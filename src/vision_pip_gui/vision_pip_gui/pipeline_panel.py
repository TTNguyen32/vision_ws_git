"""rqt control panel for the A*STAR vision pipeline.

Replaces all four tmux panes:
    Collection : Start / End / Clear map / Clear Trajectory / Remove cluster
                                                    (collect.sh s, e; the rest are new)
    Selection  : Undo / Save / Load locked / Clear  (select.sh Backspace, Enter, l, x)
    Path       : Generate / Clear                   (path.sh p, x)
    Mesh       : status only, runs by itself        (mesh.sh)
    Parameters : crop sector, mesh depth, path offset
    Status     : node, run, state, mesh, path, trunk offset
    Quit       : closes the window; run_vision_pip then stops everything

Closing the window with its X asks the same question as Quit.

The panel keeps no collecting flag of its own. State comes from the
'latest' symlink and the .collecting marker, as the shell panes did.

Every log line is printed to stdout too, which run_vision_pip shows
in its terminal and appends to logs/gui_<stamp>.log. Mesh and spline
tool output also goes there, prefixed [mesh] / [path], unless
ECHO_TOOL_LOGS=0; it is always written to the run's logs/ folder.
"""
import copy
import os
import time

from python_qt_binding.QtCore import QEvent, QTimer
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QApplication, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPlainTextEdit,
    QPushButton, QSpinBox, QVBoxLayout, QWidget)
from rqt_gui_py.plugin import Plugin

from . import common
from . import params as pr
from . import services as sv
from .cluster import ClusterSelection, ClusterStatus
from .mesh_watcher import MeshWatcher
from .subscriptions import TrunkOffset
from .tools import ToolRun

POLL_MS = 500

# Node whose parameters the sector controls set. run_vision_pip starts it
# as 'cloud_accumulator'.
NODE_NAME = os.environ.get('VISION_NODE_NAME', 'cloud_accumulator')

# Drone -> trunk axis midpoint, published by the node.
OFFSET_TOPIC = os.environ.get('VISION_OFFSET_TOPIC', '/processed/trunk_offset')
OFFSET_STALE_S = 2.0

# Cluster-removal state line, published by the node.
CLUSTER_TOPIC = os.environ.get('VISION_CLUSTER_TOPIC',
                               '/processed/cluster_status')

START = '/start_collection'
STOP = '/stop_collection'
UNDO = '/undo_normal_selection'
SAVE = '/save_normals'
LOAD_LOCKED = '/load_locked_target'
CLEAR_NORMALS = '/clearNormals'
DUMP = '/dump_normals'
PUBLISH_PATH = '/load_and_publish_path'
CLEAR_PATH = '/clear_path'
CLEAR_MAP = '/clear_map'
CLEAR_TRAJ = '/clear_trajectory'
CLUSTER_MAP = '/cluster_map'
CLUSTER_CONFIRM = '/confirm_cluster_removal'
CLUSTER_CANCEL = '/cancel_cluster_removal'

ALL_SERVICES = (START, STOP, CLEAR_MAP, UNDO, SAVE, LOAD_LOCKED,
                CLEAR_NORMALS, DUMP, PUBLISH_PATH, CLEAR_PATH, CLEAR_TRAJ,
                CLUSTER_MAP, CLUSTER_CONFIRM, CLUSTER_CANCEL)


class PipelinePanel(Plugin):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('PipelinePanel')

        self._srv = sv.TriggerCaller(context.node)
        self._coll_busy = False
        self._sel_busy = False
        self._path_busy = False
        self._cluster_busy = False
        # Armed only once OUR /cluster_map call succeeded. The status
        # topic is latched, so a panel restart replays the last line;
        # without this flag a stale 'pending' would pop a dialog for a
        # selection the node no longer holds.
        self._cluster_armed = False
        self._cluster_asked = ''       # last 'pending' line already dialogged
        self._path_job = None          # (label, out, mesh) while building
        self._quit_confirmed = False
        self._quitting = False

        # pipeline_params.yaml as launched, plus any edits applied here.
        self._params = None
        self._params_error = ''
        base = common.params_file()
        if base is None:
            self._params_error = 'PARAMS_FILE is not set'
        else:
            try:
                data = pr.load(base)
            except Exception as e:  # noqa: BLE001 - yaml raises its own
                self._params_error = str(e)
            else:
                if pr.has_all_fields(data):
                    self._params = data
                else:
                    self._params_error = (f'{os.path.basename(base)} is '
                                          'missing one of the editable keys')

        self._build_ui(context)

        self._log(f'output dir:    {common.OUTPUT_DIR}')
        self._log(f'params:        {common.params_file() or "NOT SET"}')
        self._log(f'locked target: {common.LOCKED_TARGET_FILE}')

        # Tool arguments come from pipeline_params.yaml via load_params.py.
        self._args = None
        try:
            self._args = common.load_tool_args()
        except (RuntimeError, OSError) as e:
            self._log(f'ERROR: tool parameters not loaded: {e}')
            self._log('  mesh generation and path generation are DISABLED')
        if self._args is not None:
            for what, fn in (('mesh tool', common.MESH_RECONSTRUCT),
                             ('spline tool', common.SPLINE_SCRIPT)):
                if not os.path.isfile(fn):
                    self._log(f'WARNING: {what} not found: {fn}')
            if common.conda_run_prefix() is None:
                self._log('WARNING: conda not found - mesh and path will fail')
            elif common.ECHO_TOOL_LOGS and not common.conda_streams_live():
                self._log('NOTE: this conda has no --no-capture-output; tool '
                          'output reaches the terminal when each job ends')
            self._log('tool output in terminal: '
                      + ('on' if common.ECHO_TOOL_LOGS else 'off (files only)'))

        self._offset = TrunkOffset(context.node, OFFSET_TOPIC, self)
        self._cluster = ClusterStatus(context.node, CLUSTER_TOPIC, self)
        self._cluster.changed.connect(self._on_cluster_status)
        self._psetter = sv.ParamSetter(context.node, NODE_NAME)
        if self._params is None:
            self._log(f'NOTE: parameter editing disabled - {self._params_error}')

        self._path_tool = ToolRun('path', self)
        self._path_tool.done.connect(self._path_built)

        self._mesh = None
        if self._args is not None:
            self._mesh = MeshWatcher(self._args['MESH_LIVE_ARGS'],
                                     self._args['MESH_FINAL_ARGS'], self)
            self._mesh.log.connect(self._log)
            self._mesh.status.connect(self._mesh_lbl.setText)
            self._mesh_lbl.setText('waiting for a run')
        else:
            self._mesh_lbl.setText('DISABLED (see log)')
            self._mesh_lbl.setStyleSheet('color: red;')

        # Ask before the window's X closes the pipeline.
        self._main_window = next(
            (w for w in QApplication.topLevelWidgets()
             if isinstance(w, QMainWindow)), None)
        if self._main_window is not None:
            self._main_window.installEventFilter(self)
        else:
            self._log('WARNING: main window not found - '
                      'closing it will not ask first')

        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh)
        self._timer.start(POLL_MS)

        self._log('waiting for node services ...')
        self._refresh()

    # ============================================================ UI

    def _build_ui(self, context):
        w = QWidget()
        w.setObjectName('PipelinePanelUi')
        title = 'Vision Pipeline'
        if context.serial_number() > 1:
            title += f' ({context.serial_number()})'
        w.setWindowTitle(title)

        status = QGroupBox('Status')
        form = QFormLayout(status)
        self._node_lbl = QLabel()
        self._run_lbl = QLabel()
        self._state_lbl = QLabel()
        self._mesh_lbl = QLabel()
        self._path_lbl = QLabel('-')
        self._offset_lbl = QLabel('waiting for a cylinder fit')
        self._cluster_lbl = QLabel('-')
        for lbl in (self._mesh_lbl, self._path_lbl):
            lbl.setWordWrap(True)
        form.addRow('Node:', self._node_lbl)
        form.addRow('Run:', self._run_lbl)
        form.addRow('State:', self._state_lbl)
        form.addRow('Mesh:', self._mesh_lbl)
        form.addRow('Path:', self._path_lbl)
        form.addRow('Trunk offset:', self._offset_lbl)
        form.addRow('Clusters:', self._cluster_lbl)

        def group(title, buttons):
            box = QGroupBox(title)
            col = QVBoxLayout(box)
            for b in buttons:
                col.addWidget(b)
            col.addStretch(1)
            return box

        def button(text, slot, tip):
            b = QPushButton(text)
            b.setToolTip(tip)
            b.clicked.connect(slot)
            return b

        self._start_btn = button('Start', self._on_start,
                                 'Start accumulating a new run')
        self._end_btn = button('End', self._on_end,
                               'End accumulation and build the final mesh')
        self._clear_map_btn = button('Clear map', self._on_clear_map,
                        'Discard the point cloud, normals, mesh and path. '
                        'Collection keeps running if it was.')
        self._clear_traj_btn = button('Clear trajectory', self._on_clear_traj,
                        'Clear the generated trajectory')
        self._cluster_btn = button('Remove cluster', self._on_remove_cluster,
                        'Split the map into clusters, then click a point in '
                        'RViz (Publish Point) to delete the one you clicked')
        self._undo_btn = button('Undo last', self._on_undo,
                                'Remove the most recently selected normal')
        self._save_btn = button('Save', self._on_save,
                                'Save the selection; optionally make it the locked target')
        self._load_btn = button('Load locked', self._on_load_locked,
                                'Add the locked target normals to the selection')
        self._clear_sel_btn = button('Clear all', self._on_clear_normals,
                                     'Remove every selected normal (locked file untouched)')
        self._gen_btn = button('Generate', self._on_generate,
                               'Build a path from the current normals and newest mesh')
        self._clear_path_btn = button('Clear', self._on_clear_path,
                                      'Remove the displayed path (files are kept)')
        self._quit_btn = button('Quit pipeline', self._on_quit,
                                'Close the GUI and stop every pipeline process')

        row = QHBoxLayout()
        row.addWidget(group('Collection', [self._start_btn, self._end_btn,
                                           self._clear_map_btn, self._clear_traj_btn,
                                           self._cluster_btn]))
        row.addWidget(group('Normals  (click faces in RViz)',
                            [self._undo_btn, self._save_btn,
                             self._load_btn, self._clear_sel_btn]))
        row.addWidget(group('Path', [self._gen_btn, self._clear_path_btn]))

        params_box = QGroupBox('Parameters')
        grid = QGridLayout(params_box)
        self._spins = {}
        # Each group flows down its own column, wrapping into a fresh one
        # every GRID_ROWS fields, and the next group always starts on a
        # new column. Six sector fields and three tool fields therefore
        # land as 3 + 3 + 3, and neither list is pinned to a length.
        GRID_ROWS = 3
        col0, rows = 0, 0
        for fields in (pr.FOV_FIELDS, pr.TOOL_FIELDS):
            for i, (key, label, is_int, lo, hi, step, decimals) in enumerate(fields):
                spin = QSpinBox() if is_int else QDoubleSpinBox()
                spin.setRange(lo, hi)
                spin.setSingleStep(step)
                if not is_int:
                    spin.setDecimals(decimals)
                if self._params is not None:
                    spin.setValue(pr.get(self._params, key))
                else:
                    spin.setEnabled(False)
                self._spins[key] = spin
                c, r = divmod(i, GRID_ROWS)
                grid.addWidget(QLabel(label), r, (col0 + c) * 2)
                grid.addWidget(spin, r, (col0 + c) * 2 + 1)
                rows = max(rows, r + 1)
            col0 += -(-len(fields) // GRID_ROWS)   # ceiling division

        self._apply_btn = button(
            'Apply', self._on_apply_params,
            'Use these values from the next mesh, path or run')
        self._revert_btn = button('Revert', self._on_revert_params,
                                  'Put the spin boxes back to the applied values')
        self._apply_btn.setEnabled(self._params is not None)
        self._revert_btn.setEnabled(self._params is not None)
        grid.addWidget(self._apply_btn, rows, 1)
        grid.addWidget(self._revert_btn, rows, 3)
        note = QLabel('Mesh and path values apply to the next job; '
                      'the sector applies from the next Start.')
        note.setWordWrap(True)
        grid.addWidget(note, rows + 1, 0, 1, 6)

        quit_row = QHBoxLayout()
        quit_row.addStretch(1)
        quit_row.addWidget(self._quit_btn)

        self._log_view = QPlainTextEdit()
        self._log_view.setReadOnly(True)
        self._log_view.setMaximumBlockCount(2000)
        mono = QFont('Monospace')
        mono.setStyleHint(QFont.TypeWriter)
        self._log_view.setFont(mono)

        layout = QVBoxLayout(w)
        layout.addWidget(status)
        layout.addLayout(row)
        layout.addWidget(params_box)
        layout.addLayout(quit_row)
        layout.addWidget(QLabel('Log'))
        layout.addWidget(self._log_view, 1)

        self._widget = w
        context.add_widget(w)

    def _log(self, text):
        line = f'[{time.strftime("%H:%M:%S")}] {text}'
        self._log_view.appendPlainText(line)
        print(line, flush=True)

    def _confirm(self, title, text):
        reply = QMessageBox.question(self._widget, title, text,
                                     QMessageBox.Yes | QMessageBox.No,
                                     QMessageBox.No)
        return reply == QMessageBox.Yes

    def _refresh(self):
        if self._quitting:
            return
        ready = {s: self._srv.ready(s) for s in ALL_SERVICES}
        run = common.current_run()
        collecting = common.is_collecting(run)

        missing = sum(not r for r in ready.values())
        if missing == 0:
            self._node_lbl.setText('ready')
            self._node_lbl.setStyleSheet('color: green;')
        else:
            self._node_lbl.setText(
                f'waiting for {missing} of {len(ready)} services ...')
            self._node_lbl.setStyleSheet('color: orange;')

        self._run_lbl.setText(os.path.basename(run) if run else '(none yet)')

        if collecting:
            self._state_lbl.setText('COLLECTING')
            self._state_lbl.setStyleSheet('color: red; font-weight: bold;')
        else:
            self._state_lbl.setText('idle')
            self._state_lbl.setStyleSheet('')

        self._refresh_offset()

        c, s, p = self._coll_busy, self._sel_busy, self._path_busy
        tools = self._args is not None
        self._start_btn.setEnabled(ready[START] and not collecting and not c)
        self._end_btn.setEnabled(ready[STOP] and collecting and not c)
        self._clear_map_btn.setEnabled(ready[CLEAR_MAP] and not c)
        self._clear_traj_btn.setEnabled(ready[CLEAR_TRAJ] and not p)
        # Stays enabled while armed: it is the Cancel button then.
        self._cluster_btn.setEnabled(
            ready[CLUSTER_MAP] and ready[CLUSTER_CANCEL]
            and not self._cluster_busy)
        self._cluster_btn.setText(
            'Cancel removal' if self._cluster_armed else 'Remove cluster')
        self._undo_btn.setEnabled(ready[UNDO] and not s)
        self._save_btn.setEnabled(ready[SAVE] and not s)
        self._load_btn.setEnabled(ready[LOAD_LOCKED] and not s)
        self._clear_sel_btn.setEnabled(ready[CLEAR_NORMALS] and not s)
        self._gen_btn.setEnabled(
            tools and ready[DUMP] and ready[PUBLISH_PATH] and not p)
        self._clear_path_btn.setEnabled(ready[CLEAR_PATH] and not p)

    def _refresh_offset(self):
        age = self._offset.age()
        if age is None:
            self._offset_lbl.setText('waiting for a cylinder fit')
            self._offset_lbl.setStyleSheet('')
            return
        x, y, z = self._offset.xyz
        dist = (x * x + y * y + z * z) ** 0.5
        text = (f'x {x:+.3f}  y {y:+.3f}  z {z:+.3f}  '
                f'|d| {dist:.3f} m')
        if age > OFFSET_STALE_S:
            # The node stops publishing when the fit is cleared or the
            # drone TF drops out, so say so rather than show an old value.
            self._offset_lbl.setText(f'{text}   (stale, {age:.0f}s)')
            self._offset_lbl.setStyleSheet('color: orange;')
        else:
            self._offset_lbl.setText(text)
            self._offset_lbl.setStyleSheet('')

    def _set_busy(self, attr, busy):
        setattr(self, attr, busy)
        self._refresh()

    # ==================================================== Collection

    def _on_start(self):
        self._log('button: Start')
        self._set_busy('_coll_busy', True)
        self._srv.call(START, self._start_done)

    def _start_done(self, status, message):
        self._set_busy('_coll_busy', False)

        if status != sv.OK:
            self._log(f'start failed ({status}): {message}')
            if status == sv.TIMEOUT:
                self._log('  node state unknown - check the node log '
                          'before pressing Start again')
            return

        run = message.strip()
        if not os.path.isdir(run):
            self._log(f'ERROR: node reported success but {run} '
                      'is not a directory')
            return

        latest = common.current_run()
        if latest != os.path.realpath(run):
            self._log(f'WARNING: latest -> {latest}, node said {run}')

        # Publish the marker only after the folder exists, so the mesh
        # watcher never sees a run that isn't ready yet.
        try:
            common.touch_marker(run)
        except OSError as e:
            self._log(f'ERROR: could not create marker: {e}')
            self._log('  live meshing will not start - press End')
            return

        try:
            if self._params is not None:
                dst = pr.write(self._params, os.path.join(run, 'params.yaml'),
                               'run start')
            else:
                dst = common.copy_params(run)
            self._log(f'params recorded: {os.path.basename(dst)}')
        except OSError as e:
            self._log(f'WARNING: params not recorded: {e}')

        self._log(f'started: {os.path.basename(run)}')
        self._refresh()

    def _on_end(self):
        self._log('button: End')
        run = common.current_run()
        if not common.is_collecting(run):
            self._log('  not collecting - nothing to end')
            self._refresh()
            return

        # Retract the marker first: the mesh watcher stops taking
        # snapshots and waits for the final cloud finalizeRun() writes.
        common.remove_marker(run)
        self._set_busy('_coll_busy', True)
        self._srv.call(STOP, lambda s, m: self._end_done(run, s, m))

    def _end_done(self, run, status, message):
        self._set_busy('_coll_busy', False)

        if status == sv.OK:
            self._log('stopped. Finalising - watch the Mesh status.')
        elif status == sv.UNAVAILABLE:
            # The request never reached the node, so it is still collecting.
            common.touch_marker(run)
            self._log(f'stop failed: {message} (still collecting)')
        elif status == sv.REFUSED:
            # The node only refuses when it is not collecting: stale marker.
            self._log(f'stop refused: {message}')
            self._log('  marker was stale; left removed')
        else:
            self._log(f'stop failed ({status}): {message}')
            self._log('  node state unknown - check the node log')
        self._refresh()

    def _on_clear_map(self):
        self._log('button: Clear map')
        if common.is_collecting(common.current_run()):
            text = ('Clear the point cloud and keep collecting?\n\n'
                    'Selected normals, mesh and path are cleared too. '
                    'Clicks are refused until the next live mesh. '
                    'Files already saved are kept.')
        else:
            text = ('Clear the point cloud?\n\n'
                    'The next Start begins from an empty map. Selected '
                    'normals, mesh and path are cleared too. Files already '
                    'saved are kept.')
        if not self._confirm('Clear map', text):
            self._log('  clear cancelled')
            return
        self._set_busy('_coll_busy', True)
        self._srv.call(CLEAR_MAP, self._clear_map_done)

    def _clear_map_done(self, status, message):
        self._set_busy('_coll_busy', False)
        if status == sv.OK:
            self._log(f'map cleared: {message}')
            self._path_lbl.setText('cleared')
        else:
            self._log(f'clear map failed ({status}): {message}')
            
    def _on_clear_traj(self):
        self._log('button: Clear trajectory')
        if not self._confirm('Clear trajectory',
                             'Clear the generated trajectory?\n\n'
                             'Files already saved are kept.'):
            self._log('  clear cancelled')
            return
        self._set_busy('_path_busy', True)
        self._srv.call(CLEAR_TRAJ, self._clear_traj_done)

    def _clear_traj_done(self, status, message):
        self._set_busy('_path_busy', False)
        if status == sv.OK:
            self._log(f'trajectory cleared: {message}')
            self._path_lbl.setText('cleared')
        else:
            self._log(f'clear trajectory failed ({status}): {message}')


    # =============================================== Cluster removal

    # Arm -> click in RViz -> confirm. The node refuses to arm when the
    # scan is already a single cluster, which is the normal case for a
    # clean trunk scan, so the button doubles as a check.

    def _on_remove_cluster(self):
        if self._cluster_armed:
            self._log('button: Cancel removal')
            self._set_busy('_cluster_busy', True)
            self._srv.call(CLUSTER_CANCEL, self._cluster_cancelled)
            return

        self._log('button: Remove cluster')
        self._set_busy('_cluster_busy', True)
        self._srv.call(CLUSTER_MAP, self._cluster_armed_done)

    def _cluster_armed_done(self, status, message):
        self._set_busy('_cluster_busy', False)
        if status != sv.OK:
            # success=false is the node saying "only one cluster" — an
            # answer, not a failure, so log it plainly.
            self._log(f'cluster: {message}')
            self._cluster_lbl.setText(message)
            return

        self._cluster_armed = True
        self._cluster_asked = ''
        self._log(f'cluster: {message}')
        self._cluster_lbl.setText(
            'click a point in the cluster to remove (RViz > Publish Point)')
        self._cluster_lbl.setStyleSheet('color: orange; font-weight: bold;')
        self._refresh()

    def _on_cluster_status(self, text):
        """Latched status line from the node (GUI thread, queued signal)."""
        kind = text.split('|')[0]

        if kind != 'pending':
            if kind == 'idle' and not self._cluster_busy:
                self._cluster_armed = False
                self._cluster_asked = ''
                self._cluster_lbl.setStyleSheet('')
                if self._cluster_lbl.text().startswith('click a point'):
                    self._cluster_lbl.setText('-')
            self._refresh()
            return

        if not self._cluster_armed:
            return                      # replayed line from a previous session
        if text == self._cluster_asked:
            return                      # same click, already answered
        self._cluster_asked = text

        pick = ClusterSelection.parse(text)
        if pick is None:
            self._log(f'cluster: unparsable status line: {text}')
            return

        self._log(f'cluster: picked {pick.index + 1} of {pick.total} '
                  f'({pick.points} voxels)')
        self._cluster_lbl.setText(
            f'cluster {pick.index + 1} of {pick.total} selected')

        # Modal: it blocks the poll timer, which is what we want — the
        # node holds the selection until it hears confirm or cancel.
        if self._confirm(
                'Remove cluster',
                f'Remove cluster {pick.index + 1} of {pick.total}?\n\n'
                f'{pick.points} voxels, clicked at '
                f'({pick.x:+.3f}, {pick.y:+.3f}, {pick.z:+.3f}).\n\n'
                'Its points are deleted from the map. Files already '
                'saved are kept; the mesh updates on the next build.'):
            self._set_busy('_cluster_busy', True)
            self._srv.call(CLUSTER_CONFIRM, self._cluster_removed)
        else:
            self._log('  removal cancelled')
            self._set_busy('_cluster_busy', True)
            self._srv.call(CLUSTER_CANCEL, self._cluster_cancelled)

    def _cluster_removed(self, status, message):
        self._set_busy('_cluster_busy', False)
        self._cluster_armed = False
        self._cluster_asked = ''
        self._cluster_lbl.setStyleSheet('')
        if status == sv.OK:
            self._log(f'cluster removed: {message}')
            self._cluster_lbl.setText('removed')
            self._offer_remesh()
        else:
            self._log(f'cluster removal failed ({status}): {message}')
            self._cluster_lbl.setText('removal FAILED')
        self._refresh()

    def _offer_remesh(self):
        """The cloud on disk changed; the mesh built from it has not.

        Asked rather than done automatically: a final mesh is the
        expensive job in this pipeline, and after removing the first of
        several clusters you usually want to remove the next one before
        paying for it.
        """
        if self._mesh is None:
            self._log('  mesh rebuild unavailable - mesh tool is disabled')
            return
        if not self._confirm(
                'Rebuild mesh',
                'Rebuild the mesh from the cleaned cloud?\n\n'
                'The current mesh still contains the cluster you just '
                'removed, and click-to-select and the cylinder fit both '
                'read the mesh, not the cloud.\n\n'
                'Say No if you are about to remove another cluster.'):
            self._log('  mesh not rebuilt')
            return

        handled, why = self._mesh.remesh()
        self._log(f'mesh rebuild: {why}')
        if not handled:
            self._log('  the mesh on disk still has the removed cluster in it')

    def _cluster_cancelled(self, status, message):
        self._set_busy('_cluster_busy', False)
        self._cluster_armed = False
        self._cluster_asked = ''
        self._cluster_lbl.setStyleSheet('')
        self._cluster_lbl.setText('-')
        if status != sv.OK:
            self._log(f'cluster cancel failed ({status}): {message}')
        self._refresh()

    # ==================================================== Parameters

    def _on_revert_params(self):
        self._log('button: Revert parameters')
        for key, spin in self._spins.items():
            spin.setValue(pr.get(self._params, key))

    def _on_apply_params(self):
        self._log('button: Apply parameters')
        if self._params is None:
            return

        data = copy.deepcopy(self._params)
        changed = []
        for key, spin in self._spins.items():
            new = spin.value()
            if new != pr.get(data, key):
                changed.append(f'{key}={new}')
            pr.set_value(data, key, new)

        if not changed:
            self._log('  parameters unchanged')
            return

        # Same validation that guards the launch: a bad value is rejected
        # here rather than by the tool halfway through a run.
        try:
            args = common.tool_args_for(data)
        except (RuntimeError, OSError) as e:
            self._log(f'parameters rejected: {e}')
            self._log('  nothing applied - fix the values and press Apply')
            return

        self._params = data
        self._args = args
        if self._mesh is not None:
            self._mesh.set_args(args['MESH_LIVE_ARGS'], args['MESH_FINAL_ARGS'])
        self._log('applied: ' + ', '.join(changed))

        run = common.current_run()
        if run is not None:
            try:
                dst = pr.write(data, pr.next_snapshot(run),
                               'changed: ' + ', '.join(changed))
                self._log(f'recorded: {os.path.basename(dst)}')
            except OSError as e:
                self._log(f'WARNING: snapshot not written: {e}')

        fov = {key.split('.')[-1]: pr.get(data, key)
               for key, *_ in pr.FOV_FIELDS}
        self._psetter.set_doubles(fov, self._fov_set_done)

    def _fov_set_done(self, status, message):
        if status == sv.OK:
            self._log(f'FOV sector set on the node ({message}); '
                      'applies from the next Start')
        else:
            self._log(f'FOV sector NOT set ({status}): {message}')
            self._log('  mesh and path values were still applied')

    # ===================================================== Selection

    def _simple_call(self, button, service, ok_prefix, fail_prefix):
        """Button -> one service call -> one log line."""
        self._log(f'button: {button}')
        self._set_busy('_sel_busy', True)

        def done(status, message):
            self._set_busy('_sel_busy', False)
            if status == sv.OK:
                self._log(f'{ok_prefix}: {message}')
            else:
                self._log(f'{fail_prefix} ({status}): {message}')

        self._srv.call(service, done)

    def _on_undo(self):
        self._simple_call('Undo last', UNDO, 'undo', 'undo failed')

    def _on_load_locked(self):
        self._simple_call('Load locked', LOAD_LOCKED, 'locked', 'load failed')

    def _on_clear_normals(self):
        self._log('button: Clear all')
        if not self._confirm('Clear normals',
                             'Clear ALL selected normals?\n\n'
                             'The locked target file is not changed.'):
            self._log('  clear cancelled')
            return
        self._simple_call('Clear all (confirmed)', CLEAR_NORMALS,
                          'cleared', 'clear failed')

    def _on_save(self):
        self._log('button: Save')
        self._set_busy('_sel_busy', True)
        self._srv.call(SAVE, self._save_done)

    def _save_done(self, status, message):
        self._set_busy('_sel_busy', False)
        if status != sv.OK:
            self._log(f'save failed ({status}): {message}')
            return

        saved = message.strip()
        if not os.path.isfile(saved):
            self._log(f'ERROR: node reported {saved} but it does not exist')
            return
        self._log(f'saved: {saved}')

        if not self._confirm('Locked target',
                             'Overwrite the locked target with this selection?\n\n'
                             f'{common.LOCKED_TARGET_FILE}'):
            self._log('  locked target unchanged')
            return
        try:
            common.copy_atomic(saved, common.LOCKED_TARGET_FILE)
            self._log(f'locked target updated: {common.LOCKED_TARGET_FILE}')
        except OSError as e:
            self._log(f'ERROR: could not update locked target: {e}')

    # ========================================================== Path

    def _on_generate(self):
        self._log('button: Generate path')
        run = common.current_run()
        if run is None:
            self._log('path: no run yet')
            return
        self._set_busy('_path_busy', True)
        self._srv.call(DUMP, lambda s, m: self._path_dumped(run, s, m))

    def _path_dumped(self, run, status, message):
        if status != sv.OK:
            self._log(f'path: {message}')
            self._set_busy('_path_busy', False)
            return

        normals = message.strip()
        if not os.path.isfile(normals):
            self._log(f'path: ERROR node reported {normals} '
                      'but it does not exist')
            self._set_busy('_path_busy', False)
            return

        # Final mesh once collection has ended and it exists,
        # otherwise the newest live mesh.
        ts = common.run_stamp(run)
        meshes = os.path.join(run, 'meshes')
        paths = os.path.join(run, 'paths')
        final_mesh = os.path.join(meshes, f'map_{ts}.stl')
        if not common.is_collecting(run) and os.path.isfile(final_mesh):
            label, mesh = 'final', final_mesh
            out = os.path.join(paths, f'path_{ts}.yaml')
            csv = os.path.join(paths, f'path_{ts}.csv')
        else:
            label = 'live'
            mesh = common.newest_file(meshes, 'live_', '.stl')
            num = os.path.splitext(os.path.basename(normals))[0]
            num = num[len('live_'):] if num.startswith('live_') else num
            out = os.path.join(paths, f'live_{num}.yaml')
            csv = os.path.join(paths, f'live_{num}.csv')

        if mesh is None or not os.path.isfile(mesh):
            self._log('path: no mesh available yet')
            self._set_busy('_path_busy', False)
            return

        os.makedirs(paths, exist_ok=True)
        tmp = common.tmp_name(out)
        args = [normals, mesh, '-o', tmp, '--csv', csv,
                *self._args['SPLINE_ARGS']]
        ok, why = self._path_tool.start(
            common.SPLINE_SCRIPT, args, tmp, out,
            os.path.join(run, 'logs', 'path.log'))
        if not ok:
            self._log(f'path: could not start spline: {why}')
            self._set_busy('_path_busy', False)
            return

        self._path_job = (label, out, mesh)
        self._path_lbl.setText(f'building {label} path ...')
        self._log(f'path: building {label} path from '
                  f'{os.path.basename(normals)} on {os.path.basename(mesh)}')

    def _path_built(self, ok, elapsed, detail):
        label, out, mesh = self._path_job
        if not ok:
            self._path_job = None
            self._log(f'path: spline failed ({detail}) - see path.log')
            self._path_lbl.setText('last build FAILED')
            self._set_busy('_path_busy', False)
            return
        self._srv.call(PUBLISH_PATH,
                       lambda s, m: self._path_published(elapsed, s, m))

    def _path_published(self, elapsed, status, message):
        label, out, mesh = self._path_job
        self._path_job = None
        self._set_busy('_path_busy', False)
        name = os.path.basename(out)
        if status == sv.OK:
            self._log(f'path: {label} {name}  [{os.path.basename(mesh)}]  '
                      f'{elapsed:.1f}s')
            self._path_lbl.setText(f'{name}  ({label})')
        else:
            self._log(f'path: {name} written but not published '
                      f'({status}): {message}')
            self._path_lbl.setText(f'{name} NOT published')

    def _on_clear_path(self):
        self._log('button: Clear path')
        if not self._confirm('Clear path',
                             'Clear the displayed path?\n\nFiles are kept.'):
            self._log('  clear cancelled')
            return
        self._set_busy('_path_busy', True)

        def done(status, message):
            self._set_busy('_path_busy', False)
            if status == sv.OK:
                self._log(f'path cleared: {message}')
                self._path_lbl.setText('cleared')
            else:
                self._log(f'clear path failed ({status}): {message}')

        self._srv.call(CLEAR_PATH, done)

    # ========================================================== Quit

    def _confirm_quit(self):
        text = 'Quit the WHOLE pipeline?'
        if common.is_collecting(common.current_run()):
            text += ('\n\nCollection is running and will stop '
                     'WITHOUT a final mesh.')
        jobs = []
        if self._mesh is not None and self._mesh.busy():
            jobs.append('meshing')
        if self._path_tool.running():
            jobs.append('path generation')
        if jobs:
            text += f'\n\n{" and ".join(jobs).capitalize()} will be stopped.'
        return self._confirm('Quit pipeline', text)

    def _begin_quit(self):
        self._quit_confirmed = True
        self._quitting = True
        self._timer.stop()
        for b in (self._start_btn, self._end_btn, self._clear_map_btn, self._clear_traj_btn,
                  self._cluster_btn, self._undo_btn,
                  self._save_btn, self._load_btn, self._clear_sel_btn,
                  self._gen_btn, self._clear_path_btn,
                  self._apply_btn, self._revert_btn, self._quit_btn):
            b.setEnabled(False)
        self._state_lbl.setText('shutting down ...')
        self._log('shutting down - run_vision_pip stops everything '
                  'once this window closes')

    def _on_quit(self):
        self._log('button: Quit pipeline')
        if not self._confirm_quit():
            self._log('  quit cancelled')
            return
        self._begin_quit()
        if self._main_window is not None:
            self._main_window.close()
        else:
            QApplication.instance().quit()

    def eventFilter(self, obj, event):
        # rqt sends Close twice (it saves settings in between), so only
        # ask while the quit has not been confirmed yet.
        if (obj is self._main_window and event.type() == QEvent.Close
                and not self._quit_confirmed):
            self._log('window close requested')
            if self._confirm_quit():
                self._begin_quit()
                return False
            self._log('  quit cancelled')
            event.ignore()
            return True
        return False

    # ======================================================= rqt API

    def shutdown_plugin(self):
        self._quitting = True
        self._timer.stop()
        if self._main_window is not None:
            self._main_window.removeEventFilter(self)
        if self._mesh is not None:
            self._mesh.stop()
        self._path_tool.kill()

        run = common.current_run()
        if common.is_collecting(run):
            common.remove_marker(run)
            self._log('collection was running - stopped without a final mesh')

        self._offset.destroy()
        self._cluster.destroy()
        self._psetter.destroy()
        self._srv.destroy()
        self._log('GUI closed')

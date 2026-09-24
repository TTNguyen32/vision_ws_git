"""rqt control panel for the A*STAR vision pipeline.

Replaces all four tmux panes:
    Collection : Start / End / Clear                (collect.sh s, e)
    Selection  : Undo / Save / Load locked / Clear  (select.sh Backspace, Enter, l, x)
    Path       : Generate / Clear                   (path.sh p, x)
    Mesh       : status only, runs by itself        (mesh.sh)
    Quit       : closes the window; run_vision_pip then stops everything

Closing the window with its X asks the same question as Quit.

The panel keeps no collecting flag of its own. State comes from the
'latest' symlink and the .collecting marker, as the shell panes did.

Every log line is printed to stdout too, which run_vision_pip shows
in its terminal and appends to logs/gui_<stamp>.log.
"""
import os
import time

from python_qt_binding.QtCore import QEvent, QTimer
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QApplication, QFormLayout, QGroupBox, QHBoxLayout, QLabel, QMainWindow,
    QMessageBox, QPlainTextEdit, QPushButton, QVBoxLayout, QWidget)
from rqt_gui_py.plugin import Plugin

from . import common
from . import services as sv
from .mesh_watcher import MeshWatcher
from .tools import ToolRun

POLL_MS = 500

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
ALL_SERVICES = (START, STOP, UNDO, SAVE, LOAD_LOCKED, CLEAR_NORMALS,
                DUMP, PUBLISH_PATH, CLEAR_PATH, CLEAR_MAP)


class PipelinePanel(Plugin):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('PipelinePanel')

        self._srv = sv.TriggerCaller(context.node)
        self._coll_busy = False
        self._sel_busy = False
        self._path_busy = False
        self._path_job = None          # (label, out, mesh) while building
        self._quit_confirmed = False
        self._quitting = False

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
            if not common.conda_exe():
                self._log('WARNING: conda not found - mesh and path will fail')

        self._path_tool = ToolRun(self)
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
        for lbl in (self._mesh_lbl, self._path_lbl):
            lbl.setWordWrap(True)
        form.addRow('Node:', self._node_lbl)
        form.addRow('Run:', self._run_lbl)
        form.addRow('State:', self._state_lbl)
        form.addRow('Mesh:', self._mesh_lbl)
        form.addRow('Path:', self._path_lbl)

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
        self._clear_map_btn = button(
            'Clear map', self._on_clear_map,
            'Discard the point cloud, normals, mesh and path. '
            'Collection keeps running if it was.')
        self._undo_btn = button('Undo last', self._on_undo,
                                'Remove the most recently selected normal')
        self._save_btn = button('Save', self._on_save,
                                'Save the selection; optionally make it the locked target')
        self._load_btn = button('Load locked target', self._on_load_locked,
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
        row.addWidget(group('Collection', [self._start_btn, self._end_btn, self._clear_map_btn]))
        row.addWidget(group('Normals  (click faces in RViz)',
                            [self._undo_btn, self._save_btn,
                             self._load_btn, self._clear_sel_btn]))
        row.addWidget(group('Path', [self._gen_btn, self._clear_path_btn]))

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

        c, s, p = self._coll_busy, self._sel_busy, self._path_busy
        tools = self._args is not None
        self._start_btn.setEnabled(ready[START] and not collecting and not c)
        self._end_btn.setEnabled(ready[STOP] and collecting and not c)
        self._clear_map_btn.setEnabled(ready[CLEAR_MAP] and not c)
        self._undo_btn.setEnabled(ready[UNDO] and not s)
        self._save_btn.setEnabled(ready[SAVE] and not s)
        self._load_btn.setEnabled(ready[LOAD_LOCKED] and not s)
        self._clear_sel_btn.setEnabled(ready[CLEAR_NORMALS] and not s)
        self._gen_btn.setEnabled(
            tools and ready[DUMP] and ready[PUBLISH_PATH] and not p)
        self._clear_path_btn.setEnabled(ready[CLEAR_PATH] and not p)

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
        for b in (self._start_btn, self._end_btn, self._clear_map_btn, self._undo_btn,
                  self._save_btn, self._load_btn, self._clear_sel_btn,
                  self._gen_btn, self._clear_path_btn, self._quit_btn):
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

        self._srv.destroy()
        self._log('GUI closed')

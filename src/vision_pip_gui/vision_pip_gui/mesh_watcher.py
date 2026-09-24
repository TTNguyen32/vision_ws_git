"""Mesh generation - the loop from mesh.sh, driven by a 1 s timer.

  while collecting : snapshot_NNNNN.pcd -> live_NNNNN.stl   (MESH_LIVE_ARGS)
  after collecting : global_map_TS.pcd  -> map_TS.stl       (MESH_FINAL_ARGS)

One mesh at a time. While a live mesh runs, new snapshots are not
queued: the next mesh always uses the newest snapshot, so a slow mesh
skips snapshots instead of falling further behind.
"""
import glob
import os
import time

from python_qt_binding.QtCore import QObject, QTimer, Signal

from . import common
from .tools import ToolRun

IDLE, LIVE, FINAL_WAIT, FINAL = 'idle', 'live', 'final_wait', 'final'
FINAL_WAIT_S = 120
TICK_MS = 1000


class MeshWatcher(QObject):
    log = Signal(str)
    status = Signal(str)

    def __init__(self, live_args, final_args, parent=None):
        super().__init__(parent)
        self._live_args = list(live_args)
        self._final_args = list(final_args)
        self._state = IDLE
        self._run = None
        self._last = None          # last snapshot handed to the tool
        self._wait_t0 = 0.0
        self._job = None           # (label, src, out)

        self._tool = ToolRun(self)
        self._tool.done.connect(self._on_done)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(TICK_MS)
        self.status.emit('waiting for a run')

    def busy(self):
        return self._tool.running()

    def stop(self):
        self._timer.stop()
        self._tool.kill()

    # ---------------------------------------------------------------

    def _tick(self):
        if self._tool.running():
            if self._state == LIVE:
                self._report_lag()
            return

        if self._state == IDLE:
            run = common.current_run()
            if common.is_collecting(run):
                self._run, self._last, self._state = run, None, LIVE
                self.log.emit(f'mesh: run {os.path.basename(run)}')
                self.status.emit('live - waiting for first snapshot')
            return

        if self._state == LIVE:
            if common.is_collecting(self._run):
                newest = common.newest_file(
                    os.path.join(self._run, 'maps'), 'snapshot_', '.pcd')
                if newest and newest != self._last:
                    # Recorded before the result: a failed snapshot is
                    # skipped, never retried, so it can't wedge the loop.
                    self._last = newest
                    num = os.path.basename(newest)[len('snapshot_'):-len('.pcd')]
                    out = os.path.join(self._run, 'meshes', f'live_{num}.stl')
                    self._start('live', newest, out, self._live_args)
                return

            # Marker retracted: collection has ended.
            for f in glob.glob(os.path.join(self._run, 'meshes', '.tmp_live_*.stl')):
                try:
                    os.remove(f)
                except OSError:
                    pass
            self._state = FINAL_WAIT
            self._wait_t0 = time.monotonic()
            self.log.emit('mesh: waiting for final cloud ...')
            self.status.emit('waiting for final cloud')

        if self._state == FINAL_WAIT:
            ts = common.run_stamp(self._run)
            pcd = os.path.join(self._run, 'maps', f'global_map_{ts}.pcd')
            if os.path.isfile(pcd):
                self._state = FINAL
                out = os.path.join(self._run, 'meshes', f'map_{ts}.stl')
                self._start('FINAL', pcd, out, self._final_args)
            elif time.monotonic() - self._wait_t0 >= FINAL_WAIT_S:
                self.log.emit(f'mesh: ERROR no final cloud after '
                              f'{FINAL_WAIT_S}s - check the node log')
                self._go_idle()

    def _start(self, label, src, out, args):
        tmp = common.tmp_name(out)
        log_path = os.path.join(self._run, 'logs', 'mesh.log')
        ok, why = self._tool.start(common.MESH_RECONSTRUCT,
                                   [src, tmp, *args], tmp, out, log_path)
        if not ok:
            self.log.emit(f'mesh: {label} could not start: {why}')
            if label == 'FINAL':
                self._go_idle()
            return
        self._job = (label, src, out)
        self.status.emit(f'{label} meshing {os.path.basename(src)}')

    def _on_done(self, ok, elapsed, detail):
        label, src, out = self._job
        self._job = None
        if ok:
            self.log.emit(f'mesh: {label} {os.path.basename(out)}  {elapsed:.1f}s')
        else:
            self.log.emit(f'mesh: {label} FAILED on {os.path.basename(src)} '
                          f'({detail}) - see mesh.log')

        if self._state == FINAL:
            self._go_idle()
        elif ok:
            self.status.emit(f'live - last {os.path.basename(out)} ({elapsed:.1f}s)')
        else:
            self.status.emit(f'live - last mesh FAILED ({os.path.basename(src)})')

    def _report_lag(self):
        snaps = common.sorted_files(
            os.path.join(self._run, 'maps'), 'snapshot_', '.pcd')
        behind = sum(1 for s in snaps if s > self._last) if self._last else 0
        name = os.path.basename(self._last) if self._last else '?'
        self.status.emit(f'live meshing {name}'
                         + (f' - {behind} snapshot(s) behind' if behind else ''))

    def _go_idle(self):
        self._state, self._run, self._last = IDLE, None, None
        self.log.emit('mesh: idle - waiting for the next run')
        self.status.emit('idle - waiting for the next run')

"""Run one lidar-tools script at a time through 'conda run'.

The QProcess form of mesh_one() in mesh.sh and the spline call in
path.sh:
  - output is appended to a log file and, if ECHO_TOOL_LOGS is on,
    printed to stdout with a [tag] prefix (the launcher shows stdout
    in its terminal); it never goes to the panel's own log view;
  - the tool writes a dot-prefixed temp file that is renamed on
    success, so the node never opens a partial file;
  - the tool runs under 'setsid' in its own process group, so kill()
    also stops the python that 'conda run' starts.
"""
import os
import shlex
import signal
import time

from python_qt_binding.QtCore import (QObject, QProcess,
                                      QProcessEnvironment, Signal)

from . import common


class ToolRun(QObject):
    done = Signal(bool, float, str)   # ok, elapsed seconds, detail

    def __init__(self, tag, parent=None):
        super().__init__(parent)
        self._tag = tag
        self._proc = None
        self._log = None       # open file for the current job
        self._partial = ''     # echoed text not yet ended by a newline
        self._tmp = None
        self._out = None
        self._t0 = 0.0
        self._reported = True

    def running(self):
        return self._proc is not None

    def start(self, script, args, tmp, out, log_path):
        """args must already contain tmp where the tool writes its output.

        Returns (True, '') once started, or (False, reason). 'done' is
        emitted later only if this returned True.
        """
        if self.running():
            return False, 'a job is already running'
        prefix = common.conda_run_prefix()
        if prefix is None:
            return False, 'conda not found (CONDA_EXE unset, not on PATH)'
        if not os.path.isfile(script):
            return False, f'script not found: {script}'

        argv = [*prefix, 'python', script, *args]
        header = f'=== {time.strftime("%Y-%m-%d %H:%M:%S")} {shlex.join(argv)}'
        try:
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
            self._log = open(log_path, 'a', buffering=1)
            self._log.write('\n' + header + '\n')
        except OSError as e:
            self._log = None
            return False, f'cannot write {log_path}: {e}'
        # Full command in the file; a short version in the terminal.
        first = os.path.basename(args[0]) if args else ''
        self._echo_line(f'--- {os.path.basename(script)} {first} ---')

        proc = QProcess(self)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        env = QProcessEnvironment.systemEnvironment()
        env.insert('PYTHONUNBUFFERED', '1')   # tool prints arrive as they happen
        proc.setProcessEnvironment(env)
        proc.readyReadStandardOutput.connect(self._on_output)
        proc.finished.connect(self._on_finished)
        proc.errorOccurred.connect(self._on_error)

        self._proc, self._tmp, self._out = proc, tmp, out
        self._partial = ''
        self._reported = False
        self._t0 = time.monotonic()
        proc.start('setsid', argv)
        return True, ''

    # ------------------------------------------------------- output

    def _on_output(self):
        if self._proc is None:
            return
        text = bytes(self._proc.readAllStandardOutput()).decode(
            'utf-8', errors='replace')
        if not text:
            return
        if self._log is not None:
            self._log.write(text)
        if not common.ECHO_TOOL_LOGS:
            return
        lines = (self._partial + text).split('\n')
        self._partial = lines.pop()
        for line in lines:
            self._echo_line(line)

    def _echo_line(self, line):
        if not common.ECHO_TOOL_LOGS:
            return
        # Progress bars redraw with '\r'; show only the final state.
        parts = [p for p in line.split('\r') if p.strip()]
        if parts:
            print(f'[{self._tag}] {parts[-1].rstrip()}', flush=True)

    def _close_log(self):
        if self._partial:
            self._echo_line(self._partial)
            self._partial = ''
        if self._log is not None:
            self._log.close()
            self._log = None

    # ------------------------------------------------------ results

    def _on_error(self, err):
        # Crashes are followed by finished(); only a failed start is not.
        if err == QProcess.FailedToStart:
            self._finish(False, 'could not start setsid/conda')

    def _on_finished(self, code, status):
        if self._reported:
            return   # killed, or already reported
        self._on_output()   # anything still unread
        if status != QProcess.NormalExit:
            self._finish(False, 'tool crashed')
        elif code != 0:
            self._finish(False, f'exit code {code}')
        elif not os.path.isfile(self._tmp):
            self._finish(False, 'exited 0 but wrote no output')
        else:
            try:
                os.replace(self._tmp, self._out)
            except OSError as e:
                self._finish(False, f'rename failed: {e}')
                return
            self._finish(True, '')

    def _finish(self, ok, detail):
        if self._reported:
            return
        self._reported = True
        self._close_log()
        if not ok:
            self._remove_tmp()
        elapsed = time.monotonic() - self._t0
        self._proc.deleteLater()
        self._proc = None
        self.done.emit(ok, elapsed, detail)

    def _remove_tmp(self):
        if self._tmp:
            try:
                os.remove(self._tmp)
            except OSError:
                pass

    def kill(self):
        """Stop the job without emitting 'done'."""
        if not self.running():
            return
        self._reported = True
        proc = self._proc
        pid = int(proc.processId())
        for sig, wait_ms in ((signal.SIGTERM, 1500), (signal.SIGKILL, 1000)):
            try:
                os.killpg(pid, sig)
            except (ProcessLookupError, PermissionError):
                proc.kill()
            if proc.waitForFinished(wait_ms):
                break
        self._close_log()
        self._remove_tmp()
        proc.deleteLater()
        self._proc = None

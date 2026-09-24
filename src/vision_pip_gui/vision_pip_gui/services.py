"""Non-blocking std_srvs/Trigger calls for the rqt panel.

rqt spins context.node on its own thread, so futures complete there.
Results are passed to the Qt thread through a signal before any
callback runs, so callbacks may touch widgets.
"""
from python_qt_binding.QtCore import QObject, QTimer, Signal
from std_srvs.srv import Trigger

TIMEOUT_MS = 5000

# Outcome passed to every on_done(status, message) callback.
OK = 'ok'                    # node replied success=True
REFUSED = 'refused'          # node replied success=False
UNAVAILABLE = 'unavailable'  # request never sent
TIMEOUT = 'timeout'          # sent, no reply in time: node state unknown
ERROR = 'error'              # sent, future raised


class _Bridge(QObject):
    result = Signal(object, object)   # (token, future)


class TriggerCaller:
    """on_done(status, message) runs on the Qt thread exactly once,
    unless destroy() has been called first."""

    def __init__(self, node):
        self._node = node
        self._clients = {}
        self._closed = False
        self._bridge = _Bridge()
        self._bridge.result.connect(self._on_result)

    def _client(self, name):
        if name not in self._clients:
            self._clients[name] = self._node.create_client(Trigger, name)
        return self._clients[name]

    def ready(self, name):
        return not self._closed and self._client(name).service_is_ready()

    def call(self, name, on_done):
        if self._closed:
            return
        cli = self._client(name)
        if not cli.service_is_ready():
            # Never send to a service that isn't there: the request would
            # wait forever, like 'ros2 service call' does.
            on_done(UNAVAILABLE, f'{name} is not available')
            return

        # 'finished' is only read and written on the Qt thread.
        token = {'name': name, 'on_done': on_done, 'finished': False}
        future = cli.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self._bridge.result.emit(token, f))
        QTimer.singleShot(TIMEOUT_MS, lambda: self._on_timeout(token))

    def _on_result(self, token, future):
        if self._closed or token['finished']:
            return
        token['finished'] = True
        try:
            res = future.result()
        except Exception as e:  # noqa: BLE001 - report anything
            token['on_done'](ERROR, f'{token["name"]} raised: {e}')
            return
        if res is None:
            token['on_done'](ERROR, f'{token["name"]} returned nothing')
            return
        token['on_done'](OK if res.success else REFUSED, res.message)

    def _on_timeout(self, token):
        if self._closed or token['finished']:
            return
        token['finished'] = True
        token['on_done'](
            TIMEOUT, f'{token["name"]}: no reply after {TIMEOUT_MS / 1000:g} s')

    def destroy(self):
        self._closed = True
        for cli in self._clients.values():
            self._node.destroy_client(cli)
        self._clients.clear()

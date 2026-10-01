"""Topic subscriptions for the panel.

rqt spins context.node on its own thread, so messages arrive there.
Each value crosses to the Qt thread through a signal before any widget
sees it, exactly as service replies do.
"""
import time

from geometry_msgs.msg import Vector3Stamped
from python_qt_binding.QtCore import QObject, Signal


class TrunkOffset(QObject):
    """Latest drone -> trunk axis offset, or None if nothing arrived yet."""

    received = Signal(float, float, float)   # x, y, z in the node's frame

    def __init__(self, node, topic, parent=None):
        super().__init__(parent)
        self._node = node
        self.topic = topic
        self.xyz = None
        self.stamp = 0.0          # monotonic time of the last message
        self.received.connect(self._store)
        self._sub = node.create_subscription(
            Vector3Stamped, topic, self._on_msg, 10)

    def _on_msg(self, msg):
        # rclpy thread: emit only, touch nothing else.
        self.received.emit(msg.vector.x, msg.vector.y, msg.vector.z)

    def _store(self, x, y, z):
        self.xyz = (x, y, z)
        self.stamp = time.monotonic()

    def age(self):
        """Seconds since the last message, or None if there has been none."""
        return None if self.xyz is None else time.monotonic() - self.stamp

    def destroy(self):
        self._node.destroy_subscription(self._sub)

"""Cluster-removal status for the vision pipeline panel.

The node publishes one latched line on /processed/cluster_status:

    idle
    armed|<n_clusters>
    pending|<index>|<n_clusters>|<points>|<x>|<y>|<z>

This module turns that line into a Qt signal. The point is the thread
hop: rclpy delivers the message on the executor thread, and a Qt dialog
may only be opened from the GUI thread. A signal emitted across a
QObject's thread affinity becomes a queued connection, so the panel's
slot runs on the GUI thread and QMessageBox is safe there.

Mirrors the TrunkOffset pattern in subscriptions.py; merge it in there
if you would rather keep every subscription in one file.
"""
from python_qt_binding.QtCore import QObject, Signal
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String

IDLE = 'idle'
ARMED = 'armed'
PENDING = 'pending'


class ClusterSelection:
    """One parsed 'pending|...' line."""

    __slots__ = ('index', 'total', 'points', 'x', 'y', 'z')

    def __init__(self, index, total, points, x, y, z):
        self.index = index
        self.total = total
        self.points = points
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def parse(cls, line):
        """'pending|1|3|842|0.31|0.04|0.21' -> ClusterSelection, or None.

        Returns None rather than raising: a malformed line must not take
        the panel down, and the worst case is that no dialog appears.
        """
        parts = line.split('|')
        if len(parts) != 7 or parts[0] != PENDING:
            return None
        try:
            return cls(int(parts[1]), int(parts[2]), int(parts[3]),
                       float(parts[4]), float(parts[5]), float(parts[6]))
        except ValueError:
            return None


class ClusterStatus(QObject):
    """Subscribes to the node's cluster status line."""

    changed = Signal(str)

    def __init__(self, node, topic, parent=None):
        super().__init__(parent)
        self._node = node
        self.text = IDLE

        # Must match the publisher: it is QoS(1).transient_local() and
        # RELIABLE by default. A VOLATILE subscription would connect but
        # never receive the latched line, which is the same QoS trap the
        # processed-map keepalive works around.
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self._sub = node.create_subscription(String, topic, self._on_msg, qos)

    def _on_msg(self, msg):
        # rclpy executor thread. Touch nothing but the signal.
        self.text = msg.data
        self.changed.emit(msg.data)

    def kind(self):
        return self.text.split('|')[0]

    def destroy(self):
        self._node.destroy_subscription(self._sub)

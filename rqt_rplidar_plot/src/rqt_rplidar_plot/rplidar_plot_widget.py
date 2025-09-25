import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan

from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer

import pyqtgraph as pg
import numpy as np
import threading

class RPLidarPlotWidget(QWidget):
    def __init__(self, node: Node):
        super(RPLidarPlotWidget, self).__init__()
        self.setWindowTitle('RPLIDAR Plot (PyQtGraph)')

        self.node = node

        # PyQtGraph Plot Widget
        self.plot_widget = pg.PlotWidget()
        self.plot_item = self.plot_widget.getPlotItem()
        self.plot_item.setAspectLocked(True)
        self.plot_item.showGrid(x=True, y=True)
        self.plot_item.setTitle("RPLIDAR Scan")

        self.scatter = pg.ScatterPlotItem(pen=None, brush=pg.mkBrush(255, 255, 255, 120), size=3)
        self.plot_item.addItem(self.scatter)

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.plot_widget)
        self.setLayout(layout)

        # Data storage and lock
        self._ranges = np.array([])
        self._angles = np.array([])
        self._lock = threading.Lock()

        # Use sensor data QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscriber on the node passed from the plugin context
        self.subscription = self.node.create_subscription(
            LaserScan,
            'scan',
            self._scan_callback,
            qos_profile)

        # QTimer for updating the plot
        self.timer = QTimer()
        self.timer.setInterval(33)  # ~30 Hz
        self.timer.timeout.connect(self._update_plot)
        self.timer.start()

    def _scan_callback(self, msg):
        with self._lock:
            self._ranges = np.array(msg.ranges)
            valid_indices = np.isfinite(self._ranges) & (self._ranges > 0)

            # Use a temporary variable for linspace to avoid shape mismatch
            num_points = len(msg.ranges)
            all_angles = np.linspace(msg.angle_min, msg.angle_max, num_points)

            self._ranges = self._ranges[valid_indices]
            self._angles = all_angles[valid_indices]

    def _update_plot(self):
        with self._lock:
            ranges = self._ranges
            angles = self._angles

        if len(ranges) == 0:
            return

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        self.scatter.setData(x=x, y=y)

    # No shutdown method is needed as the node lifecycle is managed by rqt
    # The QTimer will be garbage collected with the widget.
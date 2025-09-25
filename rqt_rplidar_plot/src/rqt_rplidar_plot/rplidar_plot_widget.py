import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan

from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer

import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

import numpy as np
import threading
import math

class RPLidarPlotWidget(QWidget):
    def __init__(self):
        super(RPLidarPlotWidget, self).__init__()
        self.setWindowTitle('RPLIDAR Plot')

        # Matplotlib Figure
        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111, polar=True)
        self.canvas = FigureCanvas(self.fig)
        self.toolbar = NavigationToolbar(self.canvas, self)

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        # Data storage and lock
        self._ranges = []
        self._angles = []
        self._lock = threading.Lock()

        # ROS 2 Node and Subscriber
        self.node = Node('rplidar_plot_node')

        # Use sensor data QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.node.create_subscription(
            LaserScan,
            'scan',
            self._scan_callback,
            qos_profile)

        # Thread for spinning ROS node
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.ros_thread.start()

        # QTimer for updating the plot
        self.timer = QTimer()
        self.timer.setInterval(33)  # ~30 Hz
        self.timer.timeout.connect(self._update_plot)
        self.timer.start()

    def _scan_callback(self, msg):
        with self._lock:
            self._ranges = np.array(msg.ranges)
            # Filter out infinity values
            self._ranges[np.isinf(self._ranges)] = 0.0

            # Create corresponding angles
            self._angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

    def _update_plot(self):
        with self._lock:
            # Make a copy to avoid holding the lock while plotting
            ranges = self._ranges
            angles = self._angles

        if len(ranges) == 0:
            return

        self.ax.clear()
        self.ax.scatter(angles, ranges, s=1) # s is marker size
        self.ax.set_theta_zero_location('N') # Set 0 degrees to the top
        self.ax.set_theta_direction(-1) # Clockwise
        self.ax.set_rmax(np.max(ranges) * 1.1) # Set radial limit
        self.ax.set_title("RPLIDAR Scan", va='bottom')
        self.canvas.draw()

    def shutdown(self):
        self.timer.stop()
        if self.node:
            self.node.destroy_node()
        # rclpy.shutdown() is not called here because rqt manages the context
        # and shutting it down would break other plugins.
        self.ros_thread.join()
        self.node = None
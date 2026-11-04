import sys
from threading import Event
import rclpy
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from tablet_gui.gui import MainWindow
from tablet_gui.ros_gui_node import RosGuiNode


def main(args=None):
    rclpy.init(args=args)
    node = RosGuiNode()
    app = QApplication(sys.argv)
    event = Event()
    # ROS callbacks and widget changes share one thread; no concurrent dictionary mutation.
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0) if rclpy.ok() else app.quit())
    timer.start(5)
    window = MainWindow(node, event)
    window.showMaximized()
    try:
        app.exec()
    finally:
        timer.stop()
        node.deadman_pressed = False
        node.watchdog()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

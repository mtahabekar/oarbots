"""Offscreen GUI construction, ROS event-loop and shutdown regression."""
import os
from threading import Event
import pytest
import rclpy


def test_tablet_gui_offscreen():
    os.environ['QT_QPA_PLATFORM']='offscreen'
    widgets=pytest.importorskip('PySide6.QtWidgets')
    from PySide6.QtCore import QTimer
    from tablet_gui.gui import MainWindow
    from tablet_gui.ros_gui_node import RosGuiNode
    rclpy.init()
    app=widgets.QApplication.instance() or widgets.QApplication([])
    node=RosGuiNode()
    window=MainWindow(node,Event())
    timer=QTimer()
    timer.timeout.connect(lambda:rclpy.spin_once(node,timeout_sec=0.0))
    timer.start(5)
    try:
        window.show()
        QTimer.singleShot(600,app.quit)
        app.exec()
        assert not window.grab().isNull()
        assert len(node.oarbot_settings_dict)==2
    finally:
        timer.stop(); window.close(); node.destroy_node(); rclpy.shutdown()

import numpy as np
import cv2
import pytest
from geometry_msgs.msg import PoseArray
from aruco_interfaces.msg import ArucoMarkers
from aruco_pose_estimation.pose_estimation import create_detector, pose_estimation, depth_to_pointcloud_centroid


def marker_image():
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker=cv2.aruco.generateImageMarker(dictionary, 4, 200)
    else:
        marker=cv2.aruco.drawMarker(dictionary, 4, 200)
    gray=np.full((480,640),255,np.uint8)
    gray[140:340,220:420]=marker
    return cv2.cvtColor(gray,cv2.COLOR_GRAY2RGB)


def test_synthetic_marker_pose_on_system_opencv():
    matrix=np.array([[500.,0.,320.],[0.,500.,240.],[0.,0.,1.]])
    image, poses, markers=pose_estimation(marker_image(),None,create_detector(cv2.aruco.DICT_4X4_50),
        .2,matrix,np.zeros(5),PoseArray(),ArucoMarkers())
    assert list(markers.marker_ids)==[4]
    assert poses.poses[0].position.z == pytest.approx(.5,abs=.01)
    q=poses.poses[0].orientation
    assert np.linalg.norm([q.x,q.y,q.z,q.w]) == pytest.approx(1.)


def test_depth_roi_invalid_and_valid_data():
    depth=np.zeros((20,20),dtype=np.uint16)
    corners=np.array([[[3.,3.],[16.,3.],[16.,16.],[3.,16.]]])
    matrix=np.array([[100.,0.,10.],[0.,100.,10.],[0.,0.,1.]])
    assert np.isnan(depth_to_pointcloud_centroid(depth,matrix,corners)).all()
    depth[4:16,4:16]=1000
    assert depth_to_pointcloud_centroid(depth,matrix,corners)[2] == 1.

"""Keep synthetic ROS tests on localhost, away from the cell."""
import os

os.environ['ROS_DOMAIN_ID'] = '197'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/oarbots-perception-test-logs')

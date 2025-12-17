#
# DJI Drone links and relevant frames
# Most are published by the dji_captain node
#

# UTM == Universal Transverse Mercator, globally defined frame that is usually very far that floats have problem doing math
# UTM altitude 0 = sea level
string UTM = 'utm'
# A point inside UTM, also at sea level. Use this for basically all control and navigation.
string MAP = 'map'
# ODOM == HOME_POINT == where the drone is turned on usually.
# Not necessarily at sea level. Could be above, could be below.
string ODOM = 'odom'
string HOME_POINT = 'home_point'

# The drone's body
string BASE_LINK = 'base_link'
# Same BASE_LINK, but only has the yaw component of the orientation.
string BASE_FLAT = 'base_flat_link'


# parts of the drone
string GIMBAL_CAMERA_LINK = 'gimbal_camera_link'
string WINCH_LINK = 'winch_link' # should be right-leg T junction


# Used by hook_estimator
string HOOK_LINK = 'hook_link'

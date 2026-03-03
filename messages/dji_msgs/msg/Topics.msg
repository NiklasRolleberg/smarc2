# Used by the dji_captain, namespaced as needed.
# Usually under /M350

# moving around
string MOVE_TO_SETPOINT_TOPIC = 'move_to_setpoint' # PoseStamped
string LABELED_UTM_TOPIC = 'labeled_utm_topic' # String, formatted: "utm_{zone}_{band}"

# user feedback
string CONTROLLER_INPUT_TOPIC = 'joy' # Joy
string CONTROLLER_VIBRATION_TOPIC = 'joy/set_feedback' # JoyFeedback
string TTS_TOPIC = 'speak' # String

# cameras
string GIMBAL_CAMERA_RAW_TOPIC = 'gimbal_camera/camera/image_raw' # Image
string FISHEYE_CAMERA_RAW_TOPIC = 'fisheye_camera/camera/image_raw' # Image

# object detectors in normalized image coordinates [-1,1]
# (0,0) is the center of the image, +x is right, +y is up!
# If this is not the case, FIX IT.
string ESTIMATED_BUOY_TOPIC = 'alars_detection/buoy' # PointStamped, 
string ESTIMATED_AUV_TOPIC = 'alars_detection/auv' # PointStamped
string ESTIMATED_AUV_OBB_TOPIC = "alars_detection/auv_obb" # PolygonStamped
string ESTIMATED_AUV_HEAD_TOPIC = "alars_detection/auv_head" # PolygonStamped
string ESTIMATED_BUOY_OBB_TOPIC = "alars_detection/buoy_obb" # PolygonStamped
string ESTIMATED_MIDDLE_TOPIC = 'alars_detection/middle' # PointStamped
string ESTIMATED_CNN_HOOK_ANCHOR_TOPIC = 'alars_detection/cnn' # PointStamped
string ESTIMATED_CNN_AFTER_CATCH_FLY_DIRECTION_TOPIC = 'alars_detection/cnn_vec' # PointStamped
# instead of captain subbing to image and so on
# let the consumer of camera data tell it if the camera is good, detection can happen etc.
string CAM_PROCESSOR_HAPPY_TOPIC = 'alars_detection/cam_processor_happy' # Bool

# projected points
string PROJECTED_AUV_HEAD_TOPIC = 'alars_projection/auv_head' # PolygonStamped
string PROJECTED_AUV_OBB_TOPIC = 'alars_projection/auv_obb' # PolygonStamped
string PROJECTED_BUOY_OBB_TOPIC = 'alars_projection/buoy_obb' # PolygonStamped

# object detectors in 3D
string ESTIMATED_HOOK_TOPIC = 'alars_detection/hook_est' #PoseStamped
string ESTIMATED_EKF_HOOK_TOPIC = 'alars_detection/hook_ekf' #PoseStamped

# services
string ENABLE_ALARS_DETECTOR_SERVICE_TOPIC = 'enable_alars_detector' # Trigger
string DISABLE_ALARS_DETECTOR_SERVICE_TOPIC = 'disable_alars_detector' # Trigger
string ENABLE_HOOK_ESTIMATOR_SERVICE_TOPIC = 'enable_hook_estimator' # Trigger
string DISABLE_HOOK_ESTIMATOR_SERVICE_TOPIC = 'disable_hook_estimator' # Trigger

# sensors that are not coming from PSDK
string LOAD_CELL_RAW_TOPIC = 'sensor/load_cell_raw' # Int32
string LOAD_CELL_WEIGHT_TOPIC = 'sensor/load_cell_weight' # Float32

#! /bin/bash
ROBOT_NAME=evolo
SESSION=${ROBOT_NAME}_bringup

# New variables for wasp_bt.launch and wasp_mqtt_agent.launch
AGENT_TYPE=surface
PULSE_RATE=0.5 # Hz
CONTEXT=evolo # change this to 'smarc' or something else, then connect to the same context using sim to avoid clutter

BT_LOG_MODE=compact # can be 'compact' or 'verbose'

#Dune backseat driver
DUNE_BACKSEAT_DRIVER=False #[True , False]

#Simulation
SIM=False
if [ "$SIM" = "True" ]; then
    REALSIM=simulation
    ROBOT_NAME=evolo
    USE_SIM_TIME=True
else
    REALSIM=real
    #USE_SIM_TIME=False
    USE_SIM_TIME=False #Useful for rosbags
    LOCATION_SOURCE=SBG #[SBG MQTT SERIAL]
    CAPTAIN_COM=NONE #[SERIAL MQTT]
fi

#Low controllers
tmux -2 new-session -d -s $SESSION -n 'controllers'
tmux select-window -t $SESSION:0
tmux send-keys "ros2 launch evolo_controllers evolo_controllers_launch.py" C-m

# BT, action servers etc.
tmux new-window -t $SESSION:1 -n 'bt'
tmux select-window -t $SESSION:1
tmux send-keys "ros2 launch wasp_bt wasp_bt.launch robot_name:=$ROBOT_NAME agent_type:=$AGENT_TYPE pulse_rate:=$PULSE_RATE use_sim_time:=$USE_SIM_TIME bt_log_mode:=$BT_LOG_MODE" C-m

# Action servers that are "constantly running"
tmux new-window -t $SESSION:2 -n 'servers'
tmux select-window -t $SESSION:2
tmux select-pane -t $SESSION:2.0
tmux split-window -h -t $SESSION:2.0
tmux split-window -v -t $SESSION:2.0
tmux split-window -v -t $SESSION:2.1
tmux select-layout -t $SESSION:2 tiled

#launch action servers
tmux select-pane -t $SESSION:2.0
tmux send-keys "sleep 4; ros2 run evolo_move_to move_to_server --ros-args -r __ns:=/$ROBOT_NAME -p use_sim_time:=$USE_SIM_TIME" C-m
tmux select-pane -t $SESSION:2.1
#tmux send-keys "sleep 4; ros2 run evolo_move_path move_path_server_dubins_curves --ros-args -r __ns:=/$ROBOT_NAME -p use_sim_time:=$USE_SIM_TIME" C-m
tmux send-keys "sleep 4; ros2 run evolo_move_path move_path_server_dubins_curves --ros-args -r __ns:=/$ROBOT_NAME -p use_sim_time:=$USE_SIM_TIME --params-file \$(ros2 pkg prefix evolo_move_path)/share/evolo_move_path/config/evolo_params.yaml" C-m
tmux select-pane -t $SESSION:2.2
tmux send-keys "sleep 4; ros2 run evolo_external_control externalcontrol_server --ros-args -r __ns:=/$ROBOT_NAME -p use_sim_time:=$USE_SIM_TIME" C-m
#tmux select-pane -t $SESSION:2.3
#tmux send-keys "sleep 4; ros2 run lolo_loiter server --ros-args -r __ns:=/$ROBOT_NAME -p use_sim_time:=$USE_SIM_TIME" C-m

# Mqtt bridge.
tmux new-window -t $SESSION:3 -n 'mqtt'
tmux select-window -t $SESSION:3

# To connect to smarc MQTT broker
if [ "$REALSIM" = "real" ]; then
    tmux send-keys "sleep 7; ros2 launch str_json_mqtt_bridge waraps_bridge.launch broker_addr:=20.240.40.232 broker_port:=1884 robot_name:=$ROBOT_NAME domain:=$AGENT_TYPE realsim:=$REALSIM use_sim_time:=$USE_SIM_TIME context:=$CONTEXT" C-m
else
    #tmux send-keys "sleep 7; ros2 launch str_json_mqtt_bridge waraps_bridge.launch broker_addr:=20.240.40.232 broker_port:=1884 robot_name:=$ROBOT_NAME domain:=$AGENT_TYPE realsim:=$REALSIM use_sim_time:=$USE_SIM_TIME context:=$CONTEXT" C-m
    tmux send-keys "sleep 7; ros2 launch str_json_mqtt_bridge waraps_bridge.launch broker_addr:=127.0.0.1 broker_port:=1883 robot_name:=$ROBOT_NAME domain:=$AGENT_TYPE realsim:=$REALSIM use_sim_time:=$USE_SIM_TIME context:=$CONTEXT" C-m
fi


# launch hardware drivers / connection to simulator
if [ "$REALSIM" = "real" ]; then

    #Connection to evolo captain
    tmux new-window -t $SESSION:4 -n 'Evolo captain'
    tmux select-window -t $SESSION:4
    
    if [ "$CAPTAIN_COM" = "SERIAL" ]; then
        tmux send-keys "ros2 launch evolo_serial_bridge evolo_serial_launch.py" C-m
    fi
    if [ "$CAPTAIN_COM" = "MQTT" ]; then
        tmux send-keys "ros2 launch evolo_mqtt_bridge evolo_mqtt_launch.py" C-m
    fi
    #else None
    

    #SBG driver
    tmux new-window -t $SESSION:5 -n 'SBG driver'
    tmux select-window -t $SESSION:5
    tmux send-keys "ros2 launch evolo_config sbg_launch.py robot_name:=$ROBOT_NAME" C-m

    #Odom
    tmux new-window -t $SESSION:6 -n 'Localization to Odom'
    tmux select-window -t $SESSION:6
    tmux split-window -h -t $SESSION:6.0

    if [ "$LOCATION_SOURCE" = "SBG" ]; then
        #Odom initializer
        tmux select-pane -t $SESSION:6.0
        tmux send-keys "ros2 run sbg_to_odom_initializer odom_initializer --ros-args -r __ns:=/$ROBOT_NAME -p use_sim_time:=$USE_SIM_TIME" C-m

        #sbg to odom node
        tmux select-pane -t $SESSION:6.1
        tmux send-keys "ros2 run sbg_to_odom sbg_nav_to_evolo_odom --ros-args -r __ns:=/$ROBOT_NAME -p use_sim_time:=$USE_SIM_TIME" C-m
    fi

    if [ "$LOCATION_SOURCE" = "MQTT" ] || [ "$LOCATION_SOURCE" = "SERIAL" ]; then
        #Odom initializer
        tmux select-pane -t $SESSION:6.0
        tmux send-keys "ros2 launch evolo_captain_interface evolo_captain_odom_initializer_launch.py use_sim_time:=$USE_SIM_TIME" C-m

        #sbg to odom node
        tmux select-pane -t $SESSION:6.1
        tmux send-keys "ros2 launch evolo_captain_interface evolo_captain_odom_launch.py use_sim_time:=$USE_SIM_TIME" C-m
    fi


    #Lidar driver
    tmux new-window -t $SESSION:7 -n 'lidar driver'
    tmux select-window -t $SESSION:7
    tmux send-keys "ros2 launch evolo_config lidar_launch.py ouster_ns:=$ROBOT_NAME/sensors/lidar" C-m
    
    #RTSP2web
    tmux new-window -t $SESSION:8 -n 'RTSP2web'
    tmux select-window -t $SESSION:8
    tmux send-keys "cd ~/RTSPtoWeb/ ; GO111MODULE=on go run *.go" C-m
    
    #Camera driver
    tmux new-window -t $SESSION:9 -n 'camera driver'
    tmux select-window -t $SESSION:9
    #tmux send-keys "ros2 run gscam gscam_node --ros-args -p gscam_config:="uridecodebin uri=rtsp://127.0.0.1:5541/27aec28e-6181-4753-9acd-0456a75f0289/0 source::latency=0 ! nvvidconv ! videoconvert" -p frame_id:=evolo_camera_frame -p image_encoding:=rgb8 -p sync_sink:=false -r __ns:=/evolo/gimbal_camera"
    #tmux send-keys 'ros2 run gscam gscam_node --ros-args -p gscam_config:="uridecodebin uri=rtsp://192.168.2.210 source::latency=0 ! decodebin ! videoconvert" -p frame_id:=evolo_camera_frame -p image_encoding:=rgb8 -p sync_sink:=false -r __ns:=/evolo/sensors/gimbal_camera' C-m
    GSCAM_CONFIG="rtspsrc location=rtsp://192.168.2.210 latency=0 ! rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! queue max-size-buffers=1 leaky=downstream"
    tmux send-keys "ros2 run gscam gscam_node --ros-args \
    -p gscam_config:=\"$GSCAM_CONFIG\" \
    -p frame_id:=evolo_camera_frame \
    -p image_encoding:=rgb8 \
    -p sync_sink:=false \
    -r __ns:=/$ROBOT_NAME/sensors/gimbal_camera" C-m
    
    #Gimbal driver
    tmux new-window -t $SESSION:10 -n 'gimbal driver'
    tmux select-window -t $SESSION:10
    tmux send-keys "ros2 launch z1_pro_driver z1_pro_launch.py namespace:=$ROBOT_NAME use_vehicle_altitude:=false" C-m
else #Sim
    tmux new-window -t $SESSION:4 -n 'tcp-endpoint'
    tmux select-window -t $SESSION:4
    tmux send-keys "ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1" C-m

    #TwistStamped republisher for simulator
    tmux new-window -t $SESSION:5 -n 'Twist converter'
    tmux select-window -t $SESSION:5
    tmux send-keys "ros2 run topic_tools relay /evolo/ctrl/twist_setpoint /evolo/evolo_cmd" C-m
fi

if [ "$REALSIM" = "real" ]; then
    tmux new-window -t $SESSION:11 -n 'vehicle_health'
    tmux select-window -t $SESSION:11
    tmux split-window -h -t $SESSION:11.0
    
    #Health checker
    tmux select-pane -t $SESSION:11.0
    #tmux send-keys "TODO launch health monitoring" C-m
    tmux send-keys "ros2 topic pub -r 1 /$ROBOT_NAME/smarc/vehicle_health std_msgs/msg/Int8 '{data: 0}' " C-m
    #Geofence checker
    tmux select-pane -t $SESSION:11.1
    tmux send-keys "TDODO launch geofence check"
    
else #sim
    # fake health monitoring node
    tmux new-window -t $SESSION:11 -n 'vehicle_health'
    tmux select-window -t $SESSION:11
    tmux split-window -h -t $SESSION:11.0
    
    #Health checker
    tmux select-pane -t $SESSION:11.0
    #tmux send-keys "TODO launch health monitoring" C-m
    tmux send-keys "ros2 topic pub -r 1 /$ROBOT_NAME/smarc/vehicle_health std_msgs/msg/Int8 '{data: 0}' " C-m
    #Geofence checker
    tmux select-pane -t $SESSION:11.1
    tmux send-keys "TDODO launch geofence check"
fi

#Robot description
tmux new-window -t $SESSION:12 -n 'Robot description'
tmux select-window -t $SESSION:12
tmux send-keys "ros2 launch evolo_description evolo_description.launch" C-m

# Perception
tmux new-window -t $SESSION:13 -n 'Perception'
tmux select-window -t $SESSION:13
tmux split-window -h -t $SESSION:13.0

#Pointcloud preprocessing
tmux select-pane -t $SESSION:13.0
tmux send-keys "ros2 launch pointcloud_preprocessing pointcloud_preprocessing_launch_evolo.py use_sim_time:=$USE_SIM_TIME" C-m
#occupancy grid
tmux select-pane -t $SESSION:13.1
#tmux send-keys "ros2 run clustering_segmentation clustering_segmentation --ros-args -p use_sim_time:=$USE_SIM_TIME" C-m
tmux send-keys "ros2 run clustering_segmentation clustering_segmentation --ros-args -p use_sim_time:=$USE_SIM_TIME -p DynamicStatic_clusters_segmentation:=True" C-m

#Obstacle avoidance
tmux new-window -t $SESSION:14 -n 'obstacle avoidance'
tmux select-window -t $SESSION:14
tmux send-keys "ros2 run topic_tools relay /evolo/ctrl/twist_planned /evolo/ctrl/twist_setpoint" C-m

# Logging window.
tmux new-window -t $SESSION:15 -n 'logging'
tmux select-window -t $SESSION:15

if [ "$DUNE_BACKSEAT_DRIVER" = "True" ]; then
    tmux new-window -t $SESSION:16 -n 'dune'
    tmux select-window -t $SESSION:16
    tmux send-keys "cd ~/Documents/lsts/dune; ./dune -c evolo -p Hardware" C-m

    tmux new-window -t $SESSION:17 -n 'imc_ros_bridge'
    tmux select-window -t $SESSION:17
    tmux send-keys "sleep 10; ros2 run imc_ros_bridge imc_ros2_bridge.py --ros-args -r __ns:=/$ROBOT_NAME -p use_sim_time:=$USE_SIM_TIME -p server_ip:=127.0.0.1 -p tcp_port:=7001 -p imc_src:=0x0806" C-m

    tmux new-window -t $SESSION:18 -n 'evolo imc translator'
    tmux select-window -t $SESSION:18
    tmux send-keys "ros2 run evolo_imc_translator evolo_imc_translator.py --ros-args -r __ns:=/$ROBOT_NAME -p use_sim_time:=$USE_SIM_TIME"  C-m
fi

tmux new-window -t $SESSION:19 -n 'visualization'
tmux select-window -t $SESSION:19
tmux send-keys "ros2 run rosboard rosboard_node" C-m

if [ "$REALSIM" = "real" ]; then
    tmux new-window -t $SESSION:20 -n 'message transport'
    tmux select-window -t $SESSION:20
    tmux send-keys "ros2 launch evolo_config network_bridge_evolo_tcp.launch.py" C-m
fi

tmux new-window -t $SESSION:30 -n 'zenoh router'p
tmux select-window -t $SESSION:30
tmux send-keys "ros2 run rmw_zenoh_cpp rmw_zenohd" C-m


# Set default window
tmux select-window -t $SESSION:1
tmux -2 attach-session -t $SESSION

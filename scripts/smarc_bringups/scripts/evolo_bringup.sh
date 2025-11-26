#! /bin/bash
ROBOT_NAME=evolo
SESSION=${ROBOT_NAME}_bringup
USE_SIM_TIME=False

# New variables for wasp_bt.launch and wasp_mqtt_agent.launch
AGENT_TYPE=surface
PULSE_RATE=0.5 # Hz
CONTEXT=evolo # change this to 'smarc' or something else, then connect to the same context using sim to avoid clutter

BT_LOG_MODE=compact # can be 'compact' or 'verbose'

if [ "$USE_SIM_TIME" = "True" ]; then
    REALSIM=simulation
    LINK_SUFFIX="_gt"
    ROBOT_NAME=evolo_v1
else
    REALSIM=real
    LINK_SUFFIX=""
fi

tmux -2 new-session -d -s $SESSION -n 'controllers'
tmux select-window -t $SESSION:0
tmux select-pane -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.0
tmux select-layout -t $SESSION:0 tiled
tmux select-pane -t $SESSION:0.0
tmux send-keys "ros2 launch evolo_controllers evolo_controllers_launch.py"
tmux select-pane -t $SESSION:0.1
tmux send-keys "TODO: Launch evolo description" #"sleep 2; ros2 launch lolo_description lolo_description.launch" C-m

# BT, action servers etc.
tmux new-window -t $SESSION:1 -n 'bt'
tmux select-window -t $SESSION:1
tmux send-keys "ros2 launch wasp_bt wasp_bt.launch robot_name:=$ROBOT_NAME agent_type:=$AGENT_TYPE pulse_rate:=$PULSE_RATE use_sim_time:=$USE_SIM_TIME bt_log_mode:=$BT_LOG_MODE" C-m

# controllers that are "constantly running"
tmux new-window -t $SESSION:2 -n 'servers'
tmux select-window -t $SESSION:2
tmux select-pane -t $SESSION:2.0
tmux split-window -h -t $SESSION:2.0
tmux split-window -v -t $SESSION:2.0
tmux split-window -v -t $SESSION:2.1
tmux select-layout -t $SESSION:2 tiled

#TODO launch action servers
tmux select-pane -t $SESSION:2.0
tmux send-keys "sleep 4; ros2 run evolo_move_to move_to_server --ros-args -r __ns:=/$ROBOT_NAME -p use_sim_time:=$USE_SIM_TIME" C-m
tmux select-pane -t $SESSION:2.1
tmux send-keys "sleep 4; ros2 run evolo_move_path move_path_server --ros-args -r __ns:=/$ROBOT_NAME -p use_sim_time:=$USE_SIM_TIME" C-m
#tmux select-pane -t $SESSION:2.2
#tmux send-keys "sleep 4; ros2 run lolo_emergency_action server --ros-args -r __ns:=/$ROBOT_NAME -p use_sim_time:=$USE_SIM_TIME" C-m
#tmux select-pane -t $SESSION:2.3
#tmux send-keys "sleep 4; ros2 run lolo_loiter server --ros-args -r __ns:=/$ROBOT_NAME -p use_sim_time:=$USE_SIM_TIME" C-m

# for the mqtt bridge.
tmux new-window -t $SESSION:3 -n 'mqtt_bridge'
tmux select-window -t $SESSION:3

# To connect to smarc MQTT broker
if [ "$REALSIM" = "real" ]; then
    tmux new-window -t $SESSION:3 -n 'mqtt'
    tmux select-window -t $SESSION:3
    tmux split-window -h -t $SESSION:3.0

    # Evolo / puffin broker
    tmux select-pane -t $SESSION:3.0
    tmux send-keys "ros2 launch evolo_mqtt_bridge evolo_mqtt_launch.py" C-m
    #tmux send-keys "ros2 launch evolo_serial_bridge evolo_serial_launch.py" C-m
    # Smarc broker
    tmux select-pane -t $SESSION:3.1
    tmux send-keys "sleep 7; ros2 launch str_json_mqtt_bridge waraps_bridge.launch broker_addr:=20.240.40.232 broker_port:=1884 robot_name:=$ROBOT_NAME domain:=$AGENT_TYPE realsim:=$REALSIM use_sim_time:=$USE_SIM_TIME context:=$CONTEXT" C-m
else
    tmux send-keys "sleep 7; ros2 launch str_json_mqtt_bridge waraps_bridge.launch broker_addr:=127.0.0.1 broker_port:=1883 robot_name:=$ROBOT_NAME domain:=$AGENT_TYPE realsim:=$REALSIM use_sim_time:=$USE_SIM_TIME context:=$CONTEXT"
    tmux new-window -t $SESSION:4 -n 'tcp-endpoint'
    tmux select-window -t $SESSION:4
    tmux send-keys "ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1"
fi





# launch hardware drivers if REALSIM is set to real
if [ "$REALSIM" = "real" ]; then
    
    tmux new-window -t $SESSION:4 -n 'hardware1'
    tmux select-window -t $SESSION:4
    tmux send-keys "TODO: Launch Scientist captain interface"
    tmux new-window -t $SESSION:5 -n 'lidar driver'
    tmux select-window -t $SESSION:5
    tmux send-keys "TODO: lanuch Lidar driver" C-m
    tmux new-window -t $SESSION:6 -n 'RTSP2web'
    tmux select-window -t $SESSION:6
    tmux send-keys "cd ~/RTSPtoWeb/ ; GO111MODULE=on go run *.go"
    tmux new-window -t $SESSION:7 -n 'camera driver'
    tmux select-window -t $SESSION:7
    tmux send-keys "ros2 run gscam gscam_node --ros-args -p gscam_config:="uridecodebin uri=rtsp://127.0.0.1:5541/27aec28e-6181-4753-9acd-0456a75f0289/0 source::latency=0 ! nvvidconv ! videoconvert" -p frame_id:=evolo_camera_frame -p image_encoding:=rgb8 -p sync_sink:=false -r __ns:=/evolo/gimbal_camera"
    tmux new-window -t $SESSION:8 -n 'gimbal driver'
    tmux select-window -t $SESSION:8
    tmux send-keys "TODO launch gimbal driver"
    
    echo "Launching hardware drivers in real mode."

else
    echo "Skipping hardware drivers launch in simulation mode."
fi

if [ "$REALSIM" = "real" ]; then
    tmux new-window -t $SESSION:9 -n 'vehicle_health'
    tmux select-window -t $SESSION:9
    tmux split-window -h -t $SESSION:9.0
    #Health checker
    tmux select-pane -t $SESSION:9.0
    #tmux send-keys "TODO launch health monitoring" C-m
    tmux send-keys "ros2 topic pub -r 1 /$ROBOT_NAME/smarc/vehicle_health std_msgs/msg/Int8 '{data: 0}' " C-m
    #Geofence checker
    tmux select-pane -t $SESSION:9.1
    tmux send-keys "TDODO launch geofence check"
    
else
    # fake health monitoring node
    tmux new-window -t $SESSION:9 -n 'vehicle_health'
    tmux select-window -t $SESSION:9
    tmux send-keys "ros2 topic pub -r 1 /$ROBOT_NAME/smarc/vehicle_health std_msgs/msg/Int8 '{data: 0}' " C-m
fi

# Logging window.
tmux new-window -t $SESSION:10 -n 'logging'
tmux select-window -t $SESSION:10

tmux new-window -t $SESSION:11 -n 'visualization'
tmux select-window -t $SESSION:11
tmux send-keys "TDODO launch rosboard"

tmux new-window -t $SESSION:20 -n 'zenoh router'p
tmux select-window -t $SESSION:20
tmux send-keys "ros2 run rmw_zenoh_cpp rmw_zenohd" C-m


# Set default window
tmux select-window -t $SESSION:1
tmux -2 attach-session -t $SESSION

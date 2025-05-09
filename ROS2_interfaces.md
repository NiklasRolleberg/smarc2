# ROS2 interfaces

### Topics (ROS-Standard Vehicle Interface)

/vehicle/actuator/

* /lolo/actuators/thruster_port_rpm_cmd : type=float32
* /lolo/actuators/thruster_port_rpm_fb : type = float32
* /sam/actuators/thrust_vectoring_vertical_cmd : type=float32
* /sam/actuators/thrust_vectoring_vertical_fb : type=float32


The "/lolo/actuators/thruster_port_rpm_fb" message should be published by a translation node that subscribes to "/lolo/extended/actuators/thruster_port_fb" ← Look at topic tools relay field

/vehicle/sensors/

* /lolo/sensors/gps : type=NavSatFix
* /lolo/sensors/imu : type=Imu
* /lolo/sensors/mbes : type=Pointcloud2
* /lolo/sensors/sidescan : type=Pointcloud2
* /lolo/sensors/battery: type=BatteryState
* ? /lolo/sensors/leak : type bool

/vehicle/estimate/

* /lolo/estimate/odom : type=Odometry
* /lolo/estimate/depth
* /lolo/estimate/altitude


/lolo/dr/yaw ← Help topic for control should be elsewhere!

/vehicle/control/

* /lolo/control/yaw_cmd
* /lolo/control/yawrate_cmd
* /lolo/control/speed_cmd

### Topics: Extended interface /vehicle/extended/

* /lolo/extended/sensors/mbes : type Norbit WBMS specific message will all information
* /lolo/extended/actuators/thruster_port_fb : type=Vesc specific message
* /lolo/extended/sensors/temperatures ← message with internal temperatures
* /lolo/extended/sensors/pressures ← message with internal pressures
* /lolo/extended/power_status ← message with information about what parts of the system are powered (ex EK80, MBES, lumen etc)




#### Sensor and vehicle settings

Settings should be defined as rosparams in a rosparam file. It should be possible to 

The service calls does not have to be limited to specific sensors but could also be used to change general settings on the vehicle. for example:
ros2 service call /lolo_settings smarc_service std::int="0" ← Could be standby mode / disarmed
ros2 service call /lolo_settings smarc_service std::int="1" ← Could be operational mode / armed
ros2 service call /lolo_settings smarc_service std::int="2" ← Could be operational mode + sensors powered



#### TF and Frames

UTM_ZB → map → Odom → base_link
location of map is defined by rosparam (origin_lat, origin_lon) at startup or by the first good GPS fix



#### Missions

https://api-docs.waraps.org/#/agent_communication/tasks/move_to

Generation of mission like lawnmower is done on the mission planner side. The vehicle will only receive WPs
Tasks:

* Waypoint
* Course
* Station keeping
* Change settings
* Customtask - used for experiments
    * parameters: JSON string



#### Mission plan

Unity


#### MQTT Brokers

Cloud ← YES
Vehicle: local data from onboard GUI ← YES
Vehicle: mission planner / Unity GUI ← Maybe+
Samnet:ish box ← Maybe
Laptop with unity ← No

TODO: look into mqtt relay


1st mqtt broker in the cloud/office
1-2 mqtt brokers on the vehicle
0 mqtt broker on laptop running the mission planner

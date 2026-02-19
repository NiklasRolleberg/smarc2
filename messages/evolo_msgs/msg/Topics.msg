# This file defines all the topics used on Evolo
# The goal is to have a single location that all cpp/py scripts can easily access

#Control
string EVOLO_STEERING_SETPOINT = ctrl/steering_setpoint
string EVOLO_SPEED_SETPOINT = ctrl/speed_setpoint
string TWIST_SETPOINT = ctrl/twist_setpoint #Type: TwistStamped
string TWIST_PLANNED = ctrl/twist_planned #Type: TwistStamped

#communication to and from captain
string EVOLO_CAPTAIN_TO = captain/to
string EVOLO_CAPTAIN_FROM= captain/from
string EVOLO_CAPTAIN_STATE= captain/state

string EVOLO_MQTT_RECEIVE = mqtt/fromevolo
string EVOLO_MQTT_SEND = mqtt/toevolo

#TODO LIDAR

#TDODO CAMERA & GIMBAL

#TODO SIDESCAN
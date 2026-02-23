#Data from fuelcell FCU
float32 anode_pressure
float32 cathode_pressure
float32 coolant_valve_position
float32 o2_humidity
float32 h2_humidity
float32 h2_fresh_pressure
float32 o2_fresh_pressure
float32 anode_temperature
float32 cathode_temperature
float32 o2_fresh_temp
float32 h2_fresh_temp
float32 o2_temp
float32 h2_temp
float32 stack_current
float32 stack_voltage
float32 bat_voltage
float32 min_cell_voltage
float32 max_cell_voltage
float32 avg_cell_voltage
float32 h2_concentration
float32 monitor_safety_state
float32 safety_fault
float32 startup_status
float32 shutdown_status
float32 feedback_control_setpoint
float32 stack_current_setpoint
float32 air_pump_knf_setpoint
float32 oxygen_cutoff_setpoint
float32 h2_pump_knf_setpoint
float32 coolant_pump_setpoint
float32 h2_fresh_setpoint
float32 contactor_setpoint
float32 purge_valve_setpoint
float32 o2_fresh_setpoint
float32 power_cycle_dcdc_cvm_setpoint
float32 safety_state
uint64 safety_active_mask    #     // Which errors are active NOW
uint64 safety_latched_mask	  #     // Which errors have/had been active since boot
uint8  safety_active_count   #     // How many errors active NOW
uint32 safety_total_count    #     // Total error events since boot
uint32 ms_since_boot
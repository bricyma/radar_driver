# radar_driver
assistant ROS nodes for radar driver


## delphi_srr_transformation
Transform topic `/srr_front_left/parsed_tx/srr_track` (message type: `SrrTrack`) to `/srr_rear_left/as_tx/detections` (message type: `RadarDetectionArray`)

- detect.position.x = msg.CAN_TX_DETECT_RANGE
- detect.position.y = msg.CAN_TX_DETECT_ANGLE
- detect.linear_velocity.x = msg.CAN_TX_DETECT_RANGE_RATE
- detect.amplitude = msg.CAN_TX_DETECT_AMPLITUDE
- detect.linear_velocity.z = msg.CAN_TX_DETECT_VALID_LEVEL


## vehicle_esr
Provide speed and yaw rate from `/vehicle/twist` to ESR ROS driver

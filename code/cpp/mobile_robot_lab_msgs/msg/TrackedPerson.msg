# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

uint8 TENTATIVE=0
uint8 CONFIRMED=1
uint8 LOST=2

int32 track_id
uint8 state
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
# Array header frame 기준 row-major [Pxx, Pxy, Pyx, Pyy].
float64[4] position_covariance
float64 uncertainty_radius
builtin_interfaces/Time last_measurement_stamp

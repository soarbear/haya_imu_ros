std_msgs/Header header

# Unit: dps
geometry_msgs/Vector3 angular_velocity

# Unit: (dps)^2
float64[9] angular_velocity_covariance

# Unit: m/s^2
geometry_msgs/Vector3 linear_acceleration

# Unit: (m/s)^2
float64[9] linear_acceleration_covariance

# Unit: uT
geometry_msgs/Vector3 magnetic_field

# Unit: (uT)^2
float64[9] magnetic_field_covariance

# Unit: radian, orientation[0][0:3} for 6aixs fusion, orientation[1][0:3} for 9aixs fusion
geometry_msgs/Quaternion[2] orientation

# Unit: degree, Euler angle yaw(z) -> pitch(y) -> roll(x)
geometry_msgs/Vector3 euler_ypr

# Unit: celsius degree, Internal temperature of 6axis-imu unit
float32 temperature_imu

# Unit: none, calibation index, 0x00(initial) -> 0x3f(best)
uint32 calibration_index

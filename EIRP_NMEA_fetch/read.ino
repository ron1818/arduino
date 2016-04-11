/* read data from IMU */

void Read_Gyro()
{
  gyro.read();
  
  AN[0] = gyro.g.x;
  AN[1] = gyro.g.y;
  AN[2] = gyro.g.z;
  gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
  gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
  gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
}

// Reads x,y and z accelerometer registers
void Read_Accel()
{
  compass.readAcc();
  
  AN[3] = compass.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
  AN[4] = compass.a.y >> 4;
  AN[5] = compass.a.z >> 4;
  accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
  accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
  accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
}


void Read_Compass()
{
  compass.readMag();
  
  magnetom_x = SENSOR_SIGN[6] * compass.m.x;
  magnetom_y = SENSOR_SIGN[7] * compass.m.y;
  magnetom_z = SENSOR_SIGN[8] * compass.m.z;
}


// calculate heading by mathematics
void Compass_Heading()
{
  float MAG_X;
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
  c_magnetom_x = (float)(magnetom_x - SENSOR_SIGN[6]*M_X_MIN) / (M_X_MAX - M_X_MIN) - SENSOR_SIGN[6]*0.5;
  c_magnetom_y = (float)(magnetom_y - SENSOR_SIGN[7]*M_Y_MIN) / (M_Y_MAX - M_Y_MIN) - SENSOR_SIGN[7]*0.5;
  c_magnetom_z = (float)(magnetom_z - SENSOR_SIGN[8]*M_Z_MIN) / (M_Z_MAX - M_Z_MIN) - SENSOR_SIGN[8]*0.5;
  
  // Tilt compensated Magnetic filed X:
  MAG_X = c_magnetom_x*cos_pitch+c_magnetom_y*sin_roll*sin_pitch+c_magnetom_z*cos_roll*sin_pitch;
  // Tilt compensated Magnetic filed Y:
  MAG_Y = c_magnetom_y*cos_roll-c_magnetom_z*sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-MAG_Y,MAG_X);
}

// return heading by built-in function in LSM303 library
float Compass_Heading_builtin()
{
	compass.read();
  float heading = compass.heading();
	return heading;
}



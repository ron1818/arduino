/* function to initialize objects on IMU
 * L3G
 * LSM303
 * LPS
 */

//// library
//#include <L3G.h>
//#include <LPS.h>
//#include <LSM303.h>
//
//// define objects
//LPS ps;
//L3G gyro;
//LSM303 compass;

void Gyro_Init() // L3G
{
  if (!gyro.init())
  {
    Serial.println("Failed to autodetect L3G sensor!");
    while (1);
  }
  gyro.enableDefault();

  gyro.writeReg(L3G::CTRL_REG4, 0x20); // 2000 dps full scale
  gyro.writeReg(L3G::CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
}

void Compass_Init() // LSM303 This also initializes accelerator
{
  if (!compass.init())
  {
    Serial.println("Failed to autodetect LSM303 sensor!");
    while (1);
  }
  compass.enableDefault();
	
  switch (compass.getDeviceType())
  {
    case LSM303::device_D:
      compass.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
      break;
    case LSM303::device_DLHC:
      compass.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
      break;
    default: // DLM, DLH
      compass.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
  }
}

void Ps_Init() // initialize LPS
{
	// begin LPS
  if (!ps.init())
  {
    Serial.println("Failed to autodetect LPS sensor!");
    while (1);
  }
  ps.enableDefault();
}


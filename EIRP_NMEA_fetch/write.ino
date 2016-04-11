/* output data to the serial port */

void printdata(void)
{    
      Serial.print("!");

      #if PRINT_EULER == 1
      Serial.print("ANG:");
      Serial.print(ToDeg(roll));
      Serial.print(",");
      Serial.print(ToDeg(pitch));
      Serial.print(",");
      Serial.print(ToDeg(yaw));
      #endif      
      #if PRINT_ANALOGS==1
      Serial.print(",AN:");
      Serial.print(AN[0]);  //(int)read_adc(0)
      Serial.print(",");
      Serial.print(AN[1]);
      Serial.print(",");
      Serial.print(AN[2]);  
      Serial.print(",");
      Serial.print(AN[3]);
      Serial.print (",");
      Serial.print(AN[4]);
      Serial.print (",");
      Serial.print(AN[5]);
      Serial.print(",");
      Serial.print(c_magnetom_x);
      Serial.print (",");
      Serial.print(c_magnetom_y);
      Serial.print (",");
      Serial.print(c_magnetom_z);
      #endif
      /*#if PRINT_DCM == 1
      Serial.print (",DCM:");
      Serial.print(convert_to_dec(DCM_Matrix[0][0]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[0][1]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[0][2]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[1][0]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[1][1]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[1][2]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[2][0]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[2][1]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[2][2]));
      #endif*/
      Serial.println();    
      
}

void print_pitch_roll_NMEA(void)
{
	//refer to WX series manual rev.1.000 p.32 $YXXDR-B
	//pitch and roll in degrees
  Serial.print("$YXXDR,A,");
  Serial.print(ToDeg(pitch),1);
  Serial.print(",D,PTCH,A,");
  Serial.print(ToDeg(roll),1);
  Serial.println(",D,ROLL*00");

}

void print_accelerator_NMEA(void)
{
	//refer to WX series manual rev.1.000 p.33 $YXXDR-C
	//accelerator x y z in /gravity
  Serial.print("$YXXDR,A,");
  Serial.print(float(accel_x)/GRAVITY,3);
  Serial.print(",G,XACC,A,");
  Serial.print(float(accel_y)/GRAVITY,3);
  Serial.print(",G,YACC,A,");
  Serial.print(float(accel_z)/GRAVITY,3);
  Serial.println(",G,ZACC*00");

}


void print_heading_NMEA(float compass_heading)
{
	//refer to WX series manual rev.1.000 p.25 $HCTHS
	//true heading and status
	Serial.print("$HCTHS,");
  Serial.print(compass_heading,2);
  Serial.println(",A*00");

}

void print_LPS_NMEA(void)
{
	//refer to WX series manual rev.1.000 p.30 $YXXDR-A
	//wind chill, heat index, barometric station pressure
 
	// for temeperature and pressure with IMU and I2C
	float IMUpressure = ps.readPressureMillibars();
	//float altitude = ps.pressureToAltitudeMeters(pressure);
	float IMUtemperature = ps.readTemperatureC();

  Serial.print("$YXXDR,C,");
  Serial.print(IMUtemperature,1);
  Serial.print(",C,WCHR,P,");
  Serial.print(IMUpressure/1000.0,3);
  Serial.println(",B,STNP*00");
}	


long convert_to_dec(float x)
{
  return x*10000000;
}

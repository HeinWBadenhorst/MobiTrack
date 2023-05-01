/*
   This Arduino Application receives INAV Telemetry data from an FRSky radioRF24 as input from the Drone
   which together with an on-mobile GPS and magnetometer, is used for calculating and pointing an antenna-array
   towards the flying drone. The method used to rotate around the azimuth is via stepper-motor and also a stepper-motor is used for
   adjusting the elevation of the antenna. This allows for long range flying of your drone from within a moving vehicle.
*/
// Uncomment the #define below to enable internal polling of data.
// Use only when there is no device in the S.Port chain (e.g. S.Port capable FrSky receiver) that normally polls the data.
//#define POLLING_ENABLED

#include <Wire.h>
#include <Math.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "DFRobot_RGBLCD1602.h"
#include "FrSkySportSensor.h"
#include "FrSkySportSensorInav.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportDecoder.h"
#include "nRF24L01.h" // NRF24L01 library created by TMRh20 https://github.com/TMRh20/RF24
#include "RF24.h"
#include "SPI.h"
#include <A4988.h>

//#include <BasicStepperDriver.h>
//#include <DRV8825.h>
//#include <DRV8834.h>
//#include <DRV8880.h>
//#include <MultiDriver.h>
//#include <SyncDriver.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 45
#define DIR_AZM 5
#define STEP_AZM 4
#define DIR_ELE 7
#define STEP_ELE 6
#define SLEEP 13 // optional (just delete SLEEP from everywhere if not used)
#define MS1 10
#define MS2 11
#define MS3 12

//Initialize azm and elev steppers
A4988 azm_stepper(MOTOR_STEPS, DIR_AZM, STEP_AZM, SLEEP, MS1, MS2, MS3);
A4988 ele_stepper(MOTOR_STEPS, DIR_ELE, STEP_ELE, SLEEP, MS1, MS2, MS3);


#ifdef POLLING_ENABLED
FrSkySportDecoder inavDecoder(true);  // Create inavDecoder object with polling
#else
FrSkySportDecoder inavDecoder;        // Create inavDecoder object without polling
#endif
const int SCN = 10;
const int MobileTiltAngle = 0;
const int CE = 9; //Set the CE pin connection to the nRF24
const int DIO = 8; //Set the DIO pin connection to the nRF24
static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;
unsigned long last = 0UL;
//const int MOTOR_STEPS = 200;  // change this to fit the number of steps per revolution for your motor
uint32_t currentTime, displayTime;
uint16_t inavDecoderesult;
int ReceivedMessage[1] = {000}; // Used to store value received by the NRF24L01
const uint64_t pipe = 0xE6E6E6E6E6E6; // Needs to be the same for communicating between 2 NRF24L01
int elevPotPin = A3; // potentiometer wiper (middle terminal) connected to analog pin 3
unsigned long elevPotVal = 0;  // variable to store the elevPotPin value read
unsigned long voltToDegree = 0.0277777;
unsigned long ninetyDegreeVal = 2.5;
double AngleDiff = 0;


// Create Inav sensor with default ID
FrSkySportSensorInav inav;

// The TinyGPS++ object for the mobile vehicle position
TinyGPSPlus mobileGPS;

// The serial connection to the GPS device
SoftwareSerial gpsSerial(RXPin, TXPin);

//The 4 digit 7-segment display object
//TM1637Display display(CLK, DIO);
DFRobot_RGBLCD1602 display(/*lcdCols*/16,/*lcdRows*/2);  //16 characters and 2 lines of show

//Initialize RC Telemetry Radio Link
RF24 radioRF24(CE, SCN); // NRF24L01 used SPI pins + Pin 9 and 10 on the UNO

float Heading;
float Pitch;
float Roll;
float Accx;
float Accy;
float Accz;
float Magx;
float Magy;
float Magz;
float Mag_minx;
float Mag_miny;
float Mag_minz;
float Mag_maxx;
float Mag_maxy;
float Mag_maxz;
float MobileGPSLat;
float MobileGPSLong;
float MobileElevation;
float DroneElevation;
float DroneGPSLat;
float DroneGPSLong;
double DistanceToDrone;
double DirectionToDrone;
float ElevationToDrone;
float CorrectionFactor;


/*
  Send register address and the byte value you want to write the accelerometer and
  loads the destination register with the value you send
*/
void WriteAccRegister(byte data, byte regaddress)
{
  Wire.beginTransmission(0x19);   // Use accelerometer address for regs >=0x20
  Wire.write(regaddress);
  Wire.write(data);
  Wire.endTransmission();
}

/*
  Send register address to this function and it returns byte value
  for the accelerometer register's contents
*/
byte ReadAccRegister(byte regaddress)
{
  byte data;
  Wire.beginTransmission(0x19);   // Use accelerometer address for regs >=0x20
  Wire.write(regaddress);
  Wire.endTransmission();
  delayMicroseconds(100);
  Wire.requestFrom(0x19, 1);  // Use accelerometer address for regs >=0x20
  data = Wire.read();
  Wire.endTransmission();
  delayMicroseconds(100);
  return data;
}

/*
  Send register address and the byte value you want to write the magnetometer and
  loads the destination register with the value you send
*/
void WriteMagRegister(byte data, byte regaddress)
{
  Wire.beginTransmission(0x1E);   // Else use magnetometer address
  Wire.write(regaddress);
  Wire.write(data);
  Wire.endTransmission();
  delayMicroseconds(100);
}

/*
  Send register address to this function and it returns byte value
  for the magnetometer register's contents
*/
byte ReadMagRegister(byte regaddress)
{
  byte data;
  Wire.beginTransmission(0x1E);   // Else use magnetometer address
  Wire.write(regaddress);
  Wire.endTransmission();
  delayMicroseconds(100);
  Wire.requestFrom(0x1E, 1);  // Else use magnetometer address
  data = Wire.read();
  Wire.endTransmission();
  delayMicroseconds(100);
  return data;
}

void init_Compass(void)
{
  WriteAccRegister(0x67, 0x20); // Enable accelerometer, 200Hz data output
  WriteMagRegister(0x9c, 0x00); // Enable temperature sensor, 220Hz data output
  WriteMagRegister(0x20, 0x01); // set gain to +/-1.3Gauss
  WriteMagRegister(0x00, 0x02); // Enable magnetometer constant conversions
}

/*
  Readsthe X,Y,Z axis values from the accelerometer and sends the values to the
  serial monitor.
*/
void get_Accelerometer(void)
{

  // accelerometer values
  byte xh = ReadAccRegister(0x29);
  byte xl = ReadAccRegister(0x28);
  byte yh = ReadAccRegister(0x2B);
  byte yl = ReadAccRegister(0x2A);
  byte zh = ReadAccRegister(0x2D);
  byte zl = ReadAccRegister(0x2C);
  // need to convert the register contents into a righ-justified 16 bit value
  Accx = (xh << 8 | xl);
  Accy = (yh << 8 | yl);
  Accz = (zh << 8 | zl);

}

/*
  Reads the X,Y,Z axis values from the magnetometer sends the values to the
  serial monitor.
*/
void get_Magnetometer(void)
{
  // magnetometer values
  byte xh = ReadMagRegister(0x03);
  byte xl = ReadMagRegister(0x04);
  byte yh = ReadMagRegister(0x07);
  byte yl = ReadMagRegister(0x08);
  byte zh = ReadMagRegister(0x05);
  byte zl = ReadMagRegister(0x06);
  // convert registers to ints
  Magx = (xh << 8 | xl);
  Magy = (yh << 8 | yl);
  Magz = (zh << 8 | zl);
}

/*
  Converts values to a tilt compensated heading in degrees (0 to 360)
*/
void get_TiltHeading(void)
{
  // You can use BM004_Arduino_calibrate to measure max/min magnetometer values and plug them in here.  The values
  // below are for a specific sensor and will not match yours
  Mag_minx = -221;
  Mag_miny = -275;
  Mag_minz = -262;
  Mag_maxx = 396;
  Mag_maxy = 304;
  Mag_maxz = 439;

  // use calibration values to shift and scale magnetometer measurements
  Magx = (Magx - Mag_minx) / (Mag_maxx - Mag_minx) * 2 - 1;
  Magy = (Magy - Mag_miny) / (Mag_maxy - Mag_miny) * 2 - 1;
  Magz = (Magz - Mag_minz) / (Mag_maxz - Mag_minz) * 2 - 1;
  // Normalize acceleration measurements so they range from 0 to 1
  float accxnorm = Accx / sqrt(Accx * Accx + Accy * Accy + Accz * Accz);
  float accynorm = Accy / sqrt(Accx * Accx + Accy * Accy + Accz * Accz);
  // calculate pitch and roll
  Pitch = asin(-accxnorm);
  Roll = asin(accynorm / cos(Pitch));
  // tilt compensated magnetic sensor measurements
  float magxcomp = Magx * cos(Pitch) + Magz * sin(Pitch);
  float magycomp = Magx * sin(Roll) * sin(Pitch) + Magy * cos(Roll) - Magz * sin(Roll) * cos(Pitch);
  // arctangent of y/x converted to degrees
  Heading = 180 * atan2(magycomp, magxcomp) / PI;
  if (Heading < 0)
    Heading += 360;
  Serial.print("Heading=");
  Serial.println(Heading);

  //display.write("Heading=".concat(Heading));

}

void printGPSData()
{
  // Dispatch incoming characters
  while (gpsSerial.available() > 0)
    mobileGPS.encode(gpsSerial.read());

  if (mobileGPS.location.isUpdated())
  {
    Serial.print(F("LOCATION   Fix Age="));
    Serial.print(mobileGPS.location.age());
    Serial.print(F("ms Raw Lat="));
    Serial.print(mobileGPS.location.rawLat().negative ? "-" : "+");
    Serial.print(mobileGPS.location.rawLat().deg);
    Serial.print("[+");
    Serial.print(mobileGPS.location.rawLat().billionths);
    Serial.print(F(" billionths],  Raw Long="));
    Serial.print(mobileGPS.location.rawLng().negative ? "-" : "+");
    Serial.print(mobileGPS.location.rawLng().deg);
    Serial.print("[+");
    Serial.print(mobileGPS.location.rawLng().billionths);
    Serial.print(F(" billionths],  Lat="));
    Serial.print(mobileGPS.location.lat(), 6);
    Serial.print(F(" Long="));
    Serial.println(mobileGPS.location.lng(), 6);
  }

  else if (mobileGPS.date.isUpdated())
  {
    Serial.print(F("DATE       Fix Age="));
    Serial.print(mobileGPS.date.age());
    Serial.print(F("ms Raw="));
    Serial.print(mobileGPS.date.value());
    Serial.print(F(" Year="));
    Serial.print(mobileGPS.date.year());
    Serial.print(F(" Month="));
    Serial.print(mobileGPS.date.month());
    Serial.print(F(" Day="));
    Serial.println(mobileGPS.date.day());
  }

  else if (mobileGPS.time.isUpdated())
  {
    Serial.print(F("TIME       Fix Age="));
    Serial.print(mobileGPS.time.age());
    Serial.print(F("ms Raw="));
    Serial.print(mobileGPS.time.value());
    Serial.print(F(" Hour="));
    Serial.print(mobileGPS.time.hour());
    Serial.print(F(" Minute="));
    Serial.print(mobileGPS.time.minute());
    Serial.print(F(" Second="));
    Serial.print(mobileGPS.time.second());
    Serial.print(F(" Hundredths="));
    Serial.println(mobileGPS.time.centisecond());
  }

  else if (mobileGPS.speed.isUpdated())
  {
    Serial.print(F("SPEED      Fix Age="));
    Serial.print(mobileGPS.speed.age());
    Serial.print(F("ms Raw="));
    Serial.print(mobileGPS.speed.value());
    Serial.print(F(" Knots="));
    Serial.print(mobileGPS.speed.knots());
    Serial.print(F(" MPH="));
    Serial.print(mobileGPS.speed.mph());
    Serial.print(F(" m/s="));
    Serial.print(mobileGPS.speed.mps());
    Serial.print(F(" km/h="));
    Serial.println(mobileGPS.speed.kmph());
  }

  else if (mobileGPS.course.isUpdated())
  {
    Serial.print(F("COURSE     Fix Age="));
    Serial.print(mobileGPS.course.age());
    Serial.print(F("ms Raw="));
    Serial.print(mobileGPS.course.value());
    Serial.print(F(" Deg="));
    Serial.println(mobileGPS.course.deg());
  }

  else if (mobileGPS.altitude.isUpdated())
  {
    Serial.print(F("ALTITUDE   Fix Age="));
    Serial.print(mobileGPS.altitude.age());
    Serial.print(F("ms Raw="));
    Serial.print(mobileGPS.altitude.value());
    Serial.print(F(" Meters="));
    Serial.print(mobileGPS.altitude.meters());
    Serial.print(F(" Miles="));
    Serial.print(mobileGPS.altitude.miles());
    Serial.print(F(" KM="));
    Serial.print(mobileGPS.altitude.kilometers());
    Serial.print(F(" Feet="));
    Serial.println(mobileGPS.altitude.feet());
  }

  else if (mobileGPS.satellites.isUpdated())
  {
    Serial.print(F("SATELLITES Fix Age="));
    Serial.print(mobileGPS.satellites.age());
    Serial.print(F("ms Value="));
    Serial.println(mobileGPS.satellites.value());
  }

  else if (mobileGPS.hdop.isUpdated())
  {
    Serial.print(F("HDOP       Fix Age="));
    Serial.print(mobileGPS.hdop.age());
    Serial.print(F("ms raw="));
    Serial.print(mobileGPS.hdop.value());
    Serial.print(F(" hdop="));
    Serial.println(mobileGPS.hdop.hdop());
  }

  else if (millis() - last > 5000)
  {
    Serial.println();
    if (mobileGPS.location.isValid())
    {
      static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
      double distanceToLondon =
        TinyGPSPlus::distanceBetween(
          mobileGPS.location.lat(),
          mobileGPS.location.lng(),
          LONDON_LAT,
          LONDON_LON);
      double courseToLondon =
        TinyGPSPlus::courseTo(
          mobileGPS.location.lat(),
          mobileGPS.location.lng(),
          LONDON_LAT,
          LONDON_LON);

      Serial.print(F("LONDON     Distance="));
      Serial.print(distanceToLondon / 1000, 6);
      Serial.print(F(" km Course-to="));
      Serial.print(courseToLondon, 6);
      Serial.print(F(" degrees ["));
      Serial.print(TinyGPSPlus::cardinal(courseToLondon));
      Serial.println(F("]"));
    }

    Serial.print(F("DIAGS      Chars="));
    Serial.print(mobileGPS.charsProcessed());
    Serial.print(F(" Sentences-with-Fix="));
    Serial.print(mobileGPS.sentencesWithFix());
    Serial.print(F(" Failed-checksum="));
    Serial.print(mobileGPS.failedChecksum());
    Serial.print(F(" Passed-checksum="));
    Serial.println(mobileGPS.passedChecksum());

    if (mobileGPS.charsProcessed() < 10)
      Serial.println(F("WARNING: No GPS data.  Check wiring."));

    last = millis();
    Serial.println();
  }

}


void get_GPSData()
{
  // Dispatch incoming characters
  while (gpsSerial.available() > 0)
    mobileGPS.encode(gpsSerial.read());

  if (millis() - last > 5000)
  {
    Serial.println();
    if (mobileGPS.location.isValid())
    {

      MobileGPSLat = mobileGPS.location.lat();
      MobileGPSLong = mobileGPS.location.lng();
      MobileElevation = mobileGPS.altitude.meters();
      DistanceToDrone =
        TinyGPSPlus::distanceBetween(
          MobileGPSLat,
          MobileGPSLong,
          DroneGPSLat,
          DroneGPSLat);
      DirectionToDrone =
        TinyGPSPlus::courseTo(
          MobileGPSLat,
          MobileGPSLong,
          DroneGPSLat,
          DroneGPSLat);
    }
    if (mobileGPS.charsProcessed() < 10)
      Serial.println(F("WARNING: No GPS data.  Check wiring."));
    last = millis();
    Serial.println();
  }

}


void printINAVData()
{
  // Read and decode the telemetry data, note that the data will only be decoded for sensors
  // that that have been passed to the begin method. Print the AppID of the decoded data.
  inavDecoderesult = inavDecoder.decode();
  if (inavDecoderesult != SENSOR_NO_DATA_ID) {
    Serial.print("Decoded data with AppID 0x");
    Serial.println(inavDecoderesult, HEX);
  }
  // Display data once a second to not interfeere with data decoding
  currentTime = millis();
  if (currentTime > displayTime)
  {
    displayTime = currentTime + 1000;
    Serial.println("");
    Serial.print("RSSI = "); Serial.println(inav.getRssi()); // RSSI
    Serial.print("RxBatt = "); Serial.println(inav.getRxBatt()); // RxBatt voltage in volts
    Serial.print("fuel = "); Serial.print(inav.getFuel()); Serial.println("%"); // Fuel level in percent
    // Get current/voltage sensor (FAS) data
    Serial.print("current = "); Serial.print(inav.getCurrent()); Serial.println("A"); // Current consumption in amps
    Serial.print("voltage = "); Serial.print(inav.getVoltage()); Serial.println("V"); // Battery voltage in volts
    // Get variometer sensor (FVAS) data
    Serial.print("Altitude = "); Serial.print(inav.getAltitude()); Serial.println("m"); // Altitude in m (can be nevative)
    Serial.print("Vario = "); Serial.print(inav.getVario()); Serial.println("m/s");     // Verticas speed in m/s (can be nevative)
    // Get GPS data
    Serial.print("GPS: lat = "); Serial.print(inav.getLat()); Serial.print(", lon = "); Serial.println(inav.getLon()); // Latitude and longitude in degrees decimal (positive for N/E, negative for S/W)
    Serial.print("GPS: state = "); Serial.println(inav.getGpsState());
    Serial.print("speed = "); Serial.println(inav.getSpeed()); // Speed in m/s
    // Get accelerometer sensor (TAS) data
    Serial.print("ACCX = "); Serial.print(inav.getAccX());                      // X axis acceleraton in Gs (can be negative)
    Serial.print("ACCY = "); Serial.print(inav.getAccY());                      // Y axis acceleraton in Gs (can be negative)
    Serial.print("ACCZ = "); Serial.print(inav.getAccZ()); Serial.println("G"); // Z axis acceleraton in Gs (can be negative)
    Serial.print("Heading = "); Serial.print(inav.getHeading()); Serial.println("");
    Serial.print("MODE = "); Serial.println(inav.getFlightMode());
    Serial.println("");
  }
}

void getINAVData()
{
/*  // Read and decode the telemetry data, note that the data will only be decoded for sensors
  // that that have been passed to the begin method. Print the AppID of the decoded data.
  inavDecoderesult = inavDecoder.decode();
  // Display data once a second to not interfere with data decoding
  currentTime = millis();
  if (currentTime > displayTime)
  {
    displayTime = currentTime + 1000;
    //inav.getRssi(); // RSSI
    //inav.getRxBatt(); // RxBatt voltage in volts

    DroneElevation = inav.getAltitude();
    // Get GPS data
    DroneGPSLat = inav.getLat();
    DroneGPSLong = inav.getLon();
    //inav.getSpeed();
  }
*/
    DroneGPSLat = 26;
    DroneGPSLong = -26;
}

void getINAVRadioData()
{
  // Read the radio data
  while (radioRF24.available())
  {
    radioRF24.read(ReceivedMessage, 1); // Read information from the NRF24L01
    display.write(ReceivedMessage);
    
  }
}

void point_Antenna_Azimuth()
{
  double HeadingDiff = 0;
  double CorrectionFactor = 0;
  if (DirectionToDrone > Heading) {
    if ((DirectionToDrone - Heading) <= 180) {
      HeadingDiff = DirectionToDrone - Heading;
      CorrectionFactor =  MOTOR_STEPS / 360;
      azm_stepper.rotate((int)HeadingDiff * CorrectionFactor);
      //azm_stepper.rotate((int)HeadingDiff);
    } else {
      HeadingDiff = (360 - DirectionToDrone) + Heading;
      CorrectionFactor =  MOTOR_STEPS / 360;
      azm_stepper.rotate(-(int)HeadingDiff * CorrectionFactor);
      //azm_stepper.rotate(-(int)HeadingDiff);
    }
  } else {
    if ((Heading - DirectionToDrone) <= 180) {
      HeadingDiff = Heading - DirectionToDrone;
      CorrectionFactor =  MOTOR_STEPS / 360;
      // In 1:8 microstepping mode, one revolution takes 8 times as many microsteps
      //stepper.move(8 * MOTOR_STEPS);    // forward revolution
      azm_stepper.rotate(-(int)HeadingDiff * CorrectionFactor);
    } else {
      HeadingDiff = (360 - Heading) + DirectionToDrone;
      CorrectionFactor =  MOTOR_STEPS / 360;
      azm_stepper.rotate((int)HeadingDiff * CorrectionFactor);
      //azm_stepper.rotate((int)HeadingDiff);
    }
  }
}

boolean checkElevationLimits(boolean upperLimit)
{
  if (upperLimit)
  {
    if ((elevPotVal / voltToDegree + AngleDiff) > 90)
    {
      return true;
    } else return false;
  } else if ((elevPotVal / voltToDegree - AngleDiff) < 0)
  {
    return true;
  } else return false;
}

void point_Antenna_Elevation()
{
  double elevationDiff = 0;
  elevationDiff = (DroneElevation - MobileElevation);
  double elevationAngle = tan(elevationDiff / DistanceToDrone);
  elevPotVal = analogRead(elevPotPin);
  double elevPotAngle = elevPotVal / voltToDegree;
  if (checkElevationLimits(true))
  {
    //OVER VERT LIMIT SET TO 90 DEGREES
    AngleDiff = 90 - elevPotAngle;
    ele_stepper.rotate((int)(AngleDiff  * CorrectionFactor));
  } else if (checkElevationLimits(false))
  {
    //UNDER HOR LIMIT SET TO 0 DEGREES
    AngleDiff = 0 - elevPotAngle;
    ele_stepper.rotate((int)(AngleDiff  * CorrectionFactor));

  } else
  {
    //tilt antenna to new pos
    if (elevPotAngle > elevationAngle)
    {
      AngleDiff = elevPotAngle - elevationAngle;
      ele_stepper.rotate(-(int)(AngleDiff * CorrectionFactor));
    } else
    {
      AngleDiff = elevationAngle - elevPotAngle;
      ele_stepper.rotate((int)(AngleDiff  * CorrectionFactor));
    }
  }
}

void setup() {
  Wire.begin();
  Serial.begin(9600);  // start serial for output
  gpsSerial.begin(GPSBaud);
  init_Compass();
  //display.setBrightness(0x0a);  //set the diplay to maximum brightness
  display.init();
  display.write("Test");
  azm_stepper.begin(RPM);
  ele_stepper.begin(RPM);
  // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
  // stepper.setEnableActiveState(LOW);
  azm_stepper.enable();
  ele_stepper.enable();
  //azm_stepper.setMicrostep(8);   // Set microstep mode to 1:8
  //ele_stepper.setMicrostep(8);   // Set microstep mode to 1:8
  radioRF24.begin(); // Start the NRF24L01
  radioRF24.openReadingPipe(1, pipe); // Get NRF24L01 ready to receive
  radioRF24.startListening(); // Listen to see if information received
  //inavDecoder.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_12, &inav);
}

void loop() {
  //get_Accelerometer();
  //get_Magnetometer();
  //get_TiltHeading();
  //getINAVData();
  get_GPSData();
  //printINAVData();
  printGPSData();
  //point_Antenna_Azimuth();
  //point_Antenna_Elevation();
  //display.showNumberDec(DirectionToDrone); //TM Display the Variable value;
  display.write(DirectionToDrone);
}

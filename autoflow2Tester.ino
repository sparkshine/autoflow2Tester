/**************************************************************************/
/*!
  *********************************************************
  *IMPORTANT*   FOR USE WITH BQ34Z100-G1 ONLY   *IMPORTANT*
  *********************************************************

    @file     AUTOFLOW2Tester.ino
    @author   P. Ozorio (Creasefield Ltd)
    @license  Reference Code

  This is the source code for the AUTOFLOW2 Tester.

  Property of Creasefield Ltd. Do not copy or modify without permission.

  @section  HISTORY

    v1.0-a1  - 25/08/2015 - Alpha build
    v1.0-a2  - 01/09/2015 - Replaced recursion fuction
  *********************************************************
  *IMPORTANT*   FOR USE WITH BQ34Z100-G1 ONLY   *IMPORTANT*
  *********************************************************
*/
/**************************************************************************/
// Libraries
#include <Wire.h>
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;

// Function Declaration
float measureVoltage();
void compareVoltage();
void print_title();
void readSOC();
void i2cTest();
void readRemainingCapacity();
void readVoltage();
void readFirmware();
void readAverageCurrent();
void readInstCurrent();
void shortCircuit();
void readBattTemp();
void measureTherm();
void sealPack();
void readDivider();
void calibrateVoltage(float i2cVoltage, float inaVoltage);
void failCheck();

// Constants
#define BQ34Z100 85                                                  // I2C adress for BQ34Z100                                                                    
#define FLASHVERSION 0xAAAA                                          // Check DF version matches this. Fails if not equal
#define mosfetPIN 2
const float c1 = 0.001134508, c2 = 0.000233318, c3 = 9.0327E-8;      // Steinhart & Hart Co-efficients
const float R = 14870;                                               // 

// Global Variables
float inaVoltage, i2cVoltage;
int divider;
int flashbytes[32] = {0};
int batt_temp;
uint8_t error = 0, fail = 0, calAttempts = 0;

//! Initialize Arduino
void setup()
{
  Serial.begin(9600);
  uint32_t currentFrequency;
  ina219.begin();
  Wire.begin();
  pinMode(mosfetPIN,OUTPUT);
}

//! Repeats Arduino loop
void loop()

{
  fail = 0;
  calAttempts = 0;
  error = 0;
  /* loop until i2c available */
  while (error != 0)
    i2cTest();
  delay(4000);
  readVoltage(); // Reads voltage, calibrates when needed.
  measureVoltage();
  compareVoltage();
  readSOC(); 
  shortCircuit();
  readFirmware();
  readBattTemp();
  measureTherm();
  failCheck();
  /* loop until i2c unavailable (ie. pack unplugged) */
  while (error == 0) 
    i2cTest();
}

float measureVoltage()
{
  float busvoltage = ina219.getBusVoltage_V();      // Voltage across Vin- and Gnd
  float shuntvoltage = ina219.getShuntVoltage_mV(); // Voltage across Vin+ and Vin-
  
  inaVoltage = busvoltage + (shuntvoltage / 1000);
  Serial.print("Shunt Voltage :"); Serial.println(shuntvoltage);
  Serial.print("Bus Voltage :"); Serial.println(busvoltage);
  Serial.print("Total Voltage :");Serial.println(inaVoltage);
}

void print_title()
{
  Serial.println(F("\n*****************************************************************"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*            hello world                                        *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*                                                               *\n"));
  Serial.print(F("*****************************************************************\n"));
}
/* Reads the state of charge over I2C */
void readSOC() {
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x02);
  Wire.endTransmission();

  Wire.requestFrom(BQ34Z100, 1);

  int soc = Wire.read();

  Serial.print("Battery Charge: ");
  Serial.print(soc);
  Serial.println("%");

  if (soc < 20 || soc > 60)
  {
    fail++;
    Serial.println("SOC FAIL");
  }
}

/* Checks for i2c, error = 0 when i2c is avaliable */
void i2cTest() {
  Wire.beginTransmission(BQ34Z100);
  error = Wire.endTransmission();
  delay(500);
}

void readRemainingCapacity()
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x04);
  Wire.endTransmission();

  Wire.requestFrom(BQ34Z100, 1);

  unsigned int low = Wire.read();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.requestFrom(BQ34Z100, 1);

  unsigned int high = Wire.read();

  unsigned int high1 = high << 8;

  int remain_cap = high1 + low;
}

void readVoltage()
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.requestFrom(BQ34Z100, 1);

  unsigned int low = Wire.read();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x09);
  Wire.endTransmission();

  Wire.requestFrom(BQ34Z100, 1);

  unsigned int high = Wire.read();

  unsigned int high1 = high << 8;

  i2cVoltage = high1 + low;

  Serial.print("I2C Voltage Reading: ");
  Serial.print(i2cVoltage);
  Serial.println(" mV");
  
  if (i2cVoltage < 7200 || i2cVoltage > 7600)
  {
    fail++;
    Serial.println("VOLTAGE FAIL");
  }
}
void compareVoltage()
{
   while ( i2cVoltage - inaVoltage > 20 || i2cVoltage - inaVoltage < -20)
  {
    if ( calAttempts > 2)
    {
      fail++;
      return;
    }
    readVoltage();
    measureVoltage();
    calibrateVoltage(i2cVoltage, inaVoltage);
    calAttempts++;
    delay(500);
  } 
}

void readFirmware()
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x00);
  Wire.write(0x0C);
  Wire.endTransmission();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x01);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100, 1);
  int firmware_msb = Wire.read();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100, 1);
  int firmware_lsb = Wire.read();

  firmware_msb = firmware_msb << 8;
  unsigned int firmware = firmware_lsb + firmware_msb;

  Serial.print("Flash version: ");
  Serial.println(firmware, HEX);

  if (firmware != FLASHVERSION) {
    fail++;
    Serial.println("FIRMWARE FAIL");
  }

}

void readAverageCurrent()
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x0a);
  Wire.endTransmission();

  Wire.requestFrom(BQ34Z100, 1);

  unsigned int low = Wire.read();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x0b);
  Wire.endTransmission();

  Wire.requestFrom(BQ34Z100, 1);

  unsigned int high = Wire.read();

  unsigned int high1 = high << 8;

  int avg_current = high1 + low;

  Serial.print("Average Current Draw: ");
  Serial.print(avg_current);
  Serial.println(" mA");

  if (avg_current > 2 || avg_current < -2)
  {
    fail++;
    Serial.println("CURRENT FAIL");
  }
}

void shortCircuit()
{
  digitalWrite(mosfetPIN, HIGH);
  delay(6000);
  readInstCurrent();
  delay(1000);
  digitalWrite(mosfetPIN, LOW);
}

void readInstCurrent()
{
  // Write Control command bytes to gas gauge chip: 0x00/0x01
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x00);
  Wire.write(0x18);
  Wire.endTransmission();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x01);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100, 1);
  unsigned int inst_current_lsb = Wire.read();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100, 1);
  unsigned int inst_current_msb = Wire.read();

  unsigned int temp = inst_current_msb << 8;

  unsigned int inst_current = temp + inst_current_lsb;
}

void readBattTemp() {
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x0c);
  Wire.endTransmission();

  Wire.requestFrom(BQ34Z100, 1);

  unsigned int low = Wire.read();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x0d);
  Wire.endTransmission();

  Wire.requestFrom(BQ34Z100, 1);

  unsigned int high = Wire.read();

  unsigned int high1 = high << 8;

  batt_temp = high1 + low;

  batt_temp = 0.1 * batt_temp;        // Each bit is 0.1K, so we have a value in Kelvins
  batt_temp = batt_temp - 273.15;      // Convert to degrees Celsius

  Serial.print("Battery Temperature: ");
  Serial.print(batt_temp);
  Serial.println(" C");
  if (batt_temp > 35 || batt_temp < 5)
  {
    fail++;
    Serial.println("TEMPERATURE FAIL");
  }
}

void measureTherm()
{
  float R, logRt;
  float Vo = analogRead(A0);
  float Rt = R * ( 1023.0 / (float)Vo - 1.0 );
  logRt = log(Rt);
  float T = ( 1.0 / (c1 + c2 * logRt + c3 * logRt * logRt * logRt ) ) - 273.15;
  Serial.print("Thermistor Temperature: ");
  Serial.print(" "); Serial.println(T);

  T = batt_temp - T;
  if (T > 2 || T < -2)
  {
    fail++;
    Serial.println("THERMISTOR FAIL");
  }
}

void sealPack()
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x00);
  Wire.write(0x20);
  Wire.write(0x00);
  delay(10);
  Wire.endTransmission();
}

void readDivider()
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x61);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x3e);
  Wire.write(0x68);
  Wire.endTransmission();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x3f);
  Wire.write(0x00);
  Wire.endTransmission();
  //  Serial.print("Flash: ");
  for (int i = 0; i < 32; i++)
  {
    delay(15);
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x40 + i);
    Wire.endTransmission(false);
    Wire.requestFrom(BQ34Z100, 1);
    delay(15);
    flashbytes[i] = Wire.read();
    //       Serial.print(flashbytes[i],HEX);
    //       Serial.print(" ");
  }
  divider = word(flashbytes[14], flashbytes[15]);
  Serial.print("Current Divider: ");
  Serial.println(divider);
}

void calibrateVoltage(float i2cVoltage, float inaVoltage)
{
  divider;
  readDivider();
  divider = divider * inaVoltage / i2cVoltage;
  Serial.print("Divider should be: ");
  Serial.println(divider);
  int  byte = divider & 0xFF;
  Serial.print("byte: ");
  Serial.println(byte, HEX);
  divider >>= 8;
  flashbytes[15] = byte;
  byte = divider & 0xFF;
  Serial.print("byte: ");
  Serial.println(byte, HEX);
  divider >>= 8;
  flashbytes[14] = byte;
  for (int i = 0; i < 32; i++)
  {
    Wire.beginTransmission(BQ34Z100);  // write flashbyte
    Wire.write(0x40 + i);
    Wire.write(flashbytes[i]);
    Wire.endTransmission();
    delay(10);
  }

  int chkSum = 0;  //calc checksum
  for (int i = 0; i < 32; i++)
  {
    chkSum += flashbytes[i];
  }
  int chkSumTemp = chkSum / 256;
  chkSum = chkSum - (chkSumTemp * 256);
  chkSum = 255 - chkSum;
  //write checksum
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x60);
  Wire.write(chkSum);
  Wire.endTransmission();

  readDivider();
}

void failCheck()
{
  Serial.print("Number of Fails: ");
  Serial.println(fail);
  Serial.println("------------------------------------------------");
  Serial.println("");

  if (fail == 0 )
  {
    Serial.print("All test passed... Sealing packing");
    sealPack();
  }

}

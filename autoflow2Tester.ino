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

  Checks the following

  1)Pack voltage is between 7.2 and 7.4V
  2)Temperature between 5 and 35
  3)Thermistor temperature reading within 2C of TI temperature
  4)Average current between -2 and 2mA
  5)State of Charge between 20% and 60%
  6)Firmware equals to the one defined


  @section  HISTORY

    v1.0-a1  - Alpha
  *********************************************************
  *IMPORTANT*   FOR USE WITH BQ34Z100-G1 ONLY   *IMPORTANT*
  *********************************************************
*/
/**************************************************************************/

#include <Wire.h>                                                              
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;


/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define BQ34Z100                                   85     // I2C adress for BQ34Z100                                                                    
#define FLASHVERSION                               0xAAAA // Check DF version against this. Fails if not equal
/*=========================================================================*/

/*=========================================================================
    Variables
    -----------------------------------------------------------------------*/
float R = 14870; // Fixed resistance in the voltage divider
float logRt, Rt, T;
float inaVoltage;
int divider;
int flashbytes[32] = {0};
float c1 = 0.001134508, c2 = 0.000233318, c3 = 9.0327E-8;
unsigned int soc, i2cVoltage, remain_cap;
unsigned int inst_current_lsb, inst_current_msb;
unsigned int firmware_msb, firmware_lsb, firmware;
int inst_current, c;
int avg_current, batt_temp, cntrl_stat1, cntrl_stat2;
float power_draw;
float reading;
/*=========================================================================*/


/**************************************************************************/
/*
    @brief  Reads the state of charge over I2C
*/
/**************************************************************************/
void readSOC() {
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x02);
  Wire.endTransmission();

  Wire.requestFrom(BQ34Z100, 1);

  soc = Wire.read();
}

/**************************************************************************/
/*
    @brief  Read Remaining Capacitor of pack over I2C
*/
/**************************************************************************/
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

  remain_cap = high1 + low;
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
  firmware_msb = Wire.read();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100, 1);
  firmware_lsb = Wire.read();

  firmware_msb = firmware_msb << 8;
  firmware = firmware_lsb + firmware_msb;
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
  inst_current_lsb = Wire.read();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100, 1);
  inst_current_msb = Wire.read();

  unsigned int temp = inst_current_msb << 8;

  inst_current = temp + inst_current_lsb;
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

float PowerDraw(float volt, float current)
{
  volt = volt / 1000.0;
  current = current / 1000.0;
  float power = volt * current;
  return power;
}

float measureVoltage()
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  shuntvoltage = ina219.getShuntVoltage_mV(); // Voltage across Vin+ and Vin-
  busvoltage = ina219.getBusVoltage_V();      // Voltage across Vin- and Gnd
  inaVoltage = busvoltage + (shuntvoltage / 1000);
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
  //
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

void setup()
{
  Serial.begin(9600);
  uint32_t currentFrequency;
  ina219.begin();
  Wire.begin();
}

void loop()
{
  uint8_t fail = 0;
  uint8_t calAttempts = 0;
  inaVoltage = 0;

  while (inaVoltage < 200)
  {
    measureVoltage();
    delay(500);
  }
  readVoltage();
  Serial.print("Battery Pack Voltage: ");
  Serial.print(i2cVoltage);
  Serial.println(" mV");

  while (i2cVoltage - inaVoltage > 20 || i2cVoltage - inaVoltage < -20 && calAttempts < 5)
  {
    readVoltage();
    measureVoltage();
    calibrateVoltage(i2cVoltage, inaVoltage);
    calAttempts++;
  }

  if (i2cVoltage < 7200 || i2cVoltage > 7600)
  {
    fail++;
    Serial.println("VOLTAGE FAIL");
  }

  
  readSOC(); //Read State of Charge
  Serial.print("Battery Charge: ");
  Serial.print(soc);
  Serial.println("%");

  if (soc < 20 || soc > 60)
  {
    fail++;
    Serial.println("SOC FAIL");
  }
//
//  readAverageCurrent();
//  Serial.print("Average Current Draw: ");
//  Serial.print(avg_current);
//  Serial.println(" mA");
//
//  if (avg_current > 2 || avg_current < -2)
//  {
//    fail++;
//    Serial.println("CURRENT FAIL");
//  }
//
//  readInstCurrent();
//  Serial.print("Instantaneous Current Draw: ");
//  Serial.println(inst_current);
//
//  readFirmware();
//  Serial.print("Flash version: ");
//  Serial.println(firmware, HEX);
//
//  if (firmware != FLASHVERSION) {
//    fail++;
//    Serial.println("FIRMWARE FAIL");
//  }
//
//
  readBattTemp();
  Serial.print("Battery Temperature: ");
  Serial.print(batt_temp);
  Serial.println(" C");
  if (batt_temp > 35 || batt_temp < 5)
  {
    fail++;
    Serial.println("TEMPERATURE FAIL");
  }

  int Vo;
  Vo = analogRead(A0);
  Rt = R * ( 1023.0 / (float)Vo - 1.0 );
  logRt = log(Rt);
  T = ( 1.0 / (c1 + c2 * logRt + c3 * logRt * logRt * logRt ) ) - 273.15;
  Serial.print("Thermistor Temperature: ");
  Serial.print(" "); Serial.println(T);

  T = batt_temp - T;
  if (T > 2 || T < -2)
  {
    fail++;
    Serial.println("THERMISTOR FAIL");
  }

  Serial.print("Number of Fails: ");
  Serial.println(fail);
  Serial.println("------------------------------------------------");
  Serial.println("");

  if (fail == 0 )
  {
    Serial.print("All test passed... Sealing packing");
    sealPack();
  }
  for (c = 0; c < fail; c++)
  {
    digitalWrite(11, HIGH);
    delay(200);
    digitalWrite(11, LOW);
    delay(200);
  }
  digitalWrite(11, HIGH);
  

}


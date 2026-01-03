/*******************************************************************************************************
  Program Operation - This is a simple program to configure a Ublox GPS for Garech vt-01 which accepts 
  only 4800 boud rate. Arduino connects to ublox module and reconfigures it to send (only)GPS basic (RMC,GGA,GSA) sentences at 4800 and does
  nothing else then.

  Serial monitor baud rate is set at 115200.

  source: https://stuartsprojects.github.io/2018/08/26/Generating-UBLOX-GPS-Configuration-Messages.html

*******************************************************************************************************/

#define RXpin 5              //this is the pin that the Arduino will use to receive data from the GPS
#define TXpin 4              //this is the pin that the Arduino can use to send data (commands) to the GPS

#include <SoftwareSerial.h>
#include <Arduino.h>
SoftwareSerial GPS(RXpin, TXpin);

//Baudrate change to 4800
const PROGMEM  uint8_t BaudRateCfg[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0xC0, 0x12, 0x00, 0x00, 0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEB, 0xC4};
const PROGMEM  uint8_t GpsOnlyCfg[] = {0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x16, 0x07, 0x00, 0x04, 0xFF, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x06, 0x08, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x01, 0xCB, 0x33};

const PROGMEM  uint8_t GllOutCfg[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
const PROGMEM  uint8_t GsvOutCfg[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
const PROGMEM  uint8_t VtgOutCfg[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};


const PROGMEM  uint8_t FilterOneCfg[] = {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x23, 0x00, 0x00, 0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCB, 0x5B};
const PROGMEM  uint8_t FilterTwoCfg[] = {0xB5, 0x62, 0x06, 0x17, 0x0C, 0x00, 0x00, 0x23, 0x00, 0x00, 0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC2, 0x92};


void GPS_SendConfig(const uint8_t *Progmem_ptr, uint8_t arraysize);

void setup()
{
  Serial.begin(115200);
  Serial.println("90_UBlox_GPS_Configuration Starting");
  Serial.println();

  delay(8000);
  GPS.begin(9600);
  GPS_SendConfig(GpsOnlyCfg, sizeof(GpsOnlyCfg));
  delay(3000);
  GPS_SendConfig(GllOutCfg, sizeof(GllOutCfg));
  delay(3000);
  GPS_SendConfig(GsvOutCfg, sizeof(GsvOutCfg));
  delay(3000);
  GPS_SendConfig(VtgOutCfg, sizeof(VtgOutCfg));
  delay(3000);
  GPS_SendConfig(FilterOneCfg, sizeof(FilterOneCfg));
  delay(3000);
  GPS_SendConfig(FilterTwoCfg, sizeof(FilterTwoCfg));
  delay(3000);
  GPS_SendConfig(BaudRateCfg, sizeof(BaudRateCfg));
  

  delay(8000);
  GPS.begin(4800);

  GPS_SendConfig(GpsOnlyCfg, sizeof(GpsOnlyCfg));
  delay(3000);
  GPS_SendConfig(GllOutCfg, sizeof(GllOutCfg));
  delay(3000);
  GPS_SendConfig(GsvOutCfg, sizeof(GsvOutCfg));
  delay(3000);
  GPS_SendConfig(VtgOutCfg, sizeof(VtgOutCfg));
  delay(3000);
  GPS_SendConfig(FilterOneCfg, sizeof(FilterOneCfg));
  delay(3000);
  GPS_SendConfig(FilterTwoCfg, sizeof(FilterTwoCfg));

}

void loop()
{
  //do nothing

}


void GPS_SendConfig(const uint8_t *Progmem_ptr, uint8_t arraysize)
{
  uint8_t byteread, index;

  Serial.print(F("GPSSend  "));

  for (index = 0; index < arraysize; index++)
  {
    byteread = pgm_read_byte_near(Progmem_ptr++);
    if (byteread < 0x10)
    {
      Serial.print(F("0"));
    }
    Serial.print(byteread, HEX);
    Serial.print(F(" "));
  }

  Serial.println();
  Progmem_ptr = Progmem_ptr - arraysize;                  //set Progmem_ptr back to start

  for (index = 0; index < arraysize; index++)
  {
    byteread = pgm_read_byte_near(Progmem_ptr++);
    GPS.write(byteread);
  }
  delay(100);

}

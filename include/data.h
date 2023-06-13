#pragma once
#include <Arduino.h>

struct sensorData
{
  //OPC bins:
  uint16_t bins[16];
  //Sum of all bins:
  uint16_t binsSum;
  //Mass Time of Flight:
  float bin1MToF;
  float bin3MToF;
  float bin5MToF;
  float bin7MToF;
  //Sample Flow Rate:
  float sampleFlowRate;
  //Temperature:
  float temp;
  //Humidity:
  float hum;
  //Sampling Period:
  float sPeriod;
  //Reject Count Glitch:
  uint8_t rcGlitch;
  //Reject Count Long:
  uint8_t rcLong;
  //PMs:
  float pm1;
  float pm25;
  float pm10;
  //Checksum:
  uint16_t checksum;
  bool checksumOK;

  //bme280:
  float bmeTemp;
  float bmeHum;
  float bmePress;
  float bmeAlt;
};
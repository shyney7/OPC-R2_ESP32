#include <Arduino.h>
#include <SPI.h>
#include <esp_task_wdt.h>
#include "opcr2.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//Adafruit_BME280 bme I2C
Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)
//print values prototype
void printValues(void);

void setup() {
    // put your setup code here, to run once:
    esp_task_wdt_reset();   //Reset watchdog timer
    esp_task_wdt_init(8, true); //Enable watchdog timer, countdown 8s (max)
    esp_task_wdt_add(NULL); //Add current thread to watchdog timer check list

    ssPin_OPC = 5;
    SetSSpin(HIGH);
    pinMode(ssPin_OPC, OUTPUT);
    delay(1000);

    //Start serial port
    opSerial.begin(BaudRate);
    while (!opSerial) {
        delay(100);
    }

    PrintFirmwareVer(opSerial);

    SPI.begin();

    esp_task_wdt_reset();
    InitDevice(); // init OPC
    esp_task_wdt_reset();

    PrintDataLabels(opSerial); //OPC data labels
    currentTime = millis();
    cloopTime = currentTime;
    //bme280
    unsigned status;
    status = bme.begin();
    if (!status) {
        opSerial.println("Could not find a valid BME280 sensor, check wiring!");
        opSerial.print("SensorID was: 0x"); opSerial.println(bme.sensorID(), 16);
        opSerial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        opSerial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        opSerial.print("        ID of 0x60 represents a BME 280.\n");
        opSerial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
}

void loop() {
/*     esp_task_wdt_reset(); //Reset watchdog timer
    currentTime = millis(); //millis count will reset on sketch restart
    if (currentTime >= cloopTime) {
        cloopTime += 60000; //60s loop
        esp_task_wdt_reset(); //Reset watchdog timer
        ssPin_OPC = 5;

        StartOPC();
        esp_task_wdt_reset(); //Reset watchdog timer
        //Get 10 histogram data sets (dont record first one as it will contain invalid data)
        unsigned long GetHistTime = millis();
        for (byte i=0; i<10; ++i) {
            delay(1000);
            ReadOPChist();
            if (i != 0) {
                //Print time since start (millis() returns an unsigned long of numbers of ms since program started. It wraps around in ~50 days)
                opSerial.print(millis());
                PrintData(opSerial); //Print data to serial
            }
            esp_task_wdt_reset(); //Reset watchdog timer
        }
        StopOPC();
        opSerial.println("Waiting until next cycle");
    } */

    esp_task_wdt_reset(); //Reset watchdog timer
    ssPin_OPC = 5;
    StartOPC();
    esp_task_wdt_reset(); //Reset watchdog timer
    delay(1000);
    ReadOPChist();
    esp_task_wdt_reset(); //Reset watchdog timer
    while (1)
    {
        delay(1000);
        ReadOPChist();
        opSerial.print(millis());
        PrintData(opSerial); //Print data to serial
        esp_task_wdt_reset(); //Reset watchdog timer
        printValues(); //bme280
        esp_task_wdt_reset();
    }
    
}

void printValues(void) {
    opSerial.print("Temperature = ");
    opSerial.print(bme.readTemperature());
    opSerial.println(" °C");

    opSerial.print("Pressure = ");
    opSerial.print(bme.readPressure() / 100.0F);
    opSerial.println(" hPa");

    opSerial.print("Approx. Altitude = ");
    opSerial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    opSerial.println(" m");

    opSerial.print("Humidity = ");
    opSerial.print(bme.readHumidity());
    opSerial.println(" %");

    opSerial.println();
}
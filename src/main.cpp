#include <Arduino.h>
#include <SPI.h>
#include <esp_task_wdt.h>
#include "opcr2.h"

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

    PrintFirmwareVer(opSerial);

    SPI.begin();

    esp_task_wdt_reset();
    InitDevice();
    esp_task_wdt_reset();

    PrintDataLabels(opSerial);
    currentTime = millis();
    cloopTime = currentTime;
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

    }
    
}
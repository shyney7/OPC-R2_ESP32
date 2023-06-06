#include <Arduino.h>
#include <SPI.h>
#include <esp_task_wdt.h>    //Watchdog timer
#include "opcr2.h"           //OPC-R2
#include <Adafruit_Sensor.h> //BM280
#include <Adafruit_BME280.h> //BM280
#include <RF24.h>            //NRF24L01
#include <RF24Network.h>     //NRF24L01
#include <RF24Mesh.h>        //NRF24L01

//Adafruit_BME280 bme I2C ************
Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)
//print BME280 values prototype
void printValues(void);
//END Adafruit_BME280 bme I2C ************

//NRF24L01 ************
RF24 radio(GPIO_NUM_4, GPIO_NUM_2); // CE, CSN
RF24Network network(radio);
RF24Mesh mesh(radio, network);
#define nodeID 1
uint32_t displayTimer = 0;
struct payload_t {
    unsigned long ms;
    unsigned long counter;
};
//prototype
void sendData(void);
//END NRF24L01 ************


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
    //END bme280

    //NRF24L01
    // Set the nodeID to 0 for the master node (here 1 for the slave node)
    mesh.setNodeID(nodeID);
    // Set the PA Level to MIN and disable LNA for testing & power supply related issues
    radio.begin();
    radio.setPALevel(RF24_PA_MIN, 0);
    //connect to the mesh
    opSerial.println("Connecting to the mesh...");
    if (!mesh.begin()) {
        if (radio.isChipConnected()) {
            do {
                //mesh.renewAddress() will return MESH_DEFAULT_ADDRESS on failure to connect
                opSerial.println(F("Could not connect to network. \nConnecting to the mesh..."));
            }while (mesh.renewAddress() == MESH_DEFAULT_ADDRESS);
        }
        else {
            opSerial.println(F("Radio hardware not responding."));
            while (1) {
                //hold in an infinite loop
            }
        }
    }
    //END NRF24L01
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
        sendData(); //NRF24
        esp_task_wdt_reset();
    }
    
}

//print BME280 values
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

//send RF24 Data
void sendData(void) {
    mesh.update();

    //Send to the master node every second
    if (millis() - displayTimer >= 1000) {
        displayTimer = millis();
        //Send an 'M' type message containing the current millis()
        if (!mesh.write(&displayTimer, 'M', sizeof(displayTimer))) {
            //if a write fails, check connectivity to the mesh network
            if (!mesh.checkConnection()) {
                //refresh the network address
                opSerial.println("Renewing Adress");
                if (mesh.renewAddress() == MESH_DEFAULT_ADDRESS) {
                    //If address renewal fails, reconfigure the radio and restart the mesh
                    //This allows recovery from most if not all radio errors
                    mesh.begin();
                }
            } else {
                opSerial.println("Send fail, Test OK");
            }
        } else {
            opSerial.print("Send OK: ");
            opSerial.println(displayTimer);
        }
    }

    while (network.available()) {
        RF24NetworkHeader header;
        payload_t payload;
        network.read(header, &payload, sizeof(payload));
        opSerial.print("Received packet #");
        opSerial.print(payload.counter);
        opSerial.print(" at ");
        opSerial.println(payload.ms);
    }
}
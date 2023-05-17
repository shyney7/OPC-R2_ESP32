#include <Arduino.h>
#include <SPI.h>
#include <esp_task_wdt.h>

#define FirmwareVer "OPC-R2(ESP32)"

#define opSerial Serial
#define BaudRate 9600

#define SPI_OPC_busy 0x31
#define SPI_OPC_ready 0xF3

unsigned long currentTime, cloopTime;
unsigned char SPI_in[68], SPI_in_index, ssPin_OPC;

//Prototypes
void PrintFirmwareVer(Stream &port);
void InitDevice(void);
void ReadOPCstring(unsigned char SPIcommand);
void PrintOPCstring(Stream &port);
void ReadOPChist(void);
void DiscardSPIbytes (byte NumToDiscard);
void ReadOPCconfig(Stream &port);
void PrintData(Stream &port);
void StartOPC(void);
void StopOPC(void);
void GetReadyResponse(unsigned char SPIcommand);
void SetSSpin(bool pinState);
void AddDelimiter (Stream &port);
void PrintDataLabels(Stream &port);

void setup() {
    // put your setup code here, to run once:
    esp_task_wdt_reset();   //Reset watchdog timer
    esp_task_wdt_init(8, true); //Enable watchdog timer, countdown 8s (max)
    esp_task_wdt_add(NULL); //Add current thread to watchdog timer check list

    //18,19,23,5,22,21,27
    digitalWrite(18, HIGH);
    digitalWrite(19, HIGH);
    digitalWrite(23, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(22, HIGH);
    digitalWrite(21, HIGH);
    digitalWrite(27, HIGH);
    pinMode(18, OUTPUT);
    pinMode(19, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(21, OUTPUT);
    pinMode(27, OUTPUT);

    delay(1000);
    opSerial.begin(BaudRate);

    PrintFirmwareVer(opSerial);

    SPI.begin();

    ssPin_OPC = 5;
    esp_task_wdt_reset();
    InitDevice();
    esp_task_wdt_reset();
}

void PrintFirmwareVer(Stream &port) {
    port.print("Datalogger Firmware Version: ");
    port.println(FirmwareVer);
}

void InitDevice(void) {
    esp_task_wdt_reset(); //Reset watchdog timer
    ReadOPCstring(0x10);    //Read serialstring from OPC
    ReadOPCstring(0x3F);    //Read information string from OPC
    
    //Set OPC to active mode
    StartOPC();
    esp_task_wdt_reset(); //Reset watchdog timer
    ReadOPCconfig(opSerial);    //Read configuration from OPC
}

void loop() {
    esp_task_wdt_reset(); //Reset watchdog timer
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
    }
}

//Get string (serialstr or infostr) from OPC
void ReadOPCstring(unsigned char SPIcommand) {
    GetReadyResponse(SPIcommand);
    for (SPI_in_index=0; SPI_in_index<60; ++SPI_in_index) {
        delayMicroseconds(10);
        SPI_in[SPI_in_index] = SPI.transfer(0x01); //Send dummy byte to OPC and read response
    }
    SetSSpin(HIGH);
    SPI.endTransaction();
    PrintOPCstring(opSerial);
}

//Print string (serialstr or infostr) from OPC
void PrintOPCstring(Stream &port) {
    port.write(SPI_in, 60); //Print string to serial
    port.println("");
    port.flush();
}

//Get histogram data from OPC
void ReadOPChist(void) {
    GetReadyResponse(0x30);
    for (SPI_in_index=0; SPI_in_index<64; ++SPI_in_index) {
        delayMicroseconds(10);
        SPI_in[SPI_in_index] = SPI.transfer(0x01); //Send dummy byte to OPC and read response
    }
    SetSSpin(HIGH);
    SPI.endTransaction();
    delay(10);
}

void DiscardSPIbytes (byte NumToDiscard) {
    for (SPI_in_index=0; SPI_in_index<NumToDiscard; ++SPI_in_index) {
        delayMicroseconds(10);
        SPI.transfer(0x01); //Send dummy byte to OPC
    }
}

//Get Config data (bin boundaries, etc.) from OPC
void ReadOPCconfig(Stream &port) {
    uint16_t *pUInt16;
    float *pFloat;
    //Have to read config from OPC device in this 'chunks' manner as Arduino buffer isn't big enough to hold all config data at once and OPC could timeout if Arduino took time to print/save data from the buffer during the SPI transfer sequence.
    //Instead, config data is read several times, and each time a different chunk is saved to the Arduino buffer and printed. This way there is no delay during each individual SPI transfer sequence.
    
    //Get config from OPC device (Bin Boundaries ADC (BB0-BB16) uint16_t)
    GetReadyResponse(0x3C);
    for (SPI_in_index=0; SPI_in_index<34; ++SPI_in_index) {
        delayMicroseconds(10);
        SPI_in[SPI_in_index] = SPI.transfer(0x01); //Send dummy byte to OPC and read response
    }
    DiscardSPIbytes(159); //Discard remaining bytes in SPI buffer
    SetSSpin(HIGH);
    SPI.endTransaction();
    delay(10);

    port.print(F("Bin Boundaries ADC"));
    for (SPI_in_index=0; SPI_in_index<34; SPI_in_index+=2) {
        AddDelimiter(port);
        pUInt16 = (uint16_t *)&SPI_in[SPI_in_index];
        port.print(*pUInt16, DEC);
    }
    port.println("");
    port.flush();
    //END Get config from OPC device (Bin Boundaries ADC (BB0-BB16) uint16_t)

    //Get config from OPC device (Bin Boundaries diameter(um) (BBD0-BBD16) float occupying 4 bytes each (68 bytes total))
    GetReadyResponse(0x3C);
    //Discard first 34 bytes (already read)
    DiscardSPIbytes(34);
    //Read next 68 bytes
    for (SPI_in_index=0; SPI_in_index<68; ++SPI_in_index) {
        delayMicroseconds(10);
        SPI_in[SPI_in_index] = SPI.transfer(0x01); //Send dummy byte to OPC and read response
    }
    DiscardSPIbytes(91); //Discard remaining bytes in SPI buffer
    SetSSpin(HIGH);
    SPI.endTransaction();
    delay(10);

    port.print(F("Bin Boundaries diameter(um)"));
    for (SPI_in_index=0; SPI_in_index<68; SPI_in_index+=4) {
        AddDelimiter(port);
        pFloat = (float *)&SPI_in[SPI_in_index];
        port.print(*pFloat, 2); 
    }
    port.println("");
    port.flush();
    //END Get config from OPC device (Bin Boundaries diameter(um) (BBD0-BBD16) float occupying 4 bytes each (68 bytes total))
    
    //Get config from OPC (Bin Weightings (BW0-BW15) float occupying 4 bytes each (64 bytes total))
    GetReadyResponse(0x3C);
    //Discard first 102 bytes (already read)
    DiscardSPIbytes(102);
    //Read next 64 bytes
    for (SPI_in_index=0; SPI_in_index<64; ++SPI_in_index) {
        delayMicroseconds(10);
        SPI_in[SPI_in_index] = SPI.transfer(0x01); //Send dummy byte to OPC and read response
    }
    DiscardSPIbytes(27); //Discard remaining bytes in SPI buffer
    SetSSpin(HIGH);
    SPI.endTransaction();
    delay(10);

    port.print(F("Bin Weightings"));
    for (SPI_in_index=0; SPI_in_index<64; SPI_in_index+=4) {
        AddDelimiter(port);
        pFloat = (float *)&SPI_in[SPI_in_index];
        port.print(*pFloat, 2); 
    }
    port.println("");
    port.flush();
    //END Get config from OPC (Bin Weightings (BW0-BW15) float occupying 4 bytes each (64 bytes total))

    //Get config from OPC (Misc)
    GetReadyResponse(0x3C);
    //Discard first 166 bytes (already read)
    DiscardSPIbytes(166);
    //Read next 27 bytes
    for (SPI_in_index=0; SPI_in_index<27; ++SPI_in_index) {
        delayMicroseconds(10);
        SPI_in[SPI_in_index] = SPI.transfer(0x01); //Send dummy byte to OPC and read response
    }
    SetSSpin(HIGH);
    SPI.endTransaction();
    delay(10);

    //Gain Scaling Coefficient (float occupying 4 bytes)
    port.print(F("Gain Scaling Coefficient,"));
    pFloat = (float *)&SPI_in[0];
    port.println(*pFloat, 2);

    //Sample Flow Rate (float occupying 4 bytes)
    port.print(F("Sample Flow Rate,"));
    pFloat = (float *)&SPI_in[4];
    port.println(*pFloat, 2);

    //Time of Flight to Sample Flow Rate Conversion Factor (unsigned  8bit integer occupying 1 byte)
    port.print(F("Time of Flight to Sample Flow Rate Conversion Factor,"));
    port.println(SPI_in[8], DEC);

    //M_A (Particle mass Concentration A) (float occupying 4 bytes)
    port.print(F("M_A (Particle mass Concentration A),"));
    pFloat = (float *)&SPI_in[9];
    port.println(*pFloat, 2);

    //M_B (Particle mass Concentration B) (float occupying 4 bytes)
    port.print(F("M_B (Particle mass Concentration B),"));
    pFloat = (float *)&SPI_in[13];
    port.println(*pFloat, 2);

    //M_C (Particle mass Concentration C) (float occupying 4 bytes)
    port.print(F("M_C (Particle mass Concentration C),"));
    pFloat = (float *)&SPI_in[17];
    port.println(*pFloat, 2);

    //PVP (Particle Validation Period) unsigned 8bit integer occupying 1 byte)
    port.print(F("PVP (Particle Validation Period),"));
    port.println(SPI_in[21], DEC);

    //PowerStatus (unsigned 8bit integer occupying 1 byte) Bit 0: controls laser and peripheral power, Bit 1 controls fan power
    port.print(F("PowerStatus,"));
    port.println(SPI_in[22], BIN);

    //MaxTOF (Maximum Time of Flight) (unsigned 16bit integer occupying 2 bytes)
    port.print(F("MaxTOF (Maximum Time of Flight),"));
    pUInt16 = (uint16_t *)&SPI_in[23];
    port.println(*pUInt16, DEC);

    //LaserDAC (Laser DAC) (unsigned 8bit integer occupying 1 byte)
    port.print(F("LaserDAC (Laser DAC),"));
    port.println(SPI_in[25], DEC);

    //BinWeightingIndex (0-10) is an unsigned 8bit integer occupying 1 byte that represents the bin weightings to use
    port.print(F("BinWeightingIndex (0-10),"));
    port.println(SPI_in[26], DEC);
    port.flush();
    //END Get config from OPC (Misc)
}

void StartOPC (void) {
    //Turn ON fan and laser
    GetReadyResponse(0x03);
    SPI.transfer(0x03);
    SetSSpin(HIGH);
    SPI.endTransaction();
    delay(10);

    //Wait for fan to reach full speed (and for multiple attempts for OPC firmware to turn on fan)
    for (byte i=0; i<5; ++i) {
        esp_task_wdt_reset();
        delay(1000);
    }
}

void StopOPC (void) {
    //Turn OFF fan and laser
    GetReadyResponse(0x03);
    SPI.transfer(0x00);
    SetSSpin(HIGH);
    SPI.endTransaction();
    delay(10);
}

void GetReadyResponse(unsigned char SPIcommand) {
    unsigned char Response;

    SPI.beginTransaction(SPISettings(300000, MSBFIRST, SPI_MODE1));

    //Try reading a byte here to clear out anything remnant of SD card SPI activity 
    Response = SPI.transfer(SPIcommand);
    delay(1);

    do {
        SetSSpin(LOW);
        unsigned char Tries = 0;
        do {
            Response = SPI.transfer(SPIcommand);
            if (Response != SPI_OPC_ready) delay(1);
        }
        while ((Tries++ < 20) && (Response != SPI_OPC_ready));

        if (Response != SPI_OPC_ready) {
            if (Response == SPI_OPC_busy) {
                SetSSpin(HIGH);
                Serial.println(F("ERROR Waiting 2s (for OPC comms timeout) OPC is busy"));
                Serial.flush();
                esp_task_wdt_reset();
                delay(2000);
            }
            else {
                //End SPI and wait a few seconds for it to be cleared
                SetSSpin(HIGH);
                Serial.println(F("ERROR Resetting SPI"));
                Serial.flush();
                SPI.endTransaction();
                esp_task_wdt_reset();
                delay(6000);
                esp_task_wdt_reset();
                SPI.beginTransaction(SPISettings(300000, MSBFIRST, SPI_MODE1));
            }
        }
    }
    while ((Response != SPI_OPC_ready) && (Serial.available() == 0));
    delay(10);
    esp_task_wdt_reset();
}

unsigned int MODBUS_CalcCRC(unsigned char data[], unsigned char nbrOfBytes)
{
  #define POLYNOMIAL_MODBUS 0xA001 //Generator polynomial for MODBUS crc
  #define InitCRCval_MODBUS 0xFFFF //Initial CRC value

  unsigned char _bit; // bit mask
  unsigned int crc = InitCRCval_MODBUS; // initialise calculated checksum
  unsigned char byteCtr; // byte counter

  // calculates 16-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
  {
    crc ^= (unsigned int)data[byteCtr];
    for(_bit = 0; _bit < 8; _bit++)
    {
      if (crc & 1) //if bit0 of crc is 1
      {
        crc >>= 1;
        crc ^= POLYNOMIAL_MODBUS;
      }
      else
        crc >>= 1;
    }
  }
  return crc;
}

//Convert SHT31 ST output to Temperature (C)
float ConvSTtoTemperature (unsigned int ST)
{
  return -45 + 175*(float)ST/65535;
}


//Convert SHT31 SRH output to Relative Humidity (%)
float ConvSRHtoRelativeHumidity (unsigned int SRH)
{
  return 100*(float)SRH/65535;
}

void PrintData (Stream &port) {
    unsigned char i;
    uint16_t *pUInt16;
    float *pFloat;
    float Afloat;

    //Histogram bins (unsigned 16bit integers occupying 2 bytes each)
    for (i=0; i<32; i+=2) {
        AddDelimiter(port);
        pUInt16 = (uint16_t *)&SPI_in[i];
        port.print(*pUInt16, DEC);
    }

    //MToF 8bit integer occupying 1 byte each representing the avarage time that particles sized in the stated bin took to cross the laser beam
    for (i=32; i<36; ++i) {
        AddDelimiter(port);
        Afloat = (float)SPI_in[i]/3; //convert to microseconds
        port.print(Afloat, 2);
    }

    //SFR (Sample Flow Rate) (float occupying 4 bytes) (ml/s)
    AddDelimiter(port);
    pFloat = (float *)&SPI_in[36];
    port.print(*pFloat, 3);

    //Temperature (unsigned 16bit integer occupying 2 bytes)
    AddDelimiter(port);
    pUInt16 = (uint16_t *)&SPI_in[40];
    port.print(ConvSTtoTemperature(*pUInt16), 1);

    //Relative Humidity (unsigned 16bit integer occupying 2 bytes)
    AddDelimiter(port);
    pUInt16 = (uint16_t *)&SPI_in[42];
    port.print(ConvSRHtoRelativeHumidity(*pUInt16), 1);

    //Sampling Period (float occupying 4 bytes) measure of the histograms actual sampling period in seconds
    AddDelimiter(port);
    pFloat = (float *)&SPI_in[44];
    port.print(*pFloat, 3);

    //Reject count Glitch and Reject count Long (unsigned 8bit integers occupying 1 byte each)
    for (i=48; i<50; ++i) {
        AddDelimiter(port);
        port.print(SPI_in[i], DEC);
    }

    //PM_A (float occupying 4 bytes) (ug/m3)
    AddDelimiter(port);
    pFloat = (float *)&SPI_in[50];
    port.print(*pFloat, 3);

    //PM_B (float occupying 4 bytes) (ug/m3)
    AddDelimiter(port);
    pFloat = (float *)&SPI_in[54];
    port.print(*pFloat, 3);

    //PM_C (float occupying 4 bytes) (ug/m3)
    AddDelimiter(port);
    pFloat = (float *)&SPI_in[58];
    port.print(*pFloat, 3);

    //Checksum (unsigned 16bit integer occupying 2 bytes)
    AddDelimiter(port);
    pUInt16 = (uint16_t *)&SPI_in[62];
    port.println(*pUInt16, DEC);

    //Compare recalculated Checksum with the one received from the OPC
    if (*pUInt16 == MODBUS_CalcCRC(SPI_in, 62)) {
        port.println(F("Checksum OK"));
    }
    else {
        port.println(F("Checksum ERROR"));
    }
    port.flush();
}

//Print Data Labels
void PrintDataLabels (Stream &port) {
    unsigned char i;
    port.print(F("Time(ms)"));
    for (i=0; i<16; ++i) {
        port.print(F(",Bin"));
        if (i<10)
            port.print(F("0"));
        port.print(i, DEC);
    }
    port.print(F(",MToF Bin1"));
    port.print(F(",MToF Bin3"));
    port.print(F(",MToF Bin5"));
    port.print(F(",MToF Bin7"));
    port.print(F(",SFR"));
    port.print(F(",Temp(C)"));
    port.print(F(",RH(%)"));
    port.print(F(",Sampling Period(s)"));
    port.print(F(",Reject count Glitch"));
    port.print(F(",Reject count Long"));
    port.print(F(",PM_A(ug/m3)"));
    port.print(F(",PM_B(ug/m3)"));
    port.print(F(",PM_C(ug/m3)"));
    port.print(F(",Checksum"));
    port.println();
    port.flush();
}

void AddDelimiter (Stream &port) {
    port.print(F(","));
}

void SetSSpin (bool pinState) {
    digitalWrite(ssPin_OPC, pinState);
}

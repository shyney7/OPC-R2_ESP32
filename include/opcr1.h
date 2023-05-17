/*
    OPCR1.h - Library for operating the Alphasense OPC-N2 Particle counter.
    Created by David H Hagan, March 2016.
    Modified by Marcelo Yungaicela, May 2017
		Modified for the OPC R1 by Detlef Amend, August 2019
    Released with an MIT license.
*/

#ifndef Opcr1_h
#define Opcr1_h

// Includes

struct StatusR1 {
    int fanON;
    int laserON;
    int fanDAC;
    int laserDAC;
};

struct FirmwareR1 {
    int major;
    int minor;
};

struct HistogramDataR1 {
		bool DataValid; // set TRUE on succesful initiating Data Transfer
    
		double bin0;
    double bin1;
    double bin2;
    double bin3;
    double bin4;
    double bin5;
    double bin6;
    double bin7;
    double bin8;
    double bin9;
    double bin10;
    double bin11;
    double bin12;
    double bin13;
    double bin14;
    double bin15;

    double sumAllBins; // Sum of all BINs

    // Mass Time-of-Flight
    float bin1MToF;
    float bin3MToF;
    float bin5MToF;
    float bin7MToF;

    // Sample Flow Rate
    float sfr;
		
		// Temperature
		unsigned int temp;
		
		// Humidity
		unsigned int humidity;

    // Sampling Period
    float period;
		
		// Reject Count Glitch
		byte rcg;
		
		//Reject Count LONG
		byte rcl;
    
		// PM's
    float pmA;
    float pmB;
    float pmC;
		
		// Checksum
    unsigned int checksum;
};

struct PMDataR1 {
	bool DataValid; // set TRUE on succesful initiating Data Transfer
  float pm1;
  float pm25;
  float pm10;
};

struct ConfigVarsR1 {
		
		boolean DataValid; // set TRUE on succesful initiating Data Transfer
		
    // Bin Boundaries
    unsigned int bb0;
    unsigned int bb1;
    unsigned int bb2;
    unsigned int bb3;
    unsigned int bb4;
    unsigned int bb5;
    unsigned int bb6;
    unsigned int bb7;
    unsigned int bb8;
    unsigned int bb9;
    unsigned int bb10;
    unsigned int bb11;
    unsigned int bb12;
    unsigned int bb13;
    unsigned int bb14;
    unsigned int bb15;
    unsigned int bb16;

    // Bin Boundaries diameter (um / floats)
    float bbd0;
    float bbd1;
    float bbd2;
    float bbd3;
    float bbd4;
    float bbd5;
    float bbd6;
    float bbd7;
    float bbd8;
    float bbd9;
    float bbd10;
    float bbd11;
    float bbd12;
    float bbd13;
    float bbd14;
    float bbd15;
    float bbd16;

    // Bin Weightings (floats)
    float bw0;
    float bw1;
    float bw2;
    float bw3;
    float bw4;
    float bw5;
    float bw6;
    float bw7;
    float bw8;
    float bw9;
    float bw10;
    float bw11;
    float bw12;
    float bw13;
    float bw14;
    float bw15;

    // Gain Scaling Coefficient
    float gsc;

    // Sample Flow Rate (ml/s)
    float sfr;

		// Time of Flight to Sample Flow Rate Conversion Factor
    byte tof_sfr;
		
    // Particle mass Concentration (floats)
    float M_A;
    float M_B;
    float M_C;
		
		// Particle Validation Period
    byte pvp;
    
		// Power Status
    byte powerStatus;
		
		//Maximum Time Of Flight
		float maxTOF;
    
		// LaserDAC 8 bit int
    byte laser_dac;
		
		//BinWeightingIndex
		byte bwi;
    
};

class OPCR1
{
private:
    // attributes
    uint8_t _CS;
    int _fv;

    // methods
    uint16_t _16bit_int(byte MSB, byte LSB);
    bool _compare_arrays(byte array1[], byte array2[], int length);
    float _calculate_float(byte val0, byte val1, byte val2, byte val3);
    uint32_t _32bit_int(byte val0, byte val1, byte val2, byte val3);
		bool _waitForGo(byte val);
    float _floatTruncate(float val, byte dec);
    float _oneDec(float a);

public:
    OPCR1(uint8_t chip_select);

    // attributes
    FirmwareR1 firm_ver;
    bool on();
    bool off();
    bool toggle_fan(bool state);
    bool toggle_laser(bool state);

    String read_information_string();
    String read_serial_number_string();
    ConfigVarsR1 read_configuration_variables();
    PMDataR1 read_pm_data();
    HistogramDataR1 read_histogram(bool convert_to_conc = true);
};

#endif

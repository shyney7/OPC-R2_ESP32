/*
		opcr1m.h Functions to operate the Alphasense OPC-R1 Particle counter..
	
    based on the work of David H Hagan and Marcelo Yungaicela
		
		Modified for R1 by Detlef Amend, August 2019
    Modified for PlatformIO by Marcel Oliveira Brito, May 2023

*/
#include <Arduino.h>

#define _ASSpiBegin SPI.beginTransaction(SPISettings(600000UL, MSBFIRST, SPI_MODE1));

#include <SPI.h>
#include "opcr1.h"

OPCR1::OPCR1(uint8_t chip_select)
{
  // Initiate an instance of the OPCR1 class
  // Ex. OPCR1 alpha(chip_select = A2);
  _CS = chip_select;
 
 pinMode(_CS, OUTPUT);
  SPI.begin();
  firm_ver.major = 99;
  firm_ver.minor = 99;
}

uint16_t OPCR1::_16bit_int(byte LSB, byte MSB){
  // Combine two bytes into a 16-bit unsigned int
  return ((MSB << 8) | LSB);
}

bool OPCR1::_compare_arrays(byte array1[], byte array2[], int length){
  // Compare two arrays and return a boolean
  bool result = true;

  for (int i = 0; i < length; i++){
    if (array1[i] != array2[i]){
      result = false;
    }
  }

  return result;
}

float OPCR1::_calculate_float(byte val0, byte val1, byte val2, byte val3){
  // Return an IEEE754 float from an array of 4 bytes
  union u_tag {
    byte b[4];
    float val;
  } u;

  u.b[0] = val0;
  u.b[1] = val1;
  u.b[2] = val2;
  u.b[3] = val3;

  return u.val;
}

uint32_t OPCR1::_32bit_int(byte val0, byte val1, byte val2, byte val3){
  // Return a 32-bit unsigned int from 4 bytes
  return ((val3 << 24) | (val2 << 16) | (val1 << 8) | val0);
}

bool OPCR1::_waitForGo(byte val){
	
	byte loopCounter=0;
  byte ret;
  _ASSpiBegin

  //This function was completely overhauled to fix power cycle issues by Marcel Oliveira Brito
  //This is a fix for the OPC-R1 power cycle problem
  //Try reading a byte here to clear out anything remnant of SD card SPI activity (WORKS!)
  ret = SPI.transfer(val);
  delay(1);
	
	do{ // wait for Return is 0xF3
		delay(12);
		digitalWrite(this->_CS, LOW);
		  
    do{  
      ret=SPI.transfer(val);
      if(ret != 0xF3) delay(1);
    }while((loopCounter++ < 20) && (ret != 0xF3));
    
    if (ret != 0xF3) {
      if (ret == 0x31) {
        digitalWrite(this->_CS, HIGH);
        Serial.println("ERROR Waiting 2s (for OPC comms timeout)");
        Serial.flush();
        delay(2000);
      }
      else {
        digitalWrite(this->_CS, HIGH);
        Serial.println("ERROR Resetting SPI");
        Serial.flush();
        SPI.endTransaction();
        delay(6000);
        _ASSpiBegin
      }
    }
		
	}while((ret!=0xF3) && (Serial.available()==0));

	delay(12);

	return true;
	
}

float OPCR1::_oneDec(float a){ // One Decimal 
  return round(a*10)/10;
}

float OPCR1::_floatTruncate(float val, byte dec){
  float x=val*pow(10,dec);
  float y=round(x);
  float z=x-y;
  if ((int)z==5){
    y++;
  }
  x=y/pow(10,dec);
  return x;
}

bool OPCR1::on(){
  // Turn ON the OPC and return a boolean
  // Ex.
  // $ alpha.on()
  // $ true
	
	byte ret;
	
	_ASSpiBegin
	
	if(this->_waitForGo(0x03)){
		digitalWrite(this->_CS, LOW);
		ret=SPI.transfer(0x03); // Fan&Laser ON
		digitalWrite(this->_CS, HIGH);
		//SPI.endTransaction();
	}else{
		return false;
	}
	
	SPI.endTransaction();
	
	delay(2000);
	
	if(ret!=0x03){ // Laser & Fan Command OK
		return false;
	}
	
	return true;

}

bool OPCR1::off(){
  // Turn OFF the OPC and return a boolean
  // Ex.
  // $ alpha.off()
  // $ true
	
	byte ret;
	
	_ASSpiBegin
	
	if(this->_waitForGo(0x03)){
		digitalWrite(this->_CS, LOW);
		ret=SPI.transfer(0x00); // Fan&Laser OFF
		digitalWrite(this->_CS, HIGH);
		SPI.endTransaction();
	}else{
		return false;
	}
	
	SPI.endTransaction();
	
	if(ret!=0x03){ // Laser & Fan Command OK
		return false;
	}
	
	return true;
}

String OPCR1::read_information_string(){
  // Read the information string and return a string
  // Ex.
  // $ alpha.read_information_string()

	String result="";
  byte ret;
	
	_ASSpiBegin
	
	if(this->_waitForGo(0x3F)){

		digitalWrite(this->_CS, LOW);

		// Iterate to read the entire string
		for (int i=0;i<60;i++){
			ret=SPI.transfer(0x3F);
			result+=String((char)ret);
			delayMicroseconds(10);
		}

		digitalWrite(this->_CS, HIGH);
		
		SPI.endTransaction();

		return result;
		
	}else{
		return "Error Reading Info String";
	}
}

String OPCR1::read_serial_number_string(){
  // Read the serial number string and return a string
  // Ex.
  // $ alpha.read_information_string()

  String result="";
  byte ret;
	
	_ASSpiBegin
  
  if(this->_waitForGo(0x10)){
    digitalWrite(this->_CS,LOW);
    delayMicroseconds(10);
    for(int i=0;i<60;i++){
      ret=SPI.transfer(0x10);
      result+=String((char)ret);
      delayMicroseconds(10);
    }
    digitalWrite(this->_CS,HIGH);

    SPI.endTransaction();

    return result;
    
  }else{
    return "Error Reading Serial Number String";
  }
}

struct ConfigVarsR1 OPCR1::read_configuration_variables(){
  // Read the configuration variables and return the structure
  // Ex.
  // $ alpha.read_configuration_variables();
	
  ConfigVarsR1 results;       // empty structure for the data
  byte vals[192];
	
	byte ret=0;
	byte loopCounter=0;
	
	results.DataValid=false;
	
	_ASSpiBegin

	do{ // wait for Return is 0xF3
		delay(10);
		digitalWrite(this->_CS, LOW);
		ret=SPI.transfer(0x3c);
		digitalWrite(this->_CS, HIGH);

		if(loopCounter++>20){
			return results;
		}
		
	}while(ret!=0xf3);
  
	delay(10);
	
	results.DataValid=true;

  digitalWrite(this->_CS, LOW);
  for (int i = 0; i < 193; i++){
    vals[i] = SPI.transfer(0x3c);
    delayMicroseconds(8);
  }
  digitalWrite(this->_CS, HIGH);
	SPI.endTransaction();

  // Fill in the results
  results.bb0 = this->_16bit_int(vals[0], vals[1]);
  results.bb1 = this->_16bit_int(vals[2], vals[3]);
  results.bb2 = this->_16bit_int(vals[4], vals[5]);
  results.bb3 = this->_16bit_int(vals[6], vals[7]);
  results.bb4 = this->_16bit_int(vals[8], vals[9]);
  results.bb5 = this->_16bit_int(vals[10], vals[11]);
  results.bb6 = this->_16bit_int(vals[12], vals[13]);
  results.bb7 = this->_16bit_int(vals[14], vals[15]);
  results.bb8 = this->_16bit_int(vals[16], vals[17]);
  results.bb9 = this->_16bit_int(vals[18], vals[19]);
  results.bb10 = this->_16bit_int(vals[20], vals[21]);
  results.bb11 = this->_16bit_int(vals[22], vals[23]);
  results.bb12 = this->_16bit_int(vals[24], vals[25]);
  results.bb13 = this->_16bit_int(vals[26], vals[27]);
  results.bb14 = this->_16bit_int(vals[28], vals[29]);
  results.bb15 = this->_16bit_int(vals[30], vals[31]);
  results.bb16 = this->_16bit_int(vals[32], vals[33]);

  // Bin Boundaries diameter (um / floats)
  results.bbd0 = this->_calculate_float(vals[34], vals[35], vals[36], vals[37]);
  results.bbd1 = this->_calculate_float(vals[38], vals[39], vals[40], vals[41]);
  results.bbd2 = this->_calculate_float(vals[42], vals[43], vals[44], vals[45]);
  results.bbd3 = this->_calculate_float(vals[46], vals[47], vals[48], vals[49]);
  results.bbd4 = this->_calculate_float(vals[50], vals[51], vals[52], vals[53]);
  results.bbd5 = this->_calculate_float(vals[54], vals[55], vals[56], vals[57]);
  results.bbd6 = this->_calculate_float(vals[58], vals[59], vals[60], vals[61]);
  results.bbd7 = this->_calculate_float(vals[62], vals[63], vals[64], vals[65]);
  results.bbd8 = this->_calculate_float(vals[66], vals[67], vals[68], vals[69]);
  results.bbd9 = this->_calculate_float(vals[70], vals[71], vals[72], vals[73]);
  results.bbd10 = this->_calculate_float(vals[74], vals[75], vals[76], vals[77]);
  results.bbd11 = this->_calculate_float(vals[78], vals[79], vals[80], vals[81]);
  results.bbd12 = this->_calculate_float(vals[82], vals[83], vals[84], vals[85]);
  results.bbd13 = this->_calculate_float(vals[86], vals[87], vals[88], vals[89]);
  results.bbd14 = this->_calculate_float(vals[90], vals[91], vals[92], vals[93]);
  results.bbd15 = this->_calculate_float(vals[94], vals[95], vals[96], vals[97]);
  results.bbd16 = this->_calculate_float(vals[98], vals[99], vals[100], vals[101]);

  // Bin Weightings (floats)
  results.bw0 = this->_calculate_float(vals[102], vals[103], vals[104], vals[105]);
  results.bw1 = this->_calculate_float(vals[106], vals[107], vals[108], vals[109]);
  results.bw2 = this->_calculate_float(vals[110], vals[111], vals[112], vals[113]);
  results.bw3 = this->_calculate_float(vals[114], vals[115], vals[116], vals[117]);
  results.bw4 = this->_calculate_float(vals[118], vals[119], vals[120], vals[121]);
  results.bw5 = this->_calculate_float(vals[122], vals[123], vals[124], vals[125]);
  results.bw6 = this->_calculate_float(vals[126], vals[127], vals[128], vals[129]);
  results.bw7 = this->_calculate_float(vals[130], vals[131], vals[132], vals[133]);
  results.bw8 = this->_calculate_float(vals[134], vals[135], vals[136], vals[137]);
  results.bw9 = this->_calculate_float(vals[138], vals[139], vals[140], vals[141]);
  results.bw10 = this->_calculate_float(vals[142], vals[143], vals[144], vals[145]);
  results.bw11 = this->_calculate_float(vals[146], vals[147], vals[148], vals[149]);
  results.bw12 = this->_calculate_float(vals[150], vals[151], vals[152], vals[153]);
  results.bw13 = this->_calculate_float(vals[154], vals[155], vals[156], vals[157]);
  results.bw14 = this->_calculate_float(vals[158], vals[159], vals[160], vals[161]);
  results.bw15 = this->_calculate_float(vals[162], vals[163], vals[164], vals[165]);

  // Gain Scaling Coefficient
  results.gsc = this->_calculate_float(vals[166], vals[167], vals[168], vals[169]);

  // Sample Flow Rate (ml/s)
  results.sfr = this->_calculate_float(vals[170], vals[171], vals[172], vals[173]);

	// Time-of-Flight to Sample Flow Rate ratio
	results.tof_sfr=byte(vals[174]);
	
	// Particle mass Concentration (floats)
	results.M_A = this->_calculate_float(vals[175], vals[176], vals[177], vals[178]);
  results.M_B = this->_calculate_float(vals[179], vals[180], vals[181], vals[182]);
  results.M_C = this->_calculate_float(vals[183], vals[184], vals[185], vals[186]);

	// Time-of-Flight to Sample Flow Rate ratio
	results.pvp=byte(vals[187]);

	// Power Status
	results.powerStatus=byte(vals[188]);

	//Maximum Time Of Flight
	results.maxTOF=(float)((double)this->_16bit_int(vals[189],vals[190]))/48;

  // LaserDAC
	results.laser_dac=byte(vals[191]);
	
	// BinWeightingIndex
	results.bwi=byte(vals[192]);
	
	//Serial.println(results.DataValid);

  return results;
	
}

struct PMDataR1 OPCR1::read_pm_data(){
  // Read the PM Data and reset the histogram, return the struct
  // Ex.
  // $ alpha.read_pm_data();
  PMDataR1 data;
  byte vals[14];
	
	byte ret=0;
	byte loopCounter=0;
	data.DataValid=false;
	
	_ASSpiBegin
	
  // Read the data and clear the local memory
  do{ // wait for Return is 0xF3
		delay(12);
		digitalWrite(this->_CS, LOW);
		ret=SPI.transfer(0x32);
		digitalWrite(this->_CS, HIGH);
		if(loopCounter++>20){
			return data;
		}
	}while(ret!=0xF3);
	
	data.DataValid=true;

  // Send commands and build array of data
  digitalWrite(this->_CS, LOW);

  for (int i = 0;i<14;i++){
		vals[i] = SPI.transfer(0x32);
		delayMicroseconds(10);
  }

  digitalWrite(this->_CS, HIGH);
	
	SPI.endTransaction();

  data.pm1  = this->_calculate_float(vals[0], vals[1], vals[2], vals[3]);
  data.pm25 = this->_calculate_float(vals[4], vals[5], vals[6], vals[7]);
  data.pm10 = this->_calculate_float(vals[8], vals[9], vals[10], vals[11]);

  return data;
}

struct HistogramDataR1 OPCR1::read_histogram(bool convert_to_conc){
  // Read the Histogram Data and reset the histogram, return the struct
  // convert_to_conc can be set to true if you would like the result
  // returned as concentrations (rather than raw counts) with units of
  // particles per cubic centimeter [#/cc]
  // Ex.
  // $ alpha.read_histogram(true)
	
  HistogramDataR1 data;
  byte vals[64];
	
	byte ret=0;
	byte loopCounter=0;
	data.DataValid=false;
	
	_ASSpiBegin
	
	do{
		delay(10);
		digitalWrite(this->_CS, LOW);
		ret=SPI.transfer(0x30);
		digitalWrite(this->_CS, HIGH);
		if(loopCounter++>20){
			return data;
		}
	}while(ret!=0xF3);
	
	data.DataValid=true;

	delayMicroseconds(8);

  // Send commands and build array of data
  digitalWrite(this->_CS, LOW);
  for (int i=0;i<64;i++){
		vals[i] = SPI.transfer(0x30);
		delayMicroseconds(8);
  }
  digitalWrite(this->_CS, HIGH);
	
	SPI.endTransaction();

  data.period = this->_calculate_float(vals[44], vals[45], vals[46], vals[47]);
  data.sfr    = this->_calculate_float(vals[36], vals[37], vals[38], vals[39]);

  // If convert_to_conc = True, convert from raw data to concentration
  double conv;

  if(convert_to_conc != true){
    conv=1.0;
  }
  else {
    conv=data.sfr*data.period;
  }

  // Calculate all of the values!

  data.bin0   = this->_oneDec(((float)this->_16bit_int(vals[0],vals[1])/conv));
  data.bin1   = this->_oneDec(((float)this->_16bit_int(vals[2],vals[3])/conv));
  data.bin2   = this->_oneDec(((float)this->_16bit_int(vals[4],vals[5])/conv));
  data.bin3   = this->_oneDec(((float)this->_16bit_int(vals[6],vals[7])/conv));
  data.bin4   = this->_oneDec(((float)this->_16bit_int(vals[8],vals[9])/conv));
  data.bin5   = this->_oneDec(((float)this->_16bit_int(vals[10],vals[11])/conv));
  data.bin6   = this->_oneDec(((float)this->_16bit_int(vals[12],vals[13])/conv));
  data.bin7   = this->_oneDec(((float)this->_16bit_int(vals[14],vals[15])/conv));
  data.bin8   = this->_oneDec(((float)this->_16bit_int(vals[16],vals[17])/conv));
  data.bin9   = this->_oneDec(((float)this->_16bit_int(vals[18],vals[19])/conv));
  data.bin10  = this->_oneDec(((float)this->_16bit_int(vals[20],vals[21])/conv));
  data.bin11  = this->_oneDec(((float)this->_16bit_int(vals[22],vals[23])/conv));
  data.bin12  = this->_oneDec(((float)this->_16bit_int(vals[24],vals[25])/conv));
  data.bin13  = this->_oneDec(((float)this->_16bit_int(vals[26],vals[27])/conv));
  data.bin14  = this->_oneDec(((float)this->_16bit_int(vals[28],vals[29])/conv));
  data.bin15  = this->_oneDec(((float)this->_16bit_int(vals[30],vals[31])/conv));
 
  int sumupInt=round(data.bin0*10.0);
  sumupInt+=round(data.bin1*10.0);
  sumupInt+=round(data.bin2*10.0);
  sumupInt+=round(data.bin3*10.0);
  sumupInt+=round(data.bin4*10.0);
  sumupInt+=round(data.bin5*10.0);
  sumupInt+=round(data.bin6*10.0);
  sumupInt+=round(data.bin7*10.0);
  sumupInt+=round(data.bin8*10.0);
  sumupInt+=round(data.bin9*10.0);
  sumupInt+=round(data.bin10*10.0);
  sumupInt+=round(data.bin11*10.0);
  sumupInt+=round(data.bin12*10.0);
  sumupInt+=round(data.bin13*10.0);
  sumupInt+=round(data.bin14*10.0);
  sumupInt+=round(data.bin15*10.0);

  data.sumAllBins=sumupInt/10.0;

	data.bin1MToF = int(vals[32]) / 3.0;
  data.bin3MToF = int(vals[33]) / 3.0;
  data.bin5MToF = int(vals[34]) / 3.0;
  data.bin7MToF = int(vals[35]) / 3.0;
	
	// Sample Flow Rate
	data.sfr = this->_calculate_float(vals[36],vals[37],vals[38],vals[39]);
	
	// Temperature
	data.temp=(double)this->_16bit_int(vals[40],vals[41]);

	// Humidity
	data.humidity=(double)this->_16bit_int(vals[42], vals[43]);
	
	// Sampling Period
	data.period = this->_calculate_float(vals[44],vals[45],vals[46],vals[47]);
	
	// Reject Count Glitch
	data.rcg=byte(vals[48]);
	
	//Reject Count LONG
	data.rcl=byte(vals[49]);

	// PM's
  data.pmA = this->_calculate_float(vals[50], vals[51], vals[52], vals[53]);
  data.pmB = this->_calculate_float(vals[54], vals[55], vals[56], vals[57]);
  data.pmC = this->_calculate_float(vals[58], vals[59], vals[60], vals[61]);

  data.checksum = this->_16bit_int(vals[62], vals[63]);
	
  return data;
}

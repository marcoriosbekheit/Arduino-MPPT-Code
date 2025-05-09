// ARDUINO MPPT CODE - MARCORIOS BEKHEIT - LAST UPDATE: 4-30-2021
/* 
 * Note, some variables may be defined and not used. This is due
 * to a change in the circuit. 
 * - Output voltage and current sensors couldn't be used anymore
 *   because common ground changed. 
 * - SD related functions were deleted due to SD module not working.
 */



#include <SPI.h>
#include <SD.h>
#include <TimerOne.h>
#include <math.h>

//Display Variables
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#define I2C_ADDRESS 0x3C
SSD1306AsciiAvrI2c oled;
int timeOldDisplay = millis();
const int DISPLAY_PERIOD = 1000;

// Voltage/Current Sensor Variables
double Vin, Vout, Iin, Iout;
const double Vref = 1.102; // measured from ref or 5V pin

// Voltage Sensor 1 - Vin - connect to solar panel
const double r1_1 = 182.07;
const double r1_2 = 9.4230;
const double r1_analogPin = 2;

// Current Calibration
const int calibratePin = 4;
bool runOnce = 1;

// Current Sensor 1 - Iin - connect to solar panel
const double i1_Pin = 6;
const double i1_Vneg = -2.413;
const double i1_RPos = 9.95;
const double i1_RNeg = 9.97;
double Vi1_zero = 0;

// MPPT variables
double Pprev = 0, Pnow = 0, Vprev = 0;
const int delta = 1; 
int duty = 128;
int pwmPin1 = 9; // in the example: uint8_t pin = 9;

// functions
double vSensor1();
double iSensor1();
bool isWriteOn();
void dataLog(double V1, double V2, double I1, double I2, int d); // add temperature later
void mppt(double Vnow, double I);
void freqInitialize();
void pwmDutySet();
void runDisplay(double V1, double V2, double I1, double I2, int d); // add temperature later
void currentCalibration(int isRunOnce);


void setup() {
	// Pin Mode Initialization
	pinMode(calibratePin, INPUT_PULLUP);
	
	// Frequency Initialization
	freqInitialize();
	
	// Set Reference Voltage
	analogReference(INTERNAL);
	
	// Display Set up
	oled.begin(&Adafruit128x64, I2C_ADDRESS);
	oled.setFont(Adafruit5x7);
	oled.clear();
}


void loop() {
	// calibrate current (run once)
	currentCalibration();
	
	// read data
	Vin = vSensor1();
	Iin = iSensor1(); // define
	
	// run MPPT
	mppt(Vin, Iin);
	
	// Update Display every DISPLAY_PERIOD
	int timeNowDisplay = millis();
	if(timeNowDisplay - timeOldDisplay > DISPLAY_PERIOD){
		runDisplay(Vin, Vout, Iin, Iout, duty);
		timeOldDisplay = millis();
	}
}

double vSensor1(){
	int numReads = 100;
	double sum = 0;
	// ** add up multiple inputs to get an average reading to decrease random error 
	// -- add inputs
	for(int i = 0; i < numReads; i++){
		sum += analogRead(r1_analogPin);
	}
	// -- get average from recorded inputs
	sum = sum / numReads; // this is the average bit reading
	// -- convert average bit readings to Voltage
	Vout = sum * Vref/1023.0; // units - bits * (V / bits)  = V
	
	double Vcc = Vout * (r1_1 + r1_2)/r1_2;
	
	return Vcc;
}


double iSensor1(){
	int numReads = 500;
	double avgBit = 0;
	
	// ** add up multiple inputs to get an average reading to decrease random error 
	// -- add inputs
	for(int i = 0; i < numReads; i++){
		avgBit += analogRead(i1_Pin);;
	}
	// -- get average from recorded inputs
	avgBit = avgBit / numReads; // this is the average bit reading
	
	// convert average bit readings to Voltage, Vout.
	double V1out = avgBit * Vref/1023.0; 
	
	// some math
	double Vi1 = (i1_RPos/i1_RNeg) * (V1out - i1_Vneg) + V1out;
	
	// get the voltage at Vcc 
	double Vcc_diff = Vi1 - Vi1_zero;
	
	// convert Vcc to Current
	double I1 = Vcc_diff * 1.0/0.1; // 1 A / 100 mV = 1 A / 0.1 V
	
	return I1;
}


void mppt(double Vnow, double I){
	Pnow = Vnow * I;
	
	// MPPT for Buck Converter
	if (Pnow - Pprev == 0){
		// do nothing
	}
	else if (Pnow > Pprev){
		if (Vnow > Vprev){
			duty = duty - delta;
		}
		else {
			duty = duty + delta;
		}
	}
	else {
		if (Vnow > Vprev){
			duty = duty + delta;
		}
		else {
			duty = duty - delta;
		}
	}
	
	Pprev = Pnow;
	Vprev = Vnow;
	
	if (duty < 20) duty = 20;
	if (duty > 230) duty = 230;
	
	pwmDutySet();

}

void freqInitialize(){
	Timer1.initialize(55); // 55 microsecond period ~= 18.18 khz 
	int pwmDuty = (int) duty*1023.0/255.0; // converting 8 bit to 10 bit input for
	                                       //    for Timer1.pwm()
	Timer1.pwm(pwmPin1, pwmDuty);
}

void pwmDutySet(){
	int pwmDuty = round(duty*1023.0/255.0);
	Timer1.pwm(pwmPin1, pwmDuty);
}

void runDisplay(double V1, double V2, double I1, double I2, int d){
	
	// calculate power
	double P1 = V1 * I1;
	double P2 = V2 * I2;
	
	oled.clear(); 
	oled.println(F("Input    "));
	oled.println(F("-------------------"));
	oled.println("V = " + String(V1));
	oled.println("I = " + String(I1));
	oled.println("P = " + String(P1));
	oled.println(F("--------------------"));
	oled.println("D = " + String(d));
} 


void currentCalibration() {
	if(runOnce){
		bool isCalibrateOn;
		do {
			// **** get Voffset 1
			int numReads = 1000;
			double avgBit = 0;
			double V1out = 0, V2out = 0;
			// ** add up multiple inputs to get an average reading to decrease random error 
			// -- add inputs
			for(int i = 0; i < numReads; i++){
				avgBit += analogRead(i1_Pin);;
			}
			// -- get average from recorded inputs
			avgBit = avgBit / numReads; // this is the average bit reading	
			
			// -- convert average bit readings to Voltage, Vout.
			V1out = avgBit * Vref/1023; 
			
			// find Vi_zero from Vout and other Constants
			Vi1_zero = (i1_RPos/i1_RNeg) * (V1out - i1_Vneg) + V1out;
			
			// update display
			oled.clear();
			oled.println(F("Current Calibration"));
			oled.println(F("--------------------"));
			oled.println("Vi1_zero = " + String(Vi1_zero,2));
			oled.println("V1_out = " + String(V1out,4));
			oled.println(F("(I1 = Iin, I2 = Iout)"));
			delay(2000);
			isCalibrateOn = digitalRead(calibratePin);
		} while(!isCalibrateOn);
		
		// update display
		oled.clear();
		oled.println(F("Current Calibration"));
		oled.println(F("Finished"));
	}
	runOnce = 0;
}

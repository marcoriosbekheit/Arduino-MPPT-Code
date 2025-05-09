#include <SPI.h>
#include <SD.h>
#include <TimerOne.h>
// #include <PWM.h>
#include <math.h>

/*
 * Error Definitions on fault LED:
 * 		constant on       ====> SD card initialization failed
 * 		T = 2s blinking   ====> Failed to write to file
 * 		
 * 
 * Below has been cancelled
 * Built In LED
 * 		Turns on once     ====> Frequency Initialization successful
 * 
 * Should I add a delay in the loop function?
 */


//Display Variables
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C
SSD1306AsciiAvrI2c oled;
int timeOldDisplay = millis();
const int DISPLAY_PERIOD = 1000;

// Voltage/Current Sensor Variables
double Vin, Vout, Iin, Iout;
// const double Vref = 1.090; // measured from ref or 5V pin (arduino dead)
const double Vref = 1.102; // measured from ref or 5V pin

// Voltage Sensor 1 - Vin - connect to solar panel
// const double r1_1 = 17.780;
// const double r1_2 = 5.2130;
const double r1_1 = 182.07;
const double r1_2 = 9.4230;
const double r1_analogPin = 2;

// Voltage Sensor 2 - Vout - connect to load
// const double r2_1 = 17.970;
// const double r2_2 = 5.2580;
const double r2_1 = 181.91;
const double r2_2 = 9.441;
const double r2_analogPin = 0;

// Current Calibration
const int calibratePin = 4;
bool runOnce = 1; // needed for prototyping, 
				  // can calibrate in setup() after protyping is finished
				  // and not use this variable (I don't think this should stay)

// Current Sensor 1 - Iin - connect to solar panel
const double i1_Pin = 6;
// const double i1_Vneg = -2.427; // keep the negative (arduino dead)
const double i1_Vneg = -2.413; // keep the negative
const double i1_RPos = 9.95;
const double i1_RNeg = 9.97;
double Vi1_zero = 0;

// Current Sensor 2 - Iout - connect to load
const double i2_Pin = 7;
// const double i2_Vneg = -2.465; // keep the negative (arduino dead)
const double i2_Vneg = -2.443; // keep the negative
const double i2_RPos = 10.0;
const double i2_RNeg =  9.97;
double Vi2_zero = 0;

// SD card variables
const int isWritePin  = 8; // digital read pin
const int blueLEDPin  = 7; // SD is being used, data-logging
const int greenLEDPin = 6; // SD card is no longer used, safe to remove
bool writeStatus = 1;      
bool SDoff = 0;
const int faultPin = 3; // needs to be a PWM pin or fault will not work
const int chipSelect = 10; //SD card pin
int timeOld = millis(); // get time, used to datalog every dataLogPeriod
int dataLogPeriod = 10000; // 10000 ms = 10s 



// MPPT variables
double Pprev = 0, Pnow = 0, Vprev = 0;
const int delta = 1; 
int duty = 128;
int pwmPin1 = 9; // in the example: uint8_t pin = 9;
// int32_t frequency = 18018; // 18018

// functions
double vSensor1();
double vSensor2();
double iSensor1();
double iSensor2();
bool isWriteOn();
void dataLog(double V1, double V2, double I1, double I2, int d); // add temperature later
void mppt(double Vnow, double I);
void freqInitialize();
void pwmDutySet();
void runDisplay(double V1, double V2, double I1, double I2, int d); // add temperature later
void currentCalibration(int isRunOnce);


// int num, runs=0;
// double avg;


void setup() {
//     Serial.begin(9600);
	
	// *****************************************
	// SD Card Read/Write Set up
	// *****************************************
	// see if the card is present and can be initialized:
	if (!SD.begin(chipSelect)) {
		// 		Serial.println("Card failed, or not present");
		// don't do anything more:
		analogWrite(faultPin, 255/2); // can use this a speaker
		while (1);
	}
	
	// *****************************************
	// Pin Mode Initialization
	// *****************************************
	pinMode(isWritePin, INPUT_PULLUP); // INPUT PULLUP needed to prevent short
	pinMode(calibratePin, INPUT_PULLUP); // INPUT PULLUP needed to prevent short
	
	// *****************************************
	// Frequency Initialization
	// *****************************************
	freqInitialize();
	
	// *****************************************
	// Set Reference Voltage
	// *****************************************
	analogReference(INTERNAL);
	
	// *****************************************
	// Display Set up
	// *****************************************
	oled.begin(&Adafruit128x64, I2C_ADDRESS);
	oled.setFont(Adafruit5x7);
// 	uint32_t m = micros(); // is this line needed?
	oled.clear();
	
	// *****************************************
	// Current Sensor Calibration
	// *****************************************
	// 	if(digitalRead(calibratePin)) 
// 	bool isCalibrateOn;
// 	do {
// 		// **** get Voffset 1
// 		int numReads = 1000;
// 		double avgBit = 0;
// 		double V1out = 0, V2out = 0;
// 		// ** add up multiple inputs to get an average reading to decrease random error 
// 		// -- add inputs
// 		for(int i = 0; i < numReads; i++){
// 			avgBit += analogRead(i1_Pin);;
// 		}
// 		// -- get average from recorded inputs
// 		avgBit = avgBit / numReads; // this is the average bit reading	
// 		
// 		// -- convert average bit readings to Voltage, Vout.
// 		V1out = avgBit * Vref/1023; 
// 		
// 		// find Vi_zero from Vout and other Constants
// 		Vi1_zero = (i1_RPos/i1_RNeg) * (V1out - i1_Vneg) + V1out;
// 		
// // 		Vi1_zero = sum * Vref/1023.0; // units - bits * (V / bits)  = V
// 		
// 		
// 		// **** get Voffset 2
// 		numReads = 1000;
// 		avgBit = 0;
// 		V2out = 0;
// 		// ** add up multiple inputs to get an average reading to decrease random error 
// 		// -- add inputs
// 		for(int i = 0; i < numReads; i++){
// 			avgBit += analogRead(i2_Pin);;
// 		}
// 		// -- get average from recorded inputs
// 		avgBit = avgBit / numReads; // this is the average bit reading	
// 		
// 		// -- convert average bit readings to Voltage, Vout.
// 		V2out = avgBit * Vref/1023; 
// 		
// 		// find Vi_zero from Vout and other Constants
// 		Vi2_zero = (i2_RPos/i2_RNeg) * (V2out - i2_Vneg) + V2out;
// 		
// // 		// -- convert average bit readings to Voltage, Vout.
// // 		Vi2_zero = sum * Vref/1023.0; // units - bits * (V / bits)  = V
// 		
// 		// update display
// 		oled.clear();
// 		oled.println(F("Current Calibration"));
// 		oled.println(F("--------------------"));
// 		oled.println("Vi1_zero = " + String(Vi1_zero,2));
// 		oled.println("Vi2_zero = " + String(Vi2_zero,2));
// 		oled.println("V1_out = " + String(V1out,4));
// 		oled.println("V2_out = " + String(V2out,4));
// 		oled.println(F("(I1 = Iin, I2 = Iout)"));
// 		delay(2000);
// 		isCalibrateOn = digitalRead(calibratePin);
// 	} while(!isCalibrateOn);
// 	
// 	// update display
// 	oled.clear();
// 	oled.println(F("Current Calibration"));
// 	oled.println(F("Finished"));
	
	// *****************************************
	// Notes
	// *****************************************
	// TO DO
	// * Need to calibrate ACS712 by opening all switches so that no current is running through it 
	//   while simulating a normal loop operations to determine its zero current point
	//     - open all switches / close current sources
	//     - run datalogging and mppt code for 20 times
	//     - collect data of V at zero current for ACS712
	//     - average collected data
	// * o calculate V for I-sensor from op-amp and voltage divider 
	// * x create MPPT-algorithm
	// * x test 18khz switching
	// * o test current sensor with 12v dc power supply and resistive heater
	// * o start simulating
	// * o build
	// * o setup pinMode for every used pin
	// * o change as many variables to local as possible
	// * o test delta = 1 vs 3
	// * o disable Serial.begin(9600);
	// * o instead of clearing the whole screen, clear the numbers only on the OLED
	//          - not urgent
	// * o update I1 & I2 analog pins on the diagram
	// * o automate calibration process if MOSFET open means
	//		  no current flows on both sides (input and output)
	
	/*
	 * START HERE START HERE START HERE START HERE START HERE START HERE 
	 * START HERE START HERE START HERE START HERE START HERE START HERE
	 * START HERE START HERE START HERE START HERE START HERE START HERE 
	 * START HERE START HERE START HERE START HERE START HERE START HERE  
	 * convert to 1.1 V internal reference
	 * 		test stability of internal reference
	 * 		test stability of 3.3 V rail, I can use it instead of 5V rail to offset 
	 * 			current sensor voltage
	 * 
	 * 1. find V5ref for arduino nano in use
	 * 2. find best fit resistors
	 * 3. build op-amp
	 * 4. test voltage gain
	 * 5. add to current sensor
	 */
	
	
	/*
	 * convert any constant less than 255 to short int in MPPT_2
	 * 		removing my global variables saved about 2% memory
	 * 		I can change strings to include F()
	 * 		the big users are timerone.h and SD.h
	 *	Notes:
	 * PC power:
	 * 		turning SD write on, changed Vref from 1.091 to 1.092
	 * 			once blue LED on, Vref is constant, no change
	 * 			same result with OLED screen attached
	 *		5V rail is varying by 1-2 mV
	 *		5V = 5.055 +- 0.002
	 * 		3V rail = 3.279 +- 0.001 (w/ oled + SD write)
	 * 1A/2.1A PSU:
	 * 		5V = 5.253 +- 0.003
	 * 		Vref = 1.091
	 * 		turning SD write on, changed Vref from 1.091 to 1.092
	 * 		3V = 3.278 +- 0.002
	 * 2.4A/2.4A PSU:
	 * 		5V = 5.300 +- 0.001
	 * 		Vref = 1.091
	 * 		turning SD write on, changed Vref from 1.091 to 1.092
	 * 		3V = 3.278 +- 0.002
	 * 
	 * similar result with apple 12W charger
	 * 		3V = 3.278 +- 0.002
	 */
}


void loop() {
	
	// calibrate current
	currentCalibration();
	
	// read data
	Vin = vSensor1();
	Vout = vSensor2();
	Iin = iSensor1(); // define
	Iout = iSensor2(); // define
	
	// run MPPT
	mppt(Vin, Iin);
	
	// SD card datalogging
	writeStatus = isWriteOn();
	if(writeStatus){
		
		// if SD was removed and re-inserted,
		// re-initialize the SD card to write again
		if(SDoff){
			
			// if there is an issue, turn on fault pin
			if (!SD.begin(chipSelect)) {
				analogWrite(faultPin, 255/2); // can use this when I attach a speaker to it
				digitalWrite(greenLEDPin, LOW);
				digitalWrite(blueLEDPin, LOW);
				while (1);
			}
			// if no issue, then SD is inserted and working
			//    so SDoff = 0
			//    because SD is inserted
			SDoff = 0;
		}
		
		// once SD is initialized, start data-logging every time period
		int timeNow = millis();
        if(timeNow - timeOld > dataLogPeriod){
            dataLog(Vin, Vout, Iin, Iout, duty);
			timeOld = millis();
        }
		
	} else {
		// if write status is off, then assume SD card has been removed
		SDoff = 1;
	}
	
	// Update Display every DISPLAY_PERIOD
	int timeNowDisplay = millis();
	if(timeNowDisplay - timeOldDisplay > DISPLAY_PERIOD){
		runDisplay(Vin, Vout, Iin, Iout, duty);
		timeOldDisplay = millis();
	}
}

double vSensor1(){
// 	double inputA = analogRead(r1_analogPin);
// 	double Vout = inputA * Vref/1023.0;
	
	int numReads = 100;
	double sum = 0;
	// ** add up multiple inputs to get an average reading to decrease random error 
	// -- add inputs
	for(int i = 0; i < numReads; i++){
		sum += analogRead(r1_analogPin);;
	}
	// -- get average from recorded inputs
	sum = sum / numReads; // this is the average bit reading	
	// -- convert average bit readings to Voltage
	Vout = sum * Vref/1023.0; // units - bits * (V / bits)  = V
	
	double Vcc = Vout * (r1_1 + r1_2)/r1_2;
	
	return Vcc;
}


double vSensor2(){
// 	double inputA = analogRead(r2_analogPin);
// 	double Vout = inputA * Vref/1023.0;
	
	int numReads = 100;
	double sum = 0;
	// ** add up multiple inputs to get an average reading to decrease random error 
	// -- add inputs
	for(int i = 0; i < numReads; i++){
		sum += analogRead(r2_analogPin);;
	}
	// -- get average from recorded inputs
	sum = sum / numReads; // this is the average bit reading	
	// -- convert average bit readings to Voltage
	Vout = sum * Vref/1023.0; // units - bits * (V / bits)  = V
	
	double Vcc = Vout * (r2_1 + r2_2)/r2_2;
	
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
	double V1out = avgBit * Vref/1023.0; // units - bits * (V / bits)  = V
	
	double Vi1 = (i1_RPos/i1_RNeg) * (V1out - i1_Vneg) + V1out;
	
// 	// subtract offset voltage
// 	Vout = Vout - Vi1_zero;
	
	// get the voltage at Vcc 
	double Vcc_diff = Vi1 - Vi1_zero;
	
	// convert Vcc to Current
	double I1 = Vcc_diff * 1.0/0.1; // 1 A / 100 mA = 1 A / 0.1 V
	
	return I1;
}


double iSensor2(){
// 	double inputA = analogRead(i2_Pin);
// 	double Vout = inputA * Vref/1023.0;
// 	double I2 = Vout * 1.0/0.1; // 1 A / 100 mA = 1 A / 0.1 V
// 	
// 	return I2;
	
	
	int numReads = 500;
	double avgBit = 0;
	
	// ** add up multiple inputs to get an average reading to decrease random error 
	// -- add inputs
	for(int i = 0; i < numReads; i++){
		avgBit += analogRead(i2_Pin);;
	}
	// -- get average from recorded inputs
	avgBit = avgBit / numReads; // this is the average bit reading	
	
	// convert average bit readings to Voltage, Vout.
	double V2out = avgBit * Vref/1023.0; // units - bits * (V / bits)  = V
	
	
	double Vi2 = (i2_RPos/i2_RNeg) * (V2out - i2_Vneg) + V2out;
	
	// subtract offset voltage
// 	Vout = Vout - Vi2_zero;
	
	// get the voltage at Vcc 
	double Vcc_diff = Vi2 - Vi2_zero;
	
	// convert Vcc to Current
	double I2 = Vcc_diff * 1.0/0.1; // 1 A / 100 mA = 1 A / 0.1 V
	
	return I2;
}




bool isWriteOn(){
	// 	double input = analogRead(isWritePin);
	// 	double Vout = input * Vref/1023.0;
	// 	if(Vout > Vref/2.0){...stuff...}
	bool input = digitalRead(isWritePin); // if this works, change isWritePin to = 8
	
	if(input){
		digitalWrite(greenLEDPin, LOW);
		digitalWrite(blueLEDPin, HIGH);
		return 1;
	} else {
		digitalWrite(greenLEDPin, HIGH);
		digitalWrite(blueLEDPin, LOW);
		return 0;
	}
	
}

void dataLog(double V1, double V2, double I1, double I2, int d){

	// record data to string
	String dataString = "";
	dataString = String(V1) + "," + String(V2) ;
	dataString += "," + String(I1) + "," + String(I2) + "," + String(d);
// 	dataString += "," + String(T);
	
	
	// open the file
	File dataFile = SD.open("datalog.txt", FILE_WRITE);
	
	// if the file is available, write to it:
	if (dataFile) {
		dataFile.println(dataString);
		dataFile.close();
		// print to the serial port too:
//         Serial.println(dataString);
	}
	// if the file isn't open, pop up an error:
	else {
		// 		Serial.println("error opening datalog.txt");
		digitalWrite(greenLEDPin, LOW);
		digitalWrite(blueLEDPin, LOW);
		while(1){
			analogWrite(faultPin, 255/2);
			delay(1000);  
			analogWrite(faultPin, 0);
			delay(1000); 
		}
	}
}


void mppt(double Vnow, double I){
	Pnow = Vnow * I;
	
	// MPPT for Buck Converter
	if (Pnow - Pprev == 0){
		// do nothing
	}
	else if (Pnow > Pprev){
		if (Vnow > Vprev){
			duty = duty + delta;
		}
		else {
			duty = duty - delta;
		}
	}
	else {
		if (Vnow > Vprev){
			duty = duty - delta;
		}
		else {
			duty = duty + delta;
		}
	}
	
	Pprev = Pnow;
	Vprev = Vnow;
	
	if (duty < 20) duty = 20;
	if (duty > 230) duty = 230;
	// absolute limits 
	// 		based on measured PWM frequency using digital multimeter
// 	if (duty < 3) duty = 3; // if d < 3, PWM is off
// 	if (duty > 254) duty = 254; // if d > 254, PWM is off
	
	pwmDutySet();

}

void freqInitialize(){
	// ---------------------------------------
	// using PWM.h, an outdated library
	// ---------------------------------------
	//    * this is has been tested to work
	//      but gives warning of core code altered.
	//      This could be b/c it is outdated. 
	//      Safe to use other newer option.
// 	InitTimersSafe();
// 	bool success = SetPinFrequencySafe(pwmPin1, frequency);
// 	
// 	// turn on on-board LED
// 	if(success) {
// 		//pinMode(LED_BUILTIN, OUTPUT);
// 		digitalWrite(LED_BUILTIN, HIGH);
// 		delay(2000);
// 		digitalWrite(LED_BUILTIN, LOW);
// 		
// 	} else{
// 		while(1) {
// 			digitalWrite(LED_BUILTIN, HIGH);
// 			delay(50);
// 			digitalWrite(LED_BUILTIN, LOW);
// 			delay(50);
// 		}
// 	}
// 	
// 	pwmWrite(pwmPin1,duty); // without it, PWM is not turned on
	
	// ---------------------------------------
	// using TimerOne.h, an updated library
	// ---------------------------------------
	Timer1.initialize(55); // 55 microsecond period ~= 18.18 khz 
			// breaks pin 9 and 10 analogWrite function
	int pwmDuty = (int) duty*1023.0/255.0; // converting 8 bit to 10 bit input for
	                                       //    for Timer1.pwm()
	Timer1.pwm(pwmPin1, pwmDuty);      // 50% DC on pin 9  
	
}

void pwmDutySet(){
	// 	pwmWrite(pwmPin1,duty); // this if using PWM.h 
	int pwmDuty = round(duty*1023.0/255.0);
	Timer1.pwm(pwmPin1, pwmDuty);
}

void runDisplay(double V1, double V2, double I1, double I2, int d){
	
	// ** calculate power
	double P1 = V1 * I1;
	double P2 = V2 * I2;
	
// 	// ** set up display 
// 	display.clearDisplay();
// 	display.setTextSize(1);             // Normal 1:1 pixel scale
// 	display.setTextColor(SSD1306_WHITE);        // Draw white text
// 	display.setCursor(0,0);             // Start at top-left corner
// 
// 	// ** set up text to display
// 	// -- by default String() outputs 2 decimal places
// 	// 		otherwise use String(num,numDecimalPlaces)
// 	display.println(F("Input    | Output   "));
// 	display.println(F("---------|----------"));
// 	display.println("V = " + String(V1) + "| V = " + String(V2));
// 	display.println("I = " + String(I1) + "| I = " + String(I2));
// 	display.println("P = " + String(P1) + "| P = " + String(P2));
// 	display.println(F("--------------------"));
// 	display.println("D = " + String(d) + "| e = " + String(P2/P1));
// 	
// 	// ** implement change to display
// 	display.display();
	oled.clear(); // test it with and without this line
	oled.println(F("Input    | Output   "));
	oled.println(F("---------|----------"));
	oled.println("V = " + String(V1) + "| V = " + String(V2));
	oled.println("I = " + String(I1) + "| I = " + String(I2));
	oled.println("P = " + String(P1) + "| P = " + String(P2));
	oled.println(F("--------------------"));
	oled.println("D = " + String(d) + " | e = " + String(P2/P1));
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
			
	// 		Vi1_zero = sum * Vref/1023.0; // units - bits * (V / bits)  = V
			
			
			// **** get Voffset 2
			numReads = 1000;
			avgBit = 0;
			V2out = 0;
			// ** add up multiple inputs to get an average reading to decrease random error 
			// -- add inputs
			for(int i = 0; i < numReads; i++){
				avgBit += analogRead(i2_Pin);;
			}
			// -- get average from recorded inputs
			avgBit = avgBit / numReads; // this is the average bit reading	
			
			// -- convert average bit readings to Voltage, Vout.
			V2out = avgBit * Vref/1023; 
			
			// find Vi_zero from Vout and other Constants
			Vi2_zero = (i2_RPos/i2_RNeg) * (V2out - i2_Vneg) + V2out;
			
	// 		// -- convert average bit readings to Voltage, Vout.
	// 		Vi2_zero = sum * Vref/1023.0; // units - bits * (V / bits)  = V
			
			// update display
			oled.clear();
			oled.println(F("Current Calibration"));
			oled.println(F("--------------------"));
			oled.println("Vi1_zero = " + String(Vi1_zero,2));
			oled.println("Vi2_zero = " + String(Vi2_zero,2));
			oled.println("V1_out = " + String(V1out,4));
			oled.println("V2_out = " + String(V2out,4));
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



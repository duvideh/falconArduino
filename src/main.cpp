// ***Arduino UNO connected to DWIN 960x280 screen***
// MCP2515 to read CAN bus and display readings to DWIN screen
// SoftwareSerial to communicate with DWIN screen - both send and receive
// 
// 
// 
// 
// 
// 

#include <Arduino.h>
#include <math.h>
#include <mcp_can.h> //library: coryjfowler/mcp_can
#include <SoftwareSerial.h>


//Oil temperature sensor
#define oilTempSensorDivider 5080   //defines the resistor value that is in series in the voltage divider
#define oilTempSensorPin A1         //defines the analog pin of the input voltage from the voltage divider
#define NUMSAMPLES 5                //defines the number of samples to be taken for a smooth average
const float steinconstA = 0.0004133244513630420;        //steinhart equation constant A, determined from wikipedia equations
const float steinconstB = 0.0002939949352019070;       //steinhart equation constant B, determined from wikipedia equations
const float steinconstC = -0.0000002163718111393;    //steinhart equation constant C, determined from wikipedia equations
int samples[NUMSAMPLES];                              //variable to store number of samples to be taken

//CAN bus
MCP_CAN CAN0(7); // bracketed number is number of CS pin
int ex = 0;
int why = 0;
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
int outsideTemp = 0;
int coolantTemp = 0;
int headlightState = 0;
int battCAN = 0;
int absoluteAir = 0;

//battery voltage
#define battPin A3
#define NUMBATTSAMPLES 10
float voltage = 0.0;
int battSamples[NUMBATTSAMPLES];

//SoftwareSerial
const byte rxPin = 9;
const byte txPin = 8;
SoftwareSerial dwinSerial (rxPin, txPin); //set up a new Software Serial object

//DWIN stuff
unsigned char incomingData[100];      //received chars
unsigned char textString[100];        //Chars to send
unsigned char receivedFloatArray[50]; //Array created form the useful part of the recevied char
float receivedFloat;                  //float converted from the above array using atof()
float floatVoltageValue;  //ADC analog value convertedc into voltage (stored as float)
int boostInt;              //floatVoltageValue converted into 0-180 for gauge animation
int pressureInt;           //floatVoltageValue converted into 0-120 for gauge animation
int digitsAfterDp = 0;     //number of digits after decimal point in float to send to screen
long dwinTimer;           //read and update screen values every x milliseconds only



unsigned long millis10 = 0;
unsigned long millis50 = 0;
unsigned long millis200 = 0;



//__      ______ _____ _____
// \ \    / / __ \_   _|  __ \ '/'
//  \ \  / / |  | || | | |  | |
//   \ \/ /| |  | || | | |  | |
//    \  / | |__| || |_| |__| |
//     \/   \____/_____|_____/
//
//              _   __  __                                
//             | | |  \/  |                               
//    __ _  ___| |_| \  / | ___  ___ ___  __ _  __ _  ___ 
//   / _` |/ _ \ __| |\/| |/ _ \/ __/ __|/ _` |/ _` |/ _ \ '/'
//  | (_| |  __/ |_| |  | |  __/\__ \__ \ (_| | (_| |  __/
//   \__, |\___|\__|_|  |_|\___||___/___/\__,_|\__, |\___|
//    __/ |                                     __/ |     
//   |___/                                     |___/

void getMessage (void) {   

  CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
  if (rxId == 0x128) {
    headlightState = (rxBuf[0] & 0xFF);
    }
  if (rxId == 0x353) {
    outsideTemp = rxBuf[4];
    }
  if (rxId == 0x427) {
    coolantTemp = rxBuf[0];
    battCAN = rxBuf[3];
    }
  if (rxId == 0x44D) {
    absoluteAir = rxBuf[7];
    }
}

//                      _ ______ _             _       
//                     | |  ____| |           | |      
//   _ __ ___  __ _  __| | |__  | | ___   __ _| |_ ___ 
//  | '__/ _ \/ _` |/ _` |  __| | |/ _ \ / _` | __/ __|
//  | | |  __/ (_| | (_| | |    | | (_) | (_| | |_\__ \ /*/
//  |_|  \___|\__,_|\__,_|_|    |_|\___/ \__,_|\__|___/
//                                                
void readFloats() {

  if (millis() - dwinTimer >= 17) {
    floatVoltageValue = something;                //****************to work on  
    floatVoltageValue2 = something else etc.      //** */
    boostInt = (floatVoltageValue * 18);          //** */
    pressureInt = (floatVoltageValue * 12);       //** */
    
    //----------------sending data------------
    sendOilTempInt(potReading);         //value for oil temp gauge and digital display
    sendBoostFloat(floatVoltageValue);  //vaulue for boost digital display
    sendBoostInt(boostInt);             //value for boost gauge
    sendOilPresFloat(floatVoltageValue);//value for oil pressure digital dispay
    sendOilPresInt(pressureInt);        //value for oil pressure gauge
    //----------------------------------------
    
    dwinTimer = millis();
  }
}

//                      _ _____       _       
//                     | |_   _|     | |      
//   ___  ___ _ __   __| | | |  _ __ | |_ ___ 
//  / __|/ _ \ '_ \ / _` | | | | '_ \| __/ __|
//  \__ \  __/ | | | (_| |_| |_| | | | |_\__ \ /*/
//  |___/\___|_| |_|\__,_|_____|_| |_|\__|___/
//                                                                                
void sendOilTempInt (int numberToSend) {
  
  dwinSerial.write(0x5A);                   //Header
  dwinSerial.write(0xA5);                   //Header
  dwinSerial.write(2+1+2);                  //Length: VP address + Write command + number low and high bytes
  dwinSerial.write(0x82);                   //Write command
  dwinSerial.write(0x10);                   //Write address to VP address 1000
  dwinSerial.write((byte)0x00);             //Write address
  dwinSerial.write(highByte(numberToSend)); //integer high byte
  dwinSerial.write(lowByte(numberToSend));  //integer low byte
}

void sendBoostInt (int boostNumberToSend) {
  
  dwinSerial.write(0x5A);                   //Header
  dwinSerial.write(0xA5);                   //Header
  dwinSerial.write(2+1+2);                  //Length: VP address + Write command + number low and high bytes
  dwinSerial.write(0x82);                   //Write command
  dwinSerial.write(0x12);                   //Write address to VP address 1000
  dwinSerial.write((byte)0x00);             //Write address
  dwinSerial.write(highByte(boostNumberToSend)); //integer high byte
  dwinSerial.write(lowByte(boostNumberToSend));  //integer low byte
  Serial.println(boostNumberToSend);
}

void sendOilPresInt (int presNumberToSend) {
  
  dwinSerial.write(0x5A);                   //Header
  dwinSerial.write(0xA5);                   //Header
  dwinSerial.write(2+1+2);                  //Length: VP address + Write command + number low and high bytes
  dwinSerial.write(0x82);                   //Write command
  dwinSerial.write(0x11);                   //Write address to VP address 1000
  dwinSerial.write((byte)0x75);             //Write address
  dwinSerial.write(highByte(presNumberToSend)); //integer high byte
  dwinSerial.write(lowByte(presNumberToSend));  //integer low byte
}

//                      _ ______ _             _       
//                     | |  ____| |           | |      
//   ___  ___ _ __   __| | |__  | | ___   __ _| |_ ___ 
//  / __|/ _ \ '_ \ / _` |  __| | |/ _ \ / _` | __/ __|
//  \__ \  __/ | | | (_| | |    | | (_) | (_| | |_\__ \ /*/
//  |___/\___|_| |_|\__,_|_|    |_|\___/ \__,_|\__|___/
//                                                    
void FloatToHex (float f, byte* hex) {
  byte* f_byte = reinterpret_cast<byte*>(&f); //the value of f_byte is pointer to f
  memcpy(hex, f_byte, 4);                     //hex: destination, f_byte: source, 4:number of bytes to copy (4 bytes = 32 bit (float))
}

void sendBoostFloat (float floatValue) {
  
  dwinSerial.write(0x5A);                   //Header
  dwinSerial.write(0xA5);                   //Header
  dwinSerial.write(0x07);                  //Length: VP address + Write command + Length of the float (4 bytes)
  dwinSerial.write(0x82);                   //Write command
  dwinSerial.write(0x11);                   //Write address to VP address 1100
  dwinSerial.write((byte)0x00);             //Write address
  //--

  byte hex[4] = {0}; //create a hex array for the 4 bytes

  //Serial.println(floatValue);
  FloatToHex(floatValue, hex); //Copnvert the float to the hex array
  dwinSerial.write(hex[3]); //The order is flipped (endiannes)
  dwinSerial.write(hex[2]);
  dwinSerial.write(hex[1]);
  dwinSerial.write(hex[0]);
}

void sendOilPresFloat (float presFloatValue) {
  
  dwinSerial.write(0x5A);                   //Header
  dwinSerial.write(0xA5);                   //Header
  dwinSerial.write(0x07);                  //Length: VP address + Write command + Length of the float (4 bytes)
  dwinSerial.write(0x82);                   //Write command
  dwinSerial.write(0x11);                   //Write address to VP address 1100
  dwinSerial.write((byte)0x50);             //Write address
  //--

  byte hex[4] = {0}; //create a hex array for the 4 bytes

  //Serial.println(presFloatValue);
  FloatToHex(presFloatValue, hex); //Copnvert the float to the hex array
  dwinSerial.write(hex[3]); //The order is flipped (endiannes)
  dwinSerial.write(hex[2]);
  dwinSerial.write(hex[1]);
  dwinSerial.write(hex[0]);
}


//   _____ ______ _______ _    _ _____
//  / ____|  ____|__   __| |  | |  __ \ '/'
// | (___ | |__     | |  | |  | | |__) |
//  \___ \|  __|    | |  | |  | |  ___/
//  ____) | |____   | |  | |__| | |
// |_____/|______|  |_|   \____/|_|
//
void setup() {

  //Setup serial monitor
  Serial.begin(9600);   

  //setup SoftwareSerial                             
  dwinSerial.begin(9600);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  
  //setup the CAN bus module
  if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK) Serial.print("MCP2515 Init Okay!!\r\n");
  else Serial.print("MCP2515 Init Failed!!\r\n");
  CAN0.init_Mask(0,0,0x010F0000);                // Init first mask...
  CAN0.init_Filt(0,0,0x140);                // Init first filter...
  CAN0.init_Filt(1,0,0x360);                // Init second filter...
  
  CAN0.setMode(MCP_NORMAL);         // Change to normal mode to allow messages to be transmitted

  millis10 = millis();
  millis50 = millis();
  millis200 = millis();
}


//  _      ____   ____  _____
// | |    / __ \ / __ \|  __ \ '/'
// | |   | |  | | |  | | |__) |
// | |   | |  | | |  | |  ___/
// | |___| |__| | |__| | |
// |______\____/ \____/|_|
//
void loop() {
  
  //get CAN messages
  getMessage();


  //****Oil Temperature Stuff****
  uint8_t i;
  uint8_t b;                                          //integer for loop
  float average;                                      //decimal for average
  float battAverage;
  //Serial.println(analogRead(oilTempSensorPin));
  
  if (millis() - millis10 >= 10) {
    //Oil temperature sensor
    for (i=0; i<NUMSAMPLES; i++) {                      
      samples[i] = analogRead(oilTempSensorPin);        //takes samples at number defined with a short delay between samples
      }
    //Battery voltage
    for (b=0; b<NUMBATTSAMPLES; b++) {                      
      samples[b] = analogRead(battPin);        //takes samples at number defined with a short delay between samples
      }
    millis10 = millis();
  }
  
  if (millis() - millis200 >= 200) {
    //****Oil temp sensor****
    average = 0;
    for (i=0; i< NUMSAMPLES; i++) {
      average += samples[i];                            //adds all number of samples together - '+=' i.e. num1 += num2 means 'num1 is equal to num1 plus num2'
      }
    average /= NUMSAMPLES;  
    // Serial.print("Average Analog Oil Temp Reading = ");
    // Serial.println(average);                                        //analog value at analog pin into arduino
    average = (oilTempSensorDivider*average)/(1023-average);        //conversion equation to read resistance from voltage divider
    // Serial.print("Oil Temp Sensor Resistance = ");
    // Serial.println(average);
    float steinhart;                              //steinhart equation to estimate temperature value at any resistance from curve of thermistor sensor
    steinhart = log(average);                     //lnR
    steinhart = pow(steinhart,3);                 //(lnR)^3
    steinhart *= steinconstC;                     //C*((lnR)^3)
    steinhart += (steinconstB*(log(average)));    //B*(lnR) + C*((lnR)^3)
    steinhart += steinconstA;                     //Complete equation, 1/T=A+BlnR+C(lnR)^3
    steinhart = 1.0/steinhart;                    //Inverse to isolate for T
    steinhart -= 273.15;                          //Conversion from kelvin to celcius
    // Serial.print("Temperature = ");
    // Serial.print(steinhart);                      //prints final temp in celcius
    // Serial.println(" *C");
    //****Battery voltage****
    battAverage= 0;
    for (b=0; b< NUMBATTSAMPLES; b++) {
      average += samples[b];                            //adds all number of samples together - '+=' i.e. num1 += num2 means 'num1 is equal to num1 plus num2'
      }                       
    average /= NUMBATTSAMPLES;                              //divides by number of samples to output the average

    //CAN readings
    Serial.print("Headlight State: ");
    Serial.println(headlightState);
    Serial.print("Outside Temp: ");
    Serial.println(outsideTemp);
    Serial.print("Coolant Temp: ");
    Serial.println(coolantTemp);
    Serial.print("CAN Battery voltage: ");
    Serial.println(battCAN);
    Serial.print("Absolute MAP: ");
    Serial.println(absoluteAir);
   }
}





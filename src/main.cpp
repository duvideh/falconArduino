// ***Arduino MEGA 2560 R3 clone connected to DWIN 960x280 screen***
// MCP2515 to read CAN bus and display readings to DWIN screen
// Serial1 used to communicate with DWIN
// Widgets:
//    DWIN display through UART
//    MCP2515 CAN bus communcation through SPI
//    DWIN backlight control (hopefully triggered through CAN)
//    Oil Pressure sensor 0-5v analog
//    Oil Temperature Sensor 0-5v analog
//    Battery voltage analog through voltage divider - 12v socket probably - or, if successful through CAN
//    Wideband O2 sensor 0-5v analog (to add after install)
//CAN bus parameters: (hoepfully - will test and add later)
//    Boost (MAP)
//    Coolant Temp
//    Ambient Temp
//    Headlights status
//    Intake Air temperature
//    Battery voltage (maybe)
//    trans temp???
// 
// 
// 

#include <Arduino.h>
#include <math.h>
#include <mcp_can.h> //library: coryjfowler/mcp_can



//Oil temperature and pressure sensors
#define oilTempSensorDivider 5080                       //defines the resistor value that is in series in the voltage divider
#define oilTempSensorPin A1                             //defines the analog pin of the input voltage from the voltage divider
#define oilPresSensorPin A2                             //defines the analog pin of the input from oil pressure sensure 0-5v
int presReading;                                        //int for reading of oilPresSensorPin
#define NUMSAMPLES 5                                    //defines the number of samples to be taken for a smooth average
const float steinconstA = 0.0004133244513630420;        //steinhart equation constant A, determined from wikipedia equations
const float steinconstB = 0.0002939949352019070;        //steinhart equation constant B, determined from wikipedia equations
const float steinconstC = -0.0000002163718111393;       //steinhart equation constant C, determined from wikipedia equations
int samples[NUMSAMPLES];                                //variable to store number of samples to be taken for oil temperature reading
int samplesPres [NUMSAMPLES];                           //variable to store number of samples to be taken for oil pressure reading
float steinhart;                                        //variable for steinhart equation - final oil temperature reading
uint8_t i;
uint8_t b;                                          //integer for loop
float average;                                      //decimal for average
float battAverage;
float presAverage;                

// //CAN bus
// MCP_CAN CAN0(7); // bracketed number is number of CS pin
// int ex = 0;
// int why = 0;
// long unsigned int rxId;
// unsigned char len = 0;
// unsigned char rxBuf[8];
// int outsideTemp = 0;
// int coolantTemp = 0;
// int headlightState = 0;
// int battCAN = 0;
// int absoluteAir = 0;
// int MAPvalue = 0;

// //battery voltage 
// #define battPin A3
// #define NUMBATTSAMPLES 10
// float voltage = 0.0;
// int battSamples[NUMBATTSAMPLES];

//DWIN stuff
unsigned char incomingData[100];      //received chars
unsigned char textString[100];        //Chars to send
unsigned char receivedFloatArray[50]; //Array created form the useful part of the recevied char
float receivedFloat;                  //float converted from the above array using atof()
float floatVoltageValue;              //ADC analog value convertedc into voltage (stored as float)
float floatBoostValue = 0.0;          //remapped range from adc to 0-30
float floatBoostAdjusted = 0.0;       //parameter to invert values <=14.7 
float floatLambdaValue = 0.0;         //remapped range from adc to 0.75-1.5
int   boostInt = 0;                   //floatVoltageValue converted into 0-180 for gauge animation
int   lambdaInt = 0;                  //quadratic equation to make linear lambda reading into non-linear gauge reading (-240x^2 + 780x - 450)
int   pressureInt;                    //floatVoltageValue converted into 0-120 for gauge animation
int   digitsAfterDp = 0;              //number of digits after decimal point in float to send to screen
long  dwinTimer;                      //read and update screen values every x milliseconds only
int   backlight = 0;                  //remap of ADC to 0-40 for DWIN backlight

//millis
unsigned long millis17 = 0;
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

// void getMessage (void) {   

//   CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
//   if (rxId == 0x128) {
//     headlightState = rxBuf[0]; //(rxBuf[0] & 0x00); //need to see reading first - autoheadlightson is 0x01, headlightson is 0x02
//     }
//   if (rxId == 0x353) {
//     outsideTemp = rxBuf[4];
//     }
//   if (rxId == 0x427) {
//     coolantTemp = rxBuf[0]; //also at 0x44D[4] and [5]
//     battCAN = rxBuf[3];
//     }
//   if (rxId == 0x44D) {
//     absoluteAir = rxBuf[7];  //listed as ambient air pressure - could be outside?? - 'turboBoostPressure' listed at 0x425 but no further information
//     }
// }

//                      _ _____       _       
//                     | |_   _|     | |      
//   ___  ___ _ __   __| | | |  _ __ | |_ ___ 
//  / __|/ _ \ '_ \ / _` | | | | '_ \| __/ __|
//  \__ \  __/ | | | (_| |_| |_| | | | |_\__ \ /*/
//  |___/\___|_| |_|\__,_|_____|_| |_|\__|___/
//                                                                                
void sendOilTempInt (int numberToSend) {
  
  Serial1.write(0x5A);                   //Header
  Serial1.write(0xA5);                   //Header
  Serial1.write(2+1+2);                  //Length: VP address + Write command + number low and high bytes
  Serial1.write(0x82);                   //Write command
  Serial1.write(0x10);                   //Write address to VP address 1000
  Serial1.write((byte)0x00);             //Write address
  Serial1.write(highByte(numberToSend)); //integer high byte
  Serial1.write(lowByte(numberToSend));  //integer low byte
}

void sendOilPresInt (int presNumberToSend) {
  
  Serial1.write(0x5A);                   //Header
  Serial1.write(0xA5);                   //Header
  Serial1.write(2+1+2);                  //Length: VP address + Write command + number low and high bytes
  Serial1.write(0x82);                   //Write command
  Serial1.write(0x10);                   //Write address to VP address 1010
  Serial1.write((byte)0x10);             //Write address
  Serial1.write(highByte(presNumberToSend)); //integer high byte
  Serial1.write(lowByte(presNumberToSend));  //integer low byte
}

void sendBoostInt (int boostNumberToSend) {
  
  Serial1.write(0x5A);                   //Header
  Serial1.write(0xA5);                   //Header
  Serial1.write(2+1+2);                  //Length: VP address + Write command + number low and high bytes
  Serial1.write(0x82);                   //Write command
  Serial1.write(0x12);                   //Write address to VP address 1200
  Serial1.write((byte)0x00);             //Write address
  Serial1.write(highByte(boostNumberToSend)); //integer high byte
  Serial1.write(lowByte(boostNumberToSend));  //integer low byte
}

void sendLambdaInt (int presNumberToSend) {
  
  Serial1.write(0x5A);                   //Header
  Serial1.write(0xA5);                   //Header
  Serial1.write(2+1+2);                  //Length: VP address + Write command + number low and high bytes
  Serial1.write(0x82);                   //Write command 0x82 for RAM access
  Serial1.write(0x13);                   //Write address to VP address 1350
  Serial1.write((byte)0x50);             //Write address
  Serial1.write(highByte(presNumberToSend)); //integer high byte
  Serial1.write(lowByte(presNumberToSend));  //integer low byte
}

void sendCoolantInt (int numberToSend) {
  
  Serial1.write(0x5A);                   //Header
  Serial1.write(0xA5);                   //Header
  Serial1.write(2+1+2);                  //Length: VP address + Write command + number low and high bytes
  Serial1.write(0x82);                   //Write command 0x82 for RAM access
  Serial1.write(0x10);                   //Write address to VP address 1030
  Serial1.write((byte)0x30);             //
  Serial1.write(highByte(numberToSend)); //integer high byte
  Serial1.write(lowByte(numberToSend));  //integer low byte
}

void sendAmbientInt (int numberToSend) {
  
  Serial1.write(0x5A);                   //Header
  Serial1.write(0xA5);                   //Header
  Serial1.write(2+1+2);                  //Length: VP address + Write command + number low and high bytes
  Serial1.write(0x82);                   //Write command 0x82 for RAM access
  Serial1.write(0x10);                   //Write address to VP address 1040
  Serial1.write((byte)0x40);             //
  Serial1.write(highByte(numberToSend)); //integer high byte
  Serial1.write(lowByte(numberToSend));  //integer low byte
}

void sendBacklight (int numberToSend) {
  
  Serial1.write(0x5A);                   //Header
  Serial1.write(0xA5);                   //Header
  Serial1.write(2+1+1);                  //Length: address + Write command + brightness level
  Serial1.write(0x82);                   //Write command 0x82 for RAM access
  Serial1.write((byte)0x00);             //Write address to address 0082
  Serial1.write((byte)0x82);             //
  Serial1.write(numberToSend);
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
  
  Serial1.write(0x5A);                   //Header
  Serial1.write(0xA5);                   //Header
  Serial1.write(0x07);                  //Length: VP address + Write command + Length of the float (4 bytes)
  Serial1.write(0x82);                   //Write command
  Serial1.write(0x13);                   //Write address to VP address 1300
  Serial1.write((byte)0x00);             //Write address
  //--

  byte hex[4] = {0}; //create a hex array for the 4 bytes

  //Serial.println(floatValue);
  FloatToHex(floatValue, hex); //Copnvert the float to the hex array
  Serial1.write(hex[3]); //The order is flipped (endianess)
  Serial1.write(hex[2]);
  Serial1.write(hex[1]);
  Serial1.write(hex[0]);
}


void sendVoltageFloat (float presFloatValue) {
  
  Serial1.write(0x5A);                   //Header
  Serial1.write(0xA5);                   //Header
  Serial1.write(0x07);                   //Length: VP address + Write command + Length of the float (4 bytes)
  Serial1.write(0x82);                   //Write command 0x82 for RAM access
  Serial1.write(0x10);                   //Write address to VP address 1020
  Serial1.write((byte)0x20);             //Write address
  //--

  byte hex[4] = {0}; //create a hex array for the 4 bytes

  //Serial.println(presFloatValue);
  FloatToHex(presFloatValue, hex); //Copnvert the float to the hex array
  Serial1.write(hex[3]); //The order is flipped (endiannes)
  Serial1.write(hex[2]);
  Serial1.write(hex[1]);
  Serial1.write(hex[0]);
}

void sendLambdaFloat (float floatValue) {
  
  Serial1.write(0x5A);                   //Header
  Serial1.write(0xA5);                   //Header
  Serial1.write(0x07);                   //Length: VP address + Write command + Length of the float (4 bytes)
  Serial1.write(0x82);                   //Write command 0x82 for RAM access
  Serial1.write(0x14);                   //Write address to VP address 1400
  Serial1.write((byte)0x00);             //Write address
  //--

  byte hex[4] = {0}; //create a hex array for the 4 bytes

  //Serial.println(floatValue);
  FloatToHex(floatValue, hex); //Copnvert the float to the hex array
  Serial1.write(hex[3]); //The order is flipped (endiannes)
  Serial1.write(hex[2]);
  Serial1.write(hex[1]);
  Serial1.write(hex[0]);
}

//                      _ ___  _______          _______ _   _ 
//                     | |__ \|  __ \ \        / /_   _| \ | |
//   ___  ___ _ __   __| |  ) | |  | \ \  /\  / /  | | |  \| |
//  / __|/ _ \ '_ \ / _` | / /| |  | |\ \/  \/ /   | | | . ` |
//  \__ \  __/ | | | (_| |/ /_| |__| | \  /\  /   _| |_| |\  |
//  |___/\___|_| |_|\__,_|____|_____/   \/  \/   |_____|_| \_|                                                                                   
void send2DWIN() {

  if (millis() - millis17 >= 17) {
    //floatVoltageValue = something;                //****************to work on  
    //floatVoltageValue2 = something else etc.      //** */
    boostInt = (floatVoltageValue * 18);          //** */
    pressureInt = (floatVoltageValue * 12);       //** */
    
    //----------------sending data------------
    //Oil temp + pres. analogs
    sendOilTempInt(steinhart);           //value for oil temp gauge and digital display
    sendOilPresInt(presReading);          //value for oil pressure gauge
    //CAN readings
    //sendCoolantInt(potReading);           //value for coolant temp digital display
    //sendAmbientInt(potReading);           //value for ambient temp digital display        
    //sendBoostFloat(floatBoostAdjusted);   //value for boost digital display
    //sendLambdaFloat(floatLambdaValue);    //value for lambda digital display
    //sendBoostInt(potReading);             //value for boost gauge
    //sendLambdaInt(lambdaInt);             //value for lambda gauge
    //sendVoltageFloat(floatVoltageValue);  //value for battery voltage digital display
    sendBacklight(backlight);             //value for DWIN LED backlight
    //----------------------------------------
    
    dwinTimer = millis();
  }
}


//   _____ ______ _______ _    _ _____
//  / ____|  ____|__   __| |  | |  __ \ '/'
// | (___ | |__     | |  | |  | | |__) |
//  \___ \|  __|    | |  | |  | |  ___/
//  ____) | |____   | |  | |__| | | 
// |_____/|______|  |_|   \____/|_|
//
void setup() {

  //setup Serials
  Serial.begin(9600);
  Serial1.begin(115200);       //need to make sure DWIN CFG set to 115200 baud rate                   

  
  // //setup the CAN bus module
  // if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK) Serial.print("MCP2515 Init Okay!!\r\n");
  // else Serial.print("MCP2515 Init Failed!!\r\n");
  // CAN0.init_Mask(0,0,0x010F0000);                // Init first mask...
  // CAN0.init_Filt(0,0,0x140);                // Init first filter...
  // CAN0.init_Filt(1,0,0x360);                // Init second filter...
  
  // CAN0.setMode(MCP_LISTENONLY);         // set to listen only mode

  millis17 = millis();
  millis50 = millis();
  millis200 = millis();
  
  delay(250);
}


//  _      ____   ____  _____
// | |    / __ \ / __ \|  __ \ '/'
// | |   | |  | | |  | | |__) |
// | |   | |  | | |  | |  ___/
// | |___| |__| | |__| | |
// |______\____/ \____/|_|
//
void loop() {
  
  // get CAN messages
  // getMessage();
  // Serial.print("headlightState = ");
  // Serial.println(headlightState);
  // Serial.print("outsideTemp = ");
  // Serial.println(outsideTemp);
  // Serial.print("coolantTemp = ");
  // Serial.println(coolantTemp);
  // Serial.print("battCAN = ");
  // Serial.println(battCAN);
  // Serial.print("absoluteAir = ");
  // Serial.println(absoluteAir);



  //****Oil Temperature Stuff****

  //Serial.println(analogRead(oilTempSensorPin));
  
  // if (millis() - millis200 >= 17) {
    
  //   millis17 = millis();
  // }


  if (millis() - millis200 >= 200) {
    //Oil temperature sensor
    for (i=0; i<NUMSAMPLES; i++) {                      
      samples[i] = analogRead(oilTempSensorPin);        //takes samples at number defined with a short delay between samples
      }
    average = 0;
    for (i=0; i< NUMSAMPLES; i++) {
      average += samples[i];                            //adds all number of samples together - '+=' i.e. num1 += num2 means 'num1 is equal to num1 plus num2'
      }
    average /= NUMSAMPLES;  
    // Serial.print("Average Analog Oil Temp Reading = ");
    // Serial.println(average);                                        //analog value at analog pin into arduino
    average = (oilTempSensorDivider*average)/(1023-average);           //conversion equation to read resistance from voltage divider
    // Serial.print("Oil Temp Sensor Resistance = ");
    // Serial.println(average);
    //steinhart equation to estimate temperature value at any resistance from curve of thermistor sensor
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

    //Oil pressure sensor
    for (i=0; i<NUMSAMPLES; i++) {                      
      samples[i] =  analogRead(oilPresSensorPin);        //takes samples at number defined with a short delay between samples
      }
    presAverage = 0;
    for (i=0; i< NUMSAMPLES; i++) {
      presAverage += samples[i];                            //adds all number of samples together - '+=' i.e. num1 += num2 means 'num1 is equal to num1 plus num2'
      }
    presAverage /= NUMSAMPLES;
    presReading = presAverage*0.14662756598240469208211143695015; //conversion from 0-1023 to 0-150

    // //Battery voltage
    // for (b=0; b<NUMBATTSAMPLES; b++) {                      
    //   samples[b] = analogRead(battPin);           //takes samples at number defined with a short delay between samples
    //   }
    // battAverage= 0;
    // for (b=0; b< NUMBATTSAMPLES; b++) {
    //   average += samples[b];                      //adds all number of samples together - '+=' i.e. num1 += num2 means 'num1 is equal to num1 plus num2'
    //   }                       
    // average /= NUMBATTSAMPLES;                    //divides by number of samples to output the average

  //   //CAN readings to Serial
  //   // Serial.print("Headlight State: ");
  //   // Serial.println(headlightState);
  //   // Serial.print("Outside Temp: ");
  //   // Serial.println(outsideTemp);
  //   // Serial.print("Coolant Temp: ");
  //   // Serial.println(coolantTemp);
  //   // Serial.print("CAN Battery voltage: ");
  //   // Serial.println(battCAN);
  //   // Serial.print("Absolute MAP: ");
  //   // Serial.println(absoluteAir);
    millis200 = millis();
  }
}





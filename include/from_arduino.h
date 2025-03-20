//*** test sketch get output from DWIN display serial, convert to readable text ***

#include <SoftwareSerial.h>

//software serial
const byte rxPin = 9;
const byte txPin = 8;
SoftwareSerial dwinSerial (rxPin, txPin);

//receiving data from display - to test
unsigned char incomingData[100];      //received chars
int receivedInt = 0;
unsigned char receivedFloatArray[50]; //Array created form the useful part of the recevied char
float receivedFloat;                  //float converted from the above array using atof()
float test = 10.00;
byte incomingByte;
byte previousByte;

long millis50 = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  dwinSerial.begin(9600);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  Serial.println("Starting up...");
  delay(500);
  Serial.println("Commenced.");
}

void loop() {
  // put your main code here, to run repeatedly:
//getInt();
getFloat();
sendBoostFloat(test);
//fetchText();
// if (dwinSerial.available() > 0) {
//   // while (dwinSerial.available() > 0) {
//   //   int i = dwinSerial.read();
//   //   Serial.print(i,HEX);
//   //   Serial.println(" ");
//       int i = 0;
//       while (dwinSerial.available() > 0) { //empty the buffer into our array 
//       int incomingByte = dwinSerial.read(); //read 1 character from the dwin display
//       delay(2);
//       incomingData[i] = incomingByte; //put the character in the array at position i, repeat until buffer depleted
//       i++;
//       Serial.print(incomingByte, HEX); //print the most recently read byte for checking
//       Serial.print(" ");
//      }
//   }
delay(100);
}



void fetchText() {

  if (dwinSerial.available() > 0) {   //available() returns the number of bytes stored in the serial buffer

    //Serial.println("Message incoming:");

    int i = 0; //initialise position in the buffer array

    while (dwinSerial.available() > 0) { //empty the buffer into our array 
      int incomingByte = dwinSerial.read(); //read 1 character from the dwin display
      delay(2);
      incomingData[i] = incomingByte; //put the character in the array at position i, repeat until buffer depleted
      i++;
      //Serial.print(incomingByte, HEX); //print the most recently read byte for checking
      //Serial.print(" ");
      }
    //Serial.println(" ");
    //now our array is completed, we can dissect it
    //if (incomingData[3] == (byte) 0x83) { //if VP read instruction (always 0x83, always fourth byte) is in the returned char array
                                         //DWIN will send 5A A5 (header) xx (length) 83 (VP read) xx xx xx (VP address) xx xx xx xx etc. (data to be read)
      int k = 8; //starting byte for the message - single
      //int j = 0; //string identifier, start at beggining
      //while (incomingData[k] != 0xFF) { //end-message byte is 0xFFFF
        Serial.print(incomingData[k]); Serial.println(" "); //print the message
        //receivedText[j] = incomingData[k]; // convert message into a string
        //k++;
        //j++;
        //}
    //}
    //Serial.write(receivedText,strlen(receivedText));
    ///finally, we empty the whole array to avoid garbage - if message is shorter than the preceding message, previous bytes will remain in the array
    memset(incomingData,0,sizeof(incomingData));
  }
}

void getInt() {

  if (dwinSerial.available() > 0) {   //available() returns the number of bytes stored in the serial buffer
    int i = 0; //initialise position in the buffer array
    int k = 8; //set byte position to be read
    while (dwinSerial.available() > 0) { //perform actions while there is data in the serial buffer, end actions when there is nothing left
      int incomingByte = dwinSerial.read(); //read 1 character from the dwin display
      delay(2);
      incomingData[i] = incomingByte; //put the character in the array at position i, repeat until buffer depleted
      i++;
      Serial.print(incomingByte, HEX); //print the most recently read byte for checking
      Serial.print(" ");
      }
    Serial.println(" ");
    receivedInt = incomingData[k];
    //Serial.println(receivedInt);
    ///finally, we empty the whole array to avoid garbage - if message is shorter than the preceding message, previous bytes will remain in the array
    memset(incomingData,0,sizeof(incomingData));
  }
}
void getFloat() {

  if (dwinSerial.available() > 0) {   //available() returns the number of bytes stored in the serial buffer
    int i = 0; //initialise position in the buffer array
    // int j = 8; //byte 1
    // int k = 9; //byte 2
    while (dwinSerial.available() > 0) { //perform actions while there is data in the serial buffer, end actions when there is nothing left
      incomingByte = dwinSerial.read(); //read 1 character from the dwin display
      if (incomingByte == 165 && previousByte == 90) {
        i = 1;
      }
      delay(2);
      incomingData[i] = incomingByte; //put the character in the array at position i, repeat until buffer depleted
      i++;
      if (incomingByte < 0x10) {  //add leading 0 to values < 0F
        Serial.print("0");
      }
      Serial.print(incomingByte, HEX); //print the most recently read byte for checking
      Serial.print(" ");
      //receivedFloat = dwinSerial.parseFloat();
      previousByte = incomingByte;
      }
    Serial.println(" ");
    if (incomingData[5] == 32 ) {
      Serial.print("check");
      Serial.println(" ");
      if (incomingData[8] == 1) {
        test += 0.05;
      }
      else {
        test -= 0.05;
      }
    }
    if (incomingData[5] == 48) {
      Serial.print("SAVE");
      Serial.println(" ");
    }
    //receivedFloat = (incomingData[j]+incomingData[k]);
    //Serial.println(receivedFloat);
    ///finally, we empty the whole array to avoid garbage - if message is shorter than the preceding message, previous bytes will remain in the array
    memset(incomingData,0,sizeof(incomingData));
  }
}

void FloatToHex (float f, byte* hex) {
  byte* f_byte = reinterpret_cast<byte*>(&f); //the value of f_byte is pointer to f
  memcpy(hex, f_byte, 4);                     //hex: destination, f_byte: source, 4:number of bytes to copy (4 bytes = 32 bit (float))
}

void sendBoostFloat (float floatValue) {
  
  dwinSerial.write(0x5A);                   //Header
  dwinSerial.write(0xA5);                   //Header
  dwinSerial.write(0x07);                   //Length: VP address + Write command + Length of the float (4 bytes)
  dwinSerial.write(0x82);                   //Write command 0x82 for RAM access
  dwinSerial.write(0x10);                   //Write address to VP address 1300
  dwinSerial.write((byte)0x10);             //Write address
  //--

  byte hex[4] = {0}; //create a hex array for the 4 bytes

  //Serial.println(floatValue);
  FloatToHex(floatValue, hex); //Copnvert the float to the hex array
  dwinSerial.write(hex[3]); //The order is flipped (endianess)
  dwinSerial.write(hex[2]);
  dwinSerial.write(hex[1]);
  dwinSerial.write(hex[0]);
}

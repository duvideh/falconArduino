/*****OIL TEMPERATURE SENSOR****/

#include <Arduino.h>

#define oilTempSensorDivider 2970   //defines the resistor value that is in series in the voltage divider
#define oilTempSensorPin A0         //defines the analog pin of the input voltage from the voltage divider
#define NUMSAMPLES 5                //defines the number of samples to be taken for a smooth average


const float steinconstA = 0.0004133244513630420;        //steinhart equation constant A, determined from wikipedia equations
const float steinconstB = 0.0002939949352019070;       //steinhart equation constant B, determined from wikipedia equations
const float steinconstC = -0.0000002163718111393;    //steinhart equation constant C, determined from wikipedia equations

int samples[NUMSAMPLES];                              //variable to store number of samples to be taken

uint8_t i;                                          //integer for loop
float average;                                      //decimal for average

unsigned long millis10 = 0;

void setup() {
  Serial.begin(9600);                                 //start serial monitor
}

void loop() {
  if (millis10 >= millis() +10) {
    for (i=0; i<NUMSAMPLES; i++) {                      
      samples[i] = analogRead(oilTempSensorPin);        //takes samples at number defined with a short delay between samples
      }
    millis10 = millis();
  }
  
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
    average += samples[i];                            //adds all number of samples together - '+=' i.e. num1 += num2 means 'num1 is equal to num1 plus num2'
  }
  average /= NUMSAMPLES;                              //divides by number of samples to output the average

  Serial.print("Average Analog Oil Temp Reading = ");
  Serial.println(average);                                        //analog value at analog pin into arduino
  average = (oilTempSensorDivider*average)/(1023-average);        //conversion equation to read resistance from voltage divider
  Serial.print("Oil Temp Sensor Resistance = ");
  Serial.println(average);

  float steinhart;                              //steinhart equation to estimate temperature value at any resistance from curve of thermistor sensor
  steinhart = log(average);                     //lnR
  steinhart = pow(steinhart,3);                 //(lnR)^3
  steinhart *= steinconstC;                     //C*((lnR)^3)
  steinhart += (steinconstB*(log(average)));    //B*(lnR) + C*((lnR)^3)
  steinhart += steinconstA;                     //Complete equation, 1/T=A+BlnR+C(lnR)^3
  steinhart = 1.0/steinhart;                    //Inverse to isolate for T
  steinhart -= 273.15;                          //Conversion from kelvin to celcius

  Serial.print("Temperature = ");
  Serial.print(steinhart);                      //prints final temp in celcius
  Serial.println(" *C");
  
  delay(1000);                                  //delay between readings
}
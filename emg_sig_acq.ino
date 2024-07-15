/* EMG signal reading and processing utilizing microcontrollers - by Pedro Lucca
 *
 * The following code is used to process and output wirelessly (via Bluetooth)
 * EMG signals measured by external electrodes attached to the user's arm. The
 * microcontroller used in this project is an ESP32 (Espressif) and the code was
 * developed in the Arduino IDE (and therefore uses functions from the Arduino
 * standard library).
 *
 * Only one electrode is being considered in this version of the code - utilizing
 * more electrodes requires the reevaluation of the sampling time and further
 * adjustments to the code logic (setting up more input pins, reading and
 * processing each channel).
*/

#include "BluetoothSerial.h"

//Sampling period (in microseconds):
/*
Calculated based off of the EMG signal frequency distribution and
Nyquist-Shannon sampling theorem applied to multiple signal sources
*/
const int Ts = 200;

//Cutoff frequency of EWMA filter:
float fc = 500;

//Number of samples used in the filter's window:
float Nm = floor((1/(Ts*0.000001))/(2*fc));
float invNm = 1/Nm;

float mmex = 0;

//Electrode DC voltage read value:
/*
Equal to the voltage read by the microcontroller's ADC when no movement
is being made. Utilized to rectify the alternating EMG signal.
*/
float dc = 0;

BluetoothSerial SerialBT;

//Rectfying function:
/*
Removes the DC component and outputs the absolute value of the alternating
signal.
*/
float retf(float meas){
  return abs(meas - dc);
}

void setup(){
  //Input pins (one for each electrode):
  pinMode(0, INPUT);

  //Output pins (to generate the reference voltage applied to the user's body):
  pinMode(25, OUTPUT);
  dacWrite(25, 128);

  Serial.begin(1000000);

  //Name given to the microcontroller's Bluetooth ID:
  SerialBT.begin("esp-protese");
}

void loop(){
  //Time at the start of the loop cycle (in microseconds):
  int tempo_atual = micros();

  //Applies the rectifying function to the electrode measurement:
  float val_atual = retf(analogRead(0));

  //Applies the EWMA low pass filter:
  mmex = (val_atual*(invNm))+(mmex*(1-invNm));

  //Outputs the filtered and unfiltered data through Bluetooth
  SerialBT.print(val_atual);
  SerialBT.print(",");
  SerialBT.println(mmex);

  //Waits until the loop time is greater than the sampling time to restart the loop
  while(micros() - tempo_atual < Ts){
  }
}

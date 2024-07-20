/* EMG signal reading and processing utilizing microcontrollers - by Pedro Lucca
 *
 * The following code is used to process and output wirelessly (via Bluetooth)
 * EMG signals measured by external electrodes attached to the user's arm. The
 * microcontroller used in this project is an ESP32 (Espressif) and the code was
 * developed through the Arduino IDE (and therefore uses functions from the
 * Arduino standard library).
 *
*/

#include "BluetoothSerial.h"

//Sampling period (in microseconds):
/*
Calculated based off of the EMG signal frequency distribution and
Nyquist-Shannon sampling theorem applied to multiple signal sources
*/
const unsigned long Ts = 125;

//Cutoff frequency of EWMA filter:
float fc = 500;

//Number of samples used in the filter's window:
float Nm = floor((1/(Ts*0.000001))/(2*fc));
float invNm = 1/Nm;

//Electrode DC voltage read value:
/*
Equal to the voltage read by the microcontroller's ADC when no movement
is being made. Utilized to rectify the alternating EMG signal.
*/
float dc = 0;

//Rectfying function:
/*
Removes the DC component and outputs the absolute value of the alternating
signal.
*/
float retf(float meas){
  return abs(meas - dc);
}

//Struct vector (stores each channel's information and data) with lenght
//equal to the number of electrodes

typedef struct{
	int pinNUMBER;
	float readVALUE;
	float ewma = 0.0;
} boards;
boards electrode[6];

//Inital pin configuration function
void initPINS(){
  electrode[0].pinNUMBER = 32;
  electrode[1].pinNUMBER = 33;
  electrode[2].pinNUMBER = 34;
  electrode[3].pinNUMBER = 35;
  electrode[4].pinNUMBER = 36;
  electrode[5].pinNUMBER = 39;
  for(int i = 0; i < 6; i++){
    pinMode(electrode[i].pinNUMBER, INPUT);
  }

  pinMode(25, OUTPUT);

  //Outputs the reference voltage through pin 25's DAC
  dacWrite(25, 128);
}

BluetoothSerial SerialBT;

//Signal acquisition, processing and transmission function
void readTX(){
  for(int i = 0; i < 6; i++){
    //Reads each channel's electrode
    electrode[i].readVALUE = float(analogRead(electrode[i].pinNUMBER));
    
    //Applies the rectifying function to the collected value
    electrode[i].readVALUE = retf(electrode[i].readVALUE);

    //Applies the EWMA low-pass filter to the rectified value
    electrode[i].ewma = (electrode[i].readVALUE * (invNm)) + (electrode[i].ewma * (1-invNm))]
    SerialBT.println(electrode[i].ewma);
  }
}

void setup(){
  initPINS();

  Serial.begin(115200);

  //Name given to the microcontroller's Bluetooth ID:
  SerialBT.begin("espPROTESE");
}

void loop(){
  //Stores the time at the start of the loop cycle (in microseconds):
  unsigned long tempo_atual = micros();

  readTX();

  //Waits until the loop time reaches the sampling period to restart the loop
  while(micros() - tempo_atual < Ts){
  }
}

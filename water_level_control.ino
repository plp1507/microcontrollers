/*Water level control using the HC-SR04 sensor and the Arduino UNO microcontroller - by Pedro Lucca
 *
 *This project had the goal of implementing a PID controller to control the water level in a reservoir
 *through a water pump, utilizing the HC-SR04 ultrasonic distance sensor. The following code consists
 *of the necessary microcontroller actions for the ultrasonic sensor, aswell as the code for the PID
 *controller and the necessary adjustments required by the actuator and its driving circuit.
 *
 *The controller is capable of eliminating any steady state error presented by the system, maintaining
 *its stability even in the occurrence of disturbances.
*/

//Sampling period (in milliseconds):
#define Ts 25

//EWMA window size:
#define NM 20

//Digital pins used:
#define pwmPIN 5
#define echoPIN 11
#define triggerPIN 7

//Trigger pulse duration (in microseconds):
#define triggerPULSE_DURATION 12

//Initial reference applied to the feedback system (in centimetres):
float ref = 5;

//Error variables:
/*
i_erro refers to the discrete integral (sum) of the error signal
d_erro refers to the discrete derivative of the error signal

The minimum height capable of being reproduced by the system is around
5cm. That, combined with the system's low rising time, blows up the
integral portion of the PID controller and pushes the system to instability.
i_erro starts at -17 to counteract the effects brought up by these factors.
*/
float erro = 0;
float erro_a = 0;
float i_erro = -17;
float d_erro = 0;

//PID controller multipliers:
float kp = 5.1652;
float ki = 250.8919;
float kd = 0.09;

int pwm = 0;
float invNm = 1/NM;

//Distance variables:
/*
d0 is equal to the distance measured between the height of the sensor (relative to the
bottom of the reservoir) and the lowest water level (5cm).
*/
float d0 = 24.7;
float previousDISTANCE = 0;
float filteredDISTANCE = 0;

//EWMA low pass filter:
float filter(float distance) {
  float filtered = (invNm)*distance + (1-invNm)*previousDISTANCE;
  previousDISTANCE = filtered;
  return filtered;
}

void setup() {
  //Setup of the input and output pins:
  pinMode(echoPIN, INPUT);
  pinMode(triggerPIN, OUTPUT);
  pinMode(pwmPIN, OUTPUT);

  //Serial communication speed:
  Serial.begin(115200);

  //Adjustments  to the digital controller:
  /*
  Because the PID controller is given by continuous functions of the error (integral and
  derivative), the implementation of the controller needs to adapt these functions to a
  discrete time space.
  The integral can be given by the sum of each value of error multiplied by the smallest
  amount of time between two points (which is equal to the sampling time). The discrete
  derivative is given by the difference between two adjacent points, divided by the
  smallest amount of time between the points (sampling time).
  With that, the adjustments shown below are justified - they are made with the intent of
  removing the necessity of multiplying/dividing the derivative and the integral each time
  one of them is computed.
  */
  ki = ki*(Ts*10e-3);
  kd = kd/(Ts*10e-3);
}

void loop() {
  //Stores the time at the start of the loop (in milliseconds):
  int tempoINICIAL = millis();

  //Reads any reference values transmitted through the serial port:
  if (Serial.available() > 0) {
    ref = Serial.parseFloat();
  }

  //Transmits the ultrasonic waves through the TRIGGER pin:
  digitalWrite(triggerPIN, HIGH);
  delayMicroseconds(triggerPULSE_DURATION);
  digitalWrite(triggerPIN, LOW);

  //Reads the duration of the pulse read in the ECHO pin:
  int pulseDURATION = pulseIn(ECHO_PIN, HIGH);
  float distance = pulseDURATION * 0.01715;

  //Calculates the water level through the distance read and the standard sensor distance:
  float waterLEVEL = d0 - distance;

  //Applies the EWMA filter and calculates the error signal:
  filteredDISTANCE = filter(distance);
  erro = ref - filteredDISTANCE;

  //Calculates the integral and derivative of the error:
  i_erro += erro;
  d_erro = erro - erro_a;

  //Refreshes the "previous error" variable (utilized in the derivative calculation):
  erro_a = erro;

  //PID controller:
  if (erro > 0){
    pwm = kp*erro + kd*d_erro + ki*i_erro;
  }
  else{
    /*
    Because the system behaved very differently depending on the sign of the error, the controller
    operates in two distinct modes in order to better adapt itself to the system and its inherent
    characteristics.
    */
    pwm = kp*erro + kd*d_erro - ki*i_erro;
  }

  //Limits the PWM signal to the 8-bit signal accepted by the Arduino UNO, writes the PWM value to its pin:
  pwm = constrain(pwm, 0, 255);
  analogWrite(pwmPIN, pwm);

  //Outputs the reference and the output of the system:
  Serial.print(ref);
  Serial.print(" ");
  Serial.println(filteredDISTANCE);

  //Waits until the loop execution time is bigger than the sampling time:
  while (millis() - tempoINICIAL < Ts) {}
}

#include <PID_v1.h>

#define M0_P 3
#define M0_D 2
#define M1_P 5
#define M1_D 4
#define M2_P 6
#define M2_D 7
#define M3_P 9
#define M3_D 8

#define NOQUADRATURE


//H-bridge pins
const int motorDirPin[4]  = { M0_D, M1_D, M2_D, M3_D };
const int motorPWMPin[4]  = { M0_P, M1_P, M2_P, M3_P };

//H-bridge state variables
int motorDir[4]     = { 0, 0, 0, 0 };
int motorPWM[4]     = { 0, 0, 0, 0 };


bool motorPIDenable = false;

//PID controllers
PID motorPID[4] {
  {&motorPosError[0], &motorOutput[0], &motorSetpoint[0], kp,ki,kd, DIRECT},
  {&motorPosError[1], &motorOutput[1], &motorSetpoint[1], kp,ki,kd, DIRECT},
  {&motorPosError[2], &motorOutput[2], &motorSetpoint[2], kp,ki,kd, DIRECT},
  {&motorPosError[3], &motorOutput[3], &motorSetpoint[3], kp,ki,kd, DIRECT}
};

//PID state varibles
double kp=1, ki=0, kd=0;
double motorSetpoint[4] = {0,0,0,0};    //not used, left as zero
double motorPosError[4] = {0,0,0,0};    //PID error input
double motorOutput[4] = {0,0,0,0};      //PID output

//pulse generator variables
const double maxSpeed = 2000.0;    //hz (encoder edges per second)
const double minSpeed = 0.01;    //hz
double motorSpeed[4] = {0,0,0,0};       //speed for pulse generators
unsigned long lastPulse[4] = {0,0,0,0};  //the last time the target incremented
long targetPosition[4] = {0,0,0,0};   //target for the PID

volatile long encoderPosition[4] = {0,0,0,0}; //current wheel angle (updated in interrupt)






void setup() {
  
  Serial.begin(57600);
  for ( int i = 0; i<4; i++ )
  {
    pinMode(motorDirPin[i], OUTPUT);
    pinMode(motorPWMPin[i], OUTPUT);
  }

  //set up the interrupt
  //attachInterrupt(PCINT1_vect, updateWheelSpeeds, CHANGE);
  PCICR |= 1<<PCIE1;  //enable pin change interrupts on port C
  PCMSK1 = 0x0F;  //only use adc pins 0-3 for port C interrupt

  PCICR |= 1<<PCIE0;  //enable pin change interrupts on port B
  PCMSK0 = 0x3C;  //only use pins 2-5 for port B interrupt

  sei();  //turn on interrupts

}


void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0)
  {
    int command = Serial.read();
    switch (command)
    {
      case 's':
        for ( int i = 0; i<4; i++ )
        {
          analogWrite(motorPWMPin[i], 0);
        }
        Serial.println("OK");
        break;
        
      case 'p':
        for ( int i = 0; i<4; i++ )
        {
          motorPWM[i] = Serial.parseInt();
        }
        Serial.println("OK");
        break;
      case 'd':
        for ( int i = 0; i<4; i++ )
        {
          motorDir[i] = Serial.parseInt();
        }
        Serial.println("OK");
        break;
      case 'g':
        for ( int i = 0; i<4; i++ )
        {
          digitalWrite(motorDirPin[i], motorDir[i]);
        }
        for ( int i = 0; i<4; i++ )
        {
          analogWrite(motorPWMPin[i], motorPWM[i]);
        }
        Serial.println("OK");
        break;
      case 'v':
        for ( int i = 0; i<4; i++ )
        {
          Serial.print("Motor ");
          Serial.print(i);
          Serial.print(", Dir: ");
          Serial.print(motorDir[i]);
          Serial.print(", PWM: ");
          Serial.println(motorPWM[i]);
        }
        break;
    }
  }


  //pulse generator
  for(int i=0; i<4; i++){
    theTime = micros();
    double frequency = (motorSpeed[i] > 0) ? motorSpeed[i] : -motorSpeed[i];  //set it positive
    if(!motorPIDenable) frequency = 0;  //don't deliberately build error
    frequency = (frequency > maxSpeed) ? maxSpeed : frequency;  //set a max speed
    unsigned long period = 1000000.0/ frequency;

    if(frequency < minSpeed){ //division for period won't deal well with small speeds.
      lastPulse[i] = theTime; //keep it ticking.
    }else{
      if(theTime - lastPulse[i] > period){
        lastPulse[i] += period;
        //(motorSpeed[i] > 0) ? targetPosition[i]++ : targetPosition[i]--;
#ifdef QUADRATURE
        (motorSpeed[i] > 0) ? targetPosition[i] += 2 : targetPosition[i] -= 2;  //don't limp with uneven edges
#else
        targetPosition[i] += 2;
#endif        
      }
    }
  }



  if(motorPIDenable){
    for(int i=0; i<4; i++){
      motorPosError[i] = targetPosition[i] - encoderPosition[i];  //these will overflow regularly, don't use the setpoint
      motorPID[i].Compute();
      setMotor(motorOutput[i], i);
    }
  }else{
    for(int i=0; i<4; i++){
      targetPosition[i] = encoderPosition[i];  //don't jump on next start
    }
  }


    
}

void setMotor(int speed, int motor){
  if(speed==0){
    motorPWM[motor] = 0;
    digitalWrite(motorPWMPin[motor], LOW);
  }else{
    if(speed>0){
      motorDir[motor]=0;
      motorPWM[motor] = min(255, speed);
    }else {
      motorDir[motor]=1;
      motorPWM[motor] = min(255, -speed);
    }
  analogWrite(motorPWMPin[motor], motorPWM[motor]);
  digitalWrite(motorDirPin[motor], motorDir[motor]);
  }

}

#ifdef QUADRATURE

//void updateWheelSpeeds(){
ISR(PCINT1_vect) {
  //prep on boot
  static uint8_t oldPCstate = 0;  //store the state interrupt-by-interrupt.

  //low latency tasks
  uint8_t newPCstate = PINC;  //fetch the Port C input register as soon as possible,
  
  //actual processing
  uint8_t freshEdges = (newPCstate ^ oldPCstate) & 0x0F; //which bits changed?
  
  uint8_t binState = ((newPCstate & 0x0A)>>1) ^ newPCstate;  //convert two quads to two binaries, A^B

  if((freshEdges & 0x03) == 0x03){
    //encoder glitch.
    //no action
  } else if((freshEdges & 0x03) != 0){
    if(((freshEdges ^ binState) & 0x01) == 0){
      encoderPosition[0]++;
    }else{
      encoderPosition[0]--;
    }
  }

  if((freshEdges & 0x0C) == 0x0C){
    //encoder glitch.
    //no action
  } else if((freshEdges & 0x0C) != 0){
    if(((freshEdges ^ binState) & 0x04) == 0){
      encoderPosition[1]++;
    }else{
      encoderPosition[1]--;
    }
  }

  oldPCstate = newPCstate; //remember the state of the 
}



ISR(PCINT0_vect) {
  //prep on boot
  static uint8_t oldPBstate = 0;  //store the state interrupt-by-interrupt.

  //low latency tasks
  uint8_t newPBstate = PINB;  //fetch the Port B input register as soon as possible,
  
  //actual processing
  uint8_t freshEdges = (newPBstate ^ oldPBstate) & 0x3C; //which bits changed?
  
  uint8_t binState = ((newPBstate & 0x28)>>1) ^ newPBstate;  //convert two quads to two binaries, A^B



  if((freshEdges & 0x0C) == 0x0C){
    //encoder glitch.
    //no action
  } else if((freshEdges & 0x0C) != 0){
    if(((freshEdges ^ binState) & 0x04) == 0){
      encoderPosition[2]++;
    }else{
      encoderPosition[2]--;
    }
  }

  if((freshEdges & 0x30) == 0x30){
    //encoder glitch.
    //no action
  } else if((freshEdges & 0x30) != 0){
    if(((freshEdges ^ binState) & 0x10) == 0){
      encoderPosition[3]++;
    }else{
      encoderPosition[3]--;
    }
  }


  oldPBstate = newPBstate; //remember the state of the 
}

#else

ISR(PCINT0_vect) {
  //no connections
}

ISR(PCINT1_vect) {
  //low latency tasks
  uint8_t newPCstate = PINC;  //fetch the Port C input register as soon as possible,
  
  uint8_t freshEdges = (newPCstate ^ oldPCstate) & 0x0F; //which bits changed?
  for(int i=0; i<4; i++) {
    if(freshEdges & (i<<1)) encoderPosition[i]++;
  }
  

}

#endif
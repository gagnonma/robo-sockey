#include <QTRSensors.h>
#include <Servo.h>

// Change the values below to suit your robot's motors, weight, wheel type, etc.
#define KP .08
#define KD 2
#define M1_DEFAULT_SPEED 210 //left motor
#define M2_DEFAULT_SPEED 150 //right motor
#define M1_MAX_SPEED 250
#define M2_MAX_SPEED 250
#define MIDDLE_SENSOR 1
#define NUM_SENSORS  3      // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN      // emitter is controlled by digital pin 2
#define DEBUG 0 // set to 1 if serial debug output needed
#define MOTOR1 6
#define MOTOR2 5
#define MOTOR1_a 7
#define MOTOR1_b 8
#define MOTOR2_a 4
#define MOTOR2_b 2
#define GATE 3
#define TURN_TIME_90 1200
#define TRIG_PIN 12
#define ECHO_PIN 13
int program = 0;

int timer;
Servo gate;
enum MODE{START, WALL, LINE, SCORE, FIND_GOAL};
enum MODE mode = START;
QTRSensorsRC qtrrc((unsigned char[]) {9,10,11} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];


void setup() {
  // put your setup code here, to run once:
  pinMode(MOTOR1,OUTPUT);
  pinMode(MOTOR1_a,OUTPUT);
  pinMode(MOTOR1_b,OUTPUT);
  pinMode(MOTOR2_a,OUTPUT);
  pinMode(MOTOR2_b,OUTPUT);
  pinMode(MOTOR2,OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  gate.attach(GATE);
  manual_calibration(); 
  set_motors(0,0);
 

}

int lastError = 0;
int  last_proportional = 0;
int integral = 0;


void loop() {
  // put your main code here, to run repeatedly:
  int rightMotorSpeed;
  int leftMotorSpeed;
  int motorSpeed;
  int error;
  int position;
  unsigned int sensors[3];
  
  switch(mode){
    case START:
      delay(2000);
      open_gate();
      digitalWrite(MOTOR1_a,HIGH);
      digitalWrite(MOTOR1_b,LOW);
      digitalWrite(MOTOR2_a,HIGH);
      digitalWrite(MOTOR2_b,LOW);
      analogWrite(MOTOR1,150);     // set motor speed
      analogWrite(MOTOR2,150); 
      
      if(get_distance() < 25.0){
        turn90();
        mode = WALL;
        halt();
      }
      break;
    case WALL:
      digitalWrite(MOTOR1_a,HIGH);
      digitalWrite(MOTOR1_b,LOW);
      digitalWrite(MOTOR2_a,HIGH);
      digitalWrite(MOTOR2_b,LOW);
      analogWrite(MOTOR1,150);     // set motor speed
      analogWrite(MOTOR2,150); 
      if(detect_ball() == true){
        collect_ball();
        mode = FIND_GOAL;
        break;
      }
       
      if(get_distance() < 25.0){
        turn90();
      }
      break;
    case FIND_GOAL:
      digitalWrite(MOTOR1_a,HIGH);
      digitalWrite(MOTOR1_b,LOW);
      digitalWrite(MOTOR2_a,HIGH);
      digitalWrite(MOTOR2_b,LOW);
      analogWrite(MOTOR1,150);     // set motor speed
      analogWrite(MOTOR2,150);
       
      if(get_distance() < 25.0){
        turn90();
      }
      if(on_line == true){
        mode = LINE;
      }
     
      break;
    case LINE:
      
      position = qtrrc.readLine(sensors);
      error = position - 1000;

      motorSpeed = KP * error + KD * (error - lastError);
      lastError = error;

      leftMotorSpeed = M1_DEFAULT_SPEED + motorSpeed;
      rightMotorSpeed = M2_DEFAULT_SPEED - motorSpeed;

      // set motor speeds using the two motor speed variables above
      set_motors(leftMotorSpeed, rightMotorSpeed);
      
      if(get_distance() < 25.0){
        score_goal();
        turn90();
        mode = WALL;
      }
      break;

  }
    

}

void manual_calibration() {

  int i;
      // turn on Arduino's LED to indicate we are in calibration mode
  for (i = 0; i < 150; i++)  // the calibration will take a few seconds
  {
    qtrrc.calibrate();
    delay(20);
  }
       // turn off Arduino's LED to indicate we are through with calibration
  
}

void set_motors(int motor1speed, int motor2speed)
{
  if (motor1speed > M1_MAX_SPEED ) {
    motor1speed = M1_MAX_SPEED; // limit top speed
  }
  if (motor2speed > M2_MAX_SPEED ){
    motor2speed = M2_MAX_SPEED; // limit top speed
  }
  if (motor1speed < 0){
    motor1speed = 0; // keep motor above 0
  }
  if (motor2speed < 0){
    motor2speed = 0;// keep motor speed above 0
  }
  analogWrite(MOTOR1,motor1speed);     // set motor speed
  analogWrite(MOTOR2,motor2speed);     // set motor speed
  digitalWrite(MOTOR1_a,HIGH);
  digitalWrite(MOTOR1_b,LOW);
  digitalWrite(MOTOR2_a,HIGH);
  digitalWrite(MOTOR2_b,LOW);
}

void turn90(){
      digitalWrite(MOTOR1_a,LOW);
      digitalWrite(MOTOR1_b,HIGH);
      digitalWrite(MOTOR2_a,HIGH);
      digitalWrite(MOTOR2_b,LOW);
      analogWrite(MOTOR1,150);     // set motor speed
      analogWrite(MOTOR2,0);
      delay(TURN_TIME_90);
}

void halt(){
      digitalWrite(MOTOR1_a,LOW);
      digitalWrite(MOTOR1_b,LOW);
      digitalWrite(MOTOR2_a,LOW);
      digitalWrite(MOTOR2_b,LOW);
      analogWrite(MOTOR1,0);     // set motor speed
      analogWrite(MOTOR2,0); 
}

boolean detect_ball(){
  if (ball_distance() < 5){
    return true;
  }
  return false;
}

boolean on_line(){
  int position;
  unsigned int sensors[3];
  position = qtrrc.readLine(sensors);
  if(position < 1300 && position > 700){
    return true;
  }
  else{
    return false;
  }
  
}

float get_distance(){
  float volts = analogRead(A1);
  float distance = (6787.0/(volts-3.0))-4.0;
  return distance;
}
int ball_distance(){
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  while ( digitalRead(ECHO_PIN) == 0 );

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1);
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed 
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  return cm;
}

void score_goal(){
  open_gate();
  analogWrite(MOTOR1,0);     // set motor speed
  analogWrite(MOTOR2,0);     // set motor speed
  digitalWrite(MOTOR1_a,LOW);
  digitalWrite(MOTOR1_b,LOW);
  digitalWrite(MOTOR2_a,LOW);
  digitalWrite(MOTOR2_b,LOW);
  delay(3000);
  
}

void collect_ball(){
  delay(3000);
  close_gate();
}

void open_gate(){
  gate.write(90);
}

void close_gate(){
  gate.write(0);
}


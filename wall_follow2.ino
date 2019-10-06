#define MOTOR1 6 
//left motor
#define MOTOR2 5
//right motor
#define MOTOR1_a 7
#define MOTOR1_b 8
#define MOTOR2_a 4
#define MOTOR2_b 2
#define TURN_TIME_90 1100
#define KP .9
#define KD .01
#define speedm1 140
#define speedm2 140

void setup() {
  // put your setup code here, to run once:
  pinMode(MOTOR1,OUTPUT);
  pinMode(MOTOR1_a,OUTPUT);
  pinMode(MOTOR1_b,OUTPUT);
  pinMode(MOTOR2_a,OUTPUT);
  pinMode(MOTOR2_b,OUTPUT);
  pinMode(MOTOR2,OUTPUT);
  Serial.begin(9600);
  

}

void loop() {
  // put your main code here, to run repeatedly:
      digitalWrite(MOTOR1_a,HIGH);
      digitalWrite(MOTOR1_b,LOW);
      digitalWrite(MOTOR2_a,HIGH);
      digitalWrite(MOTOR2_b,LOW);
      int rightMotorSpeed;
  int leftMotorSpeed;
  int motorSpeed;
  int error;
  float pos;
  int lastError = 0;
      

      pos = get_distancetop(); 
      error = pos - 10.0;

      motorSpeed = KP * error + KD * (error - lastError);
      lastError = error;

      leftMotorSpeed = speedm1 - motorSpeed;
      rightMotorSpeed = speedm2 + motorSpeed;

      // set motor speeds using the two motor speed variables above
      analogWrite(MOTOR1,leftMotorSpeed);     // set motor speed
      analogWrite(MOTOR2,rightMotorSpeed);
      Serial.print(error);
      Serial.print("\t ");
      Serial.print(rightMotorSpeed);
      Serial.print("\t ");
      Serial.print(leftMotorSpeed);
      Serial.print("\t ");
      Serial.println(motorSpeed);
      
      
      if(get_distance() < 9.0){
        turn90();
        halt();
        delay(2000);
      }
      delay(500);
     
}

void turn90(){
      digitalWrite(MOTOR1_a,LOW);
      digitalWrite(MOTOR1_b,HIGH);
      digitalWrite(MOTOR2_a,HIGH);
      digitalWrite(MOTOR2_b,LOW);
      analogWrite(MOTOR1,150);     // set motor speed
      analogWrite(MOTOR2,150);
      delay(TURN_TIME_90);
}

float get_distance(){
  float volts = analogRead(A1);
  float distance = (6787.0/(volts-3.0))-4.0;
  return distance;
}
void halt(){
      digitalWrite(MOTOR1_a,LOW);
      digitalWrite(MOTOR1_b,LOW);
      digitalWrite(MOTOR2_a,LOW);
      digitalWrite(MOTOR2_b,LOW);
      analogWrite(MOTOR1,0);     // set motor speed
      analogWrite(MOTOR2,0); 
}
float get_distance2(){
  float volts = analogRead(A2);
  float distance = (6787.0/(volts-3.0))-4.0;
  return distance;
}
float get_distancetop(){
  float volts = analogRead(A3);
  float distance = (6787.0/(volts-3.0))-4.0;
   Serial.print("Top distance");
  Serial.println(distance);
  return distance;
}

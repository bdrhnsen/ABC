#include <Wire.h>
#define trigPin 13
#define echoPin 12
String data;
const int Enable_B = 10;
const int Enable_A = 11;
const int inputA1 = 5;
const int inputA2 = 8;
const int inputB1 = 9;
const int inputB2 = 2;
const int required_distance=55;


int prev_error=0;
int total_error=0;
int angular_prev_error=0;
int angular_total_error=0;
int k=0;
int distance, angle;
unsigned long t;
void backward(int vel)
{
  digitalWrite(Enable_A, HIGH);
  digitalWrite(Enable_B, HIGH);
  analogWrite(Enable_A, vel);
  analogWrite(Enable_B, vel);
  digitalWrite(inputA1, HIGH);
  digitalWrite(inputA2, LOW);
  digitalWrite(inputB1, HIGH);
  digitalWrite(inputB2, LOW);
  Serial.println("going back");
  
}

void forward(int distance, int angle)
{

  int velocity=dist_controller(distance,required_distance);
  int angular_vel=angle_controller(angle);

  digitalWrite(Enable_A, HIGH);
  digitalWrite(Enable_B, HIGH);

  int q = velocity-abs(angular_vel);
  
  if(q>250)
  {
    q=255;
  }
  if((velocity)>250)
  {
    velocity=255;
  }
  if(q<0){
    q=0;
  }
  

  if(angular_vel>=0){
    analogWrite(Enable_B,velocity);
    analogWrite(Enable_A,q);
  }
  if(q < 25){
    q = 25;
  }
  if(velocity>225)
  {
    velocity=225;
  }
   if(angular_vel<0){
    analogWrite(Enable_A,velocity+30);
    analogWrite(Enable_B,q-25);
  }


  digitalWrite(inputB1, LOW);
  digitalWrite(inputB2, HIGH);
  digitalWrite(inputA1, LOW);
  digitalWrite(inputA2, HIGH);
  
}
int dist_controller(int distance,int required_distance)
{
  int Kp=7;
  int Kd=0;
  int Ki=0;
  int error=distance-required_distance;
  int velocity=error*Kp+total_error*Ki+(error-prev_error)*Kd;
  total_error=+error;
  
  prev_error=error;
//  Serial.print("Error: ");
//  Serial.println(error);
  return velocity;
}
int angle_controller(int angle){
  
  int Kp=30;
  int Kd=0;
  int Ki=0;
  int angular_error=angle;
  int angular_vel=angular_error*Kp+angular_total_error*Ki+(angular_error-angular_prev_error)*Kd;
  angular_total_error=+angular_error;
  angular_prev_error=angular_error;

  return angular_vel;
}
void Stop()
{
  digitalWrite(Enable_A, HIGH);
  digitalWrite(Enable_B, HIGH);

  digitalWrite(inputA1, LOW);
  digitalWrite(inputA2, LOW);
  digitalWrite(inputB1, LOW);
  digitalWrite(inputB2, LOW);
}
int MeasureDistForward(){
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  Serial.println(distance);
  return distance;
  
}
void Somet(){
  analogWrite(Enable_A, 130);
  analogWrite(Enable_B, 180);

    digitalWrite(inputB1, LOW);
  digitalWrite(inputB2, HIGH);
  //B sağ A sol

    digitalWrite(inputA1, LOW);
  digitalWrite(inputA2, HIGH);
}
void setup()
{
  Wire.begin();
  pinMode(Enable_A, OUTPUT);
  pinMode(Enable_B, OUTPUT);
  pinMode(inputA1, OUTPUT);
  pinMode(inputA2, OUTPUT);
  pinMode(inputB1, OUTPUT);
  pinMode(inputB2, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
 
  Serial.begin(9600); 
   
  // // Serial Communication is starting with 9600 of baudrate speed
    while (!Serial) { // Serial portun hazır olmasını bekle.
    ;
  }
}  

void loop()
{
  if(millis()-t>200){
  distance=required_distance;
  angle=0;   
  }
 
if(Serial.available() > 0 ){
    t=millis();
//read distance and angle
data = Serial.readStringUntil('\n');
int comma_pos=data.indexOf(',');
String distance_str= data.substring(0,comma_pos);
distance=distance_str.toInt();
String angle_str=data.substring(comma_pos+1,data.length());
angle = angle_str.toInt();
  //angle şekil soldaysa artı sağdaysa eksi
}
int forwardD=MeasureDistForward();
if(forwardD<25){
backward(100);
delay(100); 
}
else{
 //Somet();
forward(distance,angle);
}


}
//  forward();

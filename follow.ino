#include <Wire.h>


const int Enable_A = 10;
const int Enable_B = 11;
const int inputA1 = 5;
const int inputA2 = 8;
const int inputB1 = 9;
const int inputB2 = 2;
const int required_distance=40;

int angle_array[2000];
int distance_array[1000];
int prev_error=0;
int total_error=0;
int angular_prev_error=0;
int angular_total_error=0;
int k=0;
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

void forward()
{
  int angle;
  int distance;
  //angle generation
 
    k++;

  
  angle=angle_array[k];
  distance=distance_array[k];
  Serial.println("angle");
  Serial.println(angle);
  int velocity=dist_controller(distance,required_distance);
  int angular_vel=angle_controller(angle);
  
angular_vel=10;
  if((angular_vel)>0){
  digitalWrite(inputA1, LOW);
  digitalWrite(inputA2, HIGH);
  }
  else{
  Serial.print("burdayım");
  digitalWrite(inputA1, HIGH);
  digitalWrite(inputA2, LOW);
  }
 
//  Serial.println(distance);
//  Serial.print("velocity: ");
//  Serial.println(velocity);
  Serial.print("velocity: ");
  Serial.println(angular_vel);
  digitalWrite(Enable_A, HIGH);
  digitalWrite(Enable_B, HIGH);
  analogWrite(Enable_A,velocity);
  analogWrite(Enable_B, abs(velocity));

    digitalWrite(inputB1, LOW);
  digitalWrite(inputB2, HIGH);
  
}
int dist_controller(int distance,int required_distance)
{
  int Kp=3;
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
  
  int Kp=3;
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

void Somet(){
  analogWrite(Enable_A, 100);
   analogWrite(Enable_B, 100);
  digitalWrite(inputA1, LOW);
  digitalWrite(inputA2, HIGH);
  digitalWrite(inputB1, HIGH);
  digitalWrite(inputB2, LOW);
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

  Serial.begin(9600);                               // // Serial Communication is starting with 9600 of baudrate speed
}  

void loop()
{

  
  for(int i=0;i<2000;i++){
    if(i<1000){
      angle_array[i] = 90-i*0.18;
      distance_array[i]=90-i*0.18;
    }
    
    else{
      angle_array[i] = i*0.18-180;
      distance_array[i-1000]=i*0.18-180;
  }

  }
  forward();
}

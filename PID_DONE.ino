#include<Servo.h> //include Servo.h library
#include <Wire.h> //include library for wires


//Set-up Pins and Variables for Proximity Sensor
int trigPin = 12; //Trigger - sends signal
int echoPin = 13; //Echo - received signal
long returntime, distmm, dist5, dist4, dist3, dist2, dist1, avgdist = 0; //Define nine global variables of long type

Servo myservo; //name the serve 'myservo'


////////////////////////Variables///////////////////////
float distance = 0.0; //total distance from end to end
float elapsedTime, rtime, timePrev;        //Variables for time control
float distance_previous_error, distance_error; //distance error variables
int period = 40;  //Refresh rate period of the loop is 50ms
///////////////////////////////////////////////////////


///////////////////PID constants///////////////////////
float kp = 1.6; //1.2-1.9 Working value: 1.6
float ki = 0.044; //0.04 to 0.06 Working value: 0.044
float kd = 210; //190 - 230 Working value: 210
float distance_setpoint = 225;           //Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d = 0; //initliaze variables for PID Proportional Gain, PID Integral gain, PID derivative gain
float PID_total = 0; //creates a variable for Total PID Value

void setup() {
  // put your setup code here, to run once:
pinMode (trigPin, OUTPUT); //set trigPin #12 as output for Prox Sensor
pinMode (echoPin, INPUT); //set echoPin #13 as input for Prox Sensor
Serial.begin(9600); //Start serial Monitor to print Proximity Data
myservo.attach(9);  // attaches the servo on pin 9 to the servo object
myservo.write(90); //Put the servco at angle 90, so the balance is in the middle
delay(2500); //delay of 2.5s
rtime = millis(); //starts timer, stores it in millis()

}


void loop() {
  // put your main code here, to run repeatedly:
  
//stores last 5 distances of cart, averages them for more accurate result
dist1 = distmm;
dist2 = dist1;
dist3 = dist2;
dist4 = dist3;
dist5 = dist4;
avgdist = (dist1 + dist2 + dist3 + dist4 + dist5) / 5;


  if (millis() > rtime+period)//if the time passed is longer than the variable plus period
  {
    rtime = millis();//store the time into the same variable    
    prox();//run prox subroutine to get distance of cart
    distance_error = distance_setpoint - avgdist; //the error equals the set point subtracted by the distance of the cart
    
    //PROPORTIONAL GAIN
    PID_p = kp * distance_error;//multiply proportional gain value by error
    
    //DERIVATIVE GAIN
    float dist_diference = distance_error - distance_previous_error;     
    PID_d = kd*((distance_error - distance_previous_error)/period);
      
   //INTEGRAL GAIN
    PID_i = PID_i + (ki * distance_error);
  
//TOTAL PID VALUE  
    PID_total = PID_p + PID_i + PID_d;

//Serial.print(", ");
   // Serial.println(90);
  // Serial.println(avgdist);

//MAPS PID VALUES TO A RANGE
  PID_total = map(PID_total, -150, 150, 50, 130);

  //LIMITS MOVEMENT OF SERVO, CLAMPING
    if(PID_total < 40.0)
    {
      PID_total = 40;  
    }
    if(PID_total > 140) 
    {
      PID_total = 140;
    } 

//Serial.print(PID_total);
  
  //Moves servo to position
    myservo.write(PID_total);  

    //stores error into previous error variable
    distance_previous_error = distance_error;
  }










}



//subroutine for proximity sensor
void prox() 
{
digitalWrite(trigPin, LOW); //Turn of trigPin for 5microseconds to ensure clean pulse
delayMicroseconds(5);
digitalWrite(trigPin, HIGH); //Pulse trigPin for exactly 10microseconds to send out a sound pulse
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
//Read the time it takes for the signal to return and store to variable 'returntime'
returntime = pulseIn(echoPin, HIGH);
//Convert the measured return time to cm and inches (Math from class)
delay(10);
distmm = (returntime * 0.1845);
// Output the result in cm using the Serial Monitor

}



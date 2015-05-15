/*
Author : Simeon Simeonov
 Name : First Arduino project
 Ver: 1.0.0
 */
 
 #define PART_LM4F120
 
 #include "Energia.h"

#include <stdio.h> // for function sprintf


//--------------------- PIN definitions -----------

#define RIGHT_EDGE  A6      // A0
#define RIGHT_POS   A5      // A1
#define CENTER_POS  A4      // A2
#define LEFT_POS    A2      // A3
#define LEFT_EDGE   A1      // A4

#define LEFT_PWM    PC_6
#define RIGHT_PWM   PC_7
#define LEFT_DIR    PD_6
#define RIGHT_DIR   PD_7

//--------------- LEDS -----------
#define LEFT_LED    PB_5      // LED1
#define CENTER_LED  PB_0      // LED2
#define RIGHT_LED   PB_1      // LED1

//--------------  Buttons -----------
// PUSH1
// PUSH2

//---------------- Sensors and position ---------
#define FORWARD    1
#define LEFT       2
#define RIGHT      3
#define BACK       4


unsigned int left_pos;
unsigned int right_pos;
unsigned int center_pos;
unsigned int left_edge;
unsigned int right_edge;
int pos; 

int min_left, max_left;
int min_center, max_center;
int min_right, max_right;
int min_l_edge, max_l_edge;
int min_r_edge, max_r_edge;
int left_cal, center_cal, right_cal, l_edge_cal, r_edge_cal;

int dir;

///////////////////////////////////////////////////////////////////////////
//----------------------------------- PID ---------------------------------
///////////////////////////////////////////////////////////////////////////
#define KP   1
#define KD    16
#define MAX_SPEED      150
#define ACQUIRE_SPEED  50
#define SLOW_SPEED     MAX_SPEED - MAX_SPEED/10
#define MIN_SPEED      MAX_SPEED / 8
#define TURN_SPEED     MAX_SPEED + MAX_SPEED /8

/////////////////////////////////////////////////////////////////////////
int error;
int lastError;
int last_position;
int left_pwm;
int right_pwm;
int motorSpeed;

//--------- Common ------------
char tmp_str[64];


//=================  CALIBRATE ==========================
void callibrate (void) {
  int i;
  int tmp_value;

  min_left = 4096;
  min_right = 4096;
  min_center = 4096;
  min_l_edge = 4096;
  min_r_edge = 4096;
  max_left = 0;
  max_right =0;
  max_center = 0;
  max_l_edge = 0;
  max_r_edge = 0;

  analogWrite(LEFT_PWM,ACQUIRE_SPEED);
  analogWrite(RIGHT_PWM,ACQUIRE_SPEED);
  digitalWrite(LEFT_DIR,1);
  digitalWrite(RIGHT_DIR,0);

  for (i=0; i<1024; i++) {
    tmp_value = analogRead(LEFT_POS);
    if (tmp_value< min_left) min_left = tmp_value; 
    if (tmp_value > max_left) max_left = tmp_value; 
    tmp_value = analogRead(CENTER_POS);
    if (tmp_value < min_center) min_center = tmp_value; 
    if (tmp_value > max_center) max_center = tmp_value; 
    tmp_value = analogRead(RIGHT_POS);
    if (tmp_value < min_right) min_right = tmp_value; 
    if (tmp_value > max_right) max_right = tmp_value; 

    tmp_value = analogRead(LEFT_EDGE);
    if (tmp_value < min_l_edge) min_l_edge = tmp_value; 
    if (tmp_value > max_l_edge) max_l_edge = tmp_value; 

    tmp_value = analogRead(RIGHT_EDGE);
    if (tmp_value < min_r_edge) min_r_edge = tmp_value; 
    if (tmp_value > max_r_edge) max_r_edge = tmp_value; 


    if (i==512) {
      digitalWrite(LEFT_DIR,0);
      digitalWrite(RIGHT_DIR,1);
    }  
    delay(2);
  }

  left_cal = (max_left - min_left) / 10; 
  center_cal = (max_center - min_center)/ 10; 
  right_cal = (max_right - min_right) / 10; 
  l_edge_cal = (max_l_edge - min_l_edge)/10;
  r_edge_cal = (max_r_edge - min_r_edge)/10;

  digitalWrite(LEFT_DIR,1);
  digitalWrite(RIGHT_DIR,0);
  analogWrite(LEFT_PWM,SLOW_SPEED);
  analogWrite(RIGHT_PWM, SLOW_SPEED);
  do {
    tmp_value = analogRead(CENTER_POS);
    center_pos = ((tmp_value - min_center)*10) / center_cal;
  } 
  while (center_pos < 70);
  
  digitalWrite(LEFT_DIR,1);
  digitalWrite(RIGHT_DIR,0);
  analogWrite(LEFT_PWM,0);
  analogWrite(RIGHT_PWM,0);
}

//============================= Read sensors  and scale =====================
void read_sensors(void) {
  //-------------- Read sensors ------------
  left_edge = analogRead(LEFT_EDGE);
  left_pos = analogRead(LEFT_POS);
  center_pos = analogRead(CENTER_POS);
  right_pos = analogRead(RIGHT_POS);
  right_edge = analogRead(RIGHT_EDGE);

  //--------- Validate ----------
  if (left_pos< min_left)
    left_pos = min_left;

  if (center_pos< min_center)
    center_pos = min_center;

  if (right_pos< min_right)
    right_pos = min_right;  

  if (left_edge < min_l_edge)
    left_edge =  min_l_edge; 

  if (right_edge < min_r_edge)
    right_edge = min_r_edge;

  //--------------------- Calibrate -------------------
  left_pos = ((left_pos - min_left)*10) / left_cal;
  center_pos = ((center_pos - min_center)*10) / center_cal;
  right_pos = ((right_pos - min_right)*10) / right_cal;

  left_edge = ((left_edge - min_l_edge)*10) / l_edge_cal;
  right_edge = ((right_edge - min_r_edge)*10) / r_edge_cal;

}



//////////////////////////////////////////////////////////////////////////////


// the setup routine runs once when you press reset:
void setup()  {   
/*  
  pinMode(LEFT_EDGE,INPUT);
  pinMode(RIGHT_EDGE,INPUT);
  pinMode(LEFT_POS,INPUT);
  pinMode(CENTER_POS,INPUT);
  pinMode(RIGHT_POS,INPUT);
*/

  pinMode(LEFT_DIR,OUTPUT);
  pinMode(RIGHT_DIR,OUTPUT);
  pinMode(LEFT_PWM,OUTPUT);
  pinMode(RIGHT_PWM,OUTPUT);
  //analogFrequency(800);
  //PWMwrite(13,256,50,1000);
  
  pinMode(LEFT_LED, OUTPUT);
  pinMode(CENTER_LED, OUTPUT);
  pinMode(RIGHT_LED, OUTPUT);

  pinMode(PUSH1, INPUT_PULLUP);
  pinMode(PUSH2, INPUT_PULLUP);
  
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  while ( digitalRead(PUSH2) ==1);
  callibrate();
  dir = FORWARD;
  lastError = 0;

  sprintf(tmp_str,"Left %d - %d", min_left, max_left);
  Serial.println(tmp_str); 
  sprintf(tmp_str,"Center %d - %d", min_center, max_center);
  Serial.println(tmp_str); 
  sprintf(tmp_str,"Right %d - %d", min_left, max_left);
  Serial.println(tmp_str); 

  sprintf(tmp_str,"Callibrated %d - %d - %d", left_cal, center_cal, right_cal);
  Serial.println(tmp_str); 
  sprintf(tmp_str,"     edges  %d - %d ", l_edge_cal, r_edge_cal);
  Serial.println(tmp_str);   
  
  while ( digitalRead(PUSH2) ==1);
} 



// the loop routine runs over and over again forever:
void loop()  { 

  static int tmp_value;
  //char sequence;
  static char  found_left, found_right, found_straight;

  read_sensors();
  
  digitalWrite(LEFT_LED,LOW);
  digitalWrite(CENTER_LED,LOW);
  digitalWrite(RIGHT_LED,LOW);
  

  sprintf(tmp_str,"current %d | %d - %d - %d | %d",left_edge, left_pos, center_pos, right_pos, right_edge);
  Serial.println(tmp_str);

/*
  //-------------------- Find sequence -------------------
  if (dir== FORWARD) {

    found_left = 0; 
    found_right =0; 
    found_straight = 0;

    if ( right_edge > 30 ) found_right = 1; 
    if ( left_edge > 30 ) found_left = 1;

    if (found_right || found_left ) {
      digitalWrite(RIGHT_DIR, HIGH);
      digitalWrite(LEFT_DIR, HIGH);
      analogWrite(RIGHT_PWM,70);
      analogWrite(LEFT_PWM,70);
      do {
        delay(30);
        read_sensors();
        
      } 
      while (( left_edge > 10 ) || ( right_edge > 10 )) ;
      delay(100);
    }
    
    read_sensors();
    if ((left_pos > 20) || (center_pos > 20) || (right_pos>20)){
      found_straight = 1;
    }

    //sprintf(tmp_str,"sequences  %d - %d - %d ",found_left, found_straight, found_right);
    //Serial.println(tmp_str);

    //----------------------  Make the reaction -------------
    // Make a decision about how to turn.  The following code
    // implements a left-hand-on-the-wall strategy, where we always
    // turn as far to the left as possible.

    if(found_left) {
      digitalWrite(LEFT_LED, HIGH);
      digitalWrite(LEFT_DIR,0);
      digitalWrite(RIGHT_DIR,1);
      analogWrite(LEFT_PWM,MAX_SPEED);
      analogWrite(RIGHT_PWM, MAX_SPEED);
      dir = LEFT;
      delay(100);
    } 
    else if(found_straight) {
      dir = FORWARD;
    } 
    else if(found_right) {
      digitalWrite(RIGHT_LED, HIGH);
      digitalWrite(LEFT_DIR,1);
      digitalWrite(RIGHT_DIR,0);
      analogWrite(LEFT_PWM,MAX_SPEED);
      analogWrite(RIGHT_PWM, MAX_SPEED);
      dir = RIGHT;
      delay(100);

    }  
    //else if ((left_pos < 15) && (center_pos < 15) && (right_pos<15))  {
    else{
      dir = BACK;
      digitalWrite(LEFT_DIR,1);
      digitalWrite(RIGHT_DIR,0);
      analogWrite(LEFT_PWM,MAX_SPEED);
      analogWrite(RIGHT_PWM, MAX_SPEED);
      delay(100);
    }
  }
*/

    if ( right_edge > 90 ) {
      digitalWrite(LEFT_LED, HIGH);
      digitalWrite(LEFT_DIR,1);
      digitalWrite(RIGHT_DIR,1);
      analogWrite(LEFT_PWM,TURN_SPEED);
      analogWrite(RIGHT_PWM, MIN_SPEED);
      dir = LEFT;
    } 
    
    
    if ( left_edge > 90 ) {
      digitalWrite(RIGHT_LED, HIGH);
      digitalWrite(LEFT_DIR,1);
      digitalWrite(RIGHT_DIR,1);
      analogWrite(LEFT_PWM,MIN_SPEED);
      analogWrite(RIGHT_PWM, TURN_SPEED);
      dir = RIGHT;

    }  

  //sprintf(tmp_str,"sequences  %d - %d - %d / %d",found_left, found_straight, found_right, dir);
  //Serial.println(tmp_str);

  // Drive straight a bit more - this is enough to line up our
  // wheels with the intersection.
  //delay(300);

  switch (dir) {
  case FORWARD:
  default:

    pos = (left_pos*100 + center_pos*200 + right_pos*300)/(left_pos + center_pos + right_pos);
    error = pos - 200;
    
    //sprintf(tmp_str,"position  %d", error);
    //Serial.println(tmp_str);
    
    motorSpeed = KP * error + KD * (error - lastError);
    lastError = error;
    last_position = pos;
    left_pwm = MAX_SPEED +  motorSpeed;
    right_pwm = MAX_SPEED - motorSpeed;
    if (right_pwm > 255) right_pwm = 255;
    if (left_pwm > 255) left_pwm = 255;

    if (right_pwm <MIN_SPEED) right_pwm = 0;
    if (left_pwm <MIN_SPEED) left_pwm = 0;
/*
    digitalWrite(LEFT_DIR,1);
    digitalWrite(RIGHT_DIR,1);
    analogWrite(LEFT_PWM,left_pwm);
    analogWrite(RIGHT_PWM, right_pwm);
*/
    break;

  case LEFT:
    digitalWrite(LEFT_LED, HIGH);
    if (right_pos > 50)  {
      digitalWrite(LEFT_DIR,1);
      digitalWrite(RIGHT_DIR,1);
      analogWrite( LEFT_PWM, MAX_SPEED);
      analogWrite( RIGHT_PWM , SLOW_SPEED);
      dir = FORWARD;
      lastError = 0;
      last_position = 0;
      //sdelay(1);
    }
    break;
  case RIGHT:
    digitalWrite(RIGHT_LED, HIGH);
    if ( left_pos > 50)  {
      digitalWrite(LEFT_DIR,1);
      digitalWrite(RIGHT_DIR,1);
      analogWrite( LEFT_PWM ,SLOW_SPEED);
      analogWrite(RIGHT_PWM  , MAX_SPEED);
      dir = FORWARD;
      lastError = 0;
      last_position = 0;
      //delay(1);
    }
    break;

  }

  //sprintf(tmp_str," error %d - speed %d",error, motorSpeed);
  //sprintf(tmp_str,"DIR %d /  l %d - R %d",dir, left_pwm, right_pwm);
  //sprintf(tmp_str,"Edges : %d   %d", left_edge, right_edge);
  //Serial.println(tmp_str);

  // wait for 15 milliseconds to see the dimming effect    
  delay(15);                            
}


















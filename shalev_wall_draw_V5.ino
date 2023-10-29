// polargraph code edited by shalev: 

//libraries for us to use in this code: (compatable on pi)
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <Servo.h>
#include <Arduino.h>
#include <Bounce2.h>

// pins for motors:
#define X_DIR_PIN 2
#define X_STEP_PIN 5
#define Y_DIR_PIN 3
#define Y_STEP_PIN 6
#define STEPS_PER_REVOLUTION 200
#define STEPPER_EN_PIN 8 //enable steppers pin 

//------------------------------------------------------------------------------
// settings for the movment of the motors:
unsigned long timerStart = 0;
const unsigned long timerDuration = 6000 *(100000); // 1 minute in milliseconds delay between random drawings
const int SpeedOFMotors = 500; // set speed each motor movement
const int stepsPerMotor = 1; // set the number of steps for each motor movement
//------------------------------------------------------------------------------

#define X_SEPARATION  1300    //The horizontal distance above the two ropes mm       
#define LIMXMAX       ( X_SEPARATION*0.5)  //x-axis maximum value 0 is at the center of the artboard
#define LIMXMIN       (-X_SEPARATION*0.5)  //x-axis minimum
#define LIMYMAX         (-500)   //The maximum value of the y-axis is at the bottom of the drawing board
#define LIMYMIN         (500)  //The minimum value of the y-axis is the vertical distance from the fixed point of the left and right lines to the pen at the top of the drawing board. 
//Try to measure and place it accurately, and there will be distortion if the error is too large
//When the value is reduced, the drawing becomes thinner and taller, and when the value is increased, the drawing becomes short and fat

// the following variables are unsigned longs because the time, measured in

// milliseconds, will quickly become a bigger number than can be stored in an int.
//------------------------------------------------------------------------------
// homing and buttons defenition:

int xDirection = 1;
int yDirection = 1;
//pins:
int buttonPin1 = A2; 
int buttonPin2 = A3; 
// Variables to store the button states
int buttonState12 = 0;
int buttonState9 = 0;
//------------------------------------------------------------------------------
// Define constants for motor direction:
#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
unsigned long previousMillis = 0;
const long interval = 2000;
//------------------------------------------------------------------------------

// old code stuff for the neew code to work:
#define STEPS_PER_TURN  (2048)  
#define SPOOL_DIAMETER  (35)    
#define SPOOL_CIRC      (SPOOL_DIAMETER * 3.1416)  // 35*3.14=109.956
#define TPS             (SPOOL_CIRC / STEPS_PER_TURN)  
#define step_delay      1  
#define TPD             300  
#define M1_REEL_OUT     1    
#define M1_REEL_IN      -1     
#define M2_REEL_OUT     -1      
#define M2_REEL_IN      1     
static float laststep1 = 700 , laststep2 = 700 ; 
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//code for encoders:
#define ENCODER_A_BIT_1 (A0) // input IO for gray code bit 0 
#define ENCODER_A_BIT_0 (9) // input IO for gray code bit 1
#define ENCODER_B_BIT_1 (A1) // input IO for gray code bit 0
#define ENCODER_B_BIT_0 (12) // input IO for gray code bit 1

Bounce debouncerButton1 = Bounce(); // Debouncer instance for button 1
Bounce debouncerButton2 = Bounce(); // Debouncer instance for button 2

byte Old_Encoder_A_Read = 0; // old 2 bits read from first encoder 
byte New_Encoder_A_Read = 0; // new 2 bits read from first encoder 
int16_t Current_Encoder_A_Count = 0;// serial_printed and/or display 
int16_t Old_Encoder_A_Count = 0;// for later use

byte Old_Encoder_B_Read = 0; // old 2 bits read from second encoder 
byte New_Encoder_B_Read = 0; // new 2 bits read from second encoder 
int16_t Current_Encoder_B_Count = 0;// serial_printed and/or display 
int16_t Old_Encoder_B_Count = 0;// for later use

int Last_Pulse_R = 0;
int Last_Pulse_L = 0;
int Rate = 10;

uint32_t Current_Time = 0   ;

int Read_Encoder_A(){
     Old_Encoder_A_Read = New_Encoder_A_Read ;
     New_Encoder_A_Read = (((digitalRead(ENCODER_A_BIT_1)) << 1) + (digitalRead(ENCODER_A_BIT_0))) ;
     byte Check_Direction  = (Old_Encoder_A_Read << 2) + New_Encoder_A_Read  ; // x4 = 2 rotate left 
     switch (Check_Direction)
     {
      case 1: case 7: case 8: case 14:
      return 1 ;
      case 2: case 4: case 11: case 13:
      return -1 ;
      case 0: case 5: case 10: case 15:
      return 0 ;
      default: // need to avoide delay in return 
      return 0 ; // 
    }
}
//--------------------------
int Read_Encoder_B(){
     Old_Encoder_B_Read = New_Encoder_B_Read ;
     New_Encoder_B_Read = (((digitalRead(ENCODER_B_BIT_1)) << 1) + (digitalRead(ENCODER_B_BIT_0))) ;
     byte Check_Direction  = (Old_Encoder_B_Read << 2) + New_Encoder_B_Read  ; // x4 = 2 rotate left 
     switch (Check_Direction)
     {
      case 1: case 7: case 8: case 14:
      return 1 ;
      case 2: case 4: case 11: case 13:
      return -1 ;
      case 0: case 5: case 10: case 15:
      return 0 ;
      default: // need to avoide delay in return 
      return 0 ; // 
    }
}
//---------------------------------
void Update_Encoder_A_Count(){
  int temp_move = Read_Encoder_A();
  Current_Encoder_A_Count += temp_move;
}
//---------------------------------
void Update_Encoder_B_Count(){
  int temp_move = Read_Encoder_B();
  Current_Encoder_B_Count += temp_move;
}


//------------------------------------------------------------------------------
// parameters for manual coordinates:
long disx = 0;
long disy = 0;
//------------------------------------------------------------------------------
//Define iterator value for chaging drawings:
int b = 0;
int c = 0;
//------------------------------------------------------------------------------
// servo stuff:
Servo myservo;  // create servo object to control a servo
int countServo = 0;
// debounce for the button:
int buttonState = HIGH;  // Current state of the button (HIGH means not pressed)
int lastButtonState = HIGH;  // Previous state of the button (HIGH means not pressed)
unsigned long lastDebounceTime = 0;  // Last time the button state was toggled
unsigned long debounceDelay = 50;    // Debounce time in milliseconds
int test = 0;  // Variable to keep track of the current direction of the servo
//------------------------------------------------------------------------------
struct point { 
  float x; 
  float y; 
  float z; 
};
//------------------------------------------------------------------------------
struct point actuatorPos;
// plotter position 
float posx;
float posy;
static float posz;  // pen state
static float feed_rate = 0;
// pen state
static int ps;
#define BAUD            (115200)    
#define MAX_BUF         (64)    
int xstate;
int ystate;
//------------------------------------------------------------------------------
void move_step(int stepCount, int stepPin,int dirPin)
{
  // set the axis direction  
  if (stepCount > 0){
      digitalWrite(dirPin, HIGH);
    }
  else{
    digitalWrite(dirPin, LOW);
    }
    for (int i = 0; i < abs(stepCount); i++) {
      // These four lines result in 1 step:
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(SpeedOFMotors);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(SpeedOFMotors);
    }
        //Serial.print("moved!");
  }

//------------------------------------------------------------------------------

// theta = acos((a*a+b*b-c*c)/(2*a*b));
void FK(float l1, float l2,float &x,float &y) {
  Serial.print(x);
  Serial.print(":");
  Serial.println(y);
  float a=l1 * TPS;
  float b=X_SEPARATION;
  float c=l2 * TPS;
  float theta = acos((a*a+b*b-c*c)/(2.0*a*b));
  x = cos(theta)*l1 + LIMXMIN;
  y = sin(theta)*l1 + LIMYMIN;
            
//  float theta = (a*a+b*b-c*c)/(2.0*a*b);
//  x = theta*l1 + LIMXMIN;
//  y = sqrt (1.0 - theta * theta ) * l1 + LIMYMIN;

  Serial.print("theta");
  Serial.println((a*a+b*b-c*c)/(2.0*a*b));
  Serial.print(x);
  Serial.print(":");
  Serial.println(y);
}

//------------------------------------------------------------------------------
void IK(float x,float y,long &l1, long &l2) {
  float dy = y - LIMYMIN;
  float dx = x - LIMXMIN;
  l1 = round(sqrt(dx*dx+dy*dy) / TPS);
  dx = x - LIMXMAX;
  l2 = round(sqrt(dx*dx+dy*dy) / TPS);
}

//------------------------------------------------------------------------------
// returns angle of dy/dx as a value from 0...2PI
static float atan3(float dy, float dx) {
  float a = atan2(dy, dx);
  if (a < 0) a = (PI * 2.0) + a;
  return a;
}
//------------------------------------------------------------------------------
// instantly move the virtual plotter position
// does not validate if the move is valid
static void teleport(float x, float y) {
  posx = x;
  posy = y;
  xstate = x;
  ystate = y;
  long l1,l2;
  IK(posx, posy, l1, l2);
  laststep1 = l1;
  laststep2 = l2;
}

//------------------------------------------------------------------------------
void moveto(float x,float y) {
    long l1,l2;
    IK(x,y,l1,l2);
    long d1 = l1 - laststep1;
    long d2 = l2 - laststep2;
    
    long ad1=abs(d1);
    long ad2=abs(d2);
    
    int dir1 = d1>0 ? M1_REEL_IN : M1_REEL_OUT;
    int dir2 = d2>0 ? M2_REEL_IN : M2_REEL_OUT;
    long over = 0;
    long i;

  if(ad1>ad2) {
    for(i=0;i<ad1;++i) {
      move_step(dir1,X_STEP_PIN,X_DIR_PIN);
      over+=ad2;
      if(over>=ad1) {
        over-=ad1;
          move_step(dir2,Y_STEP_PIN,Y_DIR_PIN);
      }
      delayMicroseconds(SpeedOFMotors);
     }
  } 
  else {
    for(i=0;i<ad2;++i) {
        move_step(dir2,Y_STEP_PIN,Y_DIR_PIN);
      over+=ad1;
      if(over>=ad2) {
        over-=ad2;
        move_step(dir1,X_STEP_PIN,X_DIR_PIN);
      }
      delayMicroseconds(SpeedOFMotors);
    }
  }
  laststep1=l1;
  laststep2=l2;
  posx = x;
  posy = y;
  
//  Serial.println("move to: ");
//  Serial.println(posx);
//  Serial.println(posy);  
}
//------------------------------------------------------------------------------
static void line_safe(float x,float y) {
  // split up long lines to make them straighter?
  float dx=x-posx;
  float dy=y-posy;
  float len=sqrt(dx*dx+dy*dy);
  if(len<=TPS) {
    moveto(x,y);
    return;
  }
  long pieces=floor(len/TPS);
  float x0=posx;
  float y0=posy;
  float a;
  for(long j=0;j<=pieces;++j) {
    a=(float)j/(float)pieces;
    moveto((x-x0)*a+x0,(y-y0)*a+y0);
  }
  moveto(x,y);
}
//------------------------------------------------------------------------------
void line(float x,float y) 
{
  line_safe(x,y);
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void homing() {
//
}
//------------------------------------------------------------------------------
void setDirectionRight(int direction) 
{
  digitalWrite(X_DIR_PIN, direction);
}

//------------------------------------------------------------------------------
void setDirectionLeft(int direction) 
{
  digitalWrite(Y_DIR_PIN, direction);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void stepMotorRight() {
  digitalWrite(X_STEP_PIN, HIGH);
  delayMicroseconds(SpeedOFMotors); // Adjust this value to change motor speed
  digitalWrite(X_STEP_PIN, LOW);
  delayMicroseconds(SpeedOFMotors); // Adjust this value to change motor speed
}
//------------------------------------------------------------------------------
void stepMotorLeft() {
  digitalWrite(Y_STEP_PIN, HIGH);
  delayMicroseconds(SpeedOFMotors); // Adjust this value to change motor speed
  digitalWrite(Y_STEP_PIN, LOW);
  delayMicroseconds(SpeedOFMotors); // Adjust this value to change motor speed
}
//------------------------------------------------------------------------------
// Generate random (x,y) coordinates within the given range and send them to the printer
void sendRandomCoordinates(int xMin, int xMax, int yMin, int yMax) {
  int x = random(xMin, xMax);
  int y = random(yMin, yMax);
  // Check if the generated x and y values are within range
  if (x >= xMin && x <= xMax && y >= yMin && y <= yMax) {
    moveto(x, y);
  }
}
//------------------------------------------------------------------------------
void moveMotor(int dirPin, int stepPin, int steps, bool clockwise) {
  digitalWrite(dirPin, clockwise ? HIGH : LOW); // set motor direction
  for(int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(SpeedOFMotors);
  }
}
//------------------------------------------------------------------------------
void randomDrawing() {
  // Code to be executed after one minute
  if (c == 0) {
    myservo.write(40);
    moveto(0 ,0);
    moveto(30 ,0);
    moveto(30 ,30);
    moveto(-30 ,30);
    moveto(-30 ,-30);
    moveto(30 ,-30);
    moveto(30 ,0);
    moveto(0 ,0);
    c = 1;  
  }
  else if (c == 1) {
    myservo.write(40);
    moveto(0 ,0);
    moveto(20, 0);
    moveto(40, 14.64);
    moveto(40, 34.64);
    moveto(20, 49.29);
    moveto(0, 34.64);
    moveto(0, 14.64);
    moveto(20, 0);
    moveto(0 ,0);
    c = 2;
   }

  else if (c == 2) {
    myservo.write(40);
    moveto(0, 0);       // Start point from (0, 0)
    moveto(4.14, 7.36);
    moveto(9.39, 14.14);
    moveto(15.36, 19.39);
    moveto(21.64, 22.64);
    moveto(27.86, 23.85);
    moveto(33.57, 22.83);
    moveto(38.38, 19.57);
    moveto(41.96, 14.35);
    moveto(44.12, 7.58);
    moveto(44.68, 0.77);
    moveto(43.60, -5.10);
    moveto(41.00, -10.31);
    moveto(36.99, -15.00);
    moveto(31.81, -18.67);
    moveto(25.76, -20.95);
    moveto(19.27, -21.58);
    moveto(12.84, -20.47);
    moveto(7.00, -17.74);
    moveto(2.28, -13.60);
    moveto(0, 0);       // Return to the start point
    c = 0;
   }
}               

//------------------------------------------------------------------------------
void moveMotor(int dirPin, int stepPin, int steps) {
          digitalWrite(dirPin, steps > 0 ? HIGH : LOW); // set motor direction based on sign of steps
          for (int i = 0; i < abs(steps); i++) {
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(SpeedOFMotors);
            digitalWrite(stepPin, LOW);
            delayMicroseconds(SpeedOFMotors);
  }
}
//------------------------------------------------------------------------------
int min_x = -5000; // Adjust the minimum X-axis coordinate
int max_x = 5000;  // Adjust the maximum X-axis coordinate
int min_y = -5000; // Adjust the minimum Y-axis coordinate
int max_y = 5000;  // Adjust the maximum Y-axis coordinate
int mapAndConstrain(int encoderValue, int minValue, int maxValue) {
  // Use the map function to adjust the encoder value to the range defined by minValue and maxValue
  int mappedValue = map(encoderValue, 0, 1023, minValue, maxValue);
  // Use constrain to ensure the mapped value stays within the specified range
  return constrain(mappedValue, minValue, maxValue);
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(BAUD);
  Serial.println("Booting...");
  // motors:
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_STEP_PIN, OUTPUT);

  //encoders:
  pinMode(ENCODER_A_BIT_0, INPUT_PULLUP);
  pinMode(ENCODER_A_BIT_1, INPUT_PULLUP);
  pinMode(ENCODER_B_BIT_0, INPUT_PULLUP);
  pinMode(ENCODER_B_BIT_1, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_BIT_0), Update_Encoder_A_Count, CHANGE);// arduino nano can only D2 and D3 interupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_BIT_1), Update_Encoder_A_Count, CHANGE);// arduino nano can only D2 and D3 interupt
  
  pinMode(STEPPER_EN_PIN, OUTPUT);// invert !
  digitalWrite(STEPPER_EN_PIN, LOW);// enable motors,
  
  // buttons:
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
    // Initialize button debouncers
  debouncerButton1.attach(buttonPin1);
  debouncerButton1.interval(50); // Debounce interval in milliseconds
  debouncerButton2.attach(buttonPin2);
  debouncerButton2.interval(50); 
  
  Current_Time = millis();
  Serial.println("Booting complete, start homing...");
  myservo.attach(A5);  // attaches the servo on pin A5 to the servo object

  
  // Start homing
  //homing();
  //moveto(0 ,0); // move to middle of the board
//  teleport(0, 0); // set position to (0,0) 

  //line_safe(0 ,100);
//  Serial.println("homing done");
//  Serial.print("current laststep1:");
//  Serial.println(laststep1);
//  Serial.print("current laststep2:");
//  Serial.println(laststep2);  
//  Serial.println("   ");
//  Serial.println(TPS);  
  

  //moveto(0 ,0);
  //Serial.println("Test OK!");
  //demo1();
  // line_safe(0 , 0);
  // moveto(0 , 0);
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
int Last_Encoder_A_Count = Current_Encoder_A_Count;
int Last_Encoder_B_Count = Current_Encoder_B_Count;

void loop(){
      
     //checks for inactivity = button pressing if not - start a timer of 1 minute 
    if (millis() - timerStart >= timerDuration) {
    // Timer duration reached, execute the code
    randomDrawing();
    // Reset the timer
    timerStart = millis();
  }
  Update_Encoder_A_Count();// encoder A is updated by interupt 
  Update_Encoder_B_Count();// encoder A is updated by interupt 

  //Testing the boarders
    float test_1 = laststep1;
    float test_2 = laststep2;
    float test_x = 0;
    float test_y = 0;
    

  // Check if the encoder count increased by one step
  if (Current_Encoder_A_Count > Last_Encoder_A_Count) {
    
    FK(laststep1 + TPS,laststep2,test_x,test_y);
    // Move Y motor counter-clockwise - right up by one step
    if (test_y < max_y) {
      moveMotor(Y_DIR_PIN, Y_STEP_PIN, -stepsPerMotor);
      timerStart = millis(); // Reset the timer when there's motor activity
//      posy++;
      laststep1 += TPS;
    }
  } else if (Current_Encoder_A_Count < Last_Encoder_A_Count) {

    FK(laststep1 - TPS,laststep2,test_x,test_y);

    // Move Y motor clockwise - right down by one step
    if (test_y > min_y) {
      moveMotor(Y_DIR_PIN, Y_STEP_PIN, stepsPerMotor);
      timerStart = millis(); // Reset the timer when there's motor activity
//      posy--;
      laststep1 -= TPS;
    }
  }
//  Serial.print("current laststep1:");
//  Serial.println(laststep1);
//  Serial.print("current laststep2:");
//  Serial.println(laststep2);  
//  Serial.println("   ");
//  Serial.println(TPS);  
  

  Last_Encoder_A_Count = Current_Encoder_A_Count;

    test_y = posy;
    test_x = posx;
  // Check if the encoder count increased by one step
  if (Current_Encoder_B_Count > Last_Encoder_B_Count) {
    test_x++;
    FK(laststep1,laststep2,test_x,test_y);
    
    // Move X motor counter-clockwise - right up by one step
    if (test_x < max_x) {
      moveMotor(X_DIR_PIN, X_STEP_PIN, stepsPerMotor);
      timerStart = millis(); // Reset the timer when there's motor activity
      posx++;
    }
  } else if (Current_Encoder_B_Count < Last_Encoder_B_Count) {
      test_x--;
      FK(laststep1,laststep2,test_x,test_y);
    
    // Move X motor clockwise - right down by one step
    if (test_x > min_x) {
      moveMotor(X_DIR_PIN, X_STEP_PIN, -stepsPerMotor);
      timerStart = millis(); // Reset the timer when there's motor activity
      posx--;
    }
  }
  
  Last_Encoder_B_Count = Current_Encoder_B_Count;



   // Print encoder values
    Serial.print("Encoder A Count: ");
    Serial.print(Current_Encoder_A_Count/2);
    Serial.print(", Encoder B Count: ");
    Serial.println(Current_Encoder_B_Count/2);
  
 
            
  buttonState12 = digitalRead(buttonPin1);
  buttonState9 = digitalRead(buttonPin2);
   if (buttonState12 == LOW) {
    Serial.println("Button servo!");
        static int test = 0;  // Variable to keep track of the current direction of the servo
        if (test == 0) {
           myservo.write(40);
           delay(500);
           test = 1;
           timerStart = millis();
           } 
           else 
           {
               myservo.write(120);
               delay(500);
               test = 0;
               timerStart = millis();
               }
     }
 
}
        




  

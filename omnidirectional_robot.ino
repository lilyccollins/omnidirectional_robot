#include <PS2X_lib.h>     //for v1.6, see billporter.info for updates
#include <Servo.h>      //copy from servo code - a maximum of eight servo objects can be created

#define PS2_DAT        13     //data 7
#define PS2_CMD        11     //command 6
#define PS2_SEL        10     //attention 5
#define PS2_CLK        12     //clock 4

#define pressures   false
#define rumble      false

Servo myservoA;  // works on motor A  -  upper left placement on robot
Servo myservoB;  // works on motor B  -  upper right placement on robot
Servo myservoC;  // works on motor C  -  middle back placement on robot
Servo myservoD;  // works on motor D  -  camera stand

PS2X ps2x;       // create PS2 Controller Class

int error = 0;
byte type = 0;
byte vibrate = 0;


int pos = 0;    // variable to store the servo position
int posD = 0;


void setup(){  
  Serial.begin(57600);
  myservoA.attach(A2);  // attaches the servo on pin A2 to the servo object
  myservoB.attach(A1);  // attaches the servo on pin A1 to the servo object
  myservoC.attach(A0);  // attaches the servo on pin A0 to the servo object
  myservoD.attach(A3);  // attaches the servo on pin A3 to the servo object
  
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);


  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);   //setup pins (in this order) and settings:  GamePad(clock, command, attention, data, Pressures, Rumble)


  if(error == 0){   //for PS2 connection troubleshooting
    Serial.println("Found Controller, configured successful");
    Serial.println("HOLDING L1 or R1 will print out the ANALOG STICK values.");
    Serial.println("www.billporter.info for updates and to report bugs.");
  }
  
  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  type = ps2x.readType(); 
  switch(type) {
  case 0:
    Serial.println("Unknown Controller type");
    break;
  case 1:
    Serial.println("DualShock Controller Found");
    break;
  }
}


void loop(){
  /* You must Read Gamepad to get new values
   Read GamePad and set vibration values
   ps2x.read_gamepad(small motor on/off, larger motor strength from 0-255)
   if you don't enable the rumble, use ps2x.read_gamepad(); with no values
   you should call this at least once a second
   */
   
  if(error == 1){ //skip loop if no controller found
    return; 
  } 
  else { //DualShock Controller Found
    ps2x.read_gamepad(false, vibrate);      //false unless a command written for vibrate (if "true" large motor spins at 'vibrate' speed)
    vibrate = ps2x.Analog(PSAB_BLUE);        //this will set the large motor vibrate speed based on how hard you press the blue (X) button
  }

  int rx = ps2x.Analog(PSS_RX); // reads the value of RX on the the PS2 receiver (value between 0 and 255) 
  int maprx = map(rx,0,255,1000,2000); //reads rx, creates a mapval integer between 1000 and 2000 that motor controller reads
  int ry = ps2x.Analog(PSS_RY); // reads the value of RY on the the PS2 receiver (value between 0 and 255) 
  //int mapry=map(ry,0,255,0,90); //reads ry, creates a mapval integer between 0 and 180 that servo reads
  
  int x = ps2x.Analog(PSS_LX); // reads the value of LX on the the PS2 receiver (value between 0 and 255) 
  int y = ps2x.Analog(PSS_LY); // reads the value of LY on the the PS2 receiver (value between 0 and 255) 

  if(x == 128 && y == 127) {
    myservoA.writeMicroseconds(maprx);  //stationary rotation
    myservoB.writeMicroseconds(maprx); 
    myservoC.writeMicroseconds(maprx);  
    delay(10);
  }


  if(ry == 0 && posD <= 70) {
    for (posD = 0; posD <= 70; posD += 1) {// goes from 0 degrees to 90 degrees in steps of 1 degree
      myservoD.write(posD);                   // tell servo to go to position in variable 'posD'
      delay(15);                              // waits 15ms for the servo to reach the position
    }
  }

  if(ry == 255 && posD >= 0){
    for (posD = 70; posD >= 0; posD -= 1) {// goes from 180 degrees to 0 degrees in steps of 1 degree
      myservoD.write(posD);                   // tell servo to go to position in variable 'posD'
      delay(15);                              // waits 15ms for the servo to reach the position
    }
  }


    
  int mapx = map(x,0,255,100,-100);
  int mapy = map(y,0,255,100,-100); //values from PS2 controller are read from 0-255, read as 100 to -100 value to represent a cartesian plane with a 0 in the center to simplify calculations.
  
  float theta = atan2(mapx,mapy); // arc tangent of x/y
 
  int L = sqrt(mapx*mapx+mapy*mapy);  //Pythagorean theorem
    
  float cosa = L*cos(150*M_PI/180-theta); //150 degrees minus theta
  float cosb = L*cos(30*M_PI/180-theta);  //first value is L (hypotenuse)
  float cosc = L*cos(270*M_PI/180-theta); //second value (parentheses section) represents adjacent side along x axis
  
  int Fa = map(cosa,-142,142,1000,2000); //-142 and 142 are limits of calculations above (should be)
  int Fb = map(cosb,-142,142,1000,2000);
  int Fc = map(cosc,-142,142,1000,2000);

  if(rx == 128 && ry == 127){                    //F values indicate motor rotation, robot navigation
    myservoA.writeMicroseconds(Fa);  
    myservoB.writeMicroseconds(Fb);  
    myservoC.writeMicroseconds(Fc);  
    delay(10);
  }
  
  //Next four sets of code allow you to read stick values
  if(ps2x.Button(PSB_L2)){  
    Serial.print(Fa,DEC);
    Serial.print(",");
    Serial.print(Fb,DEC);
    Serial.print(",");
    Serial.println(Fc,DEC);
  }

  if(ps2x.Button(PSB_L1)){   
    Serial.print(ps2x.Analog(PSS_LX), DEC); 
    Serial.print(",");
    Serial.println(ps2x.Analog(PSS_LY), DEC); 
  }                                          //same values as R1
  
  if(ps2x.Button(PSB_R1)){  
    Serial.print(ps2x.Analog(PSS_RX), DEC); 
    Serial.print(",");
    Serial.println(ps2x.Analog(PSS_RY), DEC); 
  }
  
  if(ps2x.Button(PSB_R2)){    
    Serial.print(x,DEC);
    Serial.print(",");
    Serial.println(y,DEC);
  }                                          //same values as L1
  
  delay(50);
         
}


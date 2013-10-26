/*****************************************************
*   Arduino example for MD25 / SRF02 / Sharp IR /    *
*                I2C compass Robot                   *
*                                                    *
*     Original md25 code James Henderson 2012        *
*                  Ian Redmond 2013                  *
*****************************************************/

#include <Wire.h>
#include <Serial.h>

String  inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean md25_present = false;    // true if we find a md25
boolean sonar_present[8];        // true if we find SRF02 Sonar
int     sonar_count = 0;         // run through each Sonar, updating.
boolean compass_present;         // true if we find a compass
int     running_code = 0;            // macro code that runs every loop.
int     error = 0;                   // 1 = no md25, 2 = hit something!

long encoder1      = 0;          // Encoder count
long encoder2      = 0;
long lastEncoder1  = 0;          // last encoder count for calculating distance traveled.
long lastEncoder2  = 0;

long count  = 0;                  // main loop counter.

float x     =0.0;                // calculated position (dead-reckoning)
float y     =0.0;
float theta =0.0;                // radians

unsigned long lastTime;          // milliseconds 

//MD25--------------------------------------------------------------------------------------------------
#define MD25ADDRESS         0x58                              // Address of the MD25
#define CMD                 0x10
#define SOFTWAREREG         0x0D                              // Byte to read the software version
#define SPEED1              (byte)0x00                        // Byte to send speed to first motor
#define SPEED2              0x01                              // Byte to send speed to second motor
#define ENCODERONE          0x02                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06                              // Byte to read motor encoder 2
#define VOLTREAD            0x0A                              // Byte to read battery volts
#define MODE                0x0F                              // Byte to write mode
#define RESETENCODERS       0x20
//SONAR--------------------------------------------------------------------------------------------------
#define SONARADDRESS        0x70
//MY REGISTERS-------------------------------------------------------------------------------------------

byte registers[128];           

#define REG_PIN             0
#define REG_PDIR            56 
#define REG_ANALOGUE        8
#define REG_PWM             24
#define REG_SERVO           40
#define REG_SONAR           56
#define REG_MD25            64
#define REG_ENCODER1        66
#define REG_ENCODER2        70
#define REG_VOLTS           74
#define REG_VERSION         77
#define REG_IR              81
#define REG_COMPASS         89
#define REG_THETA           91
#define REG_X               93
#define REG_Y               95
#define REG_COUNT           97

// pin 0 and 1 are USB serial.
#define PIN_LED1            2
#define PIN_LED2            3
#define PIN_BUMP1           4
#define PIN_BUMP2           5
#define PIN_CLIFF1          6
#define PIN_CLIFF2          7

// geometry of robot
#define WHEEL_CIRCUMFERENCE     32.55             // cm
#define WHEEL_COUNTS_PER_REV    360.0

//#define WHEEL_SPACING           23.0              // cm  full rotation currently = 5.83 radians
#define WHEEL_SPACING           22.4              // cm  need to test

#define CM_PER_TICK             (WHEEL_CIRCUMFERENCE/WHEEL_COUNTS_PER_REV)
#define FULL_CIRCLE             WHEEL_COUNTS_PER_REV*((WHEEL_SPACING*PI)/WHEEL_CIRCUMFERENCE)

void position_update() {      // calculate new X, Y, Theta 
    int left_ticks   = encoder1 - lastEncoder1;
    int right_ticks  = encoder2 - lastEncoder2;
    lastEncoder1 = encoder1;
    lastEncoder2 = encoder2;
    
    float dist_left   = (float)left_ticks * CM_PER_TICK;
    float dist_right  = (float)right_ticks * CM_PER_TICK;
    float cos_current = cos(theta);
    float sin_current = sin(theta);
    float right_minus_left = dist_right-dist_left;
    float expr1 = WHEEL_SPACING * (dist_right + dist_left) / 2.0 / (dist_right - dist_left);
         
    if (left_ticks == right_ticks) {            // Moving in a straight line 
        x += dist_left*cos_current;
        y += dist_left*sin_current;
        }
    else {                                      // Moving in an arc 
        float right_minus_left = dist_right - dist_left;
        x     += expr1 * (sin(right_minus_left/WHEEL_SPACING + theta) - sin_current);
        y     += expr1 * (cos(right_minus_left/WHEEL_SPACING + theta) - cos_current);
        theta += right_minus_left / WHEEL_SPACING;
        }
        
    if (theta<0.0)     { theta = (2*PI)+theta; }
    if (theta>=(2*PI)) { theta = theta-(2*PI); }
    
    set_register_to_int(REG_THETA, int(theta*10000.0));  // rad * 10,000
    set_register_to_int(REG_X, int(x));          // cm
    set_register_to_int(REG_Y, int(y));          // cm
    }

char hex[ ] = "0123456789ABCDEF";

void setup(){      // Initial set-up at boot
  count = 0;
  inputString.reserve(200);
  for(int i=0; i<128; i++) registers[i]=0x00;                // clear registers
  for(int i=0; i<8;  i++) sonar_present[i]=false;
  pinMode(PIN_LED1,  OUTPUT);                               // setup IO 
  pinMode(PIN_LED2,  OUTPUT);
  pinMode(PIN_BUMP1,  INPUT);
  pinMode(PIN_CLIFF1, INPUT);
  pinMode(PIN_BUMP2,  INPUT);
  pinMode(PIN_CLIFF2, INPUT);
  digitalWrite(PIN_BUMP1,  HIGH);                            // internal pull-ups
  digitalWrite(PIN_BUMP2,  HIGH);                            // internal pull-ups
  digitalWrite(PIN_CLIFF1, HIGH);                            // internal pull-ups
  digitalWrite(PIN_CLIFF2, HIGH);                            // internal pull-ups
  
  digitalWrite(PIN_LED1, HIGH);
  digitalWrite(PIN_LED2, LOW );
  
  Serial.begin(19200);                                      // start serial port 
  while (!Serial) {                                         // wait for port to open
  }
  Serial.println("!Starting wire..");
  Wire.begin();
  delay(100);                                               // Wait for everything to power up
  for(int i=0; i<128; i++) {                                 // clear registers
    registers[i]=i;
  }
  md25_present = i2c_ping(MD25ADDRESS);
  if (md25_present) {
    Serial.print("!Found MD25 @ ");
    Serial.print(MD25ADDRESS, HEX);
    md25_setup();                                             // Cals a function that resets the encoder values to 0
    md25_update();                                            // populate registers.
    Serial.print(" ver ");
    Serial.println(md25_version());
    Serial.print("!Battery : ");
    Serial.print(md25_volts());
    Serial.println("v");
  } else {
    Serial.println("!FAILED TO FIND MD25!");
  }
  for (int i=0; i<4; i++) {
    sonar_present[i] = i2c_ping(SONARADDRESS+i);
    if (sonar_present[i]) {
      Serial.print("!Found Sonar @ ");
      Serial.print(SONARADDRESS+i, HEX);
      Serial.print(" Ver ");
      Serial.println(sonar_version(i), DEC);
      Serial.print("!measurement : ");
      Serial.println(sonar_getRange(i), DEC);     
    }
  }
//  printHelp();
  lastTime = millis();
}

void loop(){          // main execution lopp
  doSerialEvent();
  if (stringComplete) {
    do_command(inputString);
    inputString = "";          //////////////////////////////////////////////////////
    stringComplete = false;    // restarts filling the buffer
    }
  if (md25_present) md25_update();
  position_update();
  pin_update();
  analogue_update();
  set_register_to_long(REG_COUNT, count++);
  run_every(100);
//  run_reflex();
}

void doSerialEvent() {        // Called on every char that appears in input buffer..
//  if (stringComplete==false) {
    while (Serial.available()) {
      char inChar = (char)Serial.read(); 
      if (inChar == '\n') stringComplete = true; else if (int(inChar)>26) inputString += inChar;
    }
//  }
}

void run_every(int m) {
  if ((millis()-lastTime)<m) return;
  lastTime = millis();
  sonar_count = (sonar_count+1)%4;
  if (sonar_present[sonar_count]) {
    digitalWrite(PIN_LED1, HIGH);  
    registers[REG_SONAR+sonar_count] = (byte)sonar_getRange(sonar_count);
  } else {
    digitalWrite(PIN_LED1, LOW);  
  }
  // Do some stuff here!!
}

void analogue_update() {                      // update registers for ananlogue inputs 0-3. 4 and 5 ARE RESERVED FOR I2C.
  int val;
  for (int i=0; i<4; i++) {
    set_register_to_int(REG_ANALOGUE, analogRead(i));
  }
}

void set_register_to_int(int r, int d) {        // break int int 2 x byte and stick in registers.
    registers[r  ] = byte(d>>8);
    registers[r+1] = byte(d);
}

void set_register_to_long(int r, long d) {        // break int int 4 x byte and stick in registers.
    registers[r  ] = byte(d>>24);
    registers[r+1] = byte(d>>16);
    registers[r+2] = byte(d>>8);
    registers[r+3] = byte(d);
}

void pin_update() {      // Set Digitial IN registers
  byte result=0x00;
  if (digitalRead(PIN_BUMP1 )==LOW) result = result|B00010000;
  if (digitalRead(PIN_BUMP2 )==LOW) result = result|B00100000;
  if (digitalRead(PIN_CLIFF1)==LOW) result = result|B01000000;
  if (digitalRead(PIN_CLIFF2)==LOW) result = result|B10000000;      // any non-0 = bump condition STOP!!
  result = result ^ B11000000;
  registers[3]=result;
}

void do_command(String command){    // Perform command received
  //Serial.println(command);
  if (isValidCommand(command)) { 
    switch(command[0]) {
      case '?' :
        do_query(command);
        break;
      case '!' :
        do_set(command);
        break;
      case '#' :
        do_run(command);
        break;
    } 
  //Serial.println("!OK");
  }
  md25_update();
  //md25_dump();
}

void do_query(String command) {       // perform a QUERY command
  int n;
  int reg = hexAt(command, 1);        // we already have chacked that length>3.
  if (command.length()<4) n = 1; else n = hexAt(command,3);
  Serial.print("=");
  for (int i=reg; i<reg+n; i++) {
    printHex(registers[i]);
  }
  Serial.println();
}

void set_register(int r, byte d) {    // Affect writing to a register, undating physical devices etc.
  if (r>63 && r<81 && md25_present) { // md25
    md25_write(r-64,d);
  }
}
    
void do_set(String command) {    // Perform a SET command
  int reg = hexAt(command, 1);   // we already have chacked that length>3.
  int n = (command.length()-3)/2;
  if (n>0) {
    for (int i=0; i<n; i++) {
      set_register(reg+i,hexAt(command,3+(i*2)));
    }
  Serial.print("=");
  printHex(reg);
  Serial.println();
  }
  else {
    Serial.println("!bad length");
  }
}

void do_run(String command) {    // Perform a MACRO command
  int reg = hexAt(command, 1);   // we already have chacked that length>3.
  switch(reg) {
/*    case(0):                     // #00   : print '=<input pins><encoder1><encoder2>'
      Serial.print("=");
      printHex(registers[3]);
      for (int i=0; i<10; i++) printHex(registers[i+REG_MD25]);
      Serial.println();
      break;
  */    
    case(1):                     // #01   : reset encoders
      md25_encoderReset();
      Serial.println("=01");
      break;
/*      
    case(2):                     // #02   : print pose, counters
      Serial.print("! ");
      Serial.print(x,2);Serial.print(" ");
      Serial.print(y,2);Serial.print(" ");
      Serial.print(theta,2);Serial.print(" ");
      Serial.print(encoder1);Serial.print(" ");
      Serial.print(encoder2);Serial.println(" ");
      break;
  */    
    case(3):                     // #03   : print snapshot of robot sensors (for md25 python robot)
      Serial.print("=");
      printHex(registers[3]);  // bump/cliff
      dumpHex(8,4);            // analogue (IR)
      dumpHex(56,2);           // Sonar
      dumpHex(64,13);          // md25 data
      dumpHex(89,8);           // Compass, bearing, x, y
      dumpHex(REG_COUNT,4);    // counter
      Serial.println();
      break;  
      
    default:
      Serial.println("!unknown macro");
  }
}

void printHex(int n) {           // print value n as HEX '01' 'AF'
  if (n<16) Serial.print("0");
  Serial.print(n,HEX);
}

void dumpHex(int i, int n)  {    // print a sequence of register values
  for (int j=i; j<i+n; j++) {
    printHex(registers[j]);
  }
}

boolean isHex(char c) {          // is a char a HEX digit?
  return (c>='0' & c<='9') || (c>='a' && c<='f') || (c>='A' && c<='F');
}

byte fromHex(char c) {           // from HEX digit to INT
  if (c>='0' & c<='9') return (byte)c-'0';
  if (c>='a' && c<='f') return 10+(byte)c-'a';
  if (c>='A' && c<='F') return 10+(byte)c-'F';
  return 0;
}

byte hexAt(String command, int i) {  // return INT value of HEX at location i in a string
  return fromHex(command[i])*16+fromHex(command[i+1]);
}  
  
boolean isValidCommand(String command) {    // Check validity of command
  if (!(command[0]=='!' || command[0]=='?' || command[0]=='#' )) {
    Serial.print("!Not command ");
    printHex(command[0]);printHex(command[1]);printHex(command[2]);printHex(command[3]);
    return false;
  }
  for (int i=1; i<command.length();i++) {
    if (!isHex(command[i])) {
      Serial.print("!Not Hex ");
      Serial.println(command);
      return false;
    }
  }
  if (command.length()%2==0 || command.length()<3) {
      Serial.print("!Bad length ");
      Serial.println(command);
      return false;
    }
  return true;  
}
  
boolean i2c_ping(int address){              // returns True if i2c address responds
  Wire.beginTransmission(address);           
  Wire.write(0x00);
  return (Wire.endTransmission()==0);
}

void md25_update(){                         // full read from md52
  int i;
  Wire.beginTransmission(MD25ADDRESS);           
  Wire.write(0x00);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25ADDRESS, 16);                        // Request 16 bytes from MD25
  while(Wire.available() < 16);                              // Wait for 16 bytes to become available
  for (i=0; i<16; i++) registers[REG_MD25+i] = Wire.read();  // read into registers.
  encoder1 = (registers[REG_ENCODER1]<<24) + (registers[REG_ENCODER1+1]<<16) + (registers[REG_ENCODER1+2]<<8) + (registers[REG_ENCODER1+3]);
  encoder2 = (registers[REG_ENCODER2]<<24) + (registers[REG_ENCODER2+1]<<16) + (registers[REG_ENCODER2+2]<<8) + (registers[REG_ENCODER2+3]);  
}

float md25_volts(){                                 // Battery volts 
  return (float)registers[REG_VOLTS]/10.0;  
}
int md25_version(){                                 // MD25 Version 
  return (int)registers[REG_VERSION];
}

void md25_move(int sp, int rot) {
  md25_write(SPEED1,sp);
  md25_write(SPEED2,rot);
}  
  
void md25_stopMotor(){                              // Stop motors in mode 0 or 2
    md25_move(0x80, 0x80);
//  md25_write(SPEED1,128);
//  md25_write(SPEED2,128);
}  

void md25_write(int reg, byte value){               // write byte to md25
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void md25_setup(){                                  // Set-up for MD25
  md25_stopMotor();
  md25_encoderReset();
  md25_write(MODE, 0x02);                           // set to SPEED + TURN, centred at 128        
}

void md25_encoderReset(){                           // Resets the encoder values to 0
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(CMD);
  Wire.write(0x20);                                 // Putting the value 0x20 to reset encoders
  Wire.endTransmission(); 
  set_register_to_int(REG_THETA, 0);
  set_register_to_int(REG_X, 0);
  set_register_to_int(REG_Y, 0);
  theta = 0.0;
  x = 0.0;
  y = 0.0;
  }

int sonar_version(int addr){                        // Function to get software revision addr = 0..3
  Wire.beginTransmission(SONARADDRESS+addr);        // Begin communication with the SRF module
  Wire.write(0x00);                                 // Sends the command bit, when this bit is read it returns the software revision
  Wire.endTransmission(); 
  Wire.requestFrom(SONARADDRESS+addr, 1);           // Request 1 byte
  while(Wire.available() < 0);                      // While byte available
  int software = Wire.read();                       // Get byte
    
  return(software);                               
}

int sonar_getRange(int addr){                       // This function gets a ranging from the SRF02.
  addr = addr + SONARADDRESS;                       // address is 0..3
  int range = 0; 
  Wire.beginTransmission(addr);                     // Start communticating with SRF08
  Wire.write(0x00);                                 // Send Command Byte
  Wire.write(0x51);                                 // Send 0x51 to start a ranging
  Wire.endTransmission();
  delay(100);                                       // Wait for ranging to be complete
  Wire.beginTransmission(addr);                     // start communicating with SRFmodule
  Wire.write(0x02);                                 // Call the register for start of ranging data
  Wire.endTransmission();
  Wire.requestFrom(addr, 2);                        // Request 2 bytes from SRF module
  while(Wire.available() < 2);                      // Wait for data to arrive
  range = Wire.read()<<8;                           // Get high byte
  range += Wire.read();                             // Get low byte
  return(range);                                    // Returns Range
}
/*
void printHelp() {
  Serial.println("!HELP\n!'?rrnn' read nn registers, starting at rr. returns '=dd*'.\n!'!rrnn+' write to one or more registers. returns '=rr'.\n!'#nndd*' run macro nn with arguments dd. Returns '=nn'.");
  Serial.println("!all commands return a string. '!xxxx' is normally a warning. '=xxxx' is a result.");
}
*/

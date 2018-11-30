// Library can be found here: http://www.pjrc.com/teensy/td_libs_Encoder.html
#include <Encoder.h>

// Heartbeat
#define HEARTBEAT_MS 1000
unsigned long last_heartbeat;

// Wheel 1
#define WHEEL_1_PWM 44
#define WHEEL_1_DIR 35
Encoder wheel_1_encoder(21, 32);
int wheel_1_pwm;
int wheel_1_dir;
int new_wheel_1_count;
int wheel_1_count;

// Wheel 2
#define WHEEL_2_PWM 46
#define WHEEL_2_DIR 27
Encoder wheel_2_encoder(20, 24);
int wheel_2_pwm;
int wheel_2_dir;
int new_wheel_2_count;
int wheel_2_count;

// Wheel 3
#define WHEEL_3_PWM 45
#define WHEEL_3_DIR 16
Encoder wheel_3_encoder(18, 17);
int wheel_3_pwm;
int wheel_3_dir;
int new_wheel_3_count;
int wheel_3_count;

// Wheel 4
#define WHEEL_4_PWM 6
#define WHEEL_4_DIR 5
Encoder wheel_4_encoder(2, 4);
int wheel_4_pwm;
int wheel_4_dir;
int new_wheel_4_count;
int wheel_4_count;

//The command queue, ech command is devided with a comma. The command looks either like i.e. "t 90" or "m 1.5" or "g 10". 
String commandQueue;
//Flag to mark the decimal point.
boolean decimal = false;
//Flag to mark the beginning of a message
boolean messageBegin = false;
//Flag to show, that leading zeroes were deleted.
boolean deletedZero = false;

//Accuracy for the distance parameter
int decimalAccuracy = 2;
int decimalCounter = 0;
//Stores the last read character, needed to distinguish between end of message and random char.
char lastChar;
//Kills the loops if the runntime is longer, than the arduino buffer.
int breakCounter = 0;
//Used for debug only

boolean finished = false;
boolean leftCurve = false;
boolean rightCurve = false;
boolean continuous_motion = false;

void setup() {
  
  Serial.begin(115200);
  // Setup Pins
  pinMode(WHEEL_1_PWM, OUTPUT);
  pinMode(WHEEL_1_DIR, OUTPUT);
  pinMode(WHEEL_2_PWM, OUTPUT);
  pinMode(WHEEL_2_DIR, OUTPUT);
  pinMode(WHEEL_3_PWM, OUTPUT);
  pinMode(WHEEL_3_DIR, OUTPUT);
  pinMode(WHEEL_4_PWM, OUTPUT);
  pinMode(WHEEL_4_DIR, OUTPUT);

}

void loop() {

  checkHeartbeat();
  checkIncomingMessage();
  int commaPosition = commandQueue.indexOf(',');
  String command = commandQueue.substring(0, commaPosition);
  commandQueue.remove(0, commaPosition + 1);
  int beginOfNumber = command.indexOf(' ');
  String commandType = command.substring(0, beginOfNumber);
  command.remove(0, beginOfNumber + 1);
  if(commandType == "t"){
    float angle = command.toFloat();
    turning(angle);
  } else if (commandType == "g"){
    int beginOfNumber = command.indexOf(' ');
    float speed1 = command.substring(0, beginOfNumber).toFloat();
    command.remove(0, beginOfNumber + 1);
    float speed2 = command.toFloat();
    go(speed1, speed2);
  } else if (commandType == "m"){
    float distance = command.toFloat();
    moving(distance);
  }
  
}

void checkHeartbeat(){
  if (last_heartbeat + HEARTBEAT_MS <= millis())
    {
        // Set hearbeat and reset timer
        Serial.print("x ");
        Serial.print( millis()/1000 );
        last_heartbeat = millis();
        Serial.println(" n");
    }
}

void checkIncomingMessage(){
  if(Serial.peek() != -1){
    finished = false;
    parseMessageNew();
  }
}



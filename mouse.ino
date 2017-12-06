//changed velocity_left to .9 in mouse_helpers

#include <PID_v1.h>

#define PIN_DISTANCE_LEFT A0
#define PIN_DISTANCE_CENTER A6
#define PIN_DISTANCE_RIGHT A7

#define PIN_LED1 A3
#define PIN_LED2 A2
#define PIN_BUTTON A1

#define PIN_ENCODER_LEFT_A 11
#define PIN_ENCODER_LEFT_B 2
#define PIN_ENCODER_RIGHT_A 12
#define PIN_ENCODER_RIGHT_B 3

#define PIN_MOTOR_LEFT_1 9
#define PIN_MOTOR_LEFT_2 10
#define PIN_MOTOR_RIGHT_1 5
#define PIN_MOTOR_RIGHT_2 6

#define TICKS_PER_DEGREE 1.6299019607843137

// Invert encoder directions if needed
const boolean INVERT_ENCODER_LEFT = true;
const boolean INVERT_ENCODER_RIGHT = true;

// Invert motor directions if needed
const boolean INVERT_MOTOR_LEFT = true  ;
const boolean INVERT_MOTOR_RIGHT = false;

// Loop count, used for print statements
int count = 0;

//PID controller variables (linear)
double velocity_linear_setpoint;
double velocity_linear;
double velocity_linear_power;

//PID controller variable(angluar)
double velocity_angular_setpoint;
double velocity_angular;
double velocity_angular_power;

//PID controller variables(angular setpoint)
double diffDistance;
double diffDistanceSetpoint;

//PID controller variables (turn controller)
double tickCount;
double tickSetpoint;

int sum = 1;
//PID controller  
PID velocity_linear_pid(&velocity_linear, &velocity_linear_power, &velocity_linear_setpoint, .003, 0.0005, 0.00, DIRECT);
PID velocity_angular_pid(&velocity_angular, &velocity_angular_power, &velocity_angular_setpoint, 0.5, 0.05, 0.0, DIRECT);
PID angular_setpoint_controller(&diffDistance, &velocity_angular_setpoint, &diffDistanceSetpoint, 0.05, 0.003, 0.0, DIRECT);
PID turn_controller(&tickCount, &velocity_angular_setpoint, &tickSetpoint, 0.005, 0.0, 0.0, DIRECT);
  
void setup() {
 pinMode(PIN_LED1, OUTPUT);
 pinMode(PIN_LED2, OUTPUT);
 Serial.begin(9600);
 //set out initial variables
 velocity_linear = 0;
 velocity_linear_setpoint = 800.0;
 velocity_angular = 0;
 velocity_angular_setpoint = 0.0;
 diffDistanceSetpoint = 0.0;
 
 //setup the controller
 velocity_linear_pid.SetOutputLimits(-1.0, 1.0); 
 velocity_linear_pid.SetSampleTime(10);
 velocity_linear_pid.SetMode(AUTOMATIC);
 velocity_angular_pid.SetOutputLimits(-1.0, 1.0);
 velocity_angular_pid.SetSampleTime(10);
 velocity_angular_pid.SetMode(AUTOMATIC);
 angular_setpoint_controller.SetOutputLimits(-10,10);
 angular_setpoint_controller.SetSampleTime(10);
 angular_setpoint_controller.SetMode(AUTOMATIC);
 turn_controller.SetOutputLimits(-10, 10);
 turn_controller.SetSampleTime(10);
 turn_controller.SetMode(AUTOMATIC);
 
 pinSetup();

}

void loop() {
 if (sum % 1000 == 0)
 {
  //
 }
 sum += 1;

 // How fast are we going? in mm/sec and rad/sec
 velocity_linear = getLinearVelocity();
 velocity_angular = getAngularVelocity();
 double distanceRight = readDistanceRight();
 double distanceLeft = readDistanceLeft();
 diffDistance = distanceLeft - distanceRight;
 int nextTurn;
 
 if(distanceLeft < 10 && distanceRight < 10){
  //wall
  digitalWrite(PIN_LED1, HIGH);
  digitalWrite(PIN_LED2, HIGH);
  velocity_linear_power = 0.0;
  //turn(90);
 }else{
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, LOW);
  angular_setpoint_controller.Compute();
  velocity_linear_pid.Compute();
  velocity_angular_pid.Compute();
 }

 applyPowerLeft(velocity_linear_power - velocity_angular_power);
 applyPowerRight(velocity_linear_power + velocity_angular_power);


 count++;

 checkEncodersZeroVelocity();
 updateDistanceSensors();
}

void turn(double degrees){
  resetTicksLeft();
  resetTicksRight();
  
  int direc = degrees / abs(degrees);
  tickSetpoint = TICKS_PER_DEGREE * abs(degrees);
  tickCount = getTicksRight();
  
  double abs_error = abs(tickCount - tickSetpoint);
  
  while (abs_error > 20){
    Serial.print("Tick Count: ");
    Serial.println(tickCount);
    Serial.print("Tick Setpoint: ");
    Serial.println(tickSetpoint);
    
    tickCount = getTicksRight();
    velocity_angular = getAngularVelocity();
    turn_controller.Compute();
    velocity_angular_pid.Compute();

    applyPowerLeft(-1 * velocity_angular_power);
    applyPowerRight(velocity_angular_power);

    abs_error = abs(tickCount - tickSetpoint);
  }
}


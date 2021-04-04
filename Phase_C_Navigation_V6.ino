#include <PID_v1.h>
#include <DueTimer.h>
#include <math.h>
#include <Servo.h>

//Robot specs
float const pi = 3.14159; // value of pi
float dia = 5.7; // diameter of the wheel in cm.
float LENGTH = 12.8; // wheel base length in cm
double PPR1 = 1490; //pulses per revolution
double PPR2 = 1610; //pulses per revolution
float Rad_to_deg = 180.0 / PI;

//pin specifications
int const enA = 5; //PWM signal pin for motor 1
int const in1 = 4; // direction for motor 1 - right side from the back
int const EncoderA1 = 2; // phase A for motor 1
int const EncoderB1 = 3; // phase B for motor 1

int const enB = 6; //PWM signal pin for motor 2
int const in2 = 7; // direction for motor 2 - left side from the back
int const EncoderA2 = 8; // phase A for motor 2
int const EncoderB2 = 9; // phase B for motor 2

// mathematical variables
volatile int encoderValueA = 0; //pulse count from encoder1
volatile int encoderValueB = 0; //pulse count from encoder1
int interval = 50; // interval for measurement - milliseconds
int SEND_INTERVAL = 100; // interval for sending information - milliseconds
volatile long prevIntegrationTime = 0;
volatile float xc, yc, bx1, by1, bx2, by2, cx1, cy1, cx2, cy2, theta, deg;
volatile float poseX, poseY;
volatile float yawNoise = 0;
bool flag = true;


//counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

//variable for RPM and speed measurement
int rpml = 0;
int rpmr = 0;
double Vl = 0;
double Vr = 0;
volatile float distance = 0;
int CW1, CW2; //CW signifies negative direction of rotation

//PID parameters

double Kp = 2.8, Ki = 2, Kd = 0.5;
double Setpoint1, Setpoint2, Output1, Output2; //output 1 for corrected speed 1 and output 2 for corrected speed 2
PID myPID1(&Vl, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);
PID myPID2(&Vr, &Output2, &Setpoint2, Kp, Ki, Kd, DIRECT);

// Servo motor parameters

Servo myservo;//gripper
Servo myservo2;//elbow

int servo_pwm_pin = 11;  // The pin for gripper
int servo_pwm_pin2 = 12; // The pin for the elbow

// You make make both of these values to stay at the same position
int lower_angle_limit_elbow = 50; // Change this lower limit value to what u what
int upper_angle_limit_elbow = 100; // Change this upper limit value to what u what

int lower_angle_limit_gripper = -5; // Change this lower limit value to what u what
int upper_angle_limit_gripper = 180;

int i = 1;
int present_angle = lower_angle_limit_gripper;
int present_angle2 = lower_angle_limit_elbow;


// Increment value for each pulse from encoder

void updateEncoderA()
{
  encoderValueA++;
}

void updateEncoderB()
{
  encoderValueB++;
}

/*#######################################################################

                          Setup

  ########################################################################*/

void setup()
{
  // Setup Serial Monitor
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial.setTimeout(5); //blocks readin from opencv all together
  Serial1.setTimeout(5); //blocks reading from open cv

  // Set encoder as input with internal pullup
  pinMode(enA, OUTPUT); // motor 1 pin initialization
  pinMode(in1, OUTPUT); // motor 1 phase direction initiazation
  pinMode(enB, OUTPUT); // motor 2 pin initialization
  pinMode(in2, OUTPUT); // motor 2 direction initialization

  //turn the PID on
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(EncoderA1), updateEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(EncoderA2), updateEncoderB, RISING);
  Timer7.attachInterrupt(Odometry);
  Timer7.start(50000);

  //  Setup servo motors
  myservo.attach(servo_pwm_pin);
  myservo2.attach(servo_pwm_pin2);
  myservo.write(lower_angle_limit_gripper);
  myservo2.write(lower_angle_limit_elbow);
}



/*#######################################################################

                          main loop

  ########################################################################*/

void loop() {
  // Only Run This Once
  if (flag) {
    // Stays in this loop until it finds the 2*4 block location
    while ((bx1 == 0 && by1 == 0) || (bx2 == 0 && by2 == 0)) {
      compVisionMessage();
    }

    // Stays in this loop until it finds the 2*2 block location
    while ((cx1 == 0 && cy1 == 0) || (cx2 == 0 && cy2 == 0) ) {
      compVisionMessage();
    }

    // This function will take care of the task of finding the closet block and droping
    // it in the correct location

    orderForBlockPickup();


    go_location(5, 0, 20, 5); // Go Back to the origin location
    turn_Robot(heading_delta(int(deg), 0)); // Orignal Orientation
  }
  flag = false;
}

/*#######################################################################

                           orderForBlockPickup method
         This method will find the optimal block to pickup and
              drop it off in the correct drop off location

  ########################################################################*/
void orderForBlockPickup() {
  float block_pickup_info[4][3] = {{bx1, by1,  0}, {bx2, by2,  0}, {cx1, cy1,  0}, {cx2, cy2,  0}};

  for (int i = 0; i < 4; i++) {
    int lowest_distance_index = 0;
    float present_lowest_distance = 500;
    for (int j = 0; j < 4; j++) {
      if (block_pickup_info[j][2] == 1) {
        continue;
      }
      float current_block_dis = calculate_distance(xc, yc, block_pickup_info[j][0], block_pickup_info[j][1]);
      if (current_block_dis < present_lowest_distance) {
        lowest_distance_index = j;
        present_lowest_distance = current_block_dis;
      }
    }

    if (lowest_distance_index < 2) {
      //pick_block_drop(block_pickup_info[lowest_distance_index][0], block_pickup_info[lowest_distance_index][1], 135, 173); // 2*4 Block Drop Off
      pick_block_drop(block_pickup_info[lowest_distance_index][0], block_pickup_info[lowest_distance_index][1], 150 ,200 ); // 2*4 Block Drop Off
    }
    else {
      //pick_block_drop(block_pickup_info[lowest_distance_index][0], block_pickup_info[lowest_distance_index][1], 135, 0); // 2*2 Block Drop Off
      pick_block_drop(block_pickup_info[lowest_distance_index][0], block_pickup_info[lowest_distance_index][1], 152, -4); // 2*2 Block Drop Off

    }
    block_pickup_info[lowest_distance_index][2] = 1;
  }
}

/*#######################################################################

                           pick_block_drop method

  ########################################################################*/

void pick_block_drop(int goalx, int goaly, int dropx, int dropy) {
  int angleTurn;
//  Gripper("open");
//  delay(500);
  elbow("up");
  go_location(goalx, goaly, 60, 20); // Pickup location

  for (int i = 0; i < 5; i++) {
    angleTurn = robot_turn_angle(xc, yc, goalx, goaly);
    turn_Robot2(angleTurn);
  }
  Gripper("open");
  delay(100);
  elbow("down");
  delay(100);
  forward(12);


  Gripper("close");
  delay(500);
  elbow("up");
  go_location(dropx, dropy, 20, 10); // Drop off location
//  angleTurn = robot_turn_angle(xc, yc, goalx, goaly);
//  turn_Robot(angleTurn);
  //  forward(20);
  
  elbow("down");
  delay(500);
  Gripper("open");
  delay(500);
  elbow("up");
  Reverse(10); // Reverse back for 15 cm
}

/*#######################################################################

                           Forward

  ########################################################################*/
void forward(int distance_to_run) {
  int distance1 = distance;
  Setpoint1 = 10;
  Setpoint2 = 10;
  CW1 = -1;
  CW2 = 1;
  digitalWrite(in1, CW1 == 1);
  digitalWrite(in2, CW2 == 1);
  while (abs(distance1 - distance) < distance_to_run) {
    myPID1.Compute();
    analogWrite(enA, Output1);
    myPID2.Compute();
    analogWrite(enB, Output2);
    delay(100);
  }
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}
/*#######################################################################

                           Reverse

  ########################################################################*/

void Reverse(int distance_to_run) {
  int distance1 = distance;
  Setpoint1 = 10;
  Setpoint2 = 10;
  CW1 = 1;
  CW2 = -1;
  digitalWrite(in1, CW1 == 1);
  digitalWrite(in2, CW2 == 1);
  while (abs(distance1 - distance) < distance_to_run) {
    myPID1.Compute();
    analogWrite(enA, Output1);
    myPID2.Compute();
    analogWrite(enB, Output2);
    delay(100);
  }
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

/*#######################################################################

                           go_location

  ########################################################################*/
void go_location(int goalx, int goaly, int small, int stopdistance) {
  compVisionMessage(); // Reads From the Comp Multi Times
  float angleTurn = robot_turn_angle(xc, yc, goalx, goaly);
  turn_Robot(angleTurn);
  float distance_to_goal = calculate_distance(xc, yc, goalx, goaly);
  while ( distance_to_goal > stopdistance) {
    angleTurn = robot_turn_angle(xc, yc, goalx, goaly);
    double steer = 5 * (angleTurn / 90.0);

    if ( abs(angleTurn) > 60) {
      // Turn the Robot
      turn_Robot(angleTurn);
    }

    else {

      float stra_speed = map(distance_to_goal, small, 100, 8, 10); // 6,8 the best value so far
      float angular_aggressive = map(distance_to_goal, small, 100, 3, 2); // it was 3, 2

      // Wheel Speed Control
      Setpoint1 = stra_speed - steer * angular_aggressive;
      Setpoint2 = stra_speed + steer * angular_aggressive;

      if (Setpoint1 > 0) {
        CW1 = -1;
      }
      else {
        Setpoint1 = abs(Setpoint1);
        CW1 = 1;
      }

      if (Setpoint2 > 0) {
        CW2 = 1;
      }

      else {
        Setpoint2 = abs(Setpoint2);
        CW2 = -1;
      }

      digitalWrite(in1, CW1 == 1);
      digitalWrite(in2, CW2 == 1);
      myPID1.Compute();
      analogWrite(enA, Output1);
      myPID2.Compute();
      analogWrite(enB, Output2);
    }
    compVisionMessage();
    distance_to_goal = calculate_distance(xc, yc, goalx, goaly);
  }
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  compVisionMessage();
  delay(500);

}
/*#######################################################################

                                      turn Robot

  ########################################################################*/

void turn_Robot(int turnAngle) {
  // Right Turn
  if (turnAngle > 0) {
    CW1 = 1;
    CW2 = 1;
  }
  //Left Turn
  else {
    CW1 = -1;
    CW2 = -1;
  }

  float turn_heading = deg + turnAngle;
  Setpoint1 = 6;
  Setpoint2 = 6;

  digitalWrite(in1, CW1 == 1);
  digitalWrite(in2, CW2 == 1);

  float turnError = heading_delta((int)deg, (int)turn_heading);
  while ( !((-5 < turnError) && (5 > turnError)) ) {
    myPID1.Compute();
    analogWrite(enA, Output1);
    myPID2.Compute();
    analogWrite(enB, Output2);
    compVisionMessage();
    turnError = heading_delta((int)deg, (int)turn_heading);
  }
  digitalWrite(enA, 0);
  digitalWrite(enB, 0);
  Setpoint1 = 0;
  Setpoint2 = 0;
  myPID1.SetTunings(Kp, Ki, Kd);
  myPID2.SetTunings(Kp, Ki, Kd);
  delay(500);
}

/*#######################################################################

                                      turn Robot 2

  ########################################################################*/

void turn_Robot2(int turnAngle) {
  // Right Turn
  
  if (turnAngle > 0) {
    CW1 = 1;
    CW2 = 1;
  }
  //Left Turn
  else {
    CW1 = -1;
    CW2 = -1;
  }

  float turn_heading = deg + turnAngle;
//  Setpoint1 = 7;
//  Setpoint2 = 7;

  digitalWrite(in1, CW1 == 1);
  digitalWrite(in2, CW2 == 1);
  
  float turnError = heading_delta((int)deg, (int)turn_heading);
  float starting_error=turnError;
  while ( !((-2 < turnError) && (2 > turnError)) ) {
    if ((abs(starting_error)+1)<(turnError)){
      break;
    }
//    myPID1.Compute();
    analogWrite(enA, 155);
//    myPID2.Compute();
    analogWrite(enB, 155);
    for (int i = 0; i< 6; i++){
      compVisionMessage();
    }
     digitalWrite(enA, 0);
     digitalWrite(enB, 0);
    for (int i = 0; i< 15; i++){
      compVisionMessage();
    }
//    compVisionMessage();
    turnError = heading_delta((int)deg, (int)turn_heading);
  }
  digitalWrite(enA, 0);
  digitalWrite(enB, 0);
  Setpoint1 = 0;
  Setpoint2 = 0;
  myPID1.SetTunings(Kp, Ki, Kd);
  myPID2.SetTunings(Kp, Ki, Kd);
  delay(500);
}
/*#######################################################################

                          robot_turn_angle

  ########################################################################*/

float robot_turn_angle(float x1, float y1, float x2, float y2) {
  float goalHeading = heading_direction(x1, y1, x2, y2);
  float turnDirection = heading_delta((int)deg, (int)goalHeading);
  return turnDirection;
}

/*#######################################################################

                          heading_direction

  ########################################################################*/
float heading_direction(float x1, float y1, float x2, float y2) {
  float delta_y = y2 - y1;
  float delta_x = x2 - x1;
  float heading_rad = atan2(delta_x, delta_y);
  float heading_deg = Rad_to_deg * heading_rad;
  heading_deg = ((int) heading_deg) % 360;
  if (heading_deg < 0) {
    heading_deg += 360;
  }
  return heading_deg;
}

/*  ######################################################################

                          heading_delta

  ########################################################################*/

float heading_delta(int robotHeading, int goalHeading) {
  robotHeading %= 360;
  if (robotHeading < 0) {
    robotHeading += 360;
  }
  goalHeading %= 360;
  float delta1 = goalHeading - robotHeading;
  float delta2 = 360 - abs(delta1);

  float delta;
  if (abs(delta1) > abs(delta2)) {
    if (delta1 > 0) {
      delta = (-1) * delta2;
    }
    else {
      delta = delta2;
    }
  }
  else {
    delta = delta1;
  }

  return delta;
}

/*########################################################################

                          calculate_distance

  ########################################################################*/

float calculate_distance(float x1, float y1, float x2, float y2) {
  float dumydistance = sqrt( pow((x2 - x1), 2) + pow((y2 - y1), 2));
  return dumydistance;
}


/*  ######################################################################

                          compVisionMessageProcess

  ########################################################################*/
void compVisionMessage() {
  String Useful_1 = Serial1.readStringUntil('\n');
  compVisionPose(Useful_1);
  //    Serial.print("xc: "); Serial.print(xc);
  //    Serial.print(" yc: "); Serial.print(yc);
  //    Serial.print(" deg: "); Serial.println(deg);
}

/*  ######################################################################

                          compVisionPose

  ########################################################################*/

void compVisionPose(String Text_To_Read ) {
  if (Text_To_Read != "") {
    char action = Text_To_Read.charAt(0);
    String message = Text_To_Read.substring(1);

    switch (action) {
      case 'P':
        int commaIndex[2];
        commaIndex[0] = message.indexOf(',');
        commaIndex[1] = message.indexOf(',', commaIndex[0] + 1);
        xc = message.substring(0, commaIndex[0]).toInt() / 1.0;
        yc = message.substring(commaIndex[0] + 1, commaIndex[1]).toInt() / 1.0;
        deg = -(message.substring(commaIndex[1] + 1).toInt());
        break;

      case 'B': //bigger blocks
        int commaIndexb[3];
        commaIndexb[0] = message.indexOf(',');
        commaIndexb[1] = message.indexOf(',', commaIndexb[0] + 1);
        commaIndexb[2] = message.indexOf(',', commaIndexb[1] + 1);
        bx1 = message.substring(0, commaIndexb[0]).toInt() / 1.0;
        by1 = message.substring(commaIndexb[0] + 1, commaIndexb[1]).toInt() / 1.0;
        bx2 = message.substring(commaIndexb[1] + 1, commaIndexb[2]).toInt();
        by2 = message.substring(commaIndexb[2] + 1).toInt();
        break;

      case 'C': //smaller blocks
        int commaIndexc[3];
        commaIndexc[0] = message.indexOf(',');
        commaIndexc[1] = message.indexOf(',', commaIndexc[0] + 1);
        commaIndexc[2] = message.indexOf(',', commaIndexc[1] + 1);
        cx1 = message.substring(0, commaIndexc[0]).toInt() / 1.0;
        cy1 = message.substring(commaIndexc[0] + 1, commaIndexc[1]).toInt() / 1.0;
        cx2 = message.substring(commaIndexc[1] + 1, commaIndexc[2]).toInt();
        cy2 = message.substring(commaIndexc[2] + 1).toInt();
        break;

      default:
        break;
    }
  }
}

/*  ######################################################################

                            Odometry

  ########################################################################*/
void Odometry() {
  unsigned long dt_integration = micros() - prevIntegrationTime;
  float dt = dt_integration / 1000000.0; // convert to seconds

  if (encoderValueA > 0 || encoderValueB > 0) { //56 to avoid noise
    //Calculate RPM
    rpml = (float)((encoderValueA) * 60 / (PPR1 * ((micros() - previousMillis) / 1000000.0)));
    rpmr = (float)((encoderValueB) * 60 / (PPR2 * ((micros() - previousMillis) / 1000000.0)));
    Vl = (float)(rpml * PI * dia / 60);
    Vr = (float)(rpmr * PI * dia / 60);
    distance = distance + (Vl * ((micros() - previousMillis) / 1000000.0));

    prevIntegrationTime = micros();
    encoderValueA = 0;
    encoderValueB = 0;
    previousMillis = micros();
  }

}


/*  ######################################################################

                            Gripper Controlling function

  ########################################################################*/
void Gripper(String state) {
  if (state == "open") {
    while (present_angle <= upper_angle_limit_gripper) {
      present_angle = present_angle + 5;
      myservo.write(present_angle);
      delay(10);
    }
  }
  else {
    while (present_angle >= lower_angle_limit_gripper) {
      present_angle = present_angle - 5;
      myservo.write(present_angle);
      delay(50);
    }
  }
}

/*  ######################################################################

                            Elbow Controlling function

  ########################################################################*/

void elbow(String state) {
  if (state == "up") {
    while (present_angle2 <= upper_angle_limit_elbow) {
      present_angle2 = present_angle2 + 5;
      myservo2.write(present_angle2);
      delay(10);
    }

  }
  else {
    while (present_angle2 >= lower_angle_limit_elbow) {
      present_angle2 = present_angle2 - 5;
      myservo2.write(present_angle2);
      delay(10);
    }
  }
}

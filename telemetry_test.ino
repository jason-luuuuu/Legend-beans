//adjust FINISH_ERROR, CORRECTION_FACTOR

#include <Wire.h>
#include <math.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

double current_latitude;
double current_longitude;

double GOAL_LATITUDE = -31.979905653394606;
double GOAL_LONGITUDE = 115.81784769629023;
double start_latitude;
double start_longitude;
double old_latitude;
double old_longitude;
bool has_reached_goal;

int STRAIGHT_TIME = 500;

double FINISH_ERROR = 0.000005;
int CORRECTION_FACTOR = 5;

int current_bearing;
int goal_bearing;
int driving_angle;

int drive = 5;
int left = 6;
int right = 7;

int RXPin = 2;
int TXPin = 3;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup() {

  Serial.begin(9600);
  gpsSerial.begin(9600);
  has_reached_goal = false;
  
  pinMode(drive, OUTPUT);
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);
  digitalWrite(left, LOW);
  digitalWrite(right, LOW);

  bool finished = false;
  while (!finished) {
    if (gpsSerial.available() > 0)
    {
      if (gps.encode(gpsSerial.read()))
      {
        if (gps.location.isValid())
        {
          start_latitude = gps.location.lat();
          start_longitude = gps.location.lng();
          finished = true;
          Serial.println("GPS connected");
        }
      }
    }
  }
  digitalWrite(drive,HIGH);

  Serial.println("Starting to go towards target");
}

int get_bearing_from(double current_latitude, double current_longitude, double goal_latitude, double goal_longitude) {
  double lat_diff = goal_latitude-current_latitude;
  double lng_diff = goal_longitude-current_longitude;
  int output;
  if (lat_diff>=0 && lng_diff>=0){
    output = round( atan (lng_diff/lat_diff) * 180/3.14159265 );
  }
  else if (lat_diff<=0 && lng_diff<=0){
    output = 180 + round( atan (lng_diff/lat_diff) * 180/3.14159265 );
  }
  else if (lat_diff>0 && lng_diff<0){
    output = 360 + round( atan (lng_diff/lat_diff) * 180/3.14159265 );
  }
  else if (lat_diff<0 && lng_diff>0){
    output = 180 + round( atan (lng_diff/lat_diff) * 180/3.14159265 );
  }
  return(output);
}

int get_current_bearing() {
  Serial.println("getting bearing");
  bool finished = false;
  while (!finished) {
    if (gpsSerial.available() > 0)
    {
      if (gps.encode(gpsSerial.read()))
      {
        if (gps.location.isValid())
        {
          old_latitude = gps.location.lat();
          old_longitude = gps.location.lng();
          finished = true;
        }
      }
    }
  }
  digitalWrite(left,LOW);
  digitalWrite(right,LOW);
  delayCheck(STRAIGHT_TIME);
  finished = false;
  while (!finished) {
    if (gpsSerial.available() > 0)
    {
      if (gps.encode(gpsSerial.read()))
      {
        if (gps.location.isValid())
        {
          current_latitude = gps.location.lat();
          current_longitude = gps.location.lng();
          if (old_latitude != current_latitude || old_longitude != current_longitude) {
            finished = true;
          }
        }
      }
    }
  }
  int output = get_bearing_from(old_latitude, old_longitude, current_latitude, current_longitude);
  return output;
}

void delayCheck(unsigned long millisecond_sleep) {
  unsigned long starting_time = millis();
  while (starting_time + millisecond_sleep > millis()) {
    if (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        if (gps.location.isValid()) {
          double now_latitude = gps.location.lat();
          double now_longitude = gps.location.lng();
          if ((abs(GOAL_LATITUDE-now_latitude)<FINISH_ERROR) && (abs(GOAL_LONGITUDE-current_longitude)<FINISH_ERROR)) {
            Serial.println("reached destination");
            if (GOAL_LATITUDE == start_latitude && GOAL_LONGITUDE == start_longitude) {
              digitalWrite(drive,LOW);
              digitalWrite(left,LOW);
              digitalWrite(right,LOW);
              while (1) {
                delay(2000);
              }
            } else if (has_reached_goal == false) {
              digitalWrite(right,HIGH);
              digitalWrite(left,LOW);
              delay(3000);
              digitalWrite(right,LOW);
              has_reached_goal = true;
            }
            GOAL_LATITUDE = start_latitude;
            GOAL_LONGITUDE = start_longitude;
          }
        }
      }
    }
  }
}

void loop() {
        //check if car is at goal location
//        if ((abs(GOAL_LATITUDE-current_latitude)<FINISH_ERROR) && (abs(GOAL_LONGITUDE-current_longitude)<FINISH_ERROR)) {
//          Serial.print("reached destination");
//          GOAL_LATITUDE = start_latitude;
//          GOAL_LONGITUDE = start_longitude;
//        }

        //get current heading, bearing from car to goal, and the driving angle required to point car towards goal
        current_bearing = get_current_bearing();
        goal_bearing = get_bearing_from(current_latitude, current_longitude, GOAL_LATITUDE, GOAL_LONGITUDE);
        driving_angle = (goal_bearing - current_bearing + 360)%360;

        Serial.print("current bearing: ");
        Serial.println(current_bearing);
        Serial.print("goal bearing: ");
        Serial.println(goal_bearing);
        Serial.print("driving angle: ");
        Serial.println(driving_angle);

        Serial.println("executing turn");

        //turn car towards goal
        if ((driving_angle >= 0) && (driving_angle < 180)) {
          Serial.print("Turn right ");
          Serial.println(driving_angle);
          digitalWrite(left, LOW);
          digitalWrite(right, HIGH);
          delayCheck(driving_angle*CORRECTION_FACTOR);
          digitalWrite(right,LOW);
        }
        else if ((driving_angle >= 180) && (driving_angle < 360)) {
          Serial.print("Turn left ");
          Serial.println(360-driving_angle);
          digitalWrite(right, LOW);
          digitalWrite(left, HIGH);
          delayCheck((360-driving_angle)*CORRECTION_FACTOR);
          digitalWrite(left,LOW);
        }
        else {
          digitalWrite(right, LOW);
          digitalWrite(left, LOW);
          delay(1000);
        }
        

        Serial.println();
        Serial.println();
}

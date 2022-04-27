#include "Arduino.h"
#include "TinyGPS++.h"
#include "ESP32Servo.h"
#include "GpsWayPoint.h"
#include "Wire.h"
#include "QMC5883LCompass.h"
#include "PID.h"
#include "HardwareSerial.h"

// #define TESTING

#define RXD2 9
#define TXD2 10


#define ONBOARD_LED 26
#define ONBOARD_BUTTON 27
#define ONBOARD_BUTTON_DEBOUNCE_SENSITIVITY 250
#define ONBOARD_BUTTON_MULTIPRESS_SENSITIVITY 1500
#define ONBOARD_BUTTON_LONGPRESS_SENSITIVITY 1500
#define METERS_TO_IN 39.37
#define IN_TO_METERS 0.0254 //39.3700788

int anglePoints[5] = {35, 340, 300, 250, 210};
long timers[5] = {15000, 9000, 8000, 9000, 15000};

int wayPointFrame = 0;
#define WAYPOINT_SENSITIVITY 20.0 // Goal radius in inches
#define WAY_POINT_AMOUNT 7

// GpsWayPoint wayPoints[WAY_POINT_AMOUNT] = {
//   GpsWayPoint(-81.201847, 28.599488),
//   GpsWayPoint(-81.201814, 28.599579),
//   GpsWayPoint(-81.201818, 28.599666),
//   GpsWayPoint(-81.201888, 28.599735),
//   GpsWayPoint(-81.201990, 28.599749),
//   GpsWayPoint(-81.202078, 28.599709),
//   GpsWayPoint(-81.202164, 28.599615)

//   // GpsWayPoint(-81.201877, 28.599492),
//   // GpsWayPoint(-81.201848, 28.599554),
//   // GpsWayPoint(-81.201840, 28.599613),
//   // GpsWayPoint(-81.201862, 28.599664),
//   // GpsWayPoint(-81.201900, 28.599700),
//   // GpsWayPoint(-81.201960, 28.599716),
//   // GpsWayPoint(-81.202034, 28.599695),
//   // GpsWayPoint(-81.202086, 28.599651),
//   // GpsWayPoint(-81.202135, 28.599586)

//   // GpsWayPoint(-81.201866, 28.599472),
//   // GpsWayPoint(-81.201826, 28.599536),
//   // GpsWayPoint(-81.201812, 28.599602),
//   // GpsWayPoint(-81.201829, 28.599671),
//   // GpsWayPoint(-81.201883, 28.599723),
//   // GpsWayPoint(-81.201961, 28.599738),
//   // GpsWayPoint(-81.202038, 28.599725),
//   // GpsWayPoint(-81.202114, 28.599678),
//   // GpsWayPoint(-81.202175, 28.599597)
// };

WayPointDouble_t doubleWayPoints[WAY_POINT_AMOUNT] {
  // {-81.19890319133003, 28.59451402777251},
  // {-81.1987681441924, 28.59457394873267},
  // {-81.19886170451984, 28.5946503715062},
  // {-81.19876022544719, 28.59470083589266},
  // {-81.19885285617833, 28.59476218144562},
  // {-81.198851782834, 28.59455052478464}

  // {-81.20186211237700, 28.59947669657357},
  // {-81.20181947634913, 28.59958359194177},
  // {-81.20189030139484, 28.59971758875102},
  // {-81.20205831051811, 28.59970659955914},
  // {-81.20217288255853, 28.59959929165985}

  {-81.20186547100926, 28.59949543608326},
  {-81.20182447270159, 28.59955721177563},
  {-81.20182433467505, 28.59964175537418},
  {-81.20188275866738, 28.59971669247343},
  {-81.20199596596456, 28.59973633404656},
  {-81.20207897986118, 28.59969463304335},
  {-81.20214621570693, 28.59960472214249}

  // {-81.20184906724158, 28.59948771496121},
  // {-81.20181398662301, 28.59957863562100},
  // {-81.20181763986577, 28.59966641943699},
  // {-81.20188837484690, 28.59973532608406},
  // {-81.20198993870503, 28.59974937658337},
  // {-81.20207769275629, 28.59970886095603},
  // {-81.20216363200487, 28.59961527349361}
};

  // {-81.2018491, 28.5994877},
  // {-81.2018140, 28.5995786},
  // {-81.2018176, 28.5996664},
  // {-81.2018884, 28.5997353},
  // {-81.2019899, 28.5997494},
  // {-81.2020777, 28.5997089},
  // {-81.2021636, 28.5996153}

// #define WAY_POINT_AMOUNT 9
// GpsWayPoint wayPoints[WAY_POINT_AMOUNT] = {
//   GpsWayPoint(-81.201866, 28.599472),
//   GpsWayPoint(-81.201826, 28.599536),
//   GpsWayPoint(-81.201812, 28.599602),
//   GpsWayPoint(-81.201829, 28.599671),
//   GpsWayPoint(-81.201883, 28.599723),
//   GpsWayPoint(-81.201961, 28.599738),
//   GpsWayPoint(-81.202038, 28.599725),
//   GpsWayPoint(-81.202114, 28.599678),
//   GpsWayPoint(-81.202175, 28.599597)
// };


Servo servo;
Servo motor;

TinyGPSPlus gps;
long scale=10000000UL;

QMC5883LCompass compass;



int startingLaneSelection = 0;


double lat = 0;
double lon = 0;
double velocity = 0;

double azimuth = 0.0;

struct OnboardButton_t {
  uint8_t pressCount;
  bool currentState;
  bool lastState;
  unsigned long debounceTime;
  unsigned long multipressTime;
  unsigned long holdTime;
};
OnboardButton_t onboardButton = {0, false, false, 0, 0};


struct OnboardLed_t {
  uint8_t mode; // 0 = Looking For Satelites;   1 = Initial Start Menu;   2 = Index Selection Menu;   3 = Stand by; 
  unsigned long time;
  unsigned long timeReference;
  unsigned long timeChange;
  bool state;
};
OnboardLed_t onboardLed = {0, 0, 0, 0, true};

// void setOnboardLedMode(uint8_t mode) {
//   onboardLed.mode = mode;
//   onboardLed.time = millis();
// }

// float steering_kp = 6.0F;
// float steering_ki = 0.5F;
// float steering_kd = 1.5F;
float steering_kp = 1.3F;
float steering_ki = 0.2F;
float steering_kd = 0.5F;
float steeringOutput;
float angleToPoint;
float steeringAngleSetpoint;
PID steeringPID(&angleToPoint, &steeringOutput, &steeringAngleSetpoint, steering_kp, steering_ki, steering_kd, P_ON_E, DIRECT);


#define CH_NUM 3
int channel[] = {0, 0, 0};
int receiverPins[] = {39, 36, 34};
int channelUpperLimitsIn[] = {1998, 1725, 999};
int channelLowerLimitsIn[] =  {1496, 1274, 2000};
int channelUpperLimitsOut[] = {130, 115, 1};
int channelLowerLimitsOut[] = {90, 70, 0};
long channelUpdateTimers[] = {0, 0, 0, 0};
long chHighs[] = {-9000, -9000, -9000};
long chLows[] = {9000, 9000, 9000};

// bool mapSwitchDebounceFlag = false;
// void handleOnboardButton(void) {
//   switch(onboardLed.mode) {
//     case 0: 
//       if((lat != 0) && (lon != 0)) {
//         setOnboardLedMode(1);
//       }
//       break;
//     case 1:
//       onboardButton.currentState = digitalRead(ONBOARD_BUTTON);
//       if(onboardButton.currentState && (!onboardButton.lastState) && 
//       ((millis() - onboardButton.debounceTime) > ONBOARD_BUTTON_DEBOUNCE_SENSITIVITY)) {
//         onboardButton.lastState = true;
//         digitalWrite(ONBOARD_LED, HIGH);
//       } else if((!onboardButton.currentState) && onboardButton.lastState) {
//         onboardButton.lastState = false;
//         if((millis() - onboardButton.multipressTime) <= ONBOARD_BUTTON_MULTIPRESS_SENSITIVITY) {
//           onboardButton.pressCount++;
//         } else {
//           onboardButton.pressCount = 1;
//         }
//         onboardButton.multipressTime = millis();
//         onboardButton.debounceTime = millis();
//         digitalWrite(ONBOARD_LED, LOW);
//       }
//       if((onboardButton.pressCount > 0) && 
//       ((millis() - onboardButton.multipressTime) > ONBOARD_BUTTON_MULTIPRESS_SENSITIVITY)) {
//         // Serial.println("Button was pressed "+String(onboardButton.pressCount)+" times");
//         startingLaneSelection = onboardButton.pressCount;
//         digitalWrite(ONBOARD_LED, LOW);
//         onboardButton.pressCount = 0;
//         setOnboardLedMode(2);
//         // delay(500);
//         // onboardLed.mode = 2;
//         // onboardLed.time = millis();
//       }
//       break;
//     case 4:
//     case 2:
//       onboardButton.currentState = digitalRead(ONBOARD_BUTTON);
//       if(onboardButton.currentState && (!onboardButton.lastState) && 
//       ((millis() - onboardButton.debounceTime) > ONBOARD_BUTTON_DEBOUNCE_SENSITIVITY)) {
//         onboardButton.lastState = true;
//         digitalWrite(ONBOARD_LED, HIGH);
//       } else if((!onboardButton.currentState) && onboardButton.lastState) {
//         onboardButton.lastState = false;
//         if((millis() - onboardButton.multipressTime) <= ONBOARD_BUTTON_MULTIPRESS_SENSITIVITY) {
//           onboardButton.pressCount++;
//         } else {
//           onboardButton.pressCount = 1;
//         }
//         onboardButton.multipressTime = millis();
//         onboardButton.debounceTime = millis();
//         digitalWrite(ONBOARD_LED, LOW);
//       }
//       if((onboardButton.pressCount > 0) && 
//       ((millis() - onboardButton.multipressTime) > ONBOARD_BUTTON_MULTIPRESS_SENSITIVITY)) {
//         // Serial.println("Button was pressed "+String(onboardButton.pressCount)+" times");
//         digitalWrite(ONBOARD_LED, LOW);
//         if(onboardButton.pressCount == 1) {
//           setOnboardLedMode(5);
//         } else if(onboardButton.pressCount == 2) {
//           onboardButton.pressCount = 0;
//           setOnboardLedMode(1);
//         } else {
//           onboardButton.pressCount = 0;
//         }
//         // delay(500);
//         // onboardLed.mode = 2;
//         // onboardLed.time = millis();
//       }
//       // onboardButton.currentState = digitalRead(ONBOARD_BUTTON);
//       // if(onboardButton.currentState && (!onboardButton.lastState) &&
//       // ((millis() - onboardButton.debounceTime) > ONBOARD_BUTTON_DEBOUNCE_SENSITIVITY)) {
//       //   onboardButton.lastState = true;
//       //   digitalWrite(ONBOARD_LED, LOW);
//       //   onboardButton.holdTime = millis();
//       //   onboardButton.debounceTime = millis();
//       // } else if((!onboardButton.currentState) && onboardButton.lastState &&
//       // (onboardButton.debounceTime >= (1000))) {
//       //   if((millis() - onboardButton.holdTime) >= ONBOARD_BUTTON_LONGPRESS_SENSITIVITY) {
//       //     setOnboardLedMode(1);
//       //   } else {
//       //     setOnboardLedMode(5);
//       //   }
//       //   onboardButton.lastState = false;
//       //   onboardButton.debounceTime = millis();
//       // }
//       // break;
//     // case 3: 
//     // case 5:
//       break;
//   }
// }
// void handleOnboardLedState(void) {
//   switch(onboardLed.mode) {
//     case 0:
//       onboardLed.timeReference = millis();
//       onboardLed.timeChange = (onboardLed.timeReference - onboardLed.time);
//       if(onboardLed.timeChange >= 4000) {
//         digitalWrite(ONBOARD_LED, HIGH);
//         onboardLed.time = onboardLed.timeReference;
//       } else if(onboardLed.timeChange >= 3000) {
//         digitalWrite(ONBOARD_LED, LOW);
//       } else if(onboardLed.timeChange >= 2000) {
//         digitalWrite(ONBOARD_LED, HIGH);
//       } else if(onboardLed.timeChange >= 1000) {
//         digitalWrite(ONBOARD_LED, LOW);
//       }
//       break;
//     case 1:
//       if(onboardButton.pressCount == 0) {
//         onboardLed.timeReference = millis();
//         onboardLed.timeChange = (onboardLed.timeReference - onboardLed.time);
//         if(onboardLed.timeChange >= 1440) {     
//           digitalWrite(ONBOARD_LED, HIGH);         
//           onboardLed.time = onboardLed.timeReference;     
//         } else if(onboardLed.timeChange >= 240) {
//           digitalWrite(ONBOARD_LED, LOW); 
//         } else if(onboardLed.timeChange >= 160) {
//           digitalWrite(ONBOARD_LED, HIGH); 
//         } else if(onboardLed.timeChange >= 80) {
//           digitalWrite(ONBOARD_LED, LOW); 
//         }
//       }
//       break;
//     case 2:
//       // onboardLed.timeReference = millis();
//       // onboardLed.timeChange = (onboardLed.timeReference - onboardLed.time);
//       // delay(500);
//       if(startingLaneSelection > 0) {
//         digitalWrite(ONBOARD_LED, HIGH);
//         delay(500);
//         digitalWrite(ONBOARD_LED, LOW);
//         delay(500);
//         startingLaneSelection--;
//       } else {
//         delay(500);
//         setOnboardLedMode(3);
//       }
//       break;
//         // if((millis() - onboardLed.time) >= 1000) {
//         //   digitalWrite(ONBOARD_LED, HIGH);
//         //   // Serial.println("UP YOU GO!");
//         //   onboardLed.time = millis();
//         // } else if((millis() - onboardLed.time) >= 1000) {
//         //   digitalWrite(ONBOARD_LED, LOW);
//         //   // digitalWrite(ONBOARD_LED, LOW);
//         //   startingLaneSelection--;
//         //   // Serial.println("Selection Val: "+String(startingLaneSelection));
//         // }
//         // if((millis() - onboardLed.time) >= 2000) {
//           // onboardLed.time = millis();
//         // } else if((millis() - onboardLed.time) >= 1000) {
//         // }
//         // if(startingLaneSelection == 0) {
//         //   Serial.print("BIG GEY! : 164");
//         //   digitalWrite(ONBOARD_LED, LOW);
//         //   onboardLed.time = onboardLed.timeReference;
//         // }
//       // } else {
//       //   if((millis() - onboardLed.time) >= 1000) {
//       //     digitalWrite(ONBOARD_LED, HIGH);
//       //   }
//     case 3:
//       // if(startingLaneSelection == 0) {
//       onboardLed.timeChange = (onboardLed.time - millis());
//       if(onboardLed.timeChange >= 2000) {
//         digitalWrite(ONBOARD_LED, LOW);
//         setOnboardLedMode(4);
//       } else if(onboardLed.timeChange >= 1000) {
//         digitalWrite(ONBOARD_LED, HIGH);
//         // Serial.println("Stand by mode.");
//       }
//       break;
//     case 4:
//       onboardLed.timeReference = millis();
//       onboardLed.timeChange = (onboardLed.timeReference - onboardLed.time);
//       if(onboardLed.timeChange >= 4000) {
//         digitalWrite(ONBOARD_LED, LOW);
//         onboardLed.time = onboardLed.timeReference;
//       } else if(onboardLed.timeChange >= 2500) {
//         digitalWrite(ONBOARD_LED, HIGH);
//       } else if(onboardLed.timeChange >= 2000) {
//         digitalWrite(ONBOARD_LED, LOW);
//       } else if(onboardLed.timeChange >= 500) {
//         digitalWrite(ONBOARD_LED, HIGH);
//       }
//       break;
//     case 5:
//       digitalWrite(ONBOARD_LED, HIGH);
//       onboardLed.mode = 6;
//       break;
//   }
// }
// String flashRead(void) {
//   unsigned char p = 0;
//   int i=0;
//   String output;
//   for(int i=0; i<128; i++) {
//     Flash.read(flash+i, &p, 1);
//     if(p && (p != 255)) {
//       output.concat((char)p);
//       Serial.write(p);
//     }
//   }
//   Serial.println();
//   return(output);
// }
// void flashErase(void) { 
//   Flash.erase(flash); 
// }
// Max allowed length is 127 characters
// void flashWrite(String dataString) {
//   if(flashDataCopy.equals("")) {
//     flashDataCopy.concat(flashRead());
//     if(flashDataCopy.indexOf(255) != -1) {
//       flashDataCopy = "";
//     }
//   }
//   flashDataCopy.concat(dataString);
//   char data[128];
//   if(flashDataCopy.length() < 129) {
//     flashDataCopy.toCharArray(data, flashDataCopy.length()+1);
//     flashErase();
//     Flash.write(flash, (unsigned char*)data, flashDataCopy.length()); 
//     blinkRedLed();
//   }
// }
// uint16_t getReceiverChValue(uint8_t ch) {
//   long currentTime = millis(); 
//   long timeChange = (currentTime - channelUpdateTimers[ch]);
//   if(ch == 0) {
//     Serial.println("Channel 0: "+String(timeChange));
//   }
//   if(!digitalRead(receiverPins[ch]) && (timeChange > 1244)) {
//     uint16_t val = timeChange;
//     channelUpdateTimers[ch] = currentTime;
//     return(map(val, channelLowerLimitsIn[ch], channelUpperLimitsIn[ch], channelLowerLimitsOut[ch], channelUpperLimitsOut[ch]));
//   } else {
//     return(0);
//   }
// }

// void updateReceiver(void) {
//   channel[2] = map(pulseIn(receiverPins[2], HIGH), channelLowerLimitsIn[2],
//                                                    channelUpperLimitsIn[2], 
//                                                    channelLowerLimitsOut[2], 
//                                                    channelUpperLimitsOut[2]);
//   if(channel[2] == 1) {
//     for(int i=0; i<CH_NUM; i++) {
//       if(i != 2) {
//         channel[i] = map(pulseIn(receiverPins[i], HIGH), channelLowerLimitsIn[i],
//                                                      channelUpperLimitsIn[i], 
//                                                      channelLowerLimitsOut[i], 
//                                                      channelUpperLimitsOut[i]);
//       }
//     }  
//     motor.write(channel[0]);
//     servo.write(channel[1]);
//   }
// }


void updateGps(void) {
  if(Serial1.available()) {
    // digitalWrite(ONBOARD_LED, LOW);
    gps.encode(Serial1.read());
    // if(gps.location.isValid()) {
    if(gps.location.isUpdated()) {
      
      // lat = (gps.location.rawLat().deg*scale+gps.location.rawLat().billionths/100UL);
      // if(gps.location.rawLat().negative) lat=-lat;
      // lon = (gps.location.rawLng().deg*scale+gps.location.rawLng().billionths/100UL);
      // if(gps.location.rawLng().negative) lon=-lon;

      lat = gps.location.lat();
      lon = gps.location.lng();
      // velocity = gps.speed.mps();
      // lat = gps.location.rawLat().deg;
      // lon = gps.location.rawLng().deg;
      // Serial.println("lon: "+String(lon, 10)+"    lat: "+String(lat, 10));

      // Serial.println("TESTER: "+String((lon*1.0e-6)*(PI/180), 6)+"        "+String((double)((_LONG_DOUBLE)lon*(_LONG_DOUBLE)1.0e-7)));
      // Serial.println("lib lon: "+String(gps.location.lng(),4)+"    lib lat: "+String(gps.location.lat(),4)+"\n");
    }
    //  else {

    // }
  }
}

// void updateCompass(void) {
//   compass.read(&x, &y, &z, &azimuth);
//   // Serial.println(String(x)+", "+String(y)+", "+String(z));
//   Serial.println(azimuth);
//   // Serial.println(averageYaw);
// }

double degreeDifference(double currentDeg, double targetDeg) {
  double angleDiff = (targetDeg-currentDeg);
  if (angleDiff > 180.0)
    angleDiff = ((360.0-angleDiff)*(-1.0));
  if (angleDiff < -180.0)
    angleDiff += 360.0;
  return angleDiff;
}

double normalizeDegrees(double degree) {
  double orientation = fmod(degree, 360.0);
  if(orientation < 0.0)
    orientation = (orientation + 360.0);
  return orientation;
}

void updateCompass(void) {
  compass.read();
  azimuth = (double)compass.getAzimuth();
  azimuth = normalizeDegrees(azimuth-180);
  // Serial.println("    Azimuth: "+String(azimuth));
  // if(wayPointFrame >= 4) {
  //   azimuth *= -1;
  // }
  // Serial.println(String(x)+", "+String(y)+", "+String(z));
  // Serial.println(azimuth);
  // Serial.println(averageYaw);
}

// void updateCompass(void) {
//   // bno.getEvent(&event);
//   // azimuth = (event.orientation.x);
//   bno055_read_euler_hrp(&myEulerData);
//   azimuth = (float(myEulerData.h)/16.00);
//   // Serial.println(azimuth);
// }

void handleWayPointNavigation(void) {
  if(wayPointFrame < WAY_POINT_AMOUNT) {
    // float interPointdistanceMeters = wayPoints[wayPointFrame].getDistanceTo(wayPoints[wayPointFrame+1]);
    // float pointBearingAngleDegrees = wayPoints[wayPointFrame].getAngleTo(wayPoints[wayPointFrame+1]));
    // float targetAzimuth = normalizeDegrees(360-wayPoints[wayPointFrame].getAngleTo(lon, lat));
    // float targetAzimuth = normalizeDegrees(wayPoints[wayPointFrame+1].getAngleTo(wayPoints[wayPointFrame].getLongitude(), wayPoints[wayPointFrame].getLatitude()));
    // double targetAzimuth = TinyGPSPlus::courseTo(doubleWayPoints[wayPointFrame].latitude, doubleWayPoints[wayPointFrame].longitude, doubleWayPoints[wayPointFrame+1].latitude, doubleWayPoints[wayPointFrame+1].longitude);
    double targetAzimuth = TinyGPSPlus::courseTo(lat, lon, doubleWayPoints[wayPointFrame].latitude, doubleWayPoints[wayPointFrame].longitude);
    // double errorDistance2;
    // double targetAzimuth2 = GpsWayPoint::courseTo(longWayPoints[wayPointFrame].latitude, longWayPoints[wayPointFrame].longitude, longWayPoints[wayPointFrame+1].latitude, longWayPoints[wayPointFrame].longitude, &errorDistance2);
    // angleToPoint = degreeDifference(azimuth, pointBearingAngleDegrees);
    angleToPoint = degreeDifference(azimuth, targetAzimuth);
    // Serial.println("Target Azimuth: "+String(targetAzimuth)+"    DegreeDifference: "+String(angleToPoint)+"\n");
    // Serial.println(angleToPoint);
    // float distanceToPoint = wayPoints[wayPointFrame+1].getDistanceFrom(lon, lat);
    // double errorDistance = TinyGPSPlus::distanceBetween(
    //   doubleWayPoints[wayPointFrame].latitude, doubleWayPoints[wayPointFrame].longitude,
    //   doubleWayPoints[wayPointFrame+1].latitude, doubleWayPoints[wayPointFrame+1].longitude
    // );
    double errorDistance = TinyGPSPlus::distanceBetween(lat, lon, doubleWayPoints[wayPointFrame].latitude, doubleWayPoints[wayPointFrame].longitude);

    // Serial.println("Target Azimuth: "+String(targetAzimuth, 10)+"    Err Distance: "+String(errorDistance, 10));
    // Serial.print("Lat: ");
    // Serial.print(lat);
    // Serial.print("    Lon: ");
    // Serial.println(lon);

    // float errorDistance = wayPoints[wayPointFrame].getDistanceFrom(lon, lat);
    // Serial.println("Target Azimuth: "+String(targetAzimuth)+"    Error Distance: "+String(errorDistance));
    // Serial.println("Long T Azimuth: "+String(targetAzimuth2)+"    Long E Distance: "+String(errorDistance2));
    steeringPID.Compute();
    servo.write(steeringOutput);
    #ifndef TESTING
      // motor.write(130);
    #endif
    if(errorDistance <= (WAYPOINT_SENSITIVITY * IN_TO_METERS)) {
      onboardLed.state = !onboardLed.state;
      digitalWrite(ONBOARD_LED, onboardLed.state);
      wayPointFrame++;
    }
  }
}

void PPM_Receiver_Calibration(void) {
  for(int i=0; i<CH_NUM; i++) {
    channel[i] = pulseIn(receiverPins[i], HIGH);
  }
  for(int i=0; i<3; i++) {
    chHighs[i] = ((chHighs[i] < channel[i])? (channel[i]):(chHighs[i]));
    chLows[i] = ((chLows[i] > channel[i])? (channel[i]):(chLows[i]));
  }
  Serial.print("HIGHS: [ ");
  for(int i=0; i<3; i++) {
    Serial.print("Ch"+String(i)+": "+String(chHighs[i])+"\t");
  }
  Serial.print(" ]\t\tLOWS: [ ");
  for(int i=0; i<3; i++) {
    Serial.print("Ch"+String(i)+": "+String(chLows[i])+"\t");
  }
  Serial.println(" ]\n");
}

void setup() {
  // Serial.begin(115200);
  // Serial1.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, 16, 17);
  Wire.begin();
  // while (!imu.check()) {}
  // imu.SetExternalCrystal(true);
  // BNO_Init(&myBNO);
  // bno055_set_operation_mode(OPERATION_MODE_NDOF_FMC_OFF);
  // if(!bno.begin()) {}
  // compass.setAddress(0x0D);
  // compass.init();
  // compass.setMode(Mode_Continuous,ODR_10Hz,RNG_8G,OSR_512);
  compass.init();
  // compass.setCalibration(-1202, 1205, -1332, 1123, -1108, 1132);
  compass.setCalibration(-708, 1032, -935, 775, -1018, 0);   // compass.setCalibration(-1007, 1398, -1550, 1001, -1330, 0);
  // compass.setCalibration(-1007, 1398, -1550, 1001, -1330, 0);
  // compass.setSmoothing(10, true);

  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(ONBOARD_BUTTON, INPUT);
  pinMode(receiverPins[0], INPUT);
  pinMode(receiverPins[1], INPUT);
  pinMode(receiverPins[2], INPUT);  
  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);  
  
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);
  motor.setPeriodHertz(50);
  motor.attach(33);
  servo.attach(32);
  motor.write(90);
  servo.write(90);
  
  steeringPID.SetOutputLimits(channelLowerLimitsOut[1], channelUpperLimitsOut[1]);
  steeringPID.SetSampleTime(40);
  steeringPID.SetMode(AUTOMATIC);
  steeringPID.SetControllerDirection(DIRECT);

  digitalWrite(ONBOARD_LED, HIGH);
  #ifndef TESTING
    onboardLed.timeReference = millis();
    while((lon == 0) && (lat == 0)) {
      updateGps();
      onboardLed.time = millis();
      if((onboardLed.time - onboardLed.timeReference) >= 500) {
        onboardLed.state = !onboardLed.state;
        digitalWrite(ONBOARD_LED, onboardLed.state);
        onboardLed.timeReference = onboardLed.time;
      }
    }
    digitalWrite(ONBOARD_LED, HIGH);
  #endif
  // while(!digitalRead(ONBOARD_BUTTON)) {}
  onboardLed.state = false;
  digitalWrite(ONBOARD_LED, onboardLed.state);
  
}

// unsigned long timer = millis();
// int frame = 0;
// long timeHolder = millis();

void loop() {
  // Serial.println("Hello World");
  // updateCompass();
  // if(wayPointFrame >= 4) {
  //   steeringPID.SetControllerDirection(REVERSE);
  // }
  // if((millis() - timer) >= 4000) {
  //   wayPointFrame++;
  //   timer = millis();
  // }
  // #ifndef TESTING
    // updateReceiver();
  // #endif
  updateGps();
  updateCompass();


  // if(!channel[2]) {
  //   motor.write(120);

  // updateReceiver();
  // for(int i=0; i<3; i++) {
    // Serial.print("Ch "+String(i)+": "+String(channel[i])+"    ");
  // }
  // Serial.println("\n");
  // PPM_Receiver_Calibration();

  //   if((millis() - timeHolder) < timers[frame]) {
  //     angleToPoint = degreeDifference(azimuth, anglePoints[frame]);
  //     steeringPID.Compute();
  //     servo.write(steeringOutput);
  //   } else {
  //     timeHolder = millis();
  //     frame++;
  //   }
  // }

    handleWayPointNavigation();
  


  // if(!channel[2]) {
  //   // compass.calibrate();
  //   // unsigned long currentTime = millis();
  //   // averageTime = (((currentTime - sampleTime) + averageTime) / 2);
  //   // sampleTime = currentTime;
  // handleWayPointNavigation();
  // Serial.println("Current: "+String(azimuth));
  // }
  motor.write(90);
  //  else {
  //   Serial.println(averageTime);
  // }
}


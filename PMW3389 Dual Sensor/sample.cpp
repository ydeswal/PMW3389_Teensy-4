/*

  main.cpp

*/
// Include Config-file (moved for code clarity)
#include "main_config.h"
#include "vector"
#include <cmath>
const String fileVersion = __TIMESTAMP__;

// Add these constants at the top of the file
const float MAX_SINGLE_DISPLACEMENT = 10.0f; // Maximum reasonable displacement in cm per reading
const float MAX_THETA = M_PI; // Maximum rotation angle in radians

// Create Sensor Objects with Specified Slave-Select Pins
ADNS adnsA(CS_PIN_A);
ADNS adnsB(CS_PIN_B);
sensor_pair_t sensor = {adnsA, adnsB};

// Capture Task (on interrupt)
IntervalTimer captureTimer;

// Counter and Timestamp Generator
elapsedMillis millisSinceAcquisitionStart;
elapsedMicros microsSinceFrameStart;
// volatile time_t currentSampleTimestamp;
volatile time_t currentFrameTimestamp;
volatile time_t currentFrameDuration;
volatile uint32_t currentFrameCount;
volatile bool isRunning = false;

//added the next 5 lines for water pin
std::vector<int> waterFrames;
volatile uint32_t waterIndex;
int range_in_seconds[2];
volatile uint16_t waterLength;
volatile uint32_t nreps = 0;
volatile bool waterPinON = false;
String count;
char input2[20];
// =============================================================================
//   SETUP & LOOP
// =============================================================================
#include "DeviceLib/devicemanager.h"
void setup() {
  delay(400);
  initializeCommunication();

  delay(400);

  initializeSensors();

  initializeTriggering();

  while (Serial.available()) {
    Serial.read();
  }
  char matlab_input[100];
  beginAcquisition(matlab_input, 100);

}

void loop() {
    while(nreps>=currentFrameCount) {
        }
        endAcquisition();

  }


// =============================================================================
//   TASKS: INITIALIZE
// =============================================================================
inline static bool initializeCommunication() {
  // Begin Serial
  Serial.begin(115200);
  while (!Serial) {
    ;  // may only be needed for native USB port
  }
  delay(10);
  return true;
};

inline static bool initializeSensors() {
  // Begin Sensors
  sensor.left.begin();
  delay(30);
  sensor.right.begin();
  delay(30);
  return true;
};

inline static bool initializeTriggering() {
  fastPinMode(TRIGGER_PIN, OUTPUT);
  fastDigitalWrite(TRIGGER_PIN, LOW);
  fastPinMode(WATER_PIN, INPUT);
  fastDigitalWrite(WATER_PIN, LOW);
  fastPinMode(LED_PIN, OUTPUT);
  fastDigitalWrite(LED_PIN, LOW);
  delay(1);
  // Setup Sync/Trigger-Output Timing
  // FrequencyTimer2::setPeriod(1e6 / DISPLACEMENT_SAMPLE_RATE)
  return true;
};

static inline void beginAcquisition(char input[], int8_t length) {
    delay(500);
    Serial.readBytes(input, length);
    //Parse input
    char *trial_length_minutes = strtok(input,",");
    float trial_length_minutes_int = atof(trial_length_minutes);
    char *sampling_interval_ms = strtok(NULL,",");
    float sampling_interval_ms_int = atof(sampling_interval_ms);
    char *water_spacing_s = strtok(NULL,",");
    float water_spacing_s_int = atof(water_spacing_s);
    char *water_jitter_s = strtok(NULL,",");
    float water_jitter_s_int = atof(water_jitter_s);


    nreps = floor(trial_length_minutes_int*60.0*1000.0/sampling_interval_ms_int);
    Serial.println((nreps));
    Serial.println((sampling_interval_ms_int));
    Serial.println(water_jitter_s_int);
    Serial.println(water_spacing_s_int);
    Serial.println("HI");

    //get random frames for
    //getRandomFrames(int(sampling_interval_ms_int), range_in_seconds, int(nreps));

    waterLength = 100/int(sampling_interval_ms_int);

    waterIndex = 0;

    // Print units and Fieldnames (header)

    sendHeader();

    // Trigger start using class methods in ADNS library
    sensor.left.triggerAcquisitionStart();
    sensor.right.triggerAcquisitionStart();

    // Flush sensors (should happen automatically -> needs bug fix)
    sensor.left.triggerSampleCapture();
    sensor.right.triggerSampleCapture();

    // Change State
    isRunning = true;

    // Reset Elapsed Time Counter
    millisSinceAcquisitionStart = 0;

    // currentSampleTimestamp = microsSinceAcquisitionStart;
    currentFrameTimestamp = millisSinceAcquisitionStart;

    currentFrameCount = 0;

    fastDigitalWrite(TRIGGER_PIN,HIGH);

    captureTimer.begin(captureDisplacement, 1000*sampling_interval_ms_int);
}



static inline void endAcquisition() {
    // End IntervalTimer
    waterFrames.clear();
    waterPinON = false;
    fastDigitalWrite(TRIGGER_PIN, LOW);
    fastDigitalWrite(WATER_PIN, LOW);
    fastDigitalWrite(LED_PIN, LOW);
    // Trigger start using class methods in ADNS library
    sensor.left.triggerAcquisitionStop();
    sensor.right.triggerAcquisitionStop();

    // Change state
    isRunning = false;
    captureTimer.end();
}

// =============================================================================
// TASKS: TRIGGERED_ACQUISITION
// =============================================================================
void captureDisplacement() {


  fastDigitalWrite(TRIGGER_PIN,LOW);
  // Initialize container for combined & stamped sample
  sensor_sample_t currentSample;
  currentSample.timestamp = currentFrameTimestamp; // maybe fix this time stamp issue?

  // Trigger capture from each sensor
  sensor.left.triggerSampleCapture();
  sensor.right.triggerSampleCapture();
  // Store timestamp for next frame

  currentFrameCount += 1;

  currentSample.left = {'L', sensor.left.readDisplacement(units)};
  currentSample.right = {'R', sensor.right.readDisplacement(units)};

  // Send Data
  sendData(currentSample,waterPinON);
  currentFrameTimestamp = millisSinceAcquisitionStart;

  fastDigitalWrite(TRIGGER_PIN,HIGH);
  delay(1);
  fastDigitalWrite(TRIGGER_PIN,LOW);

}

// =============================================================================
// TASKS: DATA_TRANSFER
// =============================================================================

void sendHeader() {
  const String dunit = getAbbreviation(units.distance);
  const String tunit = getAbbreviation(units.time);
  // Serial.flush();
  Serial.print(String(
      String("timestamp [ms]") + delimiter + flatFieldNames[0] + " [" + dunit +
      "]" + delimiter + flatFieldNames[1] + " [" + dunit + "]" + delimiter +
      flatFieldNames[2] + " [" + tunit + "]" + delimiter + flatFieldNames[3] +
      " [" + dunit + "]" + delimiter + flatFieldNames[4] + " [" + dunit + "]" +
      delimiter + flatFieldNames[5] + " [" + tunit + "]" +delimiter + " waterPin " +delimiter + " triangle_X "+delimiter + " triangle_Y "+delimiter + " triangle_Theta " "\n"));
}

void sendData(sensor_sample_t sample, bool waterPin) {

    // Convert to String class
    const String timestamp = String(sample.timestamp);
    const String dxL = String(sample.left.p.dx, decimalPlaces);
    const String dyL = String(sample.left.p.dy, decimalPlaces);
    const String dtL = String(sample.left.p.dt, decimalPlaces);
    const String dxR = String(sample.right.p.dx, decimalPlaces);
    const String dyR = String(sample.right.p.dy, decimalPlaces);
    const String dtR = String(sample.right.p.dt, decimalPlaces);
    const String waterPinVal = (waterPin ? "1" : "0");
    const String endline = String("\n");
    // Serial.availableForWrite
    //getMovement data
    // Virmen Scale
    float scale = 128/23; //virmen distance/experimental distance

    float unitsPer2PiRotationL = 1000*168;
    float unitsPer2PiRotationR =  1000*126;
    //scale of maze taken into account, measurement in mm
    float unitsPerRotationL = 1963651.453/(3*2*2.54);  //average of three complete rotations
    float unitsPerRotationR =  1963651.233/(3*2*2.54); //average of three complete rotations
    float ballCircumferenceIn = 25.125; //measured in lab
    float ballCircumferenceCm = ballCircumferenceIn*2.54; //definition
    float ballRadiusCm  = ballCircumferenceCm/(2*M_PI); //definition


    float sensorAngleDegrees = 78; //measured in lab
    float sensorAngleRadians = sensorAngleDegrees*2*M_PI/360; //definition

    float cmPerUnitL = ballCircumferenceCm/unitsPerRotationL;
    float cmPerUnitR = ballCircumferenceCm/unitsPerRotationR;

    float dlx = sample.left.p.dx*cmPerUnitL; //convert measurements to units of cm
    float drx = sample.right.p.dx*cmPerUnitR; //convert measurements to units of cm
    float dry = sample.right.p.dy*cmPerUnitR; //convert measurements to units of cm
    float dly = sample.left.p.dy*cmPerUnitR;

    float dThetaL = (sample.left.p.dx)*2*M_PI/unitsPer2PiRotationL;
    float dThetaR = (sample.right.p.dx)*2*M_PI/unitsPer2PiRotationR;

    float dTheta = (dThetaL + dThetaR)/2 ;
    float sgn = (dTheta > 0) - (dTheta < 0);
    const float pi = 3.14159;  // Use a constant for pi


    float dyT = dry;

    float dxT = (dly-dry*cos(sensorAngleRadians))/cos(M_PI/2-sensorAngleRadians);
    float sgn2 = (dyT > 0) - (dyT < 0);
    float distance = sqrt(dxT*dxT + dyT*dyT);

    // Validate displacement values
    if (distance > MAX_SINGLE_DISPLACEMENT) {
        // Scale down the values proportionally if they exceed threshold
        float scale = MAX_SINGLE_DISPLACEMENT / distance;
        dxT *= scale;
        dyT *= scale;
        distance = MAX_SINGLE_DISPLACEMENT;
    }

    float theta = atan2((dxT), dyT);
    theta = sgn * std::min(static_cast<float>(std::exp(1.4 * std::pow(std::fabs(theta), 1.2))) - 1, pi);

    // Clamp theta to reasonable range
    theta = std::max(std::min(theta, MAX_THETA), -MAX_THETA);

    // compute the angle relative to the dx axis
    float rel_direction = atan2(dyT,dxT);
    rel_direction = rel_direction+(-152*2*M_PI/360);

    // Validate final displacement calculations
    dyT = 4.60975609756/4*sin(rel_direction)*distance*6.0;
    dxT = 4.04115037444/4*cos(rel_direction)*distance*6.0;

    const String dxTriangle = String(dxT, decimalPlaces);
    const String dyTriangle = String(dyT, decimalPlaces);

    const String dThetaTest = String(theta,decimalPlaces);
    int varile=0;
    for(int i=0;i<10;i++){
      varile+=fastDigitalRead(WATER_PIN);
      delay(2);
    }
    String variable = String(varile);

    // Print ASCII Strings
    // Serial.print(timestamp + delimiter + dxL + delimiter + dyL + delimiter +
    //              dtL + delimiter + dxR + delimiter + dyR + delimiter + dtR  +
    //              delimiter + dxTriangle + delimiter + dyTriangle + delimiter + dTheta+ delimiter + variable + delimiter+
    //              String(rel_direction)+delimiter+String(currentFrameCount)+
    //              endline);
    Serial.print("[T:");
    Serial.print(sample.timestamp);
    Serial.print("ms] Sensor1 [X: ");
    Serial.print(sample.left.p.dx, decimalPlaces);
    Serial.print(" Y: ");
    Serial.print(sample.left.p.dy, decimalPlaces);
    Serial.print("] Sensor2 [X: ");
    Serial.print(sample.right.p.dx, decimalPlaces);
    Serial.print(" Y: ");
    Serial.print(sample.right.p.dy, decimalPlaces);
    Serial.print("]");

    Serial.print(" || Total Displacement X: ");
    Serial.print(dxT, decimalPlaces);
    Serial.print(" | Total Displacement Y: ");
    Serial.println(dyT, decimalPlaces);

    Serial.print("Total Distance Traveled: ");
    Serial.print(distance, decimalPlaces);
    Serial.println(" cm");
}
/*
//outputs should be distance, rel_direction, dx, dy, dTheta
void getMovement(sensor_sample_t sample){  //fix with correct input

}*/
//#########################################################################################

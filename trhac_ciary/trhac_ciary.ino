#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount] = { 0 };

int stav = 1;               // 0=wait calib, 1=calibrate, 2=line follow, 3=stop
bool calibration_done = false;
long casstartu = 0;

// Line follower
int baseSpeed = 60;
int kp = 1.8;
int rych_tocenia = 120;
int last_position = 3500;

//--------------------------------------------------------------------
// Pins
//--------------------------------------------------------------------

const int cudel = 11;

// MX1508
#define LEFT_FWD  6
#define LEFT_BWD  5
#define RIGHT_FWD 10
#define RIGHT_BWD 9

//--------------------------------------------------------------------
// MOTORS (forward + backward)
//--------------------------------------------------------------------

void setMotors(int left, int right) {

  // LEFT motor
  if (left >= 0) {
    analogWrite(LEFT_FWD, constrain(left, 0, 255));
    analogWrite(LEFT_BWD, 0);
  } else {
    analogWrite(LEFT_FWD, 0);
    analogWrite(LEFT_BWD, constrain(-left, 0, 255));
  }

  // RIGHT motor
  if (right >= 0) {
    analogWrite(RIGHT_FWD, constrain(right, 0, 255));
    analogWrite(RIGHT_BWD, 0);
  } else {
    analogWrite(RIGHT_FWD, 0);
    analogWrite(RIGHT_BWD, constrain(-right, 0, 255));
  }
}

void zastavsa() {
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BWD, 0);
  pinMode(cudel, INPUT_PULLUP);
}

//--------------------------------------------------------------------
// Setup
//--------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  Serial.println("Start MCU");

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 15, 16, 17, 2, 3, 4, 7, 8 }, SensorCount);
  qtr.setEmitterPin(12);

  pinMode(cudel, INPUT_PULLUP);
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LEFT_FWD, LOW);
  digitalWrite(LEFT_BWD, LOW);
  digitalWrite(RIGHT_FWD, LOW);
  digitalWrite(RIGHT_BWD, LOW);

  delay(1000);
  Serial.println("Configuration done");

  stav = 1;
  Serial.println("Auto-calibration in 3 sec - prepare to move robot!");
  delay(3000);
}

//--------------------------------------------------------------------
// Main loop
//--------------------------------------------------------------------

void loop() {
  bool cudelstav = digitalRead(cudel);

  if (cudelstav == 1 && calibration_done) {
    delay(200);
    cudelstav = digitalRead(cudel);
    if (cudelstav == 0) {
      Serial.println("Zastavujem robota (stop button)");
      zastavsa();
      while (true) { ; }
    }
  }

  if (stav == 0) {
    if (cudelstav == 0) {
      stav = 1;
      Serial.println("Prechadzam na kalibraciu");
    }
  }
  else if (stav == 1) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Calibration start");

    for (uint16_t i = 0; i < 400; i++) {
      qtr.calibrate();
    }

    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(' ');
    }

    digitalWrite(LED_BUILTIN, LOW);
    calibration_done = true;
    stav = 2;

    Serial.println("Kalibracia dokoncena");
    delay(5000);
    Serial.println("Polozit na ciaru");

    casstartu = millis();
  }
  else if (stav == 2) {
    uint16_t position = qtr.readLineBlack(sensorValues);
    if (last_position < 1000 and position == 0){
      analogWrite(LEFT_FWD, rych_tocenia);
      analogWrite(LEFT_BWD, 0);
      analogWrite(RIGHT_FWD, 0);
      analogWrite(RIGHT_BWD, 0);}
    else if(last_position > 6000 and position == 0){
      analogWrite(LEFT_FWD, 0);
      analogWrite(LEFT_BWD, 0);
      analogWrite(RIGHT_FWD, rych_tocenia);
      analogWrite(RIGHT_BWD, 0);}
    else{
    int error = (int)position - 3500;
    int correction = error * kp / 100;

    // TU JE ZMENA – bez constrain!
    int leftSpeed  = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    setMotors(leftSpeed, rightSpeed);
    int last_position = position;
    }
  }
  else if (stav == 3) {
    zastavsa();
  }
}
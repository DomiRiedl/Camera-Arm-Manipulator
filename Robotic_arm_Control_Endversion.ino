#include <Arduino.h>
#include <esp_system.h>

// === Vorab-Deklarationen ===
void IRAM_ATTR handleEncA();
void IRAM_ATTR handleEncB();
void IRAM_ATTR handleEncA2();
void IRAM_ATTR handleEncB2();
void IRAM_ATTR handleEncoderA_DC();
void IRAM_ATTR handleIndex_DC();
void gotoMMSync(float z1MM, float z2MM);
void calibrateSync(class Achse &A, class Achse &B);
void calibrateDC();
void stopMotorDC();

// === Forward-Declaration für globale Achse-Pointer ===
class Achse;
Achse* achsePtr1 = nullptr;
Achse* achsePtr2 = nullptr;

// === Physikalische Einstellungen ===
const float physischeStreckeMM = 220.0;
const float zielToleranzMM     =   0.15;

// --- Pitch Polynomial Coefficients ---
const float pitchA = -3.49950640e-06;
const float pitchB =  1.92481110e-03;
const float pitchC = -0.533919725;
const float pitchD =  35.314888;

// --- Roll Mapping Data ---
const int   rollN     = 11;
const float phiData[rollN]  = {  0,  3,  6,  9, 12, 15, 18, 21, 24, 27, 30};
const float diffData[rollN] = {  0, 20, 40, 60, 80,100,120,140,160,180,200};

// === Inkrementelle Winkelsteuerung ===
float currentPitch = 0.0f;
float currentRoll  = 0.0f;

bool  pitchActive  = false;
bool  rollActive   = false;

const float MAX_STEP_PITCH =  2.0f;
const float MAX_STEP_ROLL  =  2.0f;
const int   DEADZONE_STEP  = 200;

bool useIncremental = false;

// === STEP/DIR-Pins für Achsen ===
const int stepPin1 = 32;
const int dirPin1  = 33;
const int stepPin2 = 19;
const int dirPin2  = 18;

// === Joystick-Pins ===
const int joyPinX  = 14;  // ADC1_CH2 → Roll / DC-Motor
const int joyPinY  = 27;  // ADC2_CH0 → Pitch / Sync-Achsen
const int joyPinDC = 4;   // ADC2_CH4 → DC-Motor

// === DC-Motor-Pins ===
const int PIN_PWM   = 23;  // PWM-Ausgang
const int PIN_DIR   = 22;  // Richtung
const int PIN_LIMIT = 15;  // Limit-Switch (NO, INPUT_PULLUP)
const int ENC_A_DC  = 21;  // Encoder A
const int ENC_B_DC  = 13;  // Encoder B
const int ENC_X_DC  = 2;   // Encoder Index

// === PWM-Konstanten DC ===
const uint8_t DUTY_FORWARD   = 40;
const uint8_t DUTY_BACKWARD  = 70;
const int     FREQ_FORWARD   =  70;
const int     FREQ_BACKWARD  =  90;

// === Deadzone Joystick DC ===
const int MIDPOINT = 2048;
const int DEADZONE = 200;

long targetTicks1 = 0;
long targetTicks2 = 0;
unsigned long lastStepTime1 = 0;
unsigned long lastStepTime2 = 0;
bool pulseState1 = false;
bool pulseState2 = false;

// === Achse-Klasse mit Encoder (Stepper) ===
class Achse {
public:
  Achse(int stepPin, int dirPin, int limit1, int limit2,
        int encA, int encB, bool invert, unsigned int stepDelay)
    : _stepPin(stepPin), _dirPin(dirPin),
      limitSwitch1(limit1), limitSwitch2(limit2),
      encoderPinA(encA), encoderPinB(encB),
      invertDirection(invert), stepDelayMicros(stepDelay) {}

  void setup(bool isFirst) {
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(limitSwitch1, INPUT_PULLUP);
    pinMode(limitSwitch2, INPUT_PULLUP);
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    if (isFirst) {
      achsePtr1 = this;
      attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncA, CHANGE);
      attachInterrupt(digitalPinToInterrupt(encoderPinB), handleEncB, CHANGE);
    } else {
      achsePtr2 = this;
      attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncA2, CHANGE);
      attachInterrupt(digitalPinToInterrupt(encoderPinB), handleEncB2, CHANGE);
    }
  }

  long getTicks()          const { return encoderTicks; }
  unsigned int getStepDelay() const { return stepDelayMicros; }
  long getToleranceTicks() const { return (long)(zielToleranzMM / mmProTick); }
  int getLimit1Pin()       const { return limitSwitch1; }
  int getLimit2Pin()       const { return limitSwitch2; }

  void setMaxEncoderTicksVal(long v) { maxEncoderTicks = v; }
  void setMmProTickVal(float v)      { mmProTick       = v; }
  void setCalibratedVal(bool v)      { calibrated      = v; }
  void setEncoderTicksVal(long v)    { encoderTicks    = v; }
  void setDirectionCW(bool cw)       { digitalWrite(_dirPin, invertDirection ? !cw : cw); }

  void resetEncoder() {
    encoderTicks = 0;
    lastEncoded  = 0;
  }

  public:
  // Gibt die aktuelle Position in mm zurück (0…physischeStreckeMM)
  float getPositionMM() const {
    // encoderTicks und mmProTick sind private, 
    // aber hier in der Klasse zugänglich
    long ticks = encoderTicks;
    float zMM = physischeStreckeMM - ticks * mmProTick;
    // Bei invertDirection ist die Richtung umgedreht:
    return invertDirection ? physischeStreckeMM - zMM : zMM;
  }


  void handleEncoder() {
    int msb = digitalRead(encoderPinA);
    int lsb = digitalRead(encoderPinB);
    int code = (msb << 1) | lsb;
    int sum  = (lastEncoded << 2) | code;
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderTicks++;
    else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderTicks--;
    lastEncoded = code;
  }

  long toTicks(float zMM) const {
    zMM = constrain(zMM, 0.0, physischeStreckeMM);
    zMM = physischeStreckeMM - zMM;
    long zT = (long)(zMM / mmProTick);
    if (invertDirection) zT = maxEncoderTicks - zT;
    return zT;
  }
  void stepHigh() const { digitalWrite(_stepPin, HIGH); }
  void stepLow()  const { digitalWrite(_stepPin, LOW); }

private:
  int _stepPin, _dirPin;
  int limitSwitch1, limitSwitch2;
  int encoderPinA, encoderPinB;
  bool invertDirection;
  volatile long encoderTicks = 0;
  volatile int  lastEncoded  = 0;
  long maxEncoderTicks       = 0;
  float mmProTick            = 0;
  bool calibrated            = false;
  unsigned int stepDelayMicros;
};

// Instanzen
Achse achse1(stepPin1, dirPin1, 26, 25, 34, 35, false, 210);
Achse achse2(stepPin2, dirPin2, 17, 5, 36, 39, true,  215);

// ISR für Stepper
void IRAM_ATTR handleEncA()  { if (achsePtr1) achsePtr1->handleEncoder(); }
void IRAM_ATTR handleEncB()  { if (achsePtr1) achsePtr1->handleEncoder(); }
void IRAM_ATTR handleEncA2() { if (achsePtr2) achsePtr2->handleEncoder(); }
void IRAM_ATTR handleEncB2() { if (achsePtr2) achsePtr2->handleEncoder(); }

// === DC-Motor Kontrolle ===
volatile long encoderTicksDC = 0;
volatile bool indexTriggered = false;

void IRAM_ATTR handleEncoderA_DC() {
  bool b = digitalRead(ENC_B_DC);
  if (digitalRead(ENC_A_DC) == b) encoderTicksDC++;
  else                            encoderTicksDC--;
}

void IRAM_ATTR handleIndex_DC() {
  indexTriggered = true;
}

void stopMotorDC() {
  // PWM-Wert 0 → Motor aus
  ledcWrite(PIN_PWM, 0);
}

void calibrateDC() {
  Serial.println("--- DC-Motor Calibration starts ---");
  // Rückwärts zum Limit
  digitalWrite(PIN_DIR, HIGH);
  ledcAttach(PIN_PWM, FREQ_BACKWARD, 8);
  ledcWrite(PIN_PWM, DUTY_BACKWARD);

  while (digitalRead(PIN_LIMIT) == HIGH) {
    delay(10);
  }
  stopMotorDC();
  Serial.println("DC Limit-Switch reached --> Encoder-Ticks to 0.");
  encoderTicksDC = 0;

  // Kurzer Vorschub vorwärts
  digitalWrite(PIN_DIR, LOW);
  ledcAttach(PIN_PWM, FREQ_FORWARD, 8);
  ledcWrite(PIN_PWM, DUTY_FORWARD);
  delay(200);
  stopMotorDC();
}

// === Kinematik & Mapping ===
float solveLavg(float theta) {
  float x = physischeStreckeMM / 2;
  for (int i = 0; i < 10; i++) {
    float f  = ((pitchA * x + pitchB) * x + pitchC) * x + pitchD - theta;
    float df = (3 * pitchA * x + 2 * pitchB) * x + pitchC;
    x -= f / df;
  }
  return constrain(x, 0.0, physischeStreckeMM);
}

float interpLdiff(float phi) {
  phi = fabs(phi);
  if (phi <= phiData[0]) return diffData[0];
  if (phi >= phiData[rollN - 1]) return diffData[rollN - 1];
  int idx = 0;
  while (phi > phiData[idx + 1]) idx++;
  float t = (phi - phiData[idx]) / (phiData[idx + 1] - phiData[idx]);
  float v = diffData[idx] + t * (diffData[idx + 1] - diffData[idx]);
  if (phi > 10) v *= 0.85;
  return v;
}

void calibrateSync(Achse &A, Achse &B) {
  enum Phase { LEFT, RIGHT, MID, DONE };
  Phase pA = LEFT, pB = LEFT;
  bool pulseA = false, pulseB = false;
  unsigned long tA = micros(), tB = micros();

  // Zeitpunkte für Achse A
  unsigned long t0A = micros();         // Startzeitpunkt (Beginn LEFT)
  unsigned long tLR_A = 0;              // Zeit when reached RIGHT limit
  // (optional) unsigned long tRM_A = 0; // Zeit when reached MID (falls gewünscht)

  // Zeitpunkte für Achse B
  unsigned long t0B = micros();
  unsigned long tLR_B = 0;

  long mA = 0, mB = 0;
  A.setDirectionCW(true);
  B.setDirectionCW(true);

  while (pA != DONE || pB != DONE) {
    unsigned long now = micros();

    // --- Achse A ---
    if (pA != DONE && now - tA >= A.getStepDelay()) {
      if (!pulseA) {
        A.stepHigh();
        pulseA = true;
      } else {
        A.stepLow();
        pulseA = false;

        if (pA == LEFT && digitalRead(A.getLimit1Pin()) == LOW) {
          Serial.println("Axis A: first Limit Switch triggered");
          A.resetEncoder();
          A.setDirectionCW(false);
          pA = RIGHT;
          tLR_A = micros();              // Zeitpunkt, an dem RIGHT-Limit erreicht wurde
        }
        else if (pA == RIGHT && digitalRead(A.getLimit2Pin()) == LOW) {
          Serial.println("Axis A: second Limit switch triggered");
          mA = abs(A.getTicks());
          A.setMaxEncoderTicksVal(mA);
          A.setMmProTickVal(physischeStreckeMM / mA);
          A.setDirectionCW(true);
          pA = MID;
        }
        else if (pA == MID && abs(A.getTicks() - (mA/2)) <= A.getToleranceTicks()) {
          Serial.println("Axis A: MID - position");
          A.setCalibratedVal(true);
          pA = DONE;
        }
      }
      tA = now;
    }

    // --- Achse B ---
    if (pB != DONE && now - tB >= B.getStepDelay()) {
      if (!pulseB) {
        B.stepHigh();
        pulseB = true;
      } else {
        B.stepLow();
        pulseB = false;

        if (pB == LEFT && digitalRead(B.getLimit1Pin()) == LOW) {
          Serial.println("Axis B: first Limit Switch triggered");
          B.resetEncoder();
          B.setDirectionCW(false);
          pB = RIGHT;
          tLR_B = micros();              // Zeitpunkt, an dem RIGHT-Limit erreicht wurde
        }
        else if (pB == RIGHT && digitalRead(B.getLimit2Pin()) == LOW) {
          Serial.println("Axis B: second Limit switch triggered");
          mB = abs(B.getTicks());
          B.setMaxEncoderTicksVal(mB);
          B.setMmProTickVal(physischeStreckeMM / mB);
          B.setDirectionCW(true);
          pB = MID;
        }
        else if (pB == MID && abs(B.getTicks() - (mB/2)) <= B.getToleranceTicks()) {
          Serial.println("Axis B: MID - Position");
          B.setCalibratedVal(true);
          pB = DONE;
        }
      }
      tB = now;
    }
  }

  Serial.println("Completed Calibration!");

  // Ausgabe der Zeiten
  unsigned long durationA = tLR_A - t0A;
  unsigned long durationB = tLR_B - t0B;
  Serial.printf("Axis A: Time LEFT→RIGHT = %lu µs (%.2f s)\n", durationA, durationA / 1e6);
  Serial.printf("Axis B: Time LEFT→RIGHT = %lu µs (%.2f s)\n", durationB, durationB / 1e6);
}


// Synchronized Move
void gotoMMSync(float z1MM, float z2MM) {
  long tgt1 = achse1.toTicks(z1MM);
  long tgt2 = achse2.toTicks(z2MM);
  long tol1 = achse1.getToleranceTicks();
  long tol2 = achse2.getToleranceTicks();
  bool dir1 = (tgt1 > achse1.getTicks());
  bool dir2 = (tgt2 > achse2.getTicks());
  achse1.setDirectionCW(dir1);
  achse2.setDirectionCW(dir2);

  long lastD1 = abs(achse1.getTicks() - tgt1);
  long lastD2 = abs(achse2.getTicks() - tgt2);
  bool p1 = false, p2 = false;
  unsigned long l1 = micros(), l2 = micros();
  bool done1 = false, done2 = false;

  while (!done1 || !done2) {
    unsigned long now = micros();
    // Axis 1
    if (!done1 && now - l1 >= achse1.getStepDelay()) {
      if (!p1) {
        achse1.stepHigh();
        p1 = true;
      } else {
        achse1.stepLow();
        p1 = false;
        long d1 = abs(achse1.getTicks() - tgt1);
        if (d1 > lastD1) {
          dir1 = !dir1;
          achse1.setDirectionCW(dir1);
        }
        lastD1 = d1;
        if (d1 <= tol1) done1 = true;
      }
      l1 = now;
    }
    // Axis 2
    if (!done2 && now - l2 >= achse2.getStepDelay()) {
      if (!p2) {
        achse2.stepHigh();
        p2 = true;
      } else {
        achse2.stepLow();
        p2 = false;
        long d2 = abs(achse2.getTicks() - tgt2);
        if (d2 > lastD2) {
          dir2 = !dir2;
          achse2.setDirectionCW(dir2);
        }
        lastD2 = d2;
        if (d2 <= tol2) done2 = true;
      }
      l2 = now;
    }
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2000);
  analogReadResolution(12);

  pinMode(joyPinX, INPUT);
  pinMode(joyPinY, INPUT);
  pinMode(joyPinDC, INPUT);

  achse1.setup(true);
  achse2.setup(false);

  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_LIMIT, INPUT_PULLUP);
  pinMode(ENC_A_DC, INPUT_PULLUP);
  pinMode(ENC_B_DC, INPUT_PULLUP);
  pinMode(ENC_X_DC, INPUT_PULLUP);

  // PWM-Kanal initialisieren
  ledcAttach(PIN_PWM, FREQ_FORWARD, 8);

  attachInterrupt(digitalPinToInterrupt(ENC_A_DC), handleEncoderA_DC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_X_DC), handleIndex_DC, RISING);

  // 1) Aufforderung zum Kalibrieren oder Überspringen
  Serial.println();
  Serial.println("Insert 'Start' to start the Calibration");
  Serial.println("Insert 'Skip' to skip the Calibration");
  Serial.print("> ");

  // 2) Blockierende Schleife, bis "Start" oder "Skip" eingegeben wird
  bool skipCalibration = false;
  while (true) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.equalsIgnoreCase("Start")) {
        skipCalibration = false;
        Serial.println("Stepper-Calibration started...");
        break;
      } 
      else if (cmd.equalsIgnoreCase("Skip")) {
        skipCalibration = true;
        Serial.println("Stepper-Calibration skipped");
        break;
      } 
      else {
        Serial.println("Invalid input. Please insert 'Start' or 'Skip'.");
        Serial.print("> ");
      }
    }
  }

  // 3) Je nach Auswahl Kalibrierung oder Skip
  if (skipCalibration) {
    long M   = 30000, MID = M/2;
    achse1.setMaxEncoderTicksVal(M);
    achse1.setMmProTickVal(physischeStreckeMM / M);
    achse1.setCalibratedVal(true);
    achse1.setEncoderTicksVal(MID);

    achse2.setMaxEncoderTicksVal(M);
    achse2.setMmProTickVal(physischeStreckeMM / M);
    achse2.setCalibratedVal(true);
    achse2.setEncoderTicksVal(MID);

    Serial.println("Stepper-MID-Position set");
  } 
  else {
    calibrateSync(achse1, achse2);
  }

  // 4) DC-Motor-Kalibrierung (Bleibt unverändert)
  calibrateDC();

  // 5) Steuerungsmodus abfragen (Absolut oder Inkremental)
  useIncremental = false;
  Serial.println();
  Serial.println("Choose Control Method:");
  Serial.println("  [a] Absolute movement");
  Serial.println("  [i] Inkremental movement");
  Serial.print("> ");
  while (true) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.equalsIgnoreCase("a")) {
        useIncremental = false;
        Serial.println("Modus: Absolut");
        break;
      } 
      else if (cmd.equalsIgnoreCase("i")) {
        useIncremental = true;
        Serial.println("Modus: Inkremental");
        break;
      } 
      else {
        Serial.println("Ungültig – bitte 'a' oder 'i' eingeben.");
        Serial.print("> ");
      }
    }
  }
  Serial.println("Ready for Controlling!");
}


void loop() {
  // 1) Serielle Befehle (Modus oder Reset)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'a' || c == 'A') setMode(false);
    else if (c == 'i' || c == 'I') setMode(true);
    else if (c == 't' || c == 'T') setMode(!useIncremental);
    else if (c == 'r' || c == 'R') {
      Serial.println("Starte Programm neu…");
      delay(100);
      esp_restart();
    }
  }

  // 2) DC-Motor Steuerung (unverändert)
  int joyVal = analogRead(joyPinDC);
  bool forward  = (joyVal < MIDPOINT - DEADZONE);
  bool backward = (joyVal > MIDPOINT + DEADZONE);
  if (digitalRead(PIN_LIMIT) == LOW) {
    stopMotorDC();
  }
  else if (forward) {
    ledcAttach(PIN_PWM, FREQ_FORWARD, 8);
    digitalWrite(PIN_DIR, LOW);
    ledcWrite(PIN_PWM, DUTY_FORWARD);
  }
  else if (backward) {
    ledcAttach(PIN_PWM, FREQ_BACKWARD, 8);
    digitalWrite(PIN_DIR, HIGH);
    ledcWrite(PIN_PWM, DUTY_BACKWARD);
  }
  else {
    stopMotorDC();
  }

  // 3) Winkelsteuerung (absolut oder inkrementell)
  int rawY = analogRead(joyPinY);
  int rawX = analogRead(joyPinX);   // ### Rohwert für Roll (Joystick X)
  float pitch, roll;

  if (useIncremental) {
    // inkrementelle Steuerung …
    static float prevLl = -1.0f, prevLr = -1.0f;
    if (!pitchActive) {
      if (rawY > MIDPOINT + DEADZONE_STEP) {
        float norm  = float(rawY - (MIDPOINT+DEADZONE_STEP)) / (4095 - (MIDPOINT+DEADZONE_STEP));
        float delta = norm * MAX_STEP_PITCH;
        currentPitch += delta; pitchActive = true;
        Serial.printf("Pitch +%.2f°\n", delta);
      } else if (rawY < MIDPOINT - DEADZONE_STEP) {
        float norm  = float((MIDPOINT-DEADZONE_STEP) - rawY) / (MIDPOINT-DEADZONE_STEP);
        float delta = norm * MAX_STEP_PITCH;
        currentPitch -= delta; pitchActive = true;
        Serial.printf("Pitch -%.2f°\n", delta);
      }
    } else if (abs(rawY - MIDPOINT) <= DEADZONE_STEP) {
      pitchActive = false;
    }
    currentPitch = constrain(currentPitch, -25.0f, 25.0f);

    if (!rollActive) {
      if (rawX > MIDPOINT + DEADZONE_STEP) {
        float norm  = float(rawX - (MIDPOINT+DEADZONE_STEP)) / (4095 - (MIDPOINT+DEADZONE_STEP));
        float delta = norm * MAX_STEP_ROLL;
        currentRoll += delta; rollActive = true;
        Serial.printf("Roll +%.2f°\n", delta);
      } else if (rawX < MIDPOINT - DEADZONE_STEP) {
        float norm  = float((MIDPOINT-DEADZONE_STEP) - rawX) / (MIDPOINT-DEADZONE_STEP);
        float delta = norm * MAX_STEP_ROLL;
        currentRoll -= delta; rollActive = true;
        Serial.printf("Roll -%.2f°\n", delta);
      }
    } else if (abs(rawX - MIDPOINT) <= DEADZONE_STEP) {
      rollActive = false;
    }
    currentRoll = constrain(currentRoll, -30.0f, 30.0f);

    pitch = currentPitch;
    roll  = currentRoll;
  } else {
    // absolute Steuerung …
    pitch = (rawY / 4095.0f) * 50.0f - 25.0f;
    roll  = (rawX / 4095.0f) * 40.0f - 20.0f;
  }

  // 4) Kinematik & Bewegung (unverändert)
  float Lavg = solveLavg(pitch);
  float diff = interpLdiff(roll);
  float Ll   = (roll >= 0.0f) ? Lavg - diff * 0.5f : Lavg + diff * 0.5f;
  float Lr   = (roll >= 0.0f) ? Lavg + diff * 0.5f : Lavg - diff * 0.5f;

  static float prevLeft = -1.0f, prevRight = -1.0f;
  if (fabs(Ll - prevLeft) > 1e-3 || fabs(Lr - prevRight) > 1e-3) {
    gotoMMSync(Ll, Lr);
    prevLeft  = Ll;
    prevRight = Lr;
  }

  // --- Serielle Ausgabe für Serial Plotter (eine Zeile, Semikolon als Trenner) ---
  {
    float soll1 = Ll;
    float soll2 = Lr;
    float ist1  = achse1.getPositionMM();
    float ist2  = achse2.getPositionMM();
    float err1  = soll1 - ist1;
    float err2  = soll2 - ist2;

    //Serial.print(pitch, 2);   Serial.print(';');
    //Serial.print(roll,  2);   Serial.print(';');
    Serial.print(soll1, 2);   Serial.print(';');
    Serial.print(ist1,  2);   Serial.print(';');
    Serial.print(err1,  2);   Serial.print(';');
    Serial.print(soll2, 2);   Serial.print(';');
    Serial.print(ist2,  2);   Serial.print(';');
    Serial.print(err2,  2);   Serial.print(';');
    Serial.print(rawX);       Serial.println();
  }

  delay(50);


  // 6) kurze Pause
  delay(50);
}

// Hilfs-Funktion für Modus-Toggle
void setMode(bool inc) {
  useIncremental = inc;
  Serial.print("Modus changed: ");
  Serial.println(useIncremental ? "Inkremental" : "Absolut");
}

const int potPin     = 0;       // ADC1_CH0 auf GPIO0
const float maxAngle = 330.0;   // max. mechanischer Winkel (0…330°)
int offsetRaw = 0;              // speichert den Rohwert bei Upload als Nullpunkt

// --- Filter‐Parameter ---
const float alpha = 0.1;        // Glättungsfaktor (0 < alpha < 1)
float smoothAngle = 0.0;        // hier wird der geglättete Winkel gehalten

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);       // 12 Bit (0…4095)
  analogSetAttenuation(ADC_11db); // 0–3.3 V
  delay(100);

  // 1) Offset‐Kalibrierung: aktuellen Rohwert als Nullpunkt übernehmen
  offsetRaw = analogRead(potPin);
  delay(200);

  // 2) Erste Initialisierung von smoothAngle mit dem aktuellen Winkel
  {
    int raw = analogRead(potPin);
    int diff = raw - offsetRaw;
    if (diff < 0) diff += 4096;
    float angleRaw = diff * (maxAngle / 4095.0);
    // Wrap in ±(maxAngle/2)
    if (angleRaw > (maxAngle * 0.5f)) angleRaw -= maxAngle;
    smoothAngle = angleRaw;
  }
}

void loop() {
  // 1) Rohwert lesen
  int raw = analogRead(potPin);

  // 2) Differenz zum Offset (mit Wrap-around)
  int diff = raw - offsetRaw;
  if (diff < 0) diff += 4096;  

  // 3) Roh-Winkel (0…maxAngle)
  float angleRaw = diff * (maxAngle / 4095.0);

  // 4) Wrap-Logik für ±(maxAngle/2)
  float angleSigned;
  if (angleRaw > (maxAngle * 0.5f)) {
    angleSigned = angleRaw - maxAngle;  // z.B. 329 → –1
  } else {
    angleSigned = angleRaw;             // 0…165 bleibt positiv
  }

  // 5) Exponentielle Glättung (EMA)
  smoothAngle = alpha * angleSigned + (1.0 - alpha) * smoothAngle;

  // 6) Nur den geglätteten Winkel ausgeben (Serial Plotter)
  Serial.println(smoothAngle, 1);  // Ausgabe mit 1 Nachkommastelle

  delay(100);
}

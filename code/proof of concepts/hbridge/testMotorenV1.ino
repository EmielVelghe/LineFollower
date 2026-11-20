// Arduino Nano ESP32 + DRV8833, twee DC-motoren.
// Start automatisch na inschakelen of reset. Test eerst met de wielen vrij.
const int linksIn1 = D2, linksIn2 = D3;
const int rechtsIn1 = D4, rechtsIn2 = D5;

const bool keerLinksOm = false;
const bool keerRechtsOm = false;
const int testSnelheid = 120;  // PWM op een schaal van 0 tot 255
const int maxPwm = 180;

void motor(int in1, int in2, bool omkeren, int snelheid) {
  const int pwm = constrain(snelheid, 0, maxPwm);
  analogWrite(omkeren ? in1 : in2, 0);
  analogWrite(omkeren ? in2 : in1, pwm);
}

void rijd(int links, int rechts) {
  motor(linksIn1, linksIn2, keerLinksOm, links);
  motor(rechtsIn1, rechtsIn2, keerRechtsOm, rechts);
}

void setup() {
  pinMode(linksIn1, OUTPUT);
  pinMode(linksIn2, OUTPUT);
  pinMode(rechtsIn1, OUTPUT);
  pinMode(rechtsIn2, OUTPUT);
  rijd(0, 0);
  delay(3000);

  rijd(testSnelheid, 0);
  delay(1000);
  rijd(0, 0);
  delay(1000);

  rijd(0, testSnelheid);
  delay(1000);
  rijd(0, 0);
  delay(1000);

  rijd(testSnelheid, testSnelheid);
  delay(2000);
  rijd(0, 0);
}

void loop() {
  // Klaar. Druk op RESET om de test te herhalen.
}

#include <Preferences.h>

const int aantalSensoren = 8;
const int sensorPin[aantalSensoren] = {A0, A1, A2, A3, A4, A5, A6, A7};

struct Kalibratie {
  int zwart[aantalSensoren];
  int wit[aantalSensoren];
};

bool geldig(const Kalibratie &k);

Preferences geheugen;
Kalibratie actief, nieuw;
bool gekalibreerd = false;
bool zwartGemeten = false;
bool witGemeten = false;
bool geheugenOpen = false;

bool geldig(const Kalibratie &k) {
  for (int i = 0; i < aantalSensoren; i++) {
    if (k.zwart[i] == k.wit[i] || k.zwart[i] < 0 || k.zwart[i] > 4095 ||
        k.wit[i] < 0 || k.wit[i] > 4095) {
      return false;
    }
  }
  return true;
}

void meetReferentie(int waarden[]) {
  const int aantalMetingen = 32;
  long som[aantalSensoren] = {};
  for (int meting = 0; meting < aantalMetingen; meting++) {
    for (int i = 0; i < aantalSensoren; i++) {
      som[i] += analogRead(sensorPin[i]);
    }
    delay(2);
  }
  for (int i = 0; i < aantalSensoren; i++) {
    waarden[i] = som[i] / aantalMetingen;
  }
}

void kalibreer(char opdracht) {
  if (opdracht == 'z') {
    meetReferentie(nieuw.zwart);
    zwartGemeten = true;
    Serial.println("Zwart gemeten.");
  } else if (opdracht == 'w') {
    meetReferentie(nieuw.wit);
    witGemeten = true;
    Serial.println("Wit gemeten.");
  } else {
    return;
  }

  if (!zwartGemeten || !witGemeten) {
    Serial.println(zwartGemeten ? "Zet alle sensoren boven wit en stuur w."
                                : "Zet alle sensoren boven zwart en stuur z.");
    return;
  }
  if (!geldig(nieuw)) {
    Serial.println("Onvoldoende verschil. Meet zwart en wit opnieuw.");
  } else {
    actief = nieuw;
    gekalibreerd = true;
    if (geheugenOpen && geheugen.putBytes("kalibratie", &actief, sizeof(actief)) == sizeof(actief)) {
      Serial.println("Kalibratie opgeslagen.");
    } else {
      Serial.println("Kalibratie actief tot de volgende herstart. Opslaan mislukt.");
    }
  }
  zwartGemeten = false;
  witGemeten = false;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  geheugenOpen = geheugen.begin("linefollower", false);
  if (geheugenOpen && geheugen.getBytesLength("kalibratie") == sizeof(actief)) {
    gekalibreerd = geheugen.getBytes("kalibratie", &actief, sizeof(actief)) == sizeof(actief)
                   && geldig(actief);
  }
}

void loop() {
  while (Serial.available()) {
    kalibreer(Serial.read());
  }

  Serial.print(gekalibreerd ? "V4 gekalibreerd | " : "V4 ongekalibreerd (z=zwart, w=wit) | ");
  for (int i = 0; i < aantalSensoren; i++) {
    long waarde = analogRead(sensorPin[i]);
    if (gekalibreerd) {
      waarde = map(waarde, actief.zwart[i], actief.wit[i], 0, 1023);
    } else {
      // Bij deze sensorbalk geeft zwart een hogere ruwe meting dan wit.
      waarde = 1023 - waarde / 4;
    }
    waarde = constrain(waarde, 0L, 1023L);
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(waarde);
    Serial.print("\t");
  }
  Serial.println();
  delay(200);
}

#include <Arduino_BuiltIn.h>
#include "Tasker.h"
#include <Wire.h>
#include <iarduino_OLED_txt.h>
#include "iarduino_4LED.h"
#include "SerialTransfer.h"

int ohmValue = 0;
int last_secs = 0, audioValue = 0;
long prev_seconds = 999999, konvert = 0;
int hall_sensor = 0, hall_sensor2 = 0;
int volatile voices_detected = 0;
int last_event = 0;
int did_ended = 0;
int feat_exposed = 0;  // can external features enter inside
int perc = 0;
int limiter_change = 0;
int expander_change = 0;
int noise_around = 0;
uint32_t i;
int h, m, s;
int crystall_spawn_sec = 0;
int power = 0;
int fire_ended = true;
int padding = false;
unsigned long mils = 0;
SerialTransfer pcTransfer;
iarduino_4LED QLED(7, 8);
Tasker tasker;
iarduino_OLED_txt oled(0x3d);

#define OHM_INPUT A4
#define VOLUME_INPUT A2
#define MIC_INPUT A14
#define LED_BLACK_HOLE_FAIL 45
#define LED_MATCH_EXPANDER 52
#define LED_MATCH_LIMITER 53
#define PIN_BUZZER 31
#define LED_CRYSTALL_GROW 3
#define LED_VOICE_DETECTED 23
#define HIDDEN_FEATURES 9
#define BTN_RND 36

#define seconds(s) (millis(1000 * s))


void setup() {
  Serial.begin(115200);

  pcTransfer.begin(Serial);

  pinMode(BTN_RND, OUTPUT);
  pinMode(MIC_INPUT, INPUT);
  pinMode(13, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(VOLUME_INPUT, INPUT);
  pinMode(OHM_INPUT, INPUT);
  pinMode(LED_VOICE_DETECTED, OUTPUT);
  pinMode(LED_CRYSTALL_GROW, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(LED_BLACK_HOLE_FAIL, OUTPUT);
  pinMode(HIDDEN_FEATURES, OUTPUT);
  pinMode(LED_MATCH_EXPANDER, OUTPUT);
  pinMode(LED_MATCH_LIMITER, OUTPUT);



  QLED.begin();
  QLED.point(255, 0);
  QLED.light(7);
  QLED.print("TEST");

  oled.begin(&Wire);
  oled.setFont(SmallFontRus);

  do_self_test();

  perc = random(2250);
  tasker.setInterval(match_limiter, perc);
  perc = random(4500);
  tasker.setInterval(expand_limiter, perc);

  crystall_spawn_sec = random(3750);
  tasker.setInterval(raiser_crystalls, crystall_spawn_sec);
  crystall_spawn_sec = random(2150);
  tasker.setInterval(disraiser_crystalls, crystall_spawn_sec);

  tasker.setInterval(oled_print_info, 100);
  tasker.setInterval(timelaps, 1000);
  tasker.setInterval(accum_noise, 50);
  tasker.setInterval(timebash, 100);

  mils = micros();
  srand(mils);

  Serial.print("[r&q + skynet + met9] srand=");
  Serial.println(mils);
}

void do_self_test() {

  if (!digitalPinHasPWM(LED_CRYSTALL_GROW)) {
    Serial.print("error pin does not have PWM");
    return;
  }

  for (int d = 0; d < 6; d++) {
    digitalWrite(LED_BLACK_HOLE_FAIL, HIGH);

    digitalWrite(LED_VOICE_DETECTED, HIGH);
    delay(22);
    digitalWrite(LED_VOICE_DETECTED, LOW);

    digitalWrite(LED_MATCH_LIMITER, HIGH);
    delay(22);
    digitalWrite(LED_MATCH_LIMITER, LOW);

    digitalWrite(LED_MATCH_EXPANDER, HIGH);
    delay(22);
    digitalWrite(LED_MATCH_EXPANDER, LOW);

    digitalWrite(HIDDEN_FEATURES, HIGH);
    delay(22);
    digitalWrite(HIDDEN_FEATURES, LOW);

    digitalWrite(LED_CRYSTALL_GROW, HIGH);
    delay(22);
    digitalWrite(LED_CRYSTALL_GROW, LOW);

    tone(PIN_BUZZER, 250);
    digitalWrite(PIN_BUZZER, HIGH);
    delay(5);
    digitalWrite(PIN_BUZZER, LOW);
    noTone(PIN_BUZZER);

    digitalWrite(LED_BLACK_HOLE_FAIL, LOW);
  }
}

void buzz(int time = 100, int mtone = 1000) {
  if (!time || !mtone) {
    return;
  }

  tone(PIN_BUZZER, mtone);
  digitalWrite(PIN_BUZZER, HIGH);
  delay(time);
  digitalWrite(PIN_BUZZER, LOW);
  noTone(PIN_BUZZER);
}

void timebash() {
  if (random(13) < 3) {
    if (power >= 70 && expander_change > 100 && limiter_change >= 100) {
      if (random(3) == random(4) == random(1) == random(3)) {
        digitalWrite(LED_BLACK_HOLE_FAIL, HIGH);
        buzz(10, 110);
        Serial.println("what the fuck!?!");
        power -= 20;
        expander_change -= 100;
        digitalWrite(LED_BLACK_HOLE_FAIL, LOW);
      }
    }
  }
}

void accum_noise() {
  noise_around = analogRead(MIC_INPUT);
}

void timelaps() {
  expander_change = max(0, expander_change);
  limiter_change = max(0, limiter_change);

  power = min(int(limiter_change * 3.14),
              abs(expander_change - limiter_change));

  //Serial.print("power:");
  //Serial.println(power);

  // hidden connect of constructors
  bool d = digitalRead(BTN_RND);
  // Serial.print("btn:");
  // Serial.println(d);

  feat_exposed = random(43) > 38;
  if (feat_exposed) {
    digitalWrite(HIDDEN_FEATURES, HIGH);
    if (random(33) <= 3) {
      power = 0;
      buzz(5, 200);
    } else if (random(55) <= 5) {
      buzz(5, 100);
      power++;
    }
    delay(120);
    digitalWrite(HIDDEN_FEATURES, LOW);
  }
}

////////////////////  CRYSTALLLS
void raiser_crystalls() {
  int landing = random(5) > 2;
  if (landing) {
    return;
  }
  if (power > 12) {
    for (int a = 0; a < 15; a++) {
      pinMode(LED_CRYSTALL_GROW, INPUT);
      int rr = analogRead(LED_CRYSTALL_GROW);
      Serial.print("read ");
      Serial.println(rr);
      delay(2);
      rr = map(rr, 0, 1023, 0, 255);
      Serial.print("rr2=");
      Serial.println(rr);
      int v = 0;
      if (random(2) == 1) {
        v = min(255, rr + random(35));
      } else {
        v = min(30, rr - random(35));
      }
      Serial.print("raising => ");
      Serial.println(v);
      pinMode(LED_CRYSTALL_GROW, OUTPUT);
      delay(1);
      digitalWrite(LED_CRYSTALL_GROW, HIGH);
      analogWrite(LED_CRYSTALL_GROW, v);
      delay(8);
    }
  }
}

void disraiser_crystalls() {
  int landing = random(2) == 1;
  if (landing && power < 2) {
    return;
  }
  pinMode(LED_CRYSTALL_GROW, INPUT);
  int rr = analogRead(LED_CRYSTALL_GROW);
  delay(1);

  for (int a = rr; a > rr; a--) {
    delay(3);
    if (random(3) == 1) {
      power -= 1;
    }
    pinMode(LED_CRYSTALL_GROW, OUTPUT);
    analogWrite(LED_CRYSTALL_GROW, max(random(10), random(255 / rr)));
  }
  //padding = false;
}

//////// SPACES
void match_limiter() {
  bool landing = random(25) == 1;
  if (!landing) {
    return;
  }

  int a = 0;
  int r = random(50);
  if (r) {
    limiter_change += r;
    digitalWrite(LED_MATCH_LIMITER, HIGH);
    delay(150 + r);
    digitalWrite(LED_MATCH_LIMITER, LOW);
  }
}

void expand_limiter() {
  bool landing = random(25) == 3;
  if (!landing) {
    return;
  }
  int r = random(5);
  expander_change += r;
  Serial.print("grow limiter by ");
  Serial.println(r);
  digitalWrite(LED_MATCH_EXPANDER, HIGH);
  delay(150 + r);
  digitalWrite(LED_MATCH_EXPANDER, LOW);
}

///////// OTHER
void oled_print_info() {
  oled.setCursor(0, 0);
  oled.print("pwr ");
  oled.print(power);

  oled.setCursor(0, 1);
  oled.print("lim ");
  oled.print(limiter_change);

  oled.setCursor(0, 2);
  oled.print("exp ");
  oled.print(expander_change);

  oled.setCursor(0, 3);
  oled.print("nse ");
  oled.print(noise_around);

  // Serial.print("noise:");
  // Serial.println(noise_around);

  oled.setCursor(0, 4);
  oled.print("ohm ");
  oled.print(ohmValue);

  // Serial.print("vol:");
  // Serial.println(audioValue);

  oled.setCursor(0, 5);
  oled.print("snd ");
  oled.print(audioValue);

  QLED.print(voices_detected, 0);
}

void loop() {
  tasker.loop();  // after drug dealer automated-visit at 6am

  ohmValue = analogRead(OHM_INPUT);
  // Serial.print("ohmValue:");
  // Serial.println(ohmValue);

  audioValue = analogRead(VOLUME_INPUT);
  // Serial.print("audioValue:");
  // Serial.println(audioValue);

  konvert = map(audioValue, 0, 1023, 0, 255);
  // Serial.print("konvert:");
  // Serial.println(konvert);

  if (abs(audioValue - ohmValue) > konvert) {
    fire_ended = false;
    last_event = millis();
    digitalWrite(LED_VOICE_DETECTED, HIGH);
  } else {
    if (millis() - last_event >= 1) {
      digitalWrite(LED_VOICE_DETECTED, LOW);
      if (!fire_ended) {
        voices_detected += 1;
      }
      fire_ended = true;
    }
  }
}
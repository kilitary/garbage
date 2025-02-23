#include <Arduino_BuiltIn.h>
#include "Tasker.h"
#include <Wire.h>
#include <iarduino_OLED_txt.h>
#include "iarduino_4LED.h"
#include "SerialTransfer.h"
#include <avr/wdt.h>

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
int maxmic = 0;
int expander_change = 0;
unsigned long epoch = 0;
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
iarduino_OLED_txt oled(0x3c);

#define RGB_R 1
#define RGB_G 2
#define RGB_B 3
#define OHM_INPUT A4
#define VOLUME_INPUT A2
#define MIC_INPUT A14
#define LED_BLACK_HOLE_FAIL 45
#define MAX_MIC_INPUT 48
#define LED_MATCH_EXPANDER 52
#define LED_MATCH_LIMITER 53
#define PIN_BUZZER 31
#define LED_CRYSTALL_GROW A0
#define LED_VOICE_DETECTED 23
#define HIDDEN_FEATURES 9
#define BTN_SLEEP 36

#define seconds(s) (millis(1000 * s))


void setup() {
  Serial.begin(115200);

  delay(10);

  pcTransfer.begin(Serial);

  //wd_setup();

  pinMode(BTN_SLEEP, INPUT);
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
  pinMode(MAX_MIC_INPUT, INPUT);
  pinMode(LED_MATCH_EXPANDER, OUTPUT);
  pinMode(LED_MATCH_LIMITER, OUTPUT);
  pinMode(RGB_R, OUTPUT);
  pinMode(RGB_G, OUTPUT);
  pinMode(RGB_B, OUTPUT);

  QLED.begin();
  QLED.point(255, 0);
  QLED.light(7);
  QLED.print("TEST");

  oled.begin(&Wire);
  oled.setFont(SmallFontRus);

  do_self_test();

  perc = random(1250);
  tasker.setInterval(match_limiter, perc);
  perc = random(2500);
  tasker.setInterval(expand_limiter, perc);

  crystall_spawn_sec = random(1250);
  tasker.setInterval(raiser_crystalls, crystall_spawn_sec);
  crystall_spawn_sec = random(2250);
  tasker.setInterval(disraiser_crystalls, crystall_spawn_sec);

  tasker.setInterval(regular, 1000);
  tasker.setInterval(timebash, 1000);
  tasker.setInterval(oled_dump, 55);
  tasker.setInterval(cost, 100);
  tasker.setInterval(accum_noise, 50);

  mils = millis();
  srand(mils);

  Serial.print("[r&q + skynet + met9] srand=");
  Serial.println(mils);
}

void do_self_test() {

  // if (!digitalPinHasPWM(LED_CRYSTALL_GROW)) {
  //   Serial.print("error pin does not have PWM");
  //   return;
  // }

  for (int d = 0; d < 6; d++) {
    digitalWrite(LED_BLACK_HOLE_FAIL, HIGH);
    delay(2);
    digitalWrite(LED_VOICE_DETECTED, HIGH);
    delay(2);
    digitalWrite(LED_VOICE_DETECTED, LOW);
    delay(2);
    digitalWrite(LED_MATCH_LIMITER, HIGH);
    delay(2);
    digitalWrite(LED_MATCH_LIMITER, LOW);
    delay(2);
    digitalWrite(LED_MATCH_EXPANDER, HIGH);
    delay(2);
    digitalWrite(LED_MATCH_EXPANDER, LOW);
    delay(2);
    digitalWrite(HIDDEN_FEATURES, HIGH);
    delay(2);
    digitalWrite(HIDDEN_FEATURES, LOW);
    delay(2);
    digitalWrite(LED_CRYSTALL_GROW, HIGH);
    delay(2);
    digitalWrite(LED_CRYSTALL_GROW, LOW);
    delay(2);

    tone(PIN_BUZZER, 450);
    digitalWrite(PIN_BUZZER, HIGH);
    delay(5);
    digitalWrite(PIN_BUZZER, LOW);
    noTone(PIN_BUZZER);

    digitalWrite(LED_BLACK_HOLE_FAIL, LOW);
    delay(100);
  }
}

void wd_setup() {
  cli();
  wdt_reset();
  wdt_enable(WDTO_2S);
  sei();
  Serial.println("watchdog set");
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
  static bool running = false;
  if (running) {
    return;
  }
  running = true;

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
  running = false;
}

void accum_noise() {
  noise_around = analogRead(MIC_INPUT);
}

void regular() {
  static bool running = false;
  if (running) {
    return;
  }
  running = true;

  // hidden connect of constructors

  bool sleepB = digitalRead(BTN_SLEEP);
  if (sleepB) {
    Serial.println("sleep requested");
    //set_cp
  }

  expander_change = max(0, expander_change);
  limiter_change = max(0, limiter_change);

  power = min(limiter_change + power * 4,
              abs(expander_change - limiter_change));

  //Serial.print("power:");
  //Serial.println(power)

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

  digitalWrite(RGB_R, noise_around);
  digitalWrite(RGB_G, maxmic);
  digitalWrite(RGB_B, audioValue);

  running = false;
}

////////////////////  CRYSTALLLS
void raiser_crystalls() {
  static bool running = false;
  if (running) {
    return;
  }
  running = true;
  int landing = random(5) > 2;
  if (landing) {
    return;
  }
  if (power > 12) {
    int f = random(10, 20);
    for (int a = 0; a < f; a++) {
      pinMode(LED_CRYSTALL_GROW, INPUT);
      delay(2);
      int rr = analogRead(LED_CRYSTALL_GROW);
      Serial.print("read ");
      Serial.println(rr);
      delay(2);
      rr = map(rr, 0, 1023, 0, 255);
      Serial.print("rr2=");
      Serial.println(rr);
      int v = 0;
      if (random(2) == 1) {
        v = abs(min(rr, rr + random(13)));
      } else {
        v = abs(min(rr, rr - random(13)));
      }
      Serial.print("raising => ");
      Serial.println(v);
      delay(2);
      pinMode(LED_CRYSTALL_GROW, OUTPUT);
      delay(1);
      analogWrite(LED_CRYSTALL_GROW, v);
    }
  }
  running = false;
}

void disraiser_crystalls() {
  static bool running = false;
  if (running) {
    return;
  }
  running = true;

  int landing = random(2) == 1;
  if (landing && power > 2) {
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
  running = false;
}

//////// SPACES
void match_limiter() {
  static bool running = false;
  if (running) {
    return;
  }
  running = true;

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

  running = false;
}

void expand_limiter() {
  static bool running = false;
  if (running) {
    return;
  }
  running = true;

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

  running = false;
}

///////// OTHER
void oled_dump() {
  static bool running = false;
  if (running) {
    return;
  }
  running = true;

  oled.setCursor(0, 0);
  oled.print("pwr: ");
  oled.print(power);

  oled.setCursor(0, 1);
  oled.print("lim: ");
  oled.print(limiter_change);

  oled.setCursor(0, 2);
  oled.print("exp: ");
  oled.print(expander_change);

  oled.setCursor(0, 3);
  oled.print("nse: ");
  oled.print(noise_around);

  // Serial.print("noise:");
  // Serial.println(noise_around);

  oled.setCursor(0, 4);
  oled.print("ohm: ");
  oled.print(ohmValue);

  // Serial.print("vol:");
  // Serial.println(audioValue);

  oled.setCursor(0, 5);
  oled.print("snd: ");
  oled.print(audioValue);

  oled.setCursor(0, 6);
  oled.print("epoch: ");
  oled.print(epoch);

  oled.setCursor(0, 7);
  oled.print("maxmic: ");
  oled.print(maxmic);

  QLED.print(voices_detected, 0);

  running = false;
}

void cost() {
  static bool running = false;
  if (running) {
    return;
  }
  running = true;

  maxmic = analogRead(MAX_MIC_INPUT);
  // Serial.print("maxmic:");
  // Serial.println(maxmic);

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

  running = false;
}

void loop() {
  epoch++;

  tasker.loop();  // after drug dealer automated-visit at 6am
}
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
#define LED_MATCH_EXPANDER 34
#define LED_MATCH_LIMITER 53
#define PIN_BUZZER 31
#define LED_CRYSTALL_GROW A5
#define LED_VOICE_DETECTED 23
#define HIDDEN_FEATURES 9
#define BTN_SLEEP 36

#define seconds(s) (millis(1000 * s))
#define sleep(s) (delay(seconds(s)))


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
  QLED.print("3914");

  oled.begin(&Wire);
  oled.setFont(SmallFontRus);

  self_test();

  perc = 1000 + random(1250);
  Serial.println(perc);
  tasker.setInterval(match_limiter, perc);
  perc = 1000 + random(1500);
  Serial.println(perc);
  tasker.setInterval(expand_limiter, perc);

  crystall_spawn_sec = random(1250);
  tasker.setInterval(raiser_crystalls, crystall_spawn_sec);
  crystall_spawn_sec = random(1250);
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

void self_test() {

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

void buzz(int time = 0, int mtone = 0) {
  if (!time || !mtone) {
    noTone(PIN_BUZZER);
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
        buzz(1011, 5110);
        Serial.println("what the fuck!?!");
        power -= 5;
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
    running = false;
    return;
  }

  feat_exposed = random(43) < 9;
  if (feat_exposed) {
    digitalWrite(HIDDEN_FEATURES, HIGH);
    if (random(33) <= 15) {
      if (power) {
        power -= min(5, abs(expander_change - limiter_change));
      }
      //buzz(5, 200);
    } else if (random(55) <= 35) {
      //buzz(1, 1000);
      power += min(5, abs(expander_change - limiter_change));
    } else {
      if (power) {
        power -= 2;
      }
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
  static bool moving_down = false;

  int landing = random(15) > 13;
  if (running || landing) {
    return;
  }

  running = true;

  if (power > 13) {
    int f = random(3, 10);
    for (int a = 0; a < f; a++) {
      //buzz(1, 112110);
      //delay(100);
      //buzz();
      pinMode(LED_CRYSTALL_GROW, INPUT);
      delay(1);
      int rr = analogRead(LED_CRYSTALL_GROW);
      rr = map(rr, 0, 1023, 0, 255);
      int v = 0;

      if (moving_down) {
        v = rr - random(2);
        if (v <= 5) {
          moving_down = false;
        }
      } else {
        v = rr + random(2);
        if (v >= 250) {
          moving_down = true;
        }
      }
      String str = "";
      sprintf(str.c_str(), "raising => %03d", v);
      Serial.print(str);
      pinMode(LED_CRYSTALL_GROW, OUTPUT);
      delay(10);
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
    buzz(100, a * 100);
    delay(100);
    if (random(3) == 1) {
      power -= 10;
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

  bool landing = random(125) < 15;
  if (landing) {
    running = false;
    return;
  }

  int r = random(141) > 131;
  if (r) {
    r = 1 + random(3);
    limiter_change += r;
    Serial.print("expand limit matcher by ");
    Serial.println(r);
    digitalWrite(LED_MATCH_LIMITER, HIGH);
    delay(50 + r);
    digitalWrite(LED_MATCH_LIMITER, LOW);
    //buzz(5, 10);
  }

  running = false;
}

void expand_limiter() {
  static bool running = false;

  bool landing = (power < 5)
                 || random(expander_change % 4) < random(limiter_change % 3);
  if (running || landing) {
    return;
  }

  running = true;

  int r = 1 + random(5);
  expander_change += r;
  Serial.print("expand limiter by ");
  Serial.println(r);
  digitalWrite(LED_MATCH_EXPANDER, HIGH);
  delay(50 + r);
  digitalWrite(LED_MATCH_EXPANDER, LOW);
  //buzz(5, 10);
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

  char cmd[100] = { 0x0 };
  int numByte = 0;
  while (Serial.available()) {
    // read the most recent byte (which will be from 0 to 255):
    byte b = Serial.read();
    cmd[numByte] = (char)b;
    numByte++;
  }
  if (cmd[0]) {
    Serial.print("got ");
    Serial.println(cmd);
    cmd[0] = 0x0;
  }

  tasker.loop();  // after drug dealer automated-visit at 6am
}
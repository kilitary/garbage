#include <Arduino_BuiltIn.h>
#define TASKER_MAX_TASKS 40
#include "Tasker.h"
#include <Wire.h>
#include <iarduino_OLED_txt.h>
#include "iarduino_4LED.h"
#include "SerialTransfer.h"
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include "power.h"
#include "watchdog.h"
#include <Bonezegei_Printf.h>

volatile int ohmValue = 0;
volatile unsigned long tasks_done = 0;
int last_secs = 0, audioValue = 0;
long prev_seconds = 999999, konvert = 0;
int volatile voices_detected = 0;
int last_event = 0;
int did_ended = 0;
int feat_exposed = 0;  // can external features enter inside
int perc = 0;
volatile int limiter_change = 0;
volatile int maxmic = 0;
volatile int expander_change = 0;
unsigned long epoch = 0;
volatile int noise_around = 0;
uint32_t i;
int h, m, s;
int crystall_spawn_sec = 0;
volatile int crystalls = 0;
volatile int power = 0;
int fire_ended = true;
int padding = false;
unsigned long mils = 0;

SerialTransfer pcTransfer;
iarduino_4LED QLED(7, 8);
Tasker tasker;
Bonezegei_Printf debug(&Serial);
iarduino_OLED_txt oled(0x3c);

#define RGB_R 5
#define RGB_G 4
#define RGB_B 3
#define OHM_INPUT A4
#define VOLUME_INPUT A2
#define MIC_INPUT A0
#define LED_BLACK_HOLE_FAIL 45
#define MAX_MIC_INPUT 48
#define LED_MATCH_EXPANDER 23
#define LED_MATCH_LIMITER 53
#define WAKE_UP_PIN 2
#define OUTPUT_RGB_LED 13
#define PIN_BUZZER 31
#define LED_CRYSTALL_GROW 11
#define LED_VOICE_DETECTED A14
#define HIDDEN_FEATURES 9
#define BTN_SLEEP 36

#define seconds(s) (millis(1000 * s))

void setup() {

  analogReference(INTERNAL2V56);

  Serial.begin(115200);
  while (!Serial) {}

  pcTransfer.begin(Serial);

  //wd_setup();

  pinMode(BTN_SLEEP, INPUT_PULLUP);
  pinMode(MIC_INPUT, INPUT);
  pinMode(OUTPUT_RGB_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(VOLUME_INPUT, INPUT);
  pinMode(OHM_INPUT, INPUT);
  pinMode(LED_VOICE_DETECTED, OUTPUT);
  pinMode(LED_CRYSTALL_GROW, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(WAKE_UP_PIN, INPUT);
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

  self_test(); // SetUnhandledExceptionFilter()
  
  // dont disturb user, let him relax, have good night and positive morning with wifee then go to office and make what has america have now ~60% (TRYMP EYES ONLY. DO NOT DUPLICATE OR MAP. DO NOT REMEMBER AND, FINALLY DO CONSTRAIN)

  tasker.setInterval(match_limiter, 1660);
  tasker.setInterval(expand_limiter, 1600);

  tasker.setInterval(raiser_crystalls, 1844);
  tasker.setInterval(disraiser_crystalls, 4250);

  tasker.setInterval(bh_fail_check, 888);
  tasker.setInterval(timebash, 1000);
  tasker.setInterval(oled_dump, 100);
  tasker.setInterval(cost, 1000);
  tasker.setInterval(accum_noise, 1350);

  mils = millis();
  srand(mils);

  debug.printf("\n[r&q + skynet + met9]\n");
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
void self_test() {

  if (!digitalPinHasPWM(LED_CRYSTALL_GROW)) {
    Serial.print("error pin does not have PWM");
    return;
  }

  for (int d = 0; d < 9; d++) {
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

    buzz(5, 3);
    digitalWrite(PIN_BUZZER, HIGH);
    delay(5);
    digitalWrite(PIN_BUZZER, LOW);
    buzz();

    digitalWrite(LED_BLACK_HOLE_FAIL, LOW);
    delay(10);
  }
}


void timebash() {
  tasks_done++;

  if (random(13) < 3) {
    if (power >= 70 && expander_change > 100 && limiter_change >= 100) {
      if (random(3) == random(4) == random(1) == random(3)) {
        digitalWrite(LED_BLACK_HOLE_FAIL, HIGH);
        buzz(1011, 5110);
        Serial.println("what the fuck!?!");
        if (power) {
          power -= min(power - 10, 100);
        }
        expander_change -= 100;
        limiter_change -= 100;
        digitalWrite(LED_BLACK_HOLE_FAIL, LOW);
      }
    }
  }
}

void accum_noise() {
  tasks_done++;
  noise_around = analogRead(MIC_INPUT);
}

void bh_fail_check() {
  tasks_done++;

  // hidden connect of constructors
  feat_exposed = random(43) < 9;
  if (feat_exposed) {
    digitalWrite(HIDDEN_FEATURES, HIGH);
    if (random(33) <= 15) {
      power += min(5, random(expander_change + limiter_change));
      buzz(5, 610);
    } else if (random(55) <= 35) {
      if (power) {
        buzz(5, 600);
        int aa = min(5, random(limiter_change));
        if (power >= aa) {
          power -= aa;
        }
      }
    }
    digitalWrite(HIDDEN_FEATURES, LOW);
  }
}

////////////////////  CRYSTALLLS
void raiser_crystalls() {
  tasks_done++;

  static bool moving_down = false;

  int landing = random(15) > 11;
  if (landing) {
    return;
  }

  int rr = 0, v = 0, prev = 0;

  if (power > 23) {
    int fraise = random(1, random(10));

    pinMode(LED_CRYSTALL_GROW, INPUT);
    rr = analogRead(LED_CRYSTALL_GROW);
    prev = abs(map(rr, 0, 1023, 0, 255));

    for (int a = 0; a < fraise; a++) {
      if (prev <= 3 || prev >= 255) {
        moving_down = !moving_down;
        if (!moving_down && random(100) > 70) {
          crystalls += 1;
        }
      }
      if (moving_down) {
        v = prev - random(80);
      } else {
        v = prev + random(80);
      }
      debug.printf("prev=%d v=%d moving=%d a=%d\n", prev, v, moving_down, a);
      pinMode(LED_CRYSTALL_GROW, OUTPUT);
      delay(1);
      analogWrite(LED_CRYSTALL_GROW, v);
      prev = v;
    }
  }
}

void disraiser_crystalls() {
  tasks_done++;

  int landing = random(2) == 1;
  if (landing && power < 50) {
    return;
  }
  pinMode(LED_CRYSTALL_GROW, INPUT);
  delay(1);
  int rr = analogRead(LED_CRYSTALL_GROW);

  for (int a = rr; a > rr; a--) {
    buzz(100, a * 100);
    if (random(3) == 1) {
      power -= 2;
    }
    pinMode(LED_CRYSTALL_GROW, OUTPUT);
    delay(1);
    analogWrite(LED_CRYSTALL_GROW, max(random(10), random(255 / rr)));
  }
}

//////// SPACES
void match_limiter() {
  tasks_done++;

  bool landing = random(125) < 15;
  if (landing) {
    return;
  }

  int r = random(141) > 131;
  if (r) {
    r = 1 + random(3);
    limiter_change += r;
    Serial.print("expand limit matcher by ");
    Serial.println(r);
    digitalWrite(LED_MATCH_LIMITER, HIGH);
    delay(1);
    digitalWrite(LED_MATCH_LIMITER, LOW);
    //buzz(5, 10);
  }
}

void expand_limiter() {
  tasks_done++;
  bool landing = (power < 5)
                 || random(expander_change % 4) < random(limiter_change % 3);
  if (landing) {
    return;
  }

  int r = 1 + random(5);
  expander_change += r;
  Serial.print("expand limiter by ");
  Serial.println(r);
  digitalWrite(LED_MATCH_EXPANDER, HIGH);
  delay(1);
  digitalWrite(LED_MATCH_EXPANDER, LOW);
  //buzz(5, 10);
}

///////// OTHER
void oled_dump() {
  tasks_done++;

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

  // Serial.print("noise");
  // Serial.println(noise_around);

  oled.setCursor(0, 4);
  oled.print("ohm ");
  oled.print(ohmValue);

  // Serial.print("vol");s
  // Serial.println(audioValue);

  oled.setCursor(0, 5);
  oled.print("vol ");
  oled.print(audioValue);

  oled.setCursor(0, 6);
  oled.print("epoch ");
  oled.print(epoch);

  oled.setCursor(0, 7);
  oled.print("maxmic ");
  oled.print(maxmic);

  oled.setCursor(63, 0);
  oled.print("tsks ");
  oled.print(tasks_done);

  oled.setCursor(63, 1);
  oled.print("crstls ");
  oled.print(crystalls);

  QLED.print(voices_detected, 0);
}

void cost() {
  static int prev_dist = 0;

  tasks_done++;

  maxmic = analogRead(MAX_MIC_INPUT);
  maxmic = map(maxmic, 0, 1023, 0, 255);
  // Serial.print("maxmic:");
  // Serial.println(maxmic);

  ohmValue = analogRead(OHM_INPUT);
  ohmValue = map(ohmValue, 0, 1023, 0, 255);
  // Serial.print("ohmValue:");
  // Serial.println(ohmValue);

  audioValue = analogRead(VOLUME_INPUT);
  audioValue = map(audioValue, 0, 1023, 0, 255);
  // Serial.print("audioValue:");
  // Serial.println(audioValue);

  konvert = map(audioValue, 0, 1023, 0, 255);
  // Serial.print("konvert:");
  // Serial.println(konvert);

  audioValue = analogRead(VOLUME_INPUT);
  int prev_audioValue = audioValue;
  if (abs(maxmic - audioValue) >= ohmValue) {
    digitalWrite(LED_VOICE_DETECTED, HIGH);
    last_event = millis();
    while (abs(prev_audioValue - audioValue) >= ohmValue) {
      prev_audioValue = audioValue;
      audioValue = analogRead(VOLUME_INPUT);
      audioValue = map(audioValue, 0, 1023, 0, 255);
      ohmValue = analogRead(OHM_INPUT);
      ohmValue = map(ohmValue, 0, 1023, 0, 255);
    }
    digitalWrite(LED_VOICE_DETECTED, LOW);
    voices_detected += 1;
  }
}
void read_user_commands() {
  String cmd = "";
  while (Serial.available()) {
    byte b = Serial.read();
    cmd += (char)b;
  }

  if (cmd == "down") {
    sleepNow();
  }
}

void loop() {
  tasker.loop();  // after drug dealer automated-visit at 6am

  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5 / 1023.0);

  Serial.print("voltage:");
  Serial.println(voltage);

  analogWrite(RGB_R, maxmic);
  analogWrite(RGB_G, noise_around);
  analogWrite(RGB_B, audioValue);

  read_user_commands();
  epoch++;
}
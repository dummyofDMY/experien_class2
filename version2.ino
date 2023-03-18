#include<Servo.h>
#include<MsTimer2.h>

#define PWM_L 7
#define PWM_R 8
#define IN_L_A A0
#define IN_L_B A1
#define IN_R_A A2
#define IN_R_B A3


#define SEN0 A5
#define SEN1 6
#define SEN2 9
#define SEN3 A4
#define SEN4 10
#define SEN5 11
#define SEN6 12

#define L_CO_A 2
#define L_CO_B 4
#define R_CO_A 3
#define R_CO_B 5

#define BASE_PIN 44
#define PAW_PIN 46

#define BASE_HORI 70
#define BASE_VER 110
#define PAW_OPEN 70
#define PAW_CLOSE 110

#define KP_W 30
#define KI_W 100
#define KD_W 0.01

#define V_MAX 8
#define PERIOD 0.01
#define EVENT_LENGHT 200
#define CRITIS_LENGHT 200

Servo base;
Servo paw;

//coder
int left_coder = 0;
int right_coder = 0;
double v_l = 0;
double v_r = 0;
long last_l_time = millis(), last_r_time = millis();
long this_l_time, this_r_time;

//PID类遗址
double l_e[3] = {-V_MAX, -V_MAX, -V_MAX}; //下标越小越新
double r_e[3] = {-V_MAX, -V_MAX, -V_MAX}; //下标越小越新

//Sen类遗址
int data[7] = { 0 };

//Car类遗址
int pwm_l = PWM_L, pwm_r = PWM_R, in_l_a = IN_L_A, in_l_b = IN_L_B, in_r_a = IN_R_A, in_r_b = IN_R_B;
int sen_pin[8];
double patrol_e;
double l_v, r_v;
double tar_v_l, tar_v_r;
int l_cmd, r_cmd;
bool event = false;
bool critis = false;
long init_t = millis();

void update_l()
{
  l_e[2] = l_e[1]; l_e[1] = l_e[0];
  l_e[0] = left_coder * 8.055e-3 / PERIOD - tar_v_l; // pi * 2 * 1e3 / 780 = 8.055
  left_coder = 0;
  return;
}
void update_r()
{
  r_e[2] = r_e[1]; r_e[1] = r_e[0];
  r_e[0] = right_coder * 8.055e-3 / PERIOD - tar_v_r;
  right_coder = 0;
  return;
}

void calculate_l()
{
  double de = (l_e[0] - l_e[1]) / PERIOD;
  double Ie = (l_e[0] + l_e[1] + l_e[2]) * PERIOD * 2;
  l_cmd = -KP_W * l_e[0] - KI_W * Ie - KD_W * de;
  // Serial.print("l_deep::\t");
  // Serial.print(l_e[0] * KP_W); Serial.print('\t');
  // Serial.print(de * KD_W); Serial.print('\t');
  // Serial.print(Ie * KI_W); Serial.print('\t');
  return;
}
void calculate_r()
{
  double de = (r_e[0] - r_e[1]) / PERIOD;
  double Ie = (r_e[0] + r_e[1] + r_e[2]) * PERIOD * 2;
  r_cmd = -KP_W * r_e[0] - KI_W * Ie - KD_W * de;
  // Serial.print("r_deep::\t");
  // Serial.print(r_e[0] * KP_W); Serial.print('\t');
  // Serial.print(de * KD_W); Serial.print('\t');
  // Serial.print(Ie * KI_W); Serial.print('\t');
  return;
}

void read()
{
  data[0] = digitalRead(SEN0); data[1] = digitalRead(SEN1); data[2] = digitalRead(SEN2); data[3] = digitalRead(SEN3);
  data[4] = digitalRead(SEN4); data[5] = digitalRead(SEN5); data[6] = digitalRead(SEN6);
  return;
}

void sen_print()
{
  Serial.print("Sensor::\t");
  Serial.print(data[0]); Serial.print('\t'); Serial.print(data[1]); Serial.print('\t'); Serial.print(data[2]); Serial.print('\t');
  Serial.print(data[3]); Serial.print('\t'); Serial.print(data[4]); Serial.print('\t'); Serial.print(data[5]); Serial.print('\t'); 
  Serial.print(data[6]); Serial.print('\t');
  return;
}

void l_co_count()
{
  left_coder += 2 * digitalRead(L_CO_B) - 1;
  return;
}

void r_co_count()
{
  right_coder -= 2 * digitalRead(R_CO_B) - 1;
  return;
}

void step_print()
{
    Serial.print("coder::\t");
    Serial.print(left_coder);
    Serial.print('\t');
    Serial.print(right_coder);
    Serial.print('\t');
}

void place()
{
  MsTimer2::stop();
  // send_cmd(V_MAX, -V_MAX);
  // analogWrite(PWM_L, 0);
  // analogWrite(PWM_R, 0);
  // delay(1000);
  digitalWrite(IN_L_A, LOW);
  digitalWrite(IN_L_B, HIGH);
  analogWrite(PWM_L, 255);
  digitalWrite(IN_R_A, LOW);
  digitalWrite(IN_R_B, HIGH);
  analogWrite(PWM_R, 255);
  delay(300);
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
  base.write(BASE_HORI);
  paw.write(PAW_OPEN);
  return;
}

void decide_tar_sen()
{
  int emerge_l = 0, emerge_r = 0;
  for (int i = 0; i <= 3; ++i) {
    if (1 == data[i]) {
      ++emerge_l;
    }
    if (1 == data[i + 3]) {
      ++emerge_r;
    }
  }
  if (emerge_l + emerge_r == 0) {
    return;
  }
  if (emerge_l + emerge_r >= 5) {
    tar_v_l = 0;
    tar_v_r = 0;
    if (millis() - init_t >= 40 * 1e3) {
      place();
    }
    return;
  }

  if (emerge_l >= 2 /*&& millis() - init_t >= 15 * 1e3 && emerge_r <= 0*/) {
    tar_v_l = -2.5 * V_MAX;
    tar_v_r = 1.3 * V_MAX;
    return;
  }
  if (emerge_r >= 2 /*&& millis() - init_t >= 15 * 1e3 && emerge_l <= 0*/) {
    tar_v_l = 1.3 * V_MAX;
    tar_v_r = -2.5 * V_MAX;
    return;
  }
  // //四级警报
  // if (1 == data[0]) {
  //   tar_v_l = V_MAX * -1;
  //   tar_v_r = V_MAX * 1;
  //   return;
  // }
  // if (1 == data[7]) {
  //   tar_v_l = V_MAX * 1;
  //   tar_v_r = V_MAX * -1;
  //   return;
  // }
  //三级警报
  if (1 == data[0]) {
    tar_v_l = V_MAX * -0.4;
    tar_v_r = V_MAX * 0.6;
    return;
  }
  if (1 == data[6]) {
    tar_v_l = V_MAX * 0.6;
    tar_v_r = V_MAX * -0.4;
    return;
  }
  //二级警报
  if (1 == data[1]) {
    tar_v_l = V_MAX * 0.2;
    tar_v_r = V_MAX * 1;
    return;
  }
  if (1 == data[5]) {
    tar_v_l = V_MAX * 1;
    tar_v_r = V_MAX * 0.2;
    return;
  }
  //一级警报
  if (1 == data[2]) {
    tar_v_l = V_MAX * 0.5;
    tar_v_r = V_MAX * 1.2;
    return;
  }
  if (1 == data[4]) {
    tar_v_l = V_MAX * 1.2;
    tar_v_r = V_MAX * 0.5;
    return;
  }
  if (1 == data[3]) {
    tar_v_l = V_MAX;
    tar_v_r = V_MAX;
    return;
  }
}

void l_pid_print()
{
  Serial.print("PID::l_e[3]:\t");
  for (int i = 0; i < 3; ++i){
    Serial.print(l_e[i]);
    Serial.print('\t');
  }
  return;
}

void r_pid_print()
{
  Serial.print("PID::r_e[3]:\t");
  for (int i = 0; i < 3; ++i){
    Serial.print(r_e[i]);
    Serial.print('\t');
  }
  return;
}

void data_print()
{
    Serial.print("TIME::\t");
    Serial.print(millis());
    Serial.print('\t');
    // if (event) {
    //     Serial.print("!!!\tEVEN\t!!!");
    // }
    sen_print();
    // step_print();
    // dire.print_out();
    // dire.deep_print();
    // tar_print();
    // l_pid_print();
    // r_pid_print();
    cmd_print();
    Serial.print('\n');
    return;
}

void send_cmd()
{
  // l_cmd = r_cmd = 255;
  if (l_cmd < 0) {
    digitalWrite(IN_L_A, LOW);
    digitalWrite(IN_L_B, HIGH);
  }
  else {
    digitalWrite(IN_L_A, HIGH);
    digitalWrite(IN_L_B, LOW);
  }
  analogWrite(PWM_L, abs(l_cmd) < 255 ? abs(l_cmd) : 255);
  // digitalWrite(PWM_L, HIGH);
  // Serial.print("Check");
  if (r_cmd > 0) {
    digitalWrite(IN_R_A, LOW);
    digitalWrite(IN_R_B, HIGH);
  }
  else {
    digitalWrite(IN_R_A, HIGH);
    digitalWrite(IN_R_B, LOW);
  }
  analogWrite(PWM_R, abs(r_cmd) < 255 ? abs(r_cmd) : 255);
  return;
}

void control()
{
  read();
  decide_tar_sen();
  update_l();
  update_r();
  calculate_l();
  calculate_r();
  send_cmd();
  data_print();
  // send_cmd_test();
  return;
}

void tar_print()
{
    Serial.print("target::\t");
    Serial.print(tar_v_l);
    Serial.print('\t');
    Serial.print(tar_v_r);
    Serial.print('\t');
    return;
}

void send_cmd_test()
{
  return;
}

void cmd_print()
{
  Serial.print("cmd::\t");
  Serial.print(l_cmd);
  Serial.print('\t');
  Serial.print(r_cmd);
  Serial.print('\t');
}

void pick()
{
  paw.write(PAW_OPEN);
  delay(500);
  base.write(BASE_HORI);
  delay(500);
  paw.write(PAW_CLOSE);
  delay(500);
  base.write(BASE_VER);
}

void setup() {
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(IN_L_A, OUTPUT);
  pinMode(IN_L_B, OUTPUT);
  pinMode(IN_R_A, OUTPUT);
  pinMode(IN_R_B, OUTPUT);
  pinMode(SEN0, INPUT);
  pinMode(SEN1, INPUT);
  pinMode(SEN2, INPUT);
  pinMode(SEN3, INPUT);
  pinMode(SEN4, INPUT);
  pinMode(SEN5, INPUT);
  pinMode(SEN6, INPUT);
  pinMode(L_CO_A, INPUT);
  pinMode(L_CO_B, INPUT);
  pinMode(R_CO_A, INPUT);
  pinMode(R_CO_B, INPUT);
  pinMode(BASE_PIN, OUTPUT);
  pinMode(PAW_PIN, OUTPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(L_CO_A), l_co_count, RISING);
  attachInterrupt(digitalPinToInterrupt(R_CO_A), r_co_count, RISING);
  base.attach(BASE_PIN);
  paw.attach(PAW_PIN);
  pick();
  MsTimer2::set(PERIOD*1e3, control);
  MsTimer2::start();
}

void loop() {
  // static long last_t = millis();
  // while (millis() - last_t < PERIOD * 1e3);
  // control();
}
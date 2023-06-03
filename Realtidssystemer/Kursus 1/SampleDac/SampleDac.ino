#define adcPort A0

int DacPort[] = { 2, 3, 4, 5, 6, 7, 8, 9 };
uint8_t data = 0;
bool portState[8];

unsigned long timer = 0;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 8; i++) {
    pinMode(DacPort[i], OUTPUT);
  }
  pinMode(adcPort, INPUT);
}


void loop() {
  // data = analogRead(adcPort) / 4;
  // for (uint8_t y = 0; y < 8; y++) {
  //   if (data & (1 << y)) {
  //     portState[y] = 1;
  //   } else {
  //     portState[y] = 0;
  //   }
  // }
  // for (int k = 0; k < 8; k++) {
  //   digitalWrite(DacPort[k], portState[k]);
  // }
  analogWrite(DacPort[1], 4);
}

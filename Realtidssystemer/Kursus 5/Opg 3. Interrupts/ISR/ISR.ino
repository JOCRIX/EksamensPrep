uint32_t count = 0;
uint16_t interval = 100; //Debounce time
uint32_t lastTime = 0, currentTime;

void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), ISRCount, FALLING);
}

void loop() {
  Serial.print("ISR Count: ");
  Serial.print(count);
  Serial.println();
  delay(1000);
}

void ISRCount(){
  
  if (millis() - lastTime > interval){
    count++;
  }
  lastTime = millis();
}

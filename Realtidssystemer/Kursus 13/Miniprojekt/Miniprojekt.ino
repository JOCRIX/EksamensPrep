#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <PID_v1.h>

#define speed_ 1
#define speedRef_ 2
#define motorDCY_ 3
#define p_ 4
#define i_ 5
#define d_ 6
//Pin IO
#define isrPin 2   //interupt pin
#define encPin 3   //encoder direction determination pin
#define pwmPin1 5  // PWM output pin
#define pwmPin2 6

#define setDir false
//Hardcoded values
#define encPPR 100.0  //encoder pulses per rotation

#define waitTimePIDCalc 100
#define waitTimeRef 100
#define waittimePIDGet 500
#define waittimeSpeedCalc 100.0

unsigned long timer = 0;

struct ctrl {
  float speed;
  uint16_t speedRef;
  int motorDCY;
  float p, i, d;
} data;

struct pid_setting {
  double setpoint, input, output;
  float p, i, d;
} pid_init;

struct serialData {
  uint8_t serialData[25];
  uint8_t serialDataArraySize;
};

PID pid_data(&pid_init.input,
             &pid_init.output,
             &pid_init.setpoint,
             pid_init.p,
             pid_init.i,
             pid_init.d,
             DIRECT);

uint16_t encoderCount;
bool motorDirection;
bool runDir;

SemaphoreHandle_t mutexStruct;

void setup() {
  data.p = 3;
  data.i = 8;
  data.d = 0.8;

  runDir = setDir;

  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  Serial.begin(115200);
  mutexStruct = xSemaphoreCreateMutex();
  PIDControlInit();

  pinMode(isrPin, INPUT_PULLUP);
  pinMode(encPin, INPUT_PULLUP);
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(A0, INPUT);
  attachInterrupt(digitalPinToInterrupt(isrPin), EncoderISR, RISING);

  while (!Serial)
    ;
  Serial.println("TASK1");
  //Task1 TSKSpeedCalculation (Priority 1) 
  // Task one takes roughly 60 µs
  xTaskCreate(TSKSpeedCalculation,  // Task function
              "Speed Calculation",  // Task name for humans
              150,
              NULL,  // Task parameter
              5,     // Task priority
              NULL);
  //Task2 TSKSpeedRef (Priority 4)
  //Serial.println("TASK2");
  // Task two takes roughly 112 µs.
  xTaskCreate(TSKSpeedRef,      // Task function
              "Speed Ref ADC",  // Task name for humans
              150,
              NULL,  // Task parameter
              1,     // Task priority
              NULL);

  // Task3 TSKSerialPID (Priority 3)
  // Task three takes roughly 90 µs.
  Serial.println("TASK3");
  xTaskCreate(TSKSerialPID,            // Task function
              "Serial Communication",  // Task name for humans
              150,
              NULL,  // Task parameter
              2,     // Task priority
              NULL);

  // //Task4 TSKMotorPWM (Priority 3)
  // Serial.println("TASK4");
  // xTaskCreate(TSKMotorPWM,            // Task function
  //             "Motor PWM Generator",  // Task name for humans
  //             16,
  //             NULL,  // Task parameter
  //             3,     // Task priority
  //             NULL);
  // //Task5 TSKPIDControl (Priority 4)
  // Task five takes roughly 92 µs.
  Serial.println("TASK5");
  xTaskCreate(TSKPIDControl,        // Task function
              "PID Control block",  // Task name for humans
              150,
              NULL,  // Task parameter
              4,     // Task priority
              NULL);
}


void TSKSpeedRef() {
  while (1) {
    ctrl speedRefStruct;
    //Serial.println("Task2");
    float sensorValue = analogRead(A0) / 4;

    // float sensorValue = 300;

    speedRefStruct.speedRef = sensorValue;

    // speedRefStruct.speedRef = 127;

    UPDStruct(speedRefStruct, speedRef_);
    vTaskDelay(waitTimeRef / portTICK_PERIOD_MS);
  }
}

void EncoderISR() {
  encoderCount++;
  motorDirection = digitalRead(encPin);
}

void TSKSpeedCalculation() {
  while (1) {
    //Serial.println("Task1");
    ctrl speedCal = data;

    //speedCal.speed = ((float)((encoderCount / encPPR)) / ((0.001 / 60)));

    //speedCal.speed = ((encoderCount*1.0) / (encPPR*1.0)) * (60000.0 / (millis()-timer));
    speedCal.speed = ((encoderCount * 1.0) * (1000 / (millis() - timer)));
    encoderCount = 0;
    timer = millis();
    //Serial.println(speedCal.speed);
    UPDStruct(speedCal, speed_);
    // Serial.print("Data:");
    // Serial.println(data.speed);
    vTaskDelay(waittimeSpeedCalc / portTICK_PERIOD_MS);
  }
}

// void TSKMotorPWM() {
//   while (1) {
//     Serial.println("Task4");
//     analogWrite(pwmPin, data.motorDCY);
//     vTaskDelay(500 / portTICK_PERIOD_MS);
//   }
// }

void PIDControlInit() {
  pid_data.SetMode(AUTOMATIC);
  pid_data.SetOutputLimits(-2550, 2550);
}


void FunnyFunctionDoingShit() {
  //Serial.println("This is some shit");
}

void TSKPIDControl(void *pvParameters) {
  while (1) {

    ctrl pid_ctrl = data;
    pid_init.input = pid_ctrl.speed * 0.34;
    pid_init.setpoint = pid_ctrl.speedRef;

    if (pid_init.p != pid_ctrl.p || pid_init.i != pid_ctrl.i || pid_init.d != pid_ctrl.d) {
      pid_data.SetTunings(pid_ctrl.p,
                          pid_ctrl.i,
                          pid_ctrl.d);
      pid_init.p = pid_ctrl.p;
      pid_init.i = pid_ctrl.i;
      pid_init.d = pid_ctrl.d;
      UPDStruct(pid_ctrl, p_ + i_ + d_);
    }
    pid_data.Compute();



    pid_ctrl.motorDCY = pid_ctrl.speedRef + (pid_init.output / 10);

    if (pid_ctrl.motorDCY >= 255) {
      pid_ctrl.motorDCY = 255;
    } else if (pid_ctrl.motorDCY <= 0) {
      pid_ctrl.motorDCY = 0;
    }


    if (motorDirection != setDir) {
      runDir = !runDir;
    }

    if (runDir == true) {
      digitalWrite(pwmPin1, false);
      analogWrite(pwmPin2, pid_ctrl.motorDCY);
    } else {
      digitalWrite(pwmPin2, false);
      analogWrite(pwmPin1, pid_ctrl.motorDCY);
    }

    UPDStruct(pid_ctrl, motorDCY_);
    vTaskDelay(waitTimePIDCalc / portTICK_PERIOD_MS);
  }
}




uint8_t GetSerialDataArrSize(uint8_t *arr) {
  uint8_t arrSize = 0;
}

serialData GetSerialData() {
  serialData incomingData;
  uint8_t moreData = 0, currentChar, terminator = '#', index = 0;
  while ((Serial.available() > 0) && (moreData == 0)) {
    currentChar = Serial.read();
    //Serial.print(currentChar);
    if (currentChar != terminator) {
      incomingData.serialData[index] = currentChar;
      index++;
    } else {
      incomingData.serialData[index] = '#';
      incomingData.serialDataArraySize = index;
      index = 0;
      moreData = 1;
    }
  }
  return incomingData;
}

uint8_t *GetPIDValues(serialData *incomingData) {
  static uint8_t values[3];
  values[0] = GetPVal(incomingData);
  values[1] = GetIVal(incomingData);
  values[2] = GetDVal(incomingData);
  return &values[0];
}

uint8_t GetPVal(serialData *incomingData) {
  uint8_t pVal = 0, currentChar = 0, index = 0, pFound = 0;

  while (incomingData->serialData[index] != '|' && (index < (uint8_t)sizeof(incomingData->serialData)) && pFound == 0) {
    currentChar = (char)incomingData->serialData[index];
    if (currentChar != 'P') {
      index++;
    } else {
      do {
        currentChar = (char)incomingData->serialData[index];
        if ((char)currentChar == '%' && pFound == 0) {
          index++;
          pVal = (uint8_t)incomingData->serialData[index] - '0';  //Convert to int
          pFound = 1;
        } else {
          index++;
        }
      } while (pFound == 0);
    }
  }
  return pVal;
}
uint8_t GetIVal(serialData *incomingData) {

  uint8_t iVal = 0, currentChar = 0, index = 0, pFound = 0;

  while ((index < (uint8_t)sizeof(incomingData->serialData) && (pFound == 0))) {
    currentChar = (char)incomingData->serialData[index];
    if (currentChar != 'I') {
      index++;
    } else {
      do {
        currentChar = (char)incomingData->serialData[index];
        if ((char)currentChar == '%' && pFound == 0) {
          index++;
          iVal = (uint8_t)incomingData->serialData[index] - '0';  //Convert to int
          pFound = 1;
        } else {
          index++;
        }
      } while (pFound == 0);
    }
  }
  return iVal;
}
uint8_t GetDVal(serialData *incomingData) {
  uint8_t dVal = 0, currentChar = 0, index = 0, pFound = 0;

  while ((index < (uint8_t)sizeof(incomingData->serialData) && (pFound == 0))) {
    currentChar = (char)incomingData->serialData[index];
    if (currentChar != 'D') {
      index++;
    } else {
      do {
        currentChar = (char)incomingData->serialData[index];
        if ((char)currentChar == '%' && pFound == 0) {
          index++;
          dVal = (uint8_t)incomingData->serialData[index] - '0';  //Convert to int
          pFound = 1;
        } else {
          index++;
        }
      } while (pFound == 0);
    }
  }
  return dVal;
}

void ConvToFloatLoadStruct(uint8_t *receivedValues) {
  ctrl newPIDVals;
  newPIDVals.p = ((float)*receivedValues);
  receivedValues++;
  newPIDVals.i = ((float)*receivedValues);
  receivedValues++;
  newPIDVals.d = ((float)*receivedValues) / 10;

  UPDStruct(newPIDVals, (p_ + i_ + d_));
}

void TSKSerialPID(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static serialData incomingData;

  while (1) {
    //Serial.println("TSK3");
    if (Serial.available() > 0) {
      incomingData = GetSerialData();
    }
    uint8_t *receivedValues = GetPIDValues(&incomingData);
    ConvToFloatLoadStruct(receivedValues);
    /* 
  Serial.print("Final Values");
  Serial.println();
  Serial.print("P-VAL: ");
  Serial.println(data.p);
  Serial.print("I-VAL: ");
  Serial.println(data.i);
  Serial.print("D-VAL: ");
  Serial.println(data.d);
*/
    //vTaskDelayUntil(&xLastWakeTime, 250 / portTICK_PERIOD_MS);
    vTaskDelay(waittimePIDGet / portTICK_PERIOD_MS);
  }
}


void UPDStruct(ctrl updStruct, uint8_t index) {

  if (xSemaphoreTake(mutexStruct, 10) == pdTRUE) {
    switch (index) {
      case speed_:
        data.speed = updStruct.speed;
        break;
      case speedRef_:
        data.speedRef = updStruct.speedRef;
        break;
      case motorDCY_:
        data.motorDCY = updStruct.motorDCY;
        break;
      case p_:
        data.p = updStruct.p;
        break;
      case i_:
        data.i = updStruct.i;
        break;
      case d_:
        data.d = updStruct.d;
        break;
      case p_ + i_ + d_:
        data.p = updStruct.p;
        data.i = updStruct.i;
        data.d = updStruct.d;
        break;

      default:
        data = updStruct;
        break;
    }

    xSemaphoreGive(mutexStruct);
  }
}

void loop() {
}

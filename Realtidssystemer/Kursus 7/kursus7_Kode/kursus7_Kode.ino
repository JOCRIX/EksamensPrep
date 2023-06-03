#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).

#define outPin1 2
#define outPin2 3

bool out1State = false;
bool out2State = false;


// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;

// define two Tasks for DigitalRead & AnalogRead
void Task1(void *pvParameters);
void Task2(void *pvParameters);



// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);


  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if (xSerialSemaphore == NULL)  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ((xSerialSemaphore) != NULL)
      xSemaphoreGive((xSerialSemaphore));  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  // Now set up two Tasks to run independently.
  xTaskCreate(
    Task1, "Task 1"  // A name just for humans
    ,
    128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,
    NULL  //Parameters for the task
    ,
    1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    NULL);  //Task Handle

  xTaskCreate(
    Task2, "Task 2"  // A name just for humans
    ,
    128  // Stack size
    ,
    NULL  //Parameters for the task
    ,
    2  // Priority
    ,
    NULL);  //Task Handle

  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
}

void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/



void Task1(void *pvParameters __attribute__((unused)))  // This is a Task.
{
  /*
    DigitalReadSerial
    Reads a digital input on pin 2, prints the result to the serial monitor

    This example code is in the public domain.
  */
  for (;;)  // A Task shall never return or exit.
  {

    digitalWrite(outPin1, out1State);
    out1State = !out1State;

    //Serial.println("||aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa||");

    if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE) {
      Serial.println("||aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa||");

      xSemaphoreGive(xSerialSemaphore);  // Now free or "Give" the Serial Port for others.
    }


    vTaskDelay(2);  // one tick delay (15ms) in between reads for stability
  }
}

void Task2(void *pvParameters __attribute__((unused)))  // This is a Task.
{

  for (;;) {
    digitalWrite(outPin2, out2State);
    out2State = !out2State;

    //Serial.println("||bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb||");

    if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE) {
      Serial.println("||bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb||");

      xSemaphoreGive(xSerialSemaphore);  // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(2);  // one tick delay (15ms) in between reads for stability
  }
}

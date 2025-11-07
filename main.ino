#include <Arduino.h>
#include <stdarg.h>

const int PIN_POT     = 36;
const int PIN_CURRENT = 39;
const int PIN_PWM     = 18;

const int PWM_CHANNEL    = 0;
const int PWM_FREQUENCY  = 20000;
const int PWM_RESOLUTION = 8;

const int ADC_MAX = 4095;
const TickType_t POT_SAMPLE_MS = pdMS_TO_TICKS(100);
const TickType_t PROTECTION_SAMPLE_MS = pdMS_TO_TICKS(150);
const int CURRENT_THRESHOLD_RAW = 2000;

QueueHandle_t setpointQueue;
QueueHandle_t faultQueue;
SemaphoreHandle_t serialMutex;

typedef struct {
  uint8_t percent;
} Setpoint_t;

typedef enum {
  FAULT_NONE = 0,
  FAULT_START = 1,
  FAULT_CLEAR = 2
} FaultEvent_t;

void safeSerialPrintf(const char* fmt, ...) {
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100))) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
    xSemaphoreGive(serialMutex);
  }
}

void setupPWM() {
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PIN_PWM, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);
}

void SpeedReadTask(void* pv) {
  Setpoint_t sp;
  for (;;) {
    int raw = analogRead(PIN_POT);
    int scaled = (raw * 100) / ADC_MAX;
    if (scaled < 0) scaled = 0;
    if (scaled > 100) scaled = 100;
    sp.percent = (uint8_t)scaled;
    if (xQueueSend(setpointQueue, &sp, pdMS_TO_TICKS(10)) != pdPASS) {
      Setpoint_t dummy;
      xQueueReceive(setpointQueue, &dummy, 0);
      xQueueSend(setpointQueue, &sp, pdMS_TO_TICKS(10));
    }
    vTaskDelay(POT_SAMPLE_MS);
  }
}

void MotorCtrlTask(void* pv) {
  Setpoint_t sp;
  bool faultLatched = false;
  const int maxDuty = (1 << PWM_RESOLUTION) - 1;
  for (;;) {
    uint8_t evt;
    if (xQueueReceive(faultQueue, &evt, 0) == pdTRUE) {
      if (evt == FAULT_START) {
        safeSerialPrintf("[MotorCtrl] FAULT_START! stopping motor\n");
        faultLatched = true;
        ledcWrite(PWM_CHANNEL, 0);
        for (;;) {
          uint8_t evt2;
          if (xQueueReceive(faultQueue, &evt2, portMAX_DELAY) == pdTRUE) {
            if (evt2 == FAULT_CLEAR) {
              safeSerialPrintf("[MotorCtrl] FAULT_CLEAR! resuming\n");
              faultLatched = false;
              break;
            } else {
              safeSerialPrintf("[MotorCtrl] Ignored extra FAULT_START while latched\n");
            }
          }
        }
      }
    }
    if (faultLatched) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (xQueueReceive(setpointQueue, &sp, pdMS_TO_TICKS(200)) == pdTRUE) {
      int duty = (sp.percent * maxDuty) / 100;
      ledcWrite(PWM_CHANNEL, duty);
      safeSerialPrintf("[MotorCtrl] Setpoint %u%% -> duty %d\n", sp.percent, duty);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void ProtectionTask(void* pv) {
  for (;;) {
    int curRaw = analogRead(PIN_CURRENT);
    if (curRaw > CURRENT_THRESHOLD_RAW) {
      safeSerialPrintf("[Protection] Overcurrent detected raw=%d -> sending FAULT_START\n", curRaw);
      uint8_t evt = FAULT_START;
      xQueueSend(faultQueue, &evt, pdMS_TO_TICKS(10));
      vTaskDelay(pdMS_TO_TICKS(500));
      const TickType_t hold = pdMS_TO_TICKS(3000);
      vTaskDelay(hold);
      evt = FAULT_CLEAR;
      xQueueSend(faultQueue, &evt, pdMS_TO_TICKS(10));
      safeSerialPrintf("[Protection] Auto-cleared fault after hold\n");
    }
    vTaskDelay(PROTECTION_SAMPLE_MS);
  }
}

void DisplayTask(void* pv) {
  for (;;) {
    safeSerialPrintf("[Status] FreeHeap=%u\n", xPortGetFreeHeapSize());
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  setpointQueue = xQueueCreate(4, sizeof(Setpoint_t));
  faultQueue    = xQueueCreate(4, sizeof(uint8_t));
  serialMutex   = xSemaphoreCreateMutex();
  if (!setpointQueue || !faultQueue || !serialMutex) {
    Serial.println("RTOS object creation failed!");
    while (1) delay(1000);
  }
  analogSetPinAttenuation(PIN_POT, ADC_11db);
  analogSetPinAttenuation(PIN_CURRENT, ADC_11db);
  setupPWM();
  xTaskCreate(SpeedReadTask, "SpeedRead", 2048, NULL, 2, NULL);
  xTaskCreate(MotorCtrlTask, "MotorCtrl", 3072, NULL, 3, NULL);
  xTaskCreate(ProtectionTask, "Protection", 2048, NULL, 4, NULL);
  xTaskCreate(DisplayTask, "Display", 2048, NULL, 1, NULL);
  safeSerialPrintf("[Setup] System initialized\n");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

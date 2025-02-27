//--------------------------------------------------------
// Configuration Options
//--------------------------------------------------------
// #define SENDER_MODULE       // Uncomment for sender mode; comment out for receiver mode.
#define MODULE_OCTAVE 4     // Module's octave number (0-8)

// Uncomment the following to disable threads and ISRs for testing execution time.
// #define DISABLE_THREADS
// #define DISABLE_ISRS
// #define TEST_SCANKEYS       // When defined, run test code (one-iteration loops) for execution timing

//--------------------------------------------------------
// Includes and Definitions
//--------------------------------------------------------
#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <HardwareTimer.h>
#include <ES_CAN.h>  // CAN bus communication

// Pin definitions
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN  = A5;

const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;

const int OUT_PIN = D11;
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT  = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// Global variable for current phase step size (for sound generation)
// Volatile because it is accessed in tasks and in the ISR.
volatile uint32_t currentStepSize = 0;

// Global structure for sharing system state (the key matrix inputs)
struct SystemState {
  std::bitset<32> inputs;
  SemaphoreHandle_t mutex;
} sysState;

// Precomputed phase step sizes for the 12 musical keys (C, C♯, D, etc.)
// S = (2^32 * f) / 22000
const uint32_t stepSizes[12] = {
  51080511,  // C
  54113906,  // C♯
  57362353,  // D
  60709836,  // D♯
  64341089,  // E
  68195786,  // F
  72268860,  // F♯
  76537834,  // G
  81084437,  // G♯
  85899334,  // A (440Hz)
  90965186,  // A♯
  96468917   // B
};

// Hardware timer used for generating the sound sample rate.
HardwareTimer sampleTimer(TIM1);

//--------------------------------------------------------
// Advanced: Thread-Safe Knob Class
//--------------------------------------------------------
class Knob {
public:
    Knob(int lowerLimit = 0, int upperLimit = 8)
      : lower(lowerLimit), upper(upperLimit), rotation(upperLimit),
        prevState(0), lastDir(0)
    {
        mutex = xSemaphoreCreateMutex();
    }
    void update(bool bitA, bool bitB) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        uint8_t curr = ((bitB ? 1 : 0) << 1) | (bitA ? 1 : 0);
        int delta = 0;
        if (prevState == 0b00 && curr == 0b01) {
            delta = +1;
        } else if (prevState == 0b01 && curr == 0b00) {
            delta = -1;
        } else if (prevState == 0b11 && curr == 0b10) {
            delta = +1;
        } else if (prevState == 0b10 && curr == 0b11) {
            delta = -1;
        } else if ((prevState == 0b00 && curr == 0b11) ||
                   (prevState == 0b11 && curr == 0b00) ||
                   (prevState == 0b01 && curr == 0b10) ||
                   (prevState == 0b10 && curr == 0b01)) {
            delta = lastDir;
        }
        if (delta != 0) { lastDir = delta; }
        int newRot = rotation + delta;
        if (newRot < lower) newRot = lower;
        if (newRot > upper) newRot = upper;
        rotation = newRot;
        prevState = curr;
        xSemaphoreGive(mutex);
    }
    void setLimits(int lowerLimit, int upperLimit) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        lower = lowerLimit;
        upper = upperLimit;
        if (rotation < lower) rotation = lower;
        if (rotation > upper) rotation = upper;
        xSemaphoreGive(mutex);
    }
    int read() {
        int value;
        xSemaphoreTake(mutex, portMAX_DELAY);
        value = rotation;
        xSemaphoreGive(mutex);
        return value;
    }
    int readAtomic() {
        return __atomic_load_n(&rotation, __ATOMIC_RELAXED);
    }
private:
    int lower;
    int upper;
    int rotation;
    uint8_t prevState;
    int lastDir;
    SemaphoreHandle_t mutex;
};

// Create an instance for knob 3 (the right-hand knob)
Knob knob3;

//--------------------------------------------------------
// Global Variables for CAN Receive Queue and Global RX Buffer
//--------------------------------------------------------
QueueHandle_t msgInQ;
volatile uint8_t global_RX_Message[8] = {0};

//--------------------------------------------------------
// Global Variables for CAN Transmit Queue and Semaphore (Sender Mode)
//--------------------------------------------------------
#ifdef SENDER_MODULE
QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;
#endif

//--------------------------------------------------------
// CAN Receive ISR: Called whenever a CAN message is received.
// It retrieves the message and places it in the receive queue.
//--------------------------------------------------------
#ifndef DISABLE_ISRS
void CAN_RX_ISR(void) {
    uint8_t RX_Message_ISR[8];
    uint32_t ID;
    CAN_RX(ID, RX_Message_ISR);
    xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}
#endif

//--------------------------------------------------------
// CAN Transmit ISR: Called when a mailbox becomes available.
// It gives the semaphore so that the transmit task can send a message.
//--------------------------------------------------------
#ifdef SENDER_MODULE
#ifndef DISABLE_ISRS
void CAN_TX_ISR(void) {
    xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}
#endif
#endif

//--------------------------------------------------------
// Decode Task: Processes received CAN messages.
// It blocks until a message is available, copies it to a global buffer,
// and (if in receiver mode) updates the current step size.
//--------------------------------------------------------
void decodeTask(void * pvParameters) {
    uint8_t local_RX_Message[8] = {0};
    for (;;) {
        xQueueReceive(msgInQ, local_RX_Message, portMAX_DELAY);
        // Copy to global RX buffer (protected by sysState.mutex)
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        memcpy((void*)global_RX_Message, local_RX_Message, 8);
        xSemaphoreGive(sysState.mutex);
#ifndef SENDER_MODULE
        // In receiver mode, update currentStepSize based on the received message.
        if (local_RX_Message[0] == 'R') {
            __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
        } else if (local_RX_Message[0] == 'P') {
            uint8_t note = local_RX_Message[2];    // Note number (0-11)
            uint8_t octave = local_RX_Message[1];    // Octave number
            uint32_t newStep = stepSizes[note];
            if (octave > MODULE_OCTAVE) {
                newStep = newStep << (octave - MODULE_OCTAVE);
            } else if (octave < MODULE_OCTAVE) {
                newStep = newStep >> (MODULE_OCTAVE - octave);
            }
            __atomic_store_n(&currentStepSize, newStep, __ATOMIC_RELAXED);
        }
#endif
    }
}

#ifdef SENDER_MODULE
//--------------------------------------------------------
// CAN Transmit Task: Waits for messages from the transmit queue
// and sends them when a mailbox is available.
//--------------------------------------------------------
void CAN_TX_Task(void * pvParameters) {
    uint8_t msgOut[8];
    for (;;) {
        xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
        xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
        CAN_TX(0x123, msgOut);
    }
}
#endif

//--------------------------------------------------------
// Function: setOutMuxBit()
// Description: Sets a multiplexer bit using the key matrix.
//--------------------------------------------------------
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

//--------------------------------------------------------
// Function: setRow()
// Description: Selects a given row in the key matrix.
//--------------------------------------------------------
void setRow(uint8_t rowIdx) {
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, (rowIdx & 0x01) ? HIGH : LOW);
  digitalWrite(RA1_PIN, (rowIdx & 0x02) ? HIGH : LOW);
  digitalWrite(RA2_PIN, (rowIdx & 0x04) ? HIGH : LOW);
  digitalWrite(REN_PIN, HIGH);
}

//--------------------------------------------------------
// Function: readCols()
// Description: Reads the four columns of the key matrix.
//--------------------------------------------------------
std::bitset<4> readCols() {
  std::bitset<4> result;
  result[0] = (digitalRead(C0_PIN) == HIGH);
  result[1] = (digitalRead(C1_PIN) == HIGH);
  result[2] = (digitalRead(C2_PIN) == HIGH);
  result[3] = (digitalRead(C3_PIN) == HIGH);
  return result;
}

//--------------------------------------------------------
// ISR: sampleISR()
// Description: Called at 22kHz to generate the sawtooth wave.
// It applies a log taper volume control using the knob rotation.
//--------------------------------------------------------
#ifndef DISABLE_ISRS
void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  int knobRotation = knob3.readAtomic();
  Vout = Vout >> (8 - knobRotation);
  analogWrite(OUTR_PIN, Vout + 128);
}
#endif

//--------------------------------------------------------
// Task: scanKeysTask()
// Description: Scans the key matrix (rows 0-3) every 20ms, decodes knob 3,
// updates local key states, and (if in sender mode) queues a CAN message.
// In receiver mode, it updates localStepSize from keys.
//--------------------------------------------------------
#ifndef TEST_SCANKEYS
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static bool firstIteration = true;
  static std::bitset<32> prevInputs;
  
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    std::bitset<32> inputs;
    uint32_t localStepSize = 0;
    for (uint8_t row = 0; row < 4; row++) {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> cols = readCols();
      for (uint8_t col = 0; col < 4; col++) {
        inputs[row * 4 + col] = cols[col];
#ifndef SENDER_MODULE
        if (row < 3 && (row * 4 + col < 12)) {
          if (!cols[col]) {
            localStepSize = stepSizes[row * 4 + col];
          }
        }
#endif
      }
    }
    
    if (firstIteration) {
      prevInputs = inputs;
      firstIteration = false;
    }
    
    // If a key change is detected among the first 12 keys, create a CAN message.
    for (int i = 0; i < 12; i++) {
      if (inputs[i] != prevInputs[i]) {
#ifdef SENDER_MODULE
        uint8_t TX_Message[8] = {0};
        if (!inputs[i]) {
          TX_Message[0] = 'P';
        } else {
          TX_Message[0] = 'R';
        }
        TX_Message[1] = MODULE_OCTAVE;  // Use the module's configured octave.
        TX_Message[2] = i;              // Note number.
        xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
#endif
        break; // Send only one message per scan iteration.
      }
    }
    prevInputs = inputs;
    
    // Update knob 3 from row 3 (columns 0 and 1).
    bool knobB = inputs[12];
    bool knobA = inputs[13];
    knob3.update(knobA, knobB);
    
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = inputs;
    xSemaphoreGive(sysState.mutex);
    
#ifndef SENDER_MODULE
    __atomic_store_n(&currentStepSize, localStepSize, __ATOMIC_RELAXED);
#endif
  }
}
#else
//--------------------------------------------------------
// Test Version: scanKeysTaskTest()
// Description: Runs one iteration of scanKeysTask in worst-case mode.
// It generates a key press message for each of the 12 keys regardless of input.
//--------------------------------------------------------
void scanKeysTaskTest() {
#ifdef SENDER_MODULE
  for (int i = 0; i < 12; i++) {
    uint8_t TX_Message[8] = {0};
    TX_Message[0] = 'P';           // Simulate a key press for worst-case.
    TX_Message[1] = MODULE_OCTAVE; // Module's octave.
    TX_Message[2] = i;             // Note number.
    // In test mode, use non-blocking queue send.
    xQueueSend(msgOutQ, TX_Message, 0);
  }
#endif
  // (In receiver mode, you might simulate updating localStepSize similarly.)
}
#endif

//--------------------------------------------------------
// Task: updateDisplayTask()
// Description: Updates the OLED display every 100ms with "Hello World!",
// key matrix state (hex), current knob 3 rotation, and the latest received CAN message.
//--------------------------------------------------------
void updateDisplayTask(void * pvParameters) {
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(2, 10, "Hello World!");
    
    u8g2.setCursor(2, 20);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    uint32_t inputs_hex = sysState.inputs.to_ulong();
    xSemaphoreGive(sysState.mutex);
    u8g2.print(inputs_hex, HEX);
    
    u8g2.setCursor(2, 30);
    u8g2.print("Volume: ");
    u8g2.print(knob3.read());
    
    // Display the latest received CAN message.
    u8g2.setCursor(66, 30);
    uint8_t tempMsg[8] = {0};
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    memcpy(tempMsg, (void*)global_RX_Message, 8);
    xSemaphoreGive(sysState.mutex);
    u8g2.print((char) tempMsg[0]);
    u8g2.print(tempMsg[1]);
    u8g2.print(tempMsg[2]);
    
    u8g2.sendBuffer();
    digitalToggle(LED_BUILTIN);
  }
}

//--------------------------------------------------------
// setup()
// Description: Initializes pins, display, UART, hardware timer, CAN bus,
// creates queues, semaphores, and tasks, and starts the scheduler.
//--------------------------------------------------------
void setup() {
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);
  
  setOutMuxBit(DRST_BIT, LOW);
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);
  
  Serial.begin(9600);
  Serial.println("Hello World");
  
  // Initialize hardware timer for sound generation at 22kHz.
#ifndef DISABLE_ISRS
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();
#else
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.resume();
#endif
  
  // Initialize CAN bus in loopback mode.
  CAN_Init(true);
  setCANFilter(0x123, 0x7ff);
#ifndef DISABLE_ISRS
  // Register the CAN RX ISR.
  CAN_RegisterRX_ISR(CAN_RX_ISR);
#ifdef SENDER_MODULE
  // Register the CAN TX ISR.
  CAN_RegisterTX_ISR(CAN_TX_ISR);
#endif
#endif
  CAN_Start();
  
  // Create the CAN receive queue (36 items, each 8 bytes)
  msgInQ = xQueueCreate(36, 8);
  
#ifdef SENDER_MODULE
  // Create the CAN transmit queue. Increase its size in test mode if needed.
  #ifdef TEST_SCANKEYS
    msgOutQ = xQueueCreate(384, 8);
  #else
    msgOutQ = xQueueCreate(36, 8);
  #endif
  // Create a counting semaphore with max count 3 (mailbox slots) and initial count 3.
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);
#endif
  
  sysState.mutex = xSemaphoreCreateMutex();
  
#ifndef DISABLE_THREADS
  TaskHandle_t scanKeysHandle = NULL;
#ifndef TEST_SCANKEYS
  xTaskCreate(scanKeysTask, "scanKeys", 64, NULL, 2, &scanKeysHandle);
#else
  // In test mode, do not create the task.
#endif
  
  TaskHandle_t displayHandle = NULL;
  xTaskCreate(updateDisplayTask, "updateDisplay", 256, NULL, 1, &displayHandle);
  
  TaskHandle_t decodeHandle = NULL;
  xTaskCreate(decodeTask, "decodeTask", 128, NULL, 2, &decodeHandle);
  
#ifdef SENDER_MODULE
  TaskHandle_t canTxHandle = NULL;
  xTaskCreate(CAN_TX_Task, "CAN_TX_Task", 128, NULL, 3, &canTxHandle);
#endif
#endif  // DISABLE_THREADS
  
#ifdef TEST_SCANKEYS
  // Measure execution time of 32 iterations of scanKeysTaskTest.
  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++) {
    scanKeysTaskTest();
  }
  uint32_t elapsed = micros() - startTime;
  Serial.print("32 iterations of scanKeysTaskTest took: ");
  Serial.print(elapsed);
  Serial.println(" microseconds");
  while(1);  // Halt after test.
#endif
  
#ifndef TEST_SCANKEYS
  vTaskStartScheduler();
#endif
}

//--------------------------------------------------------
// loop()
// Description: Empty because FreeRTOS tasks run independently (or test code halts).
//--------------------------------------------------------
void loop() {
  // Nothing to do here.
}

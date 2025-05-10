#include "mbed.h"
#include "arm_book_lib.h"
#include <time.h>



#define NUMBER_OF_KEYS                           4
#define BLINKING_TIME_GAS_ALARM               1000
#define BLINKING_TIME_OVER_TEMP_ALARM          500
#define BLINKING_TIME_GAS_AND_OVER_TEMP_ALARM  100
#define NUMBER_OF_AVG_SAMPLES                   100
#define OVER_TEMP_LEVEL                         50
#define TIME_INCREMENT_MS                       10
#define DEBOUNCE_KEY_TIME_MS                    40
#define KEYPAD_NUMBER_OF_ROWS                    4
#define KEYPAD_NUMBER_OF_COLS                    4
#define EVENT_MAX_STORAGE                        5
#define EVENT_NAME_MAX_LENGTH                   14



typedef enum {
    MATRIX_KEYPAD_SCANNING,
    MATRIX_KEYPAD_DEBOUNCE,
    MATRIX_KEYPAD_KEY_HOLD_PRESSED
} matrixKeypadState_t;

typedef struct systemEvent {
    time_t seconds;
    char typeOfEvent[EVENT_NAME_MAX_LENGTH];
} systemEvent_t;


DigitalIn alarmTestButton(BUTTON1);
DigitalIn mq2(PE_12);

DigitalOut alarmLed(LED1);
DigitalOut incorrectCodeLed(LED3);
DigitalOut systemBlockedLed(LED2);

DigitalInOut sirenPin(PE_10);

UnbufferedSerial uartUsb(USBTX, USBRX, 115200);

AnalogIn lm35(A1);

DigitalOut keypadRowPins[KEYPAD_NUMBER_OF_ROWS] = {PB_3, PB_5, PC_7, PA_15};
DigitalIn keypadColPins[KEYPAD_NUMBER_OF_COLS]  = {PB_12, PB_13, PB_15, PC_6};



bool alarmState = OFF;
bool incorrectCode = false;
bool overTempDetector = OFF;

int numberOfIncorrectCodes = 0;
int numberOfHashKeyReleasedEvents = 0;
int keyBeingCompared = 0;
char codeSequence[NUMBER_OF_KEYS] = {'1', '8', '0', '5'};
char keyPressed[NUMBER_OF_KEYS] = {'0', '0', '0', '0'};
int accumulatedTimeAlarm = 0;

bool alarmLastState = OFF;
bool gasLastState = OFF;
bool tempLastState = OFF;
bool ICLastState = OFF;
bool SBLastState = OFF;

bool gasDetectorState = OFF;
bool overTempDetectorState = OFF;

float potentiometerReading = 0.0;
float lm35ReadingsAverage = 0.0;
float lm35ReadingsSum = 0.0;
float lm35ReadingsArray[NUMBER_OF_AVG_SAMPLES];
float lm35TempC = 0.0;

int accumulatedDebounceMatrixKeypadTime = 0;
int matrixKeypadCodeIndex = 0;
char matrixKeypadLastKeyPressed = '\0';
char matrixKeypadIndexToCharArray[] = {
    '1', '2', '3', 'A',
    '4', '5', '6', 'B',
    '7', '8', '9', 'C',
    '*', '0', '#', 'D',
};
matrixKeypadState_t matrixKeypadState;

int eventsIndex = 0;
systemEvent_t arrayOfStoredEvents[EVENT_MAX_STORAGE];

//=====[Declarations (prototypes) of public functions]=========================

void inputsInit();
void outputsInit();
void alarmActivationUpdate();
void alarmDeactivationUpdate();
void uartTask();
void availableCommands();
bool areEqual();
void eventLogUpdate();
void systemElementStateUpdate(bool lastState, bool currentState, const char* elementName);
float celsiusToFahrenheit(float tempInCelsiusDegrees);
float analogReadingScaledWithTheLM35Formula(float analogReading);
void lm35ReadingsArrayInit();
void matrixKeypadInit();
char matrixKeypadScan();
char matrixKeypadUpdate();
void displayEventLog();
void initTime();

//=====[Main function, the program entry point after power on or reset]========

int main()
{
    inputsInit();
    outputsInit();
    matrixKeypadInit();
    initTime();
    lm35ReadingsArrayInit();
    
    // Initial prompt
    printf("Enter Code to Deactivate Alarm\r\n");
    
    while (true) {
        alarmActivationUpdate();
        alarmDeactivationUpdate();
        uartTask();
        eventLogUpdate();
        ThisThread::sleep_for(chrono::milliseconds(TIME_INCREMENT_MS));
    }
}

//=====[Implementations of public functions]===================================

void inputsInit()
{
    alarmTestButton.mode(PullUp);
    mq2.mode(PullUp);
}

void outputsInit()
{
    alarmLed = OFF;
    incorrectCodeLed = OFF;
    systemBlockedLed = OFF;
    sirenPin = OFF;
}

void alarmActivationUpdate()
{
    gasDetectorState = !mq2;
    lm35ReadingsSum = 0;
    for (int i = 0; i < NUMBER_OF_AVG_SAMPLES; i++) {
        lm35ReadingsSum += lm35ReadingsArray[i];
    }
    lm35ReadingsAverage = lm35ReadingsSum / NUMBER_OF_AVG_SAMPLES;
    lm35TempC = analogReadingScaledWithTheLM35Formula(lm35ReadingsAverage);
    overTempDetectorState = lm35TempC > OVER_TEMP_LEVEL;
    
    if (gasDetectorState || overTempDetectorState || !alarmTestButton) {
        if (!alarmState) {
            alarmState = ON;
            sirenPin = ON;
            time_t currentTime;
            time(&currentTime);
            printf("Alarm Triggered at: %s", ctime(&currentTime));
            
            // Store event
            if (eventsIndex < EVENT_MAX_STORAGE) {
                arrayOfStoredEvents[eventsIndex].seconds = currentTime;
                strcpy(arrayOfStoredEvents[eventsIndex].typeOfEvent, "ALARM_TRIGGER");
                eventsIndex++;
            } else {
                // Shift events and add new one
                for (int i = 0; i < EVENT_MAX_STORAGE - 1; i++) {
                    arrayOfStoredEvents[i] = arrayOfStoredEvents[i + 1];
                }
                arrayOfStoredEvents[EVENT_MAX_STORAGE - 1].seconds = currentTime;
                strcpy(arrayOfStoredEvents[EVENT_MAX_STORAGE - 1].typeOfEvent, "ALARM_TRIGGER");
            }
        }
    }
}

void alarmDeactivationUpdate()
{
    char key = matrixKeypadUpdate();
    
    if (key != '\0' && key != '#') {
        if (matrixKeypadCodeIndex < NUMBER_OF_KEYS) {
            keyPressed[matrixKeypadCodeIndex] = key;
            matrixKeypadCodeIndex++;
        }
        
        if (matrixKeypadCodeIndex == NUMBER_OF_KEYS) {
            if (areEqual()) {
                alarmState = OFF;
                sirenPin = OFF;
                alarmLed = OFF;
                incorrectCode = false;
                numberOfIncorrectCodes = 0;
                matrixKeypadCodeIndex = 0;
                printf("Alarm Deactivated\r\n");
            } else {
                incorrectCode = true;
                numberOfIncorrectCodes++;
                matrixKeypadCodeIndex = 0;
                printf("Incorrect Code\r\n");
            }
        }
    }
    
    if (key == '#') {
        displayEventLog();
    }
    
    if (numberOfIncorrectCodes >= 3) {
        systemBlockedLed = ON;
    }
    
    incorrectCodeLed = incorrectCode;
}

void uartTask()
{
    // Update temperature readings
    static int index = 0;
    lm35ReadingsArray[index] = lm35.read();
    index = (index + 1) % NUMBER_OF_AVG_SAMPLES;
}

bool areEqual()
{
    for (int i = 0; i < NUMBER_OF_KEYS; i++) {
        if (keyPressed[i] != codeSequence[i]) {
            return false;
        }
    }
    return true;
}

void eventLogUpdate()
{
    systemElementStateUpdate(gasLastState, gasDetectorState, "GAS_DETECTOR");
    systemElementStateUpdate(tempLastState, overTempDetectorState, "OVER_TEMP");
    systemElementStateUpdate(alarmLastState, alarmState, "ALARM");
    systemElementStateUpdate(ICLastState, incorrectCode, "INCORRECT_CODE");
    systemElementStateUpdate(SBLastState, systemBlockedLed, "SYSTEM_BLOCKED");
}

void systemElementStateUpdate(bool lastState, bool currentState, const char* elementName)
{
    if (lastState != currentState) {
        time_t currentTime;
        time(&currentTime);
        if (eventsIndex < EVENT_MAX_STORAGE) {
            arrayOfStoredEvents[eventsIndex].seconds = currentTime;
            strncpy(arrayOfStoredEvents[eventsIndex].typeOfEvent, elementName, EVENT_NAME_MAX_LENGTH - 1);
            arrayOfStoredEvents[eventsIndex].typeOfEvent[EVENT_NAME_MAX_LENGTH - 1] = '\0';
            eventsIndex++;
        } else {
            for (int i = 0; i < EVENT_MAX_STORAGE - 1; i++) {
                arrayOfStoredEvents[i] = arrayOfStoredEvents[i + 1];
            }
            arrayOfStoredEvents[EVENT_MAX_STORAGE - 1].seconds = currentTime;
            strncpy(arrayOfStoredEvents[EVENT_MAX_STORAGE - 1].typeOfEvent, elementName, EVENT_NAME_MAX_LENGTH - 1);
            arrayOfStoredEvents[EVENT_MAX_STORAGE - 1].typeOfEvent[EVENT_NAME_MAX_LENGTH - 1] = '\0';
        }
    }
}

float celsiusToFahrenheit(float tempInCelsiusDegrees)
{
    return (tempInCelsiusDegrees * 9.0 / 5.0) + 32.0;
}

float analogReadingScaledWithTheLM35Formula(float analogReading)
{
    return (analogReading * 3.3 / 0.01);
}

void lm35ReadingsArrayInit()
{
    for (int i = 0; i < NUMBER_OF_AVG_SAMPLES; i++) {
        lm35ReadingsArray[i] = 0.0;
    }
}

void matrixKeypadInit()
{
    matrixKeypadState = MATRIX_KEYPAD_SCANNING;
    int pinIndex = 0;
    for (pinIndex = 0; pinIndex < KEYPAD_NUMBER_OF_COLS; pinIndex++) {
        (keypadColPins[pinIndex]).mode(PullUp);
    }
}

char matrixKeypadScan()
{
    int row = 0;
    int col = 0;
    int i = 0;

    for (row = 0; row < KEYPAD_NUMBER_OF_ROWS; row++) {
        for (i = 0; i < KEYPAD_NUMBER_OF_ROWS; i++) {
            keypadRowPins[i] = ON;
        }
        keypadRowPins[row] = OFF;
        for (col = 0; col < KEYPAD_NUMBER_OF_COLS; col++) {
            if (keypadColPins[col] == OFF) {
                return matrixKeypadIndexToCharArray[row * KEYPAD_NUMBER_OF_ROWS + col];
            }
        }
    }
    return '\0';
}

char matrixKeypadUpdate()
{
    char keyDetected = '\0';
    char keyReleased = '\0';

    switch (matrixKeypadState) {
    case MATRIX_KEYPAD_SCANNING:
        keyDetected = matrixKeypadScan();
        if (keyDetected != '\0') {
            matrixKeypadLastKeyPressed = keyDetected;
            accumulatedDebounceMatrixKeypadTime = 0;
            matrixKeypadState = MATRIX_KEYPAD_DEBOUNCE;
        }
        break;

    case MATRIX_KEYPAD_DEBOUNCE:
        if (accumulatedDebounceMatrixKeypadTime >= DEBOUNCE_KEY_TIME_MS) {
            keyDetected = matrixKeypadScan();
            if (keyDetected == matrixKeypadLastKeyPressed) {
                matrixKeypadState = MATRIX_KEYPAD_KEY_HOLD_PRESSED;
            } else {
                matrixKeypadState = MATRIX_KEYPAD_SCANNING;
            }
        }
        accumulatedDebounceMatrixKeypadTime += TIME_INCREMENT_MS;
        break;

    case MATRIX_KEYPAD_KEY_HOLD_PRESSED:
        keyDetected = matrixKeypadScan();
        if (keyDetected != matrixKeypadLastKeyPressed) {
            if (keyDetected == '\0') {
                keyReleased = matrixKeypadLastKeyPressed;
            }
            matrixKeypadState = MATRIX_KEYPAD_SCANNING;
        }
        break;

    default:
        matrixKeypadInit();
        break;
    }
    return keyReleased;
}

void displayEventLog()
{
    printf("\r\n=== Recent Alarm Events ===\r\n");
    for (int i = 0; i < min(eventsIndex, EVENT_MAX_STORAGE); i++) {
        printf("Event %d: %s at %s", i + 1, 
               arrayOfStoredEvents[i].typeOfEvent, 
               ctime(&arrayOfStoredEvents[i].seconds));
    }
    printf("==========================\r\n");
}

void initTime()
{
    // Initialize system time (no hardware RTC)
    // Set to a known time for consistency
    set_time(1746204836); // Jan 1, 2025, 00:00:00 UTC
}

//=====[Implementations of private functions]==================================

void availableCommands()
{
    printf("Available commands:\r\n");
    printf("Enter 4-digit code to deactivate alarm\r\n");
    printf("Press '#' to display event log\r\n");
}

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "clock.h"
#include "wait.h"
#include "nvic.h"
#include "uart0.h"
#include "motor_control.h"
#include "i2c0.h"
#include <math.h>  // for atan2 and sqrt
#include "eeprom.h"
#include <stdlib.h>


//IR Decoder PD0
//   PhA0 (PD6) and PhB0 (PD7) are connected to 256 cpr optical encoder
// Motor 0: PWM PA6, DIR PA7 --- OP sen PC7
// Motor 1: PWM PF2, DIR PF3 --- OP sen PC5

// Define the IR remote signals
const char RIGHT_ARROW[] =  "00100000110111110110000010011111";
const char LEFT_ARROW[] =   "00100000110111111110000000011111";
const char UP_ARROW[] =     "00100000110111110000001011111101";
const char DOWN_ARROW[] =   "00100000110111111000001001111101";
const char STOP[] =         "00100000110111110011111011000001";
#define SLEEP_PIN       PORTF, 1

#define IR_PIN          PORTD, 0
#define OP_PIN_PC5      PORTC, 5
#define OP_PIN_PC7      PORTC, 7



uint32_t targetinterruptCount_PC5 = 0;
uint32_t targetinterruptCount_PE2 = 0;
bool shouldStopMotors = false;
uint32_t OptTime0=0;
uint32_t OptTime1=0;
volatile uint32_t OptCount0=0;
volatile uint32_t OptCount1=0;
// Struct to represent the IR protocol parameters
typedef struct
{
    uint32_t InitTime;
    uint32_t Logic_0;
    uint32_t Logic_1;
    uint32_t Leniency;
} IRProtocol;

// Enum to represent the different buttons on the remote
typedef enum
{
    BUTTON_RIGHT_ARROW,
    BUTTON_LEFT_ARROW,
    BUTTON_UP_ARROW,
    BUTTON_DOWN_ARROW,
    BUTTON_STOP,
    BUTTON_UNKNOWN,

} RemoteButton;

// Struct to represent the IR signal data
typedef struct
{
    char data[33]; // 33 to include null terminator
    uint32_t count;
    bool ReceivingBit_Flag;
    RemoteButton button;
} IRSignal;

// Initialize the IR protocol parameters
const IRProtocol irProtocol = {
                               .InitTime = 13500000, // Start signal duration ns
                               .Logic_0 = 1125000,      // '0' signal duration --2T
                               .Logic_1 = 2250000,      // '1' signal duration -- 4T
                               .Leniency = (2250000/4)*0.5        // Tolerance for timing discrepancies 0.5T
};

// Initialize the IR signal
IRSignal irSignal = {
                     .data = {0},
                     .count = 0,
                     .ReceivingBit_Flag = false,
                     .button = BUTTON_UNKNOWN
};


void initHw()
{
    // System clock to 40 MHz
    initSystemClockTo40Mhz();
    // Enable Wide Timer 0,1,2
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R2| SYSCTL_RCGCWTIMER_R0| SYSCTL_RCGCWTIMER_R1 |SYSCTL_RCGCWTIMER_R3;

    _delay_cycles(3);
    // Enable ports
    enablePort(PORTD);
    enablePort(PORTF);
    enablePort(PORTC);


    //**************************Remote DCD**********************************
    // Configure IR pin DCD
    setPinAuxFunction(IR_PIN, GPIO_PCTL_PD0_WT2CCP0);
    selectPinDigitalInput(IR_PIN);

    //**************************OPB876N55***********************************
    // Configure IR pin OPB876N55 optical PortC pin 5
    setPinAuxFunction(OP_PIN_PC5, GPIO_PCTL_PC5_WT0CCP1);
    selectPinDigitalInput(OP_PIN_PC5);

    // Configure IR pin OPB876N55 optical PortC pin 7
    setPinAuxFunction(OP_PIN_PC7, GPIO_PCTL_PC7_WT1CCP1);
    selectPinDigitalInput(OP_PIN_PC7);
    //**********************************************************************

    selectPinPushPullOutput(SLEEP_PIN);

}


//**************************MPU6050 I2C*******************************************
// MPU-6050 I2C Address and Registers
#define MPU6050_ADDRESS     0x68
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_WHO_AM_I    0x75
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
int16_t gyroBias[3] = {0, 0, 0};
float initialPitch = 0;
float initialtilt = 90.0f;

void initMPU6050() {
    writeI2c0Register(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);
}

void readMpu6050Data(int16_t* accelData, int16_t* gyroData) {
    uint8_t buffer[14];
    readI2c0Registers(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, buffer, 14);
    int i;
    for (i = 0; i < 3; i++) {
        accelData[i] = (int16_t)(buffer[2*i] << 8 | buffer[2*i + 1]);
        gyroData[i] = (int16_t)(buffer[2*i + 8] << 8 | buffer[2*i + 9]);
    }
}

void calibrateGyroscope() {
    int numReadings = 1000;
    int16_t gyroData[3];
    int16_t accelData[3];
    int i;
    for (i = 0; i < numReadings; i++) {
        readMpu6050Data(accelData, gyroData);
        gyroBias[0] += gyroData[0];
        gyroBias[1] += gyroData[1];
        gyroBias[2] += gyroData[2];
        waitMicrosecond(1000);
    }

    gyroBias[0] /= numReadings;
    gyroBias[1] /= numReadings;
    gyroBias[2] /= numReadings;

    float ax = accelData[0] / 16384.0;
    float ay = accelData[1] / 16384.0;
    float az = accelData[2] / 16384.0;

    initialPitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
    // tilt is set to 90 degrees above
}

char* itoa(int value, char* result, int base) {
    if (base < 2 || base > 36) {
        *result = '\0';
        return result;
    }

    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;
    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz"[35 + (tmp_value - value * base)];
    } while (value);
    if (tmp_value < 0 && base == 10)
        *ptr++ = '-';
    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr-- = *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}


void CountOptical_PC7_L(void)
{

    OptTime0= WTIMER1_TBV_R /40;
    WTIMER1_TBV_R =0; // reset value of widetimer 1 counter
    WTIMER1_ICR_R = TIMER_ICR_CBECINT;

}
void CountOptical_PC5_R(void)
{

    OptTime1= WTIMER0_TBV_R /40 ;
    WTIMER0_TBV_R =0; // reset value of widetimer 1 counter
    WTIMER0_ICR_R = TIMER_ICR_CBECINT;
}


void GPIOPortC_Handler(void)
{
    // Read the raw interrupt status to identify which pin(s) caused the interrupt
    uint32_t status = GPIO_PORTC_RIS_R; // This contains the raw interrupt status

    // Check if PC5 caused the interrupt
    if (status & (1 << 5))
    {
        // Handle the interrupt for PC5 (e.g., increase counter, toggle LED, etc.)
        OptCount1++;

        // Clear the interrupt for PC5
        GPIO_PORTC_ICR_R = (1 << 5);
    }

    // Check if PC5 caused the interrupt
    if (status & (1 << 7))
    {
        // Handle the interrupt for PC5 (e.g., increase counter, toggle LED, etc.)
        OptCount0++;

        // Clear the interrupt for PC5
        GPIO_PORTC_ICR_R = (1 << 7);
    }
}

//******************** DECODER *******************************************************
void enableWideTimer_DCD()
{
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER2_CFG_R = 4;
    WTIMER2_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
    WTIMER2_CTL_R = TIMER_CTL_TAEVENT_NEG;
    WTIMER2_TAV_R = 0;
    WTIMER2_CTL_R |= TIMER_CTL_TAEN;


}

void SetUp_Interrupt()
{
    //DCD
    enablePort(PORTD);
    selectPinInterruptFallingEdge(IR_PIN);

    //OPTICAL
    //enablePort(PORTE);
    enablePort(PORTC);
    enablePinInterrupt(OP_PIN_PC7);
    selectPinInterruptRisingEdge(OP_PIN_PC7);
    enableNvicInterrupt(INT_GPIOC);


    enablePort(PORTC);
    enablePinInterrupt(OP_PIN_PC5);
    selectPinInterruptRisingEdge(OP_PIN_PC5);
    enableNvicInterrupt(INT_GPIOC);





}

void DECODE_ISR()
{
    uint32_t time = 0;
    enableWideTimer_DCD();
    time = WTIMER2_TAR_R * 25;  // convert ns

    if (time >= irProtocol.InitTime - irProtocol.Leniency && time <= irProtocol.InitTime + irProtocol.Leniency)
    {
        irSignal.ReceivingBit_Flag = true;
        irSignal.count = 0;
    }

    if (irSignal.ReceivingBit_Flag)
    {
        if (time >= irProtocol.Logic_0 - irProtocol.Leniency && time <= irProtocol.Logic_0 + irProtocol.Leniency)
        {
            irSignal.data[irSignal.count] = '0';
            irSignal.count++;
        } else if (time >= irProtocol.Logic_1 - irProtocol.Leniency && time <= irProtocol.Logic_1 + irProtocol.Leniency) {
            irSignal.data[irSignal.count] = '1';           irSignal.count++;
        }

        if (irSignal.count >= 32)
        {
            irSignal.ReceivingBit_Flag = false;
            irSignal.data[irSignal.count] = '\0';  // Null-terminate the string

            // Output the received data string over UART for debugging
            char debugString[50];
            snprintf(debugString, sizeof(debugString), "data = %s \n", irSignal.data);
            putsUart0(debugString);


            if (strcmp(irSignal.data, RIGHT_ARROW) == 0)
            {
                irSignal.button = BUTTON_RIGHT_ARROW;
            }
            else if (strcmp(irSignal.data, LEFT_ARROW) == 0)
            {
                irSignal.button = BUTTON_LEFT_ARROW;
            }
            else if (strcmp(irSignal.data, UP_ARROW) == 0)
            {
                irSignal.button = BUTTON_UP_ARROW;
            }
            else if (strcmp(irSignal.data, DOWN_ARROW) == 0)
            {
                irSignal.button = BUTTON_DOWN_ARROW;
            }

            else if (strcmp(irSignal.data, STOP) == 0)
            {
                irSignal.button = BUTTON_STOP;
            }

            else
            {
                irSignal.button = BUTTON_UNKNOWN;
            }
        }
    }

    clearPinInterrupt(IR_PIN);
}

//*******************GPIOC_OPB876N55 SENSOR**********************************

void enableWTimerMode(uint8_t num)
{
    switch (num)
    {
    case 0:
        WTIMER0_CTL_R &= ~TIMER_CTL_TBEN;               // Turn off counter before reconfiguring
        WTIMER0_CFG_R = 4;                              // Configures as 32-bit counter (A only)
        WTIMER0_TBMR_R = TIMER_TBMR_TBCMR | TIMER_TBMR_TBMR_CAP | TIMER_TBMR_TBCDIR;
        // Configure for edge time mode, count up
        WTIMER0_CTL_R = TIMER_CTL_TBEVENT_POS;          // Measure time from negative edge to negative edge
        WTIMER0_IMR_R = TIMER_IMR_CBEIM;
        WTIMER0_TBV_R = 0;                              // Reset counter for first period
        WTIMER0_CTL_R |= TIMER_CTL_TBEN;                // Turn on counter
        enableNvicInterrupt(INT_WTIMER0B);
        break;

    case 1:
        WTIMER1_CTL_R &= ~TIMER_CTL_TBEN;               // Turn off counter before reconfiguring
        WTIMER1_CFG_R = 4;                              // Configures as 32-bit counter (A only)
        WTIMER1_TBMR_R = TIMER_TBMR_TBCMR | TIMER_TBMR_TBMR_CAP | TIMER_TBMR_TBCDIR;
        // Configure for edge time mode, count up
        WTIMER1_CTL_R = TIMER_CTL_TBEVENT_POS;          // Measure time from negative edge to negative edge
        WTIMER1_IMR_R = TIMER_IMR_CBEIM;
        WTIMER1_TBV_R = 0;                              // Reset counter for first period
        WTIMER1_CTL_R |= TIMER_CTL_TBEN;                // Turn on counter
        enableNvicInterrupt(INT_WTIMER1B);
        break;
    case 2:
        //configuration  (WTIMER3A)
        WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;
        WTIMER3_CFG_R = 4;
        WTIMER3_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
        WTIMER3_CTL_R = TIMER_CTL_TAEVENT_POS;
        WTIMER3_IMR_R = TIMER_IMR_CAEIM;
        WTIMER3_TAV_R = 0;
        WTIMER3_CTL_R |= TIMER_CTL_TAEN;
        NVIC_EN3_R = 1 << (INT_WTIMER3A-16-96);
        break;
    }
}

void disableWTimerMode(uint8_t num)
{
    switch (num)
    {
    case 0:
        WTIMER0_CTL_R &= ~TIMER_CTL_TBEN;               // Turn off counter
        disableNvicInterrupt(INT_WTIMER0B);
        break;
    case 1:
        WTIMER1_CTL_R &= ~TIMER_CTL_TBEN;               // Turn off counter
        disableNvicInterrupt(INT_WTIMER1B);
        break;
    case 2:
        //configuration for channel 3 (WTIMER3)
        WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;
        NVIC_DIS3_R = 1 << (INT_WTIMER3A-16-96);
        break;

    }
}

//**************************************EM1***************************************

#define NUMBER_OF_EVENTS 1
bool timeMode = false;
uint32_t frequency = 0;
uint32_t time = 0;
uint32_t time1 = 0;
uint32_t time2 = 0;
uint32_t time3 = 0;

#define MAX_CHARS 80
#define MAX_FIELDS 6

typedef struct USER_DATA
{
    char buffer[MAX_CHARS + 1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
}
USER_DATA;



typedef struct EVENT_DATA
{
    int angle;
    int distance;


} EVENT_DATA;

EVENT_DATA events[NUMBER_OF_EVENTS];




void getsUart0(USER_DATA *data)
{
    uint8_t count = 0;
    char c = NULL;

    while (true)
    {
        c = getcUart0();

        if ((c == 8 || c == 127) && count > 0)
        {
            count--;
        }
        else if (c == 13)
        {
            data->buffer[count] = '\0';
            return;
        }
        else if (c >= 32)
        {
            data->buffer[count] = c;
            count++;
        }

        if (count == MAX_CHARS)
        {
            data->buffer[count] = '\0';
            return;
        }
    }
}

void parseFields(USER_DATA *data)
{
    int i;
    int len = strlen(data->buffer);
    char *buffer = data->buffer;
    data->fieldCount = 0;
    char prevType = 'd';    // Assume the previous character type is a delimiter

    for (i = 0; i < len && data->fieldCount < MAX_FIELDS; i++)
    {
        char c = buffer[i];
        char currType;

        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))
        {
            currType = 'a';
        }
        else if ((c >= '0' && c <= '9') || c == '-' || c == '.')
        {
            currType = 'n';
        }
        else
        {
            currType = 'd';
            buffer[i] = '\0';   // Convert delimiter to NULL character
        }

        if (prevType == 'd' && (currType == 'a' || currType == 'n'))
        {
            data->fieldType[data->fieldCount] = currType;
            data->fieldPosition[data->fieldCount] = i;
            data->fieldCount++;
        }

        prevType = currType;
    }
}

char *getFieldString(USER_DATA *data, uint8_t fieldNumber)
{
    if (fieldNumber < data->fieldCount)
    {
        return &(data->buffer[data->fieldPosition[fieldNumber]]);
    }
    else
    {
        return NULL;
    }
}


int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber)
{
    int32_t fieldValue = 0;

    // Check if fieldNumber is in range and fieldType is numeric
    if (fieldNumber < data->fieldCount && data->fieldType[fieldNumber] == 'n')
    {
        // Convert the field string to an integer and return the value
        fieldValue = atoi(&data->buffer[data->fieldPosition[fieldNumber]]);
    }

    return fieldValue;
}


bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    if (strcmp(strCommand, &data->buffer[data->fieldPosition[0]]) != 0)
    {
        return false;
    }
    uint8_t argumentCount = data->fieldCount - 1;
    if (argumentCount < minArguments)
    {
        return false;
    }

    return true;
}


void saveDeviceParameters(EVENT_DATA *events, uint8_t numberOfEvents)
{

    uint8_t i;
    for (i = 0; i < numberOfEvents; i++)
    {
        uint16_t eepromAddress = i*sizeof(EVENT_DATA);
        writeEeprom(eepromAddress, events[i].angle);
        writeEeprom(eepromAddress+1, events[i].distance);

    }
}

void loadDeviceParameters(EVENT_DATA *events, uint8_t numberOfEvents)
{
    uint8_t i ;
    for (i = 0; i < numberOfEvents; i++)
    {
        uint16_t eepromAddress = i*sizeof(EVENT_DATA);
        events[i].angle = readEeprom(eepromAddress );
        events[i].distance = readEeprom(eepromAddress +1);


    }
}


void displayEvents(EVENT_DATA *events, uint8_t numberOfEvents)
{
    putsUart0("Event | Angle (Degrees)\r\n");
    putsUart0("-----------------------\r\n");
    uint8_t i;
    for (i = 0; i < numberOfEvents; i++)
    {
        char eventString[100];
        sprintf(eventString, "%5u | %d\r\n", i, events[i].angle);
        putsUart0(eventString);
    }
}
void clearAngle(EVENT_DATA *events, uint8_t eventIndex, uint8_t numberOfEvents)
{
    if (eventIndex < numberOfEvents)
    {
        events[eventIndex].angle = 0;

    }
}



//***************************autobalance************************************************

#define WHEEL_TEETH 40
#define WHEEL_DIAMETER 0.1 // Set the actual diameter of your wheel in meters
#define PI 3.14159265
float distancePerTooth = (PI * WHEEL_DIAMETER) / WHEEL_TEETH;
float targetDistance = 0.7;
// PID constants
float Kp = 0;  // Proportional gain
float Ki = 0; // Integral gain
float Kd = 0; // Derivative gain

int Val_def0 = 540;
int Val_def1 = 775;

// PID variables
float previousError = 0;
float integral = 0;


#define GYRO_SCALE_FACTOR 131.0 // ±250 degrees/sec scale
#define GYRO_UPDATE_INTERVAL 0.01
#define CALIBRATION_FACTOR 0.75

float initialZAxisAngle = 0;
float zAxisDeviation = 0;
float zAxisOffset = 0; // Gyro offset


bool FWD = false;
bool BWD = false;
bool CCW = false;
bool CW =  false;

void calibrateGyro() {
    int16_t gyroData[3];
    float sum = 0;
    const int samples = 100;
    int i;
    for (i = 0; i < samples; i++) {
        readMpu6050Data(NULL, gyroData);
        sum += gyroData[2] / GYRO_SCALE_FACTOR;
        waitMicrosecond(100); // Wait for 10 milliseconds between readings
    }

    zAxisOffset = sum / samples;
}
// Function to set the initial Z-axis angle as a reference
void setInitialZAxisAngle() {
    waitMicrosecond(100);
    int16_t gyroData[3];
    readMpu6050Data(NULL, gyroData); // Read only gyro data
    initialZAxisAngle = 0;
    initialZAxisAngle = ((gyroData[2] / GYRO_SCALE_FACTOR) - zAxisOffset) * GYRO_UPDATE_INTERVAL; // Convert to degrees

}

void autobalance() {
    // Read accelerometer and gyroscope data
    int16_t accelData[3], gyroData[3];
    readMpu6050Data(accelData, gyroData);

    // Calculate tilt angle from accelerometer data
    float ax = accelData[0] / 16384.0;
    float az = accelData[2] / 16384.0;
    float tilt = atan2(-ax, az) * 180.0 / M_PI + initialtilt;

    unsigned int pwm0 ;
    unsigned int pwm1 ;


    float desiredTilt, error, output;
    desiredTilt = 87.0;

    Kp = 1;
    Ki = 0.5;
    Kd = 0.1;

    error = tilt - desiredTilt;
    integral += error;
    float derivative = error - previousError;
    output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    // Calculate maxOutput for each motor based on the PID output
    int maxOutputPWM1 = 1023-Val_def1; // Maximum output range for PWM1 (1023 - 700)
    int maxOutputPWM0 = Val_def0-0; // Maximum output range for PWM0 (600 - 0)

    // Scale the output to match the PWM ranges
    int scaledOutputPWM1 = (int)(maxOutputPWM1 * (output / maxOutputPWM1));
    int scaledOutputPWM0 = (int)(maxOutputPWM0 * (output / maxOutputPWM0));
    scaledOutputPWM1=abs(scaledOutputPWM1);
    scaledOutputPWM0=abs(scaledOutputPWM0);
    // Ensure PWM values are within their valid ranges
    pwm1 = (Val_def1 + scaledOutputPWM1 > 1023) ? 1023 : ((Val_def1 + scaledOutputPWM1 < Val_def1) ? Val_def1 : Val_def1 + scaledOutputPWM1);
    pwm0 = (Val_def0 - scaledOutputPWM0 < 0) ? 0 : ((Val_def0 - scaledOutputPWM0 > Val_def0) ? Val_def0 : Val_def0 - scaledOutputPWM0);

    if (tilt < desiredTilt - 5) {
        setPinValue(SLEEP_PIN, 1);
        setMotor(0, pwm0, 1);
        setMotor(1, pwm1, 0);

    }
    else if (tilt > desiredTilt + 5)
    {
        setPinValue(SLEEP_PIN, 1);
        setMotor(0, pwm1, 0);
        setMotor(1, pwm0, 1);

    }
    else
    {
        setPinValue(SLEEP_PIN, 0);
        setMotor(0, 0, 0);
        setMotor(1, 0, 0);

        error=0;
        integral=0;
        derivative=0;
        waitMicrosecond(3000);

    }

}


void turnDegrees(int desiredAngle) {
    float gyroAngle = 0;
    int16_t gyroData[3];
    float targetAngle = desiredAngle * CALIBRATION_FACTOR;

    // Start turning
    if(CCW==true)
    {
    setMotor(0, 800, 0); // left turn
    setMotor(1, 800, 0);
    }
    else if(CW==true)
    {
     setMotor(0, 580, 1);
     setMotor(1, 580, 1);
    }

    while (abs(gyroAngle) < targetAngle) {
        readMpu6050Data(NULL, gyroData); // Read only gyro data

        // Integrate gyro data
        gyroAngle += (gyroData[2] / GYRO_SCALE_FACTOR) * GYRO_UPDATE_INTERVAL; // Convert to degrees

        // Short delay to simulate consistent time step
        waitMicrosecond(10000); // 10 milliseconds
    }

    // Stop motors slightly before reaching the desired angle
    setMotor(0, 0, 0);
    setMotor(1, 0, 0);
}


//*****************************Run Straight*********************

float MAX_TILT_ANGLE;


void goStraightWithBalance() {
    int16_t accelData[3], gyroData[3];
    float tilt, error, derivative, output;
    static float currentZAxisAngle = 0;


    // PID constants for balancing
    float Kps = 0.1; // Proportional gain
    float Kis = 0.05; // Integral gain
    float Kds = 0.01; // Derivative gain

    static float integral = 0;
    static float previousError = 0;


    // Read accelerometer and gyroscope data
    readMpu6050Data(accelData, gyroData);

    // Calculate tilt angle from accelerometer data
    float ax = accelData[0] / 16384.0;
    float az = accelData[2] / 16384.0;
    tilt = atan2(-ax, az) * 180.0 / M_PI + initialtilt;


    // Update current Z-axis angle
    currentZAxisAngle += ((gyroData[2] / GYRO_SCALE_FACTOR) - zAxisOffset) * GYRO_UPDATE_INTERVAL;
    zAxisDeviation = currentZAxisAngle - initialZAxisAngle;



    // PID control for balance
    error = tilt - MAX_TILT_ANGLE; // Desired tilt angle
    integral += error;
    derivative = error - previousError;
    output = Kps * error + Kis * integral + Kds * derivative;
    previousError = error;



    // Motor speed calculations
    unsigned int pwm0 = Val_def0 ;
    unsigned int pwm1 = Val_def1 ;


    // Calculate maxOutput for each motor based on the PID output
    int maxOutputPWM1 = 1023-Val_def1; // Maximum output range for PWM1 (1023 - 700)
    int maxOutputPWM0 = Val_def0-0; // Maximum output range for PWM0 (600 - 0)

    // Scale the output to match the PWM ranges
    int scaledOutputPWM1 = (int)(maxOutputPWM1 * ((output) / maxOutputPWM1));
    int scaledOutputPWM0 = (int)(maxOutputPWM0 * ((output) / maxOutputPWM0));
    scaledOutputPWM1=abs(scaledOutputPWM1);
    scaledOutputPWM0=abs(scaledOutputPWM0);


    // Ensure PWM values are within their valid ranges
    pwm1 = (Val_def1 + scaledOutputPWM1 > 1023) ? 1023 : ((Val_def1 + scaledOutputPWM1 < Val_def1) ? Val_def1 : Val_def1 + scaledOutputPWM1);
    pwm0 = (Val_def0 - scaledOutputPWM0 < 0) ? 0 : ((Val_def0 - scaledOutputPWM0 > Val_def0) ? Val_def0 : Val_def0 - scaledOutputPWM0);


    if (tilt<(MAX_TILT_ANGLE -5))
    {

        if(FWD==true)
        {
            pwm0+=40;
        }

        setMotor(0, pwm0, 1);
        setMotor(1, pwm1, 0);
        if(BWD==true)
        {
            waitMicrosecond(500);
        }
        char str1[200];
        snprintf(str1, sizeof(str1), "Motor0: %d Motor1: %d CurAxis=%.2f Til=%.2f\toutput=%.2f\n", pwm0, pwm1, currentZAxisAngle, tilt,output);
        putsUart0(str1);

    }


    else if (tilt > (MAX_TILT_ANGLE +5))
    {


        setPinValue(SLEEP_PIN, 1);

        if(BWD==true)
        {
            pwm0-=100;
        }
        setMotor(0,pwm1 , 0);
        setMotor(1,pwm0, 1);
        if(BWD==true)
        {
            waitMicrosecond(5000);
        }




    }
    else
    {
        pwm0=Val_def0;
        pwm1=Val_def1;
        error = 0;
        output=0;
        integral = 0;
        derivative = 0;
        previousError = error;


    }

}





int main(void)
    {


    waitMicrosecond(1000000);
    initHw();
    initUart0();
    setUart0BaudRate(115200, 40e6);
    SetUp_Interrupt();
    initMotorControl();
    initI2c0();
    initMPU6050();
    initEeprom();
    float distanceTraveled=0;

    // Enable interrupts for GPIO Port C & D
    enablePinInterrupt(IR_PIN);
    enableNvicInterrupt(INT_GPIOD);

    RemoteButton lastButton = BUTTON_UNKNOWN; // To track the last button state

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);


    // Load the event data from EEPROM
    loadDeviceParameters(events, NUMBER_OF_EVENTS);

    USER_DATA data;

    while (1)
    {


        bool inputReceived = false;

        int16_t accelData[3];
        int16_t gyroData[3];
        readMpu6050Data(accelData, gyroData);

        float ax = accelData[0] / 16384.0;
        //        float ay = accelData[1] / 16384.0;
        float az = accelData[2] / 16384.0;

        float tilt = atan2(-ax, az) * 180.0 / M_PI + initialtilt;

        enableWTimerMode(0);
        enableWTimerMode(1);
        enableWTimerMode(2);


        while (time3 < 10000000)
        {
            time3 = WTIMER3_TAV_R/40;
            if (kbhitUart0())
            {
                inputReceived = true;
                break;
            }

        }

        disableWTimerMode(2);

        if (!inputReceived)
        {

            if (irSignal.button != lastButton)
            {
                switch (irSignal.button)
                {


                case BUTTON_UP_ARROW:
                {

                    BWD = false;
                    FWD=false;

                    setInitialZAxisAngle();
                    calibrateGyro();
                    FWD = true; // Start moving forward
                    setPinValue(SLEEP_PIN, 1);
                    OptCount0 = 0;
                    OptCount1 = 0;
                    int F=1;
                    if(FWD==true && F==1)
                    {
                        setPinValue(SLEEP_PIN, 1);
                        setMotor(0, 780, 0);
                        setMotor(1, 520, 1);
                        waitMicrosecond(7000);
                        setMotor(0, 0, 0);
                        setMotor(1, 0, 0);

                        F =0;
                    }

                    if (FWD == true)
                    {
                        MAX_TILT_ANGLE = 60.0;
                    }

                    while (FWD==true)
                    {

                        distanceTraveled = (OptCount0 * distancePerTooth);


                        setPinValue(SLEEP_PIN, 1);

                        goStraightWithBalance();
                        if(distanceTraveled>=targetDistance)
                        {

                            BWD = false;
                            FWD=false;
                            OptCount0 = 0;
                            OptCount1 = 0;
                            setPinValue(SLEEP_PIN, 0);
                            setMotor(0, 0, 0);
                            setMotor(1, 0, 0);
                            waitMicrosecond(150000);

                        }


                    }



                    break;
                }



                case BUTTON_DOWN_ARROW: // Backward
                {
                    BWD = false;
                    FWD=false;
                    setInitialZAxisAngle();
                    calibrateGyro();
                    BWD = true; // Start moving forward
                    setPinValue(SLEEP_PIN, 1);
                    OptCount0 = 0;
                    OptCount1 = 0;

                    int F=1;
                    if(FWD==true && F==1)
                    {
                        setPinValue(SLEEP_PIN, 1);
                        setMotor(0, 520, 0);
                        setMotor(1, 780, 1);
                        waitMicrosecond(7000);
                        setMotor(0, 0, 0);
                        setMotor(1, 0, 0);

                        F =0;
                    }

                    if (BWD == true)
                    {
                        MAX_TILT_ANGLE = 120.0;
                    }

                    while (BWD==true)
                    {

                        distanceTraveled = (OptCount0* distancePerTooth);
                        setPinValue(SLEEP_PIN, 1);
                        goStraightWithBalance();

                        if(distanceTraveled>=targetDistance)
                        {


                            BWD = false;
                            FWD=false;
                            OptCount0 = 0;
                            OptCount1 = 0;
                            setPinValue(SLEEP_PIN, 0);
                            setMotor(0, 0, 0);
                            setMotor(1, 0, 0);
                            waitMicrosecond(150000);


                        }


                    }
                }

                break;

                case BUTTON_LEFT_ARROW:
                    CW=false;
                    CCW=true;
                    setPinValue(SLEEP_PIN, 1);
                    turnDegrees(abs(events[0].angle));
                    waitMicrosecond(1000);
                    CW=false;
                    CCW=false;
                    break;

                case BUTTON_RIGHT_ARROW:

                    CCW=false;
                    CW=true;
                    setPinValue(SLEEP_PIN, 1);
                    turnDegrees(abs(events[0].angle));
                    waitMicrosecond(1000);
                    CW=false;
                    CCW=false;
                    break;


                case BUTTON_STOP:
                    OptCount0=0;
                    OptCount1=0;
                    setMotor(0, 0, 0);
                    setMotor(1, 0, 0);

                    waitMicrosecond(1000);

                    break;
                }

                lastButton = irSignal.button;  // Update lastButton after handling

                // If a button press was handled, reset to UNKNOWN for next detection
                if (irSignal.button != BUTTON_UNKNOWN)
                {

                    irSignal.button = BUTTON_UNKNOWN;
                }
            }



            waitMicrosecond(200);
            autobalance();

            continue;
        }



        // Get input from the user
        getsUart0(&data);

        // Parse the input
        parseFields(&data);

        // Handle the "event" command
        if (isCommand(&data, "reset", 0))
        {

            putsUart0("System is rebooting...\r\n");
            // Reset the microcontroller
            NVIC_APINT_R = NVIC_APINT_SYSRESETREQ;
            waitMicrosecond(1000000);
            putsUart0("=>System is successfully rebooted.\r\n");

        }

        else if (isCommand(&data, "set", 2) && strcmp(getFieldString(&data, 1), "angle") == 0)
        {
            int angle = getFieldInteger(&data, 2);
            events[0].angle = angle;
            saveDeviceParameters(events, NUMBER_OF_EVENTS);
            putsUart0("Angle set successfully.\r\n");

        }

        else if (isCommand(&data, "angle", 1) && strcmp(getFieldString(&data, 1), "clear") == 0)
        {
            clearAngle(events, 0, NUMBER_OF_EVENTS);
            saveDeviceParameters(events, NUMBER_OF_EVENTS);
            putsUart0("Angle cleared successfully.\r\n");
        }


        else if (isCommand(&data, "rotate", 1) && strcmp(getFieldString(&data, 1), "cw") == 0)
        {
            CW=true;
            CCW=false;
            setPinValue(SLEEP_PIN, 1);
            turnDegrees(abs(events[0].angle));
            waitMicrosecond(3000);
        }
        else if (isCommand(&data, "rotate", 1) && strcmp(getFieldString(&data, 1), "ccw") == 0)
        {
            CW=false;
            CCW=true;
            setPinValue(SLEEP_PIN, 1);
            turnDegrees(abs(events[0].angle));
            waitMicrosecond(3000);
        }



        else if (isCommand(&data, "angle", 0))
        {

            displayEvents(events, NUMBER_OF_EVENTS);
        }


        else if (isCommand(&data, "tilt", 0))
        {
            char str1[100];
            snprintf(str1, sizeof(str1), "Tilt=%.1f\n",tilt);
            putsUart0(str1);
        }


        else if (isCommand(&data, "forward", 0))
        {
            setInitialZAxisAngle();
            calibrateGyro();
            FWD = true; // Start moving forward
            setPinValue(SLEEP_PIN, 1);
            OptCount0 = 0;
            OptCount1 = 0;

            // Somewhere in your code where FWD and BWD are set
            if (FWD == true) {
                MAX_TILT_ANGLE = 80.0;
            } else if (BWD == true) {
                MAX_TILT_ANGLE = 130.0;
            }

            while (FWD==true)
            {

                distanceTraveled = (OptCount0 + OptCount1) / 2.0 * distancePerTooth;


                setPinValue(SLEEP_PIN, 1);

                goStraightWithBalance();
                if(distanceTraveled>=targetDistance)
                {

                    FWD=false;
                    OptCount0 = 0;
                    OptCount1 = 0;

                }
            }
        }


        else if (isCommand(&data, "reverse", 0))
        {
            setInitialZAxisAngle();
            calibrateGyro();
            BWD = true; // Start moving forward
            setPinValue(SLEEP_PIN, 1);
            OptCount0 = 0;
            OptCount1 = 0;


            if (FWD == true) {
                MAX_TILT_ANGLE = 80.0;
            } else if (BWD == true) {
                MAX_TILT_ANGLE = 130.0;
            }

            while (BWD==true)
            {

                distanceTraveled = (OptCount0 + OptCount1) / 2.0 * distancePerTooth;


                setPinValue(SLEEP_PIN, 1);

                goStraightWithBalance();
                if(distanceTraveled>=targetDistance)
                {

                    BWD=false;
                    OptCount0 = 0;
                    OptCount1 = 0;

                }


            }
        }


        // Handle invalid commands
        else
        {
            putsUart0(data.buffer);
            putsUart0("-Invalid command.\r\n");
        }


    }
}



#include <Arduino.h>
#include <qei.h>
#include <sysctl.h>
#include <gpio.h>
#include <pin_map.h>
#include <pins_energia.h>
#include <interrupt.h>

//  Hardware Headers
#include <inc/hw_gpio.h>
#include <inc/hw_ints.h>

#define debug 0 //0 = final mode , 1 = debug mode

const float GEAR_RATIO = 1 ;
const int ENC_MIN = 0;
const int ENC_MAX = 0xFFFFFFFF;//If position reset enabled, specifies pulses after which position integrator resets.
const int QEI_PPR = 1024;//Pulse Per Revolution
const int QEI_PREDIVISOR = QEI_VELDIV_1;//Clock Predivisor Used
int QEI_PERIOD;

//Hex Mappings for Alternate Function Definitions( As in official sdk's driverlib/pin_map.h)
#define GPIO_PD6_PHA0           0x00031806
#define GPIO_PD7_PHB0           0x00031C06
#define GPIO_PC5_PHA1           0x00021406
#define GPIO_PC6_PHB1           0x00021806

//Constants
#define LED1 PE_1
#define LED2 PD_3
#define BLINK_DELAY_1 1e6
#define BLINK_DELAY_2 1e6

//Prototypes
//  Interrupt Callbacks when velocity timer expires
void QEI0_rpm(void);
void QEI1_rpm(void);

//Global vars
float rpm0 = 0.0;
float rpm1 = 0.0;
int vel0 = 0;
int vel1 = 0;
unsigned long curr_time = micros();
unsigned long prev_time = micros();
unsigned int led1_prev = micros();
unsigned int led2_prev = micros();

uint32_t seq = 1;
bool led1_mode= 0 ,led2_mode = 0; //0 = static, 1 = blinking
bool led1_state= 0 ,led2_state = 0; //0 = off, 1 = on
bool flagenc0 = 0, flagenc1 = 0;

void setup()
{
    extern void UART_Interrupt();
    //Setup Serial
    Serial.begin(921600);
    Serial.setTimeout(100);

    String handshake = "";
    int startTime = millis();

    while (1){
        Serial.println("Started");
        handshake = Serial.readString();

        if (handshake == "++\n"){
            Serial.println("Received Ack");
            break;
        }
        else if (millis() - startTime > 1000){
            SysCtlReset();
        }
    }

    //Enable QEI Peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);


    //Configure Alternate Functions of Pins
    //  Set Pins to be PHA0 and PHB0
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);
    //  Set Pins to be PHA1 and PHB1
    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);

    // Unlocking pin D7 by using the device LOCK key
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;//Apply the unlock key
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0b10000000;// Commit the pin to keep it in GPIO mode
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    //Setting Pin Types
    GPIOPinTypeQEI(GPIO_PORTD_BASE, (GPIO_PIN_6 | GPIO_PIN_7));//0b11000000
    GPIOPinTypeQEI(GPIO_PORTC_BASE, (GPIO_PIN_5 | GPIO_PIN_6));//0b01100000


    //Configuring QEI0
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP),ENC_MAX);
    QEIEnable(QEI0_BASE);
    QEIPositionSet(QEI0_BASE, 0);

    //Configuring QEI1
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP),ENC_MAX);
    QEIEnable(QEI1_BASE);
    QEIPositionSet(QEI1_BASE, 0);

    //Internal Pullup enable(For certain encoders)
    GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_5 | GPIO_PIN_6,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_6 | GPIO_PIN_7,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPU);


    //Set Number of clock ticks to find velocity over
    QEI_PERIOD = SysCtlClockGet()/100; //10ms time period

    //Velocity Capture Configure
    QEIVelocityConfigure(QEI0_BASE, QEI_PREDIVISOR, QEI_PERIOD );
    QEIVelocityConfigure(QEI1_BASE, QEI_PREDIVISOR, QEI_PERIOD );

    //Enable Velocity Capture
    QEIVelocityEnable(QEI0_BASE);
    QEIVelocityEnable(QEI1_BASE);

    //Interrupt Enable for QEI
    QEIIntEnable(QEI0_BASE,QEI_INTTIMER);
    QEIIntEnable(QEI1_BASE,QEI_INTTIMER);

    //Register the trigger functions
    QEIIntRegister(QEI1_BASE, &QEI1_rpm);
    QEIIntRegister(QEI0_BASE, &QEI0_rpm);

    //Enable Static Configuration of Interrupts
    IntEnable(INT_QEI0);
    IntEnable(INT_QEI1);

    //Enable Interrupts
    IntMasterEnable();


    pinMode(RED_LED,OUTPUT);
    pinMode(BLUE_LED,OUTPUT);
    pinMode(GREEN_LED,OUTPUT);

    Serial.setTimeout(1);
}

void loop()
{   
    if (Serial.available()){
        String handshake = Serial.readString();
        if (handshake == "--\n"){
            SysCtlReset();
        }
    }
    
    curr_time = micros();

    if(curr_time - prev_time >= 10000 )
    {
        prev_time = curr_time;
         if(debug){
                digitalWrite(GREEN_LED,1);
                Serial.print("sq: ");    //Sequence
                Serial.print(seq);    
                Serial.print("\tts: ");
                Serial.print(millis());
                Serial.print("\tr0: ");   //Rpm0
                Serial.print(rpm1);
                Serial.print("\tr1: ");   //Rpm1
                Serial.print(rpm0);
                Serial.print("\tp0: ");   //Pulses0
                Serial.print(vel1);
                Serial.print("\tp1: ");   //Pulses1
                Serial.print(vel0);
                Serial.print("\ttime: ");
                Serial.print(curr_time-prev_time);
                Serial.println();
                prev_time = curr_time;
            }
            else{
                Serial.print('$');
                Serial.print(vel1);
                Serial.print(' ');
                Serial.print(vel0);
                Serial.print(' ');
                Serial.print(rpm1);
                Serial.print(' ');
                Serial.print(rpm0);
                Serial.print(' ');
                Serial.print(seq % 1000);
                Serial.println('&');
            }
        seq++;
    }
  
    if (led1_mode){
        if (curr_time - led1_prev >= BLINK_DELAY_1)
        {
            digitalWrite(LED1,led1_state);
            digitalWrite(BLUE_LED,led1_state);
            led1_state = not led1_state;
            led1_prev = curr_time;
        }
    }
     }

void QEI0_rpm(void){
    QEIIntClear(QEI0_BASE, QEI_INTTIMER);
    vel0=QEIVelocityGet(QEI0_BASE)*QEIDirectionGet(QEI0_BASE);
    rpm0 = (1*vel0*60)*GEAR_RATIO/(QEI_PPR*4*10.0/1000); //number of pulses*60s/(PPR*number of edges(4)*TimePeriod)
    if (flagenc0 == 1)
    digitalWrite(BLUE_LED,1);
    else
    digitalWrite(BLUE_LED,0);
    flagenc0 = 1;
}

void QEI1_rpm(void)
{
    QEIIntClear(QEI1_BASE, QEI_INTTIMER);
    vel1 =QEIVelocityGet(QEI1_BASE)* QEIDirectionGet(QEI1_BASE);
    rpm1 = (1*vel1*60)*GEAR_RATIO/(QEI_PPR*4*10.0/1000) ;//number of pulses*60s/(PPR*number of edges(4)*TimePeriod)
    if (flagenc1 == 1)
    digitalWrite(RED_LED,1);
    else
    digitalWrite(RED_LED,0);
    flagenc1 = 1;}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <wiringPi.h>
#include <pigpio.h>
#include <wiringSerial.h>
#include <stdbool.h>
#include "gpio_pwm.h"


/* constants */
#define PWM0_PINNO         26
#define PWM1_PINNO         23
#define TRIG_PINNO         25
#define SIG1K_PINNO        21
#define SIG2K_PINNO        22

#define MAXPWM_FREQ        19200000

/***************************************************//**
*  \brief clockMain() - generate a master clcok using a PWM port
*  it is used to generate signals.
*  @param freq the frequency of the clock should be.
*  @param pinNo the GPIO pin Number in term wpi.
*  @param phase the mark:space ratio.
*              1 - 1/8 2 - 1/4 3 - 3:8 4 - 1/2
*              5 - 5/8 6 - 3/4 7 - 7/8
*******/
void clockMain(unsigned int freq, int pinNo, int phase)
{
    unsigned int result, lfreq = freq;
    int value, pwmC, pwmR = 16;

    result = freq*pwmR;
    result = MAXPWM_FREQ/result;

    pwmC = (int)result;

    value = phase*2;

    pinMode(pinNo, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(pwmC);
    pwmSetRange(pwmR);
    pwmWrite(pinNo,value);

    return;
}

unsigned int debugData[4][128] = {};
int debugIdx0 = 0, debugIdx1 = 0, debugIdx2 = 0, debugIdx3 = 0;

void isrPin26()
{
    static unsigned int clkCounter = 0;
    static unsigned int value0 = 0, value1 = 0;

    unsigned int curValue = 0, value2k = 0;

    clkCounter++;
    if(!(clkCounter%500))
    {
        curValue = micros();
        debugData[0][debugIdx0] = curValue;
        //value0 = value2k;
        debugIdx0++;
    }
    if(!(clkCounter%1000))
    {
        curValue = micros();
        debugData[1][debugIdx1] = curValue;
        //value1 = curValue;
        debugIdx1++;
    }
    if(debugIdx1 > 128)
    {
        for(;;);
    }
}

/***************************************************//**
*  \brief sineSig() - generate a sinewave signal.
*  it uses a LTB to generate vary PWM pulse and then 
*  pass through a low-pass filter.
*  @param freq the frequency of the sinewave signal.
*  @param pinNo the GPIO pin Number in term wpi. it is output the signal
**********/
void sineSig(int freq, int pinNo)
{
    /* N/A */
    return;
}

/***************************************************//**
*  \brief rectSig() - generate a rectangle signal within a seperate thread.
*  (it uses a ISR to drive a GPIO level to form a signal.
*  the interrupt generated with a input clock.)
*  @param sigNum the number of rectangle signal.
*  @param prectArray the GPIO pin Number array in term wpi. they are output the signal
**********/

struct rectSigpmts rectSigs[8];
bool rectEnd = true;

void rectSig(struct rectSigpmts *prectArray, int sigNum)
{
    /* for base timing clock 100KHz */
    int lfreq100k = 100000, delayValue = 5, pin100k = PWM1_PINNO;
    /* for 1KHz rectangle signal */
    int lfreq1k = 1000, phase1k = 0, pin1k = SIG1K_PINNO;
    /* for 2KHz rectangle signal */
    int lfreq2k = 2000, phase2k = 2, pin2k = SIG2K_PINNO;
    
    /* locol variabls */
    bool isHi100k = false, isHi1k = false, isHi2k = false;
    unsigned int count100k, changeLevel1k, start1k;
    unsigned int changeLevel2k, start2k;

    /* set locol variables */
    count100k = 0;
    changeLevel1k = 1000000/lfreq1k/10; //500
    changeLevel2k = 1000000/lfreq2k/10; //250


    /* set sigle output GPIO pins */
    pinMode(pin100k, OUTPUT);digitalWrite(pin100k, HIGH);
    isHi100k = true;

    /* for rectangle phase shift */
    if(phase1k > 4)
    {
        phase1k -= 4;
        pinMode(pin1k, OUTPUT);digitalWrite(pin1k, LOW);
        isHi100k = false;
    }else{
        pinMode(pin1k, OUTPUT);digitalWrite(pin2k, HIGH);
        isHi1k = true;
    }
    if(phase2k > 4)
    {
        phase2k -= 4;
        pinMode(pin2k, OUTPUT);digitalWrite(pin2k, LOW);
        isHi2k = false;
    }else{
        pinMode(pin2k, OUTPUT);digitalWrite(pin2k, HIGH);
        isHi2k = true;
    }
    start1k = (changeLevel1k/4)*phase1k;
    start2k = (changeLevel2k/4)*phase2k; 


    /* re set the signal paramter array */
    //no implement

    /* simplify to generate two rectangle signals 1KHz and 2KHz with 90 degree shifting */
    digitalWrite(pin100k, HIGH);isHi100k = true;
    //value = micros(); count100k += value; count1k += value; count2k += value;
    
    while(!rectEnd) //generate signals loop
    {
        delayMicroseconds(delayValue);
        count100k++;
        /* output 100k clock duty 50% */
        if(isHi100k)
        {
            digitalWrite(pin100k, LOW);
            isHi100k = false;
        }
        else{
            digitalWrite(pin100k, HIGH);
            isHi100k = true;
        }

        /* output 1k rectangle signal duty 50% */
        //value = micros();
        if(!((count100k + start1k)%changeLevel1k))
        {
            if(isHi1k)
            {
                digitalWrite(pin1k, LOW);
                isHi1k = false;
            }
            else{
                digitalWrite(pin1k, HIGH);
                isHi1k = true;
            }
            //count1k = changeLevel1k;
        }

        /* output 2k rectangle signal duty 50% */
        if(!((count100k + start2k)%changeLevel2k))
        {
            if(isHi2k)
            {
                digitalWrite(pin2k, LOW);
                isHi2k = false;
            }
            else{
                digitalWrite(pin2k, HIGH);
                isHi2k = true;
            }
            //count2k = changeLevel2k;
        }
        //count100k++;
    }
}

/***************************************************//**
*  \brief signalISR() - signal generating handler.
*  it is a ISR triggered by the clock input in a GPIO pin.
*  the GPIO pin is set to a input port and the input clock's fall edge
*  triggers a interrupt.
*  @param counter the interrupt counter.
*  @param duty the duty of the rectangle signal.
*  @param pinNo the GPIO pin Number in term wpi. it is output the signal
**********/
void signalISR()
{
    //static unsigned int clkCounter = 0;
    //static unsigned int micr1k, micr2k;
    static bool isHi1k = true, isHi2k = false;
    //static int dltmicr1k, dltmicr2k;
    //static int cont = 0;

    //unsigned int micNum;
    //int lodlt, lodlt1;
    //count100k, changeLevel1k, start1k;
    //unsigned int changeLevel2k, start2k;
    //piHiPri(90);
    //clkCounter++;
    //micNum = micros();
//    dltmicr1k = micNum - micr1k;
    //lodlt = micNum - micr2k;
    //lodlt1 = micNum - micr1k;
#if 1
    /* low clock create output signal */
    if(isHi2k)
    {
        digitalWrite(SIG2K_PINNO, LOW);
        isHi2k = false;
    }
    else{
        digitalWrite(SIG2K_PINNO, HIGH);
        isHi2k = true;
    }

#else 
    if((lodlt > 500) /*|| (cont >= 100)*/)
    {
        cont++;
        if(cont >= 25)
        {
            cont = 0;
        }
    }
    
    //if(!(clkCounter%25)) //2KHz
    if(lodlt > (dltmicr2k)) //2KHz
    {
        if(isHi2k)
        {
            digitalWrite(SIG2K_PINNO, LOW);
            isHi2k = false;
        }
        else{
            digitalWrite(SIG2K_PINNO, HIGH);
            isHi2k = true;
        }
        dltmicr2k = 500 - lodlt;
        if(dltmicr2k > 350)
        {
            dltmicr2k = 250;
        }
        micr2k = micNum;
        clkCounter = 0;
    }
    //if(!(clkCounter%50))//1KHz
#if 0
    if(lodlt1 > (500 + dltmicr1k)) //1KHz
    {
        if(isHi1k)
        {
            digitalWrite(SIG1K_PINNO, LOW);
            isHi1k = false;
        }
        else{
            digitalWrite(SIG1K_PINNO, HIGH);
            isHi1k = true;
        }
        micr1k = lodlt1 - 400;
        if(dltmicr1k > 100)
        {
            dltmicr1k = 50;
        }
        micr1k = micNum;

    }
#endif
#endif
    return;
}

int main()
{
    struct rectSigpmts *prectArray = NULL;
    int sigNum = 0;
    int lfreq100k = 100000, pwm100kpin = PWM1_PINNO, interPin = PWM0_PINNO;
    if(wiringPiSetup() < 0)return 1;
    
    //piHiPri(5);
    rectEnd = false;
#if 0 /* method 1 of the generation */
    rectSig(prectArray, sigNum);
#else /* method 2 of the generation */
    /* set output pin & signal parameters */
        /* for 1KHz rectangle signal */
    int lfreq1k = 1000, phase1k = 0, pin1k = SIG1K_PINNO;
    /* for 2KHz rectangle signal */
    int lfreq2k = 2000, phase2k = 2, pin2k = SIG2K_PINNO;

    /* for start the rectangle signals */
    pinMode(pin1k, OUTPUT);digitalWrite(pin1k, LOW);

    pinMode(pin2k, OUTPUT);digitalWrite(pin2k, HIGH);
/*
    if(phase1k > 4)
    {
        phase1k -= 4;
        pinMode(pin1k, OUTPUT);digitalWrite(pin1k, LOW);
        isHi100k = false;
    }else{
        pinMode(pin1k, OUTPUT);digitalWrite(pin2k, HIGH);
        isHi1k = true;
    }
    if(phase2k > 4)
    {
        phase2k -= 4;
        pinMode(pin2k, OUTPUT);digitalWrite(pin2k, LOW);
        isHi2k = false;
    }else{
        pinMode(pin2k, OUTPUT);digitalWrite(pin2k, HIGH);
        isHi2k = true;
    }
*/
    //start1k = (changeLevel1k/4)*phase1k;
    //start2k = (changeLevel2k/4)*phase2k; 

    /* set pin 23 as PWM output as 100KHz with 50% duty */
    int n = 10 * 2; // set phase shift, 20*m, m = [0 - 7]
    pinMode(pwm100kpin, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(60);
    pwmSetRange(80);
    pwmWrite(pwm100kpin, n);

    /* set pin 26 as input & rising edge interrupt */
    pinMode(interPin, INPUT); 
    pullUpDnControl(interPin, PUD_UP);
    //wiringPiISR(interPin,INT_EDGE_RISING, isrPin26); //test jitter
    wiringPiISR(interPin,INT_EDGE_FALLING, signalISR);//signal generator

    //while(1)
    {
        delay(60000);
    }

#endif

    return 0;
}


/* the GPIO Signal header file is used to generate signals output from GPIO */

struct rectSigpmts
{
    int pinNo;
    int freq;
    int phase;
};

/****************************************************//*
* Signal generation functions - the functions to generating vary signals
*  clockMain() - generate a master clcok
*  sineSig()   - generate a sine signal
*  rectSig()   - generate a ractangle signal
*  signalISR() - interrup handler to generate signal
********************/
void clockMain(unsigned int freq, int pinNo, int phase);
void sineSig(int freq, int pinNo);
void rectSig(struct rectSigpmts *prectArray, int sigNum);
void signalISR();
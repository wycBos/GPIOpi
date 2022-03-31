/**************************************
 * the pigpio application to generate two wave
 * code protype comes from pigpio example
 * *****************/

#include <stdio.h>
#include <wiringPi.h>
#include <pigpio.h>

#if 1
int gpios[]={5,6,13,12,};
gpioPulse_t pulses[]=
{
   {0x1020, 0x2040, 125}, 
   {0x3020, 0x0040, 125}, 
   {0x2060, 0x1000, 125}, 
   {0x0060, 0x3000, 125}, 
   {0x1040, 0x2020, 125}, 
   {0x3040, 0x0020, 125}, 
   {0x2000, 0x1060, 125}, 
   {0x0000, 0x3060, 125}, 
};
#else
int gpios[] = {5};
int gpios6[] = {6};
gpioPulse_t pulses[]=
{
   {0x20, 0x80, 50},
//   {0x70, 0x110, 50},
   {0x80, 0x20, 50},
//   {0x110, 0x70, 50},

};
gpioPulse_t pulses6[]=
{
   {0x40, 0x80, 50},
//   {0x70, 0x110, 50},
   {0x80, 0x40, 50},
//   {0x110, 0x70, 50},

};
#endif
int main(int argc, char *argv[])
{
   int g, wid=-1, wid6 = -1;
 
   if (gpioInitialise() < 0) return 1;

   for (g=0; g<sizeof(gpios)/sizeof(gpios[0]); g++)
      gpioSetMode(gpios[g], PI_OUTPUT);

   gpioWaveClear();
   gpioWaveAddGeneric(sizeof(pulses)/sizeof(pulses[0]), pulses);
   wid = gpioWaveCreate();

   if (wid >= 0)
   {
      gpioWaveTxSend(wid, PI_WAVE_MODE_REPEAT);
      time_sleep(90);
      gpioWaveTxStop();
      gpioWaveDelete(wid);
   }

   gpioTerminate();
}

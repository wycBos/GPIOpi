/**************************************
 * the pigpio application to generate two wave
 * code protype comes from pigpio example
 * *****************/


#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <pigpio.h>

#define ADS1X15_EXE

#ifdef ADS1X15_EXE
   #include "ADS1x15.h"
#endif

#ifndef ADS1X15_EXE

#if 1
int gpios[]={5,6,13,12,};
gpioPulse_t pulses[]=
{
/* the rectangule test. two 2KHz and two 1KHz included *//*
   {0x1020, 0x2040, 125}, 
   {0x3020, 0x0040, 125}, 
   {0x2060, 0x1000, 125}, 
   {0x0060, 0x3000, 125}, 
   {0x1040, 0x2020, 125}, 
   {0x3040, 0x0020, 125}, 
   {0x2000, 0x1060, 125}, 
   {0x0000, 0x3060, 125},
*/
   {0x1060, 0x0000, 5}, //1
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //2
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //3
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //4
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //5
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x1000, 5}, //6
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //7
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //8
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //9
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //10
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x1020, 0x0040, 5}, //11
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //12
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //13
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //14
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //15
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x1000, 5}, //16
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //17
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //18
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //19
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

   {0x0020, 0x0000, 5}, //20
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5}, 
   {0x0020, 0x0000, 5}, 
   {0x0000, 0x0020, 5},  

};
#else //pwm
int gpios[] = {5,6};
gpioPulse_t pulses[]=
{
   {0x60, 0x00, 20},
   {0x40, 0x20, 20},
   {0x60, 0x00, 24},
   {0x40, 0x20, 16},
   {0x60, 0x0, 29},
   {0x40, 0x20, 11},
   {0x60, 0x0, 33},
   {0x40, 0x20, 7},
   {0x60, 0x0, 36},
   {0x40, 0x20, 4},

   {0x20, 0x40, 39},
   {0x0, 0x60, 1},
   {0x20, 0x40, 39},
   {0x0, 0x60, 1},
   {0x20, 0x40, 39},
   {0x0, 0x60, 1},
   {0x20, 0x40, 38},
   {0x0, 0x60, 2},
   {0x20, 0x40, 35},
   {0x0, 0x60, 5},

   {0x20, 0x40, 31},
   {0x0, 0x60, 9},
   {0x20, 0x40, 27},
   {0x0, 0x60, 13},
   {0x20, 0x40, 22},
   {0x0, 0x60, 18},
   {0x20, 0x40, 17},
   {0x0, 0x60, 23},
   {0x20, 0x40, 12},
   {0x0, 0x60, 28},

   {0x20, 0x40, 8},
   {0x0, 0x60, 32},
   {0x20, 0x40, 4},
   {0x0, 0x60, 36},
   {0x20, 0x40, 1},
   {0x0, 0x60, 39},
   {0x20, 0x40, 0},
   {0x0, 0x60, 40},
   {0x20, 0x40, 0},
   {0x0, 0x60, 40},

   {0x20, 0x40, 0},
   {0x0, 0x60, 40},
   {0x20, 0x40, 3},
   {0x0, 0x60, 37},
   {0x20, 0x40, 6},
   {0x0, 0x60, 34},
   {0x20, 0x40, 10},
   {0x0, 0x60, 30},
   {0x20, 0x40, 15},
   {0x0, 0x60, 25},      
};
#endif
/* message for testing serial port */
char contimeas[4]   ={0x80,0x06,0x03,0x77};

int main(int argc, char *argv[])
{
   int g, fd, wid=-1;
 
   if(wiringPiSetup() < 0)return 1;
    if((fd = serialOpen("/dev/serial0",9600)) < 0)return 1;
   
   printf("serial test start ...\n");
/*
   for(int n = 0; n < 4; n++)
   {
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      time_sleep(2);
   }
*/
   if (gpioInitialise() < 0) return 1;

   for (g=0; g<sizeof(gpios)/sizeof(gpios[0]); g++)
      gpioSetMode(gpios[g], PI_OUTPUT);

   gpioWaveClear();
   gpioWaveAddGeneric(sizeof(pulses)/sizeof(pulses[0]), pulses);
   wid = gpioWaveCreate();
/*
   for(int n = 0; n < 4; n++)
   {
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      time_sleep(2);
   }
*/
   if (wid >= 0)
   {
      gpioWaveTxSend(wid, PI_WAVE_MODE_REPEAT);
      time_sleep(100);
/*      
   for(int n = 0; n < 14; n++)
   {
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      time_sleep(2);
   }
*/
      gpioWaveTxStop();
      gpioWaveDelete(wid);
   }
/*
   for(int n = 0; n < 4; n++)
   {
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      time_sleep(2);
   }
*/
   gpioTerminate();
/*   
   for(int n = 0; n < 4; n++)
   {
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      serialPrintf(fd,contimeas);
      time_sleep(2);
   }
*/
}

#else //ADS1X15_EXE
/* basic routines */
//int _read_config(ads1x15_p s);
//int _write_config(ads1x15_p s);
//int _write_comparator_thresholds(ads1x15_p s, int high, int low);
//int _update_comparators(ads1x15_p s);
//int _update_config(ads1x15_p s);

/* routines for I2C communications */
int _read_config(ads1x15_p s)
{
   unsigned char buf[8];

   buf[0] = CONFIG_REG;

   //lgI2cWriteDevice(s->i2ch, buf, 1);  // set config register
   i2cWriteDevice(s->i2ch, buf, 1);

   //lgI2cReadDevice(s->i2ch, buf, 2);
   i2cReadDevice(s->i2ch, buf, 2);

   s->configH = buf[0];
   s->configL = buf[1];

   buf[0] = COMPARE_LOW_REG;

   //lgI2cWriteDevice(s->i2ch, buf, 1); // set low compare register
   i2cWriteDevice(s->i2ch, buf, 1);

   //lgI2cReadDevice(s->i2ch, buf, 2);
   i2cReadDevice(s->i2ch, buf, 2);

   s->compare_low = (buf[0] << 8) | buf[1];

   buf[0] = COMPARE_HIGH_REG;

   //lgI2cWriteDevice(s->i2ch, buf, 1); // set high compare register
   i2cWriteDevice(s->i2ch, buf, 1);

   //lgI2cReadDevice(s->i2ch, buf, 2);
   i2cReadDevice(s->i2ch, buf, 2);

   s->compare_high = (buf[0] << 8) | buf[1];

   buf[0] = CONVERSION_REG;

   //lgI2cWriteDevice(s->i2ch, buf, 1); // set conversion register
   i2cWriteDevice(s->i2ch, buf, 1);

   s->channel = (s->configH >> 4) & 7;
   s->gain = (s->configH >> 1) & 7;
   s->voltage_range = _GAIN[s->gain];
   s->single_shot = s->configH & 1;
   s->sps = (s->configL >> 5) & 7;
   s->comparator_mode = (s->configL >> 4) & 1;
   s->comparator_polarity = (s->configL >> 3) & 1;
   s->comparator_latch = (s->configL >> 2) & 1;
   s->comparator_queue = s->configL & 3;

   if (s->comparator_queue != 3)
      s->set_queue = s->comparator_queue;
   else
      s->set_queue = 0;

   return 0;
}

int _write_config(ads1x15_p s)
{
   unsigned char buf[8];

   buf[0] = CONFIG_REG;
   buf[1] = s->configH;
   buf[2] = s->configL;

   //lgI2cWriteDevice(s->i2ch, buf, 3);
   i2cWriteDevice(s->i2ch, buf, 3);

   buf[0] = CONVERSION_REG;

   //lgI2cWriteDevice(s->i2ch, buf, 1);
   i2cWriteDevice(s->i2ch, buf, 1);

   return 0;
}

int _write_comparator_thresholds(ads1x15_p s, int high, int low)
{
   unsigned char buf[8];

   if (high > 32767) high = 32767;
   else if (high < -32768) high = -32768;

   if (low > 32767) low = 32767;
   else if (low < -32768) low = -32768;

   s->compare_high = high;
   s->compare_low = low;

   buf[0] = COMPARE_LOW_REG;
   buf[1] = (low >> 8) & 0xff;
   buf[2] = low & 0xff;

   //lgI2cWriteDevice(s->i2ch, buf, 3);
   i2cWriteDevice(s->i2ch, buf, 3);

   buf[0] = COMPARE_HIGH_REG;
   buf[1] = (high >> 8) & 0xff;
   buf[2] = high & 0xff;

   //lgI2cWriteDevice(s->i2ch, buf, 3);
   i2cWriteDevice(s->i2ch, buf, 3);

   buf[0] = CONVERSION_REG;

   //lgI2cWriteDevice(s->i2ch, buf, 1);
   i2cWriteDevice(s->i2ch, buf, 1);

   return 0;
}

int _update_comparators(ads1x15_p s)
{
   int h, l;

   if (s->alert_rdy >= ADS1X15_ALERT_TRADITIONAL)
   {
      h = s->vhigh * 32768.0 / s->voltage_range;
      l = s->vlow * 32768.0 / s->voltage_range;

      return _write_comparator_thresholds(s, h, l);
   }

   return 0;
}

int _update_config(ads1x15_p s)
{
   int H, L;

   H = s->configH;
   L = s->configL;

   s->configH = ((1 << 7) | (s->channel << 4) |
      (s->gain << 1) | s->single_shot);

   s->configL = ((s->sps << 5) | (s->comparator_mode << 4) |
      (s->comparator_polarity << 3) | (s->comparator_latch << 2) |
      s->comparator_queue);

   if ((H != s->configH) || (L != s->configL)) _write_config(s);

   return 0;
}

/* routines for ADS1x15 opertions */
int ADS1X15_set_channel(ads1x15_p s, int channel)
{
   if (channel < 0) channel = 0;
   else if (channel > 7) channel = 7;

   s->channel = channel;

   _update_config(s);

   return channel;
}

float ADS1X15_set_voltage_range(ads1x15_p s, float vrange)
{
   int val, i;

   val = 7;

   for (i=0; i<8; i++)
   {
      if (vrange > _GAIN[i])
      {
         val = i;
         break;
      }
   }

   if (val > 0) val = val - 1;

   s->gain = val;

   s->voltage_range = _GAIN[val];

   _update_comparators(s);

   _update_config(s);

   return s->voltage_range;
}

int ADS1X15_set_sample_rate(ads1x15_p s, int rate)
{
   int val, i;

   val = 7;

   for (i=0; i<8; i++)
   {
      if (rate <= s->SPS[i])
      {
         val = i;
         break;
      }
   }

   s->sps = val;
   _update_config(s);

   return s->SPS[val];
}

ads1x15_p ADS1X15_open(int sbc, int bus, int device, int flags)
{
   ads1x15_p s;

   //s = calloc(1, sizeof(ads1x15_t));
   s = calloc(1, sizeof(ads1x15_t));

   if (s == NULL) return NULL;

   s->sbc = sbc;         // sbc connection
   s->bus = bus;         // I2C bus
   s->device = device;   // I2C device
   s->flags = flags;     // I2C flags

   s->SPS = _SPS_1115;   // default

   //s->i2ch = lgI2cOpen(bus, device, flags);
   s->i2ch = i2cOpen(bus, device, flags);

   if (s->i2ch < 0)
   {
      free(s);
      return NULL;
   }

   _read_config(s);

   //ADS1X15_alert_never(s); // switch off ALERT/RDY pin.

   return s;
}

int ADS1X15_read(ads1x15_p s)
{
   unsigned char buf[8];

   if (s->single_shot) _write_config(s);
  
   //lgI2cReadDevice(s->i2ch, buf, 2);
   i2cReadDevice(s->i2ch, buf, 2);

   return (buf[0]<<8) + buf[1];
}

float ADS1X15_read_voltage(ads1x15_p s)
{
   return ADS1X15_read(s) * s->voltage_range / 32768.0;
}

ads1x15_p ADS1115_open(int sbc, int bus, int device, int flags)
{
   ads1x15_p s;

   s = ADS1X15_open(sbc, bus, device, flags);

   if (s) s->SPS = _SPS_1115;

   return s;
}

ads1x15_p ADS1X15_close(ads1x15_p s)
{
   if (s != NULL)
   {
      //lgI2cClose(s->i2ch);
      i2cClose(s->i2ch);
      free(s);
      s = NULL;
   }
   return s;
}

int main(int argc, char *argv[])
{
   int h;
   int err;
   int cb_id;
   ads1x15_p adc=NULL;
   //double end_time;
   int end_time, seconds;
   int micros;

   int g, fd, wid=-1;
 
   if(wiringPiSetup() < 0)return 1;
    if((fd = serialOpen("/dev/serial0",9600)) < 0)return 1;
   
   printf("serial test start ...\n");

   if (gpioInitialise() < 0) return 1;

   printf("pigpio initialized. \n");


   adc = ADS1115_open(0, 1, 0x48, 0);

   if (adc == NULL) return -2;

   printf("ADS1115 start. \n");

   ADS1X15_set_channel(adc, ADS1X15_A0);
   ADS1X15_set_voltage_range(adc, 3.3);
   ADS1X15_set_sample_rate(adc, 0); // set minimum sampling rate
   //ADS1X15_alert_when_high_or_low(adc, 3, 1); // alert outside these voltages

   if (0 /*(ALERT >= 0*/) /* ALERT pin connected */
   {
      //h = lgGpiochipOpen(0);

      //if (h <0) return -3;

      /* got a handle, now open the GPIO for alerts */
      //err = lgGpioClaimAlert(h, 0, LG_BOTH_EDGES, ALERT, -1);
      //if (err < 0) return -4;
      //lgGpioSetAlertsFunc(h, ALERT, cbf, adc);
   }

   //end_time = lguTime() + 120;
   gpioTime(1, &seconds, &micros);
   end_time = seconds + 20;

   while (seconds < end_time)
   {
      //lguSleep(0.2);
      gpioSleep(0, 1, 0);
      gpioTime(1, &seconds, &micros);

      printf("%.2f\n", ADS1X15_read_voltage(adc));
   }

   ADS1X15_close(adc);

   //if (ALERT >= 0) /* ALERT pin connected */
   //{
   //   lgGpioSetAlertsFunc(h, ALERT, NULL, NULL);
   //   lgGpioFree(h, ALERT);
   //   lgGpiochipClose(h);
   //}

   return 0;
}
#endif //ADS1X15_EXE

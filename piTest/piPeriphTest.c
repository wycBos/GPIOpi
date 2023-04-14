/*
gcc -Wall -pthread -o piPeriphx piPeriphTest.c ... -lpigpio
sudo ./piPeriphx

*** WARNING ************************************************
*                                                          *
* All the tests make extensive use of gpio 25 (pin 22).    *
* Ensure that either nothing or just a LED is connected to *
* gpio 25 before running any of the tests.                 *
*                                                          *
* Some tests are statistical in nature and so may on       *
* occasion fail.  Repeated failures on the same test or    *
* many failures in a group of tests indicate a problem.    *
************************************************************
*                                                          *
* The file is modified for testing the Handhelder SPI, I2C,*
* and UART.                                                *
* It supports MAX11612 (I2C), MCP4822 (SPI_DAC), ADS131M04 *
* (SPI_ADC), MTC415T (UART), LASER_DISTANCE (UART)...      *
*                                                          *
* Using the ta() (searial/UART), tb() (I2C), and tc() (SPI)*
************************************************************
*
*/

#include <stdio.h>
#include <ncurses.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "pigpio.h"
#include "waveFormNew.h"
#include "piSerial.h"

#define USERDATA 18249013

//#define GPIO 21 //25

void CHECK(int t, int st, int got, int expect, int pc, char *desc)
{
   if ((got >= (((1E2 - pc) * expect) / 1E2)) && (got <= (((1E2 + pc) * expect) / 1E2)))
   {
      printf("TEST %2d.%-2d PASS (%s: %d)\n", t, st, desc, expect);
   }
   else
   {
      fprintf(stderr,
              "TEST %2d.%-2d FAILED got %d (%s: %d)\n",
              t, st, got, desc, expect);
   }
}

#if 0 // the other pigpio test routines which is not used now.
void t0() // get pigpio version, hardware version
{
   printf("\nTesting pigpio C I/F\n");

   printf("pigpio version %d.\n", gpioVersion());

   printf("Hardware revision %d.\n", gpioHardwareRevision());
}

void t1() // GPIO Pin operating, e.g. GPIO_25
{
   int v;

   printf("Mode/PUD/read/write tests.\n");

   gpioSetMode(GPIO, PI_INPUT);
   v = gpioGetMode(GPIO);
   CHECK(1, 1, v, 0, 0, "set mode, get mode");

   gpioSetPullUpDown(GPIO, PI_PUD_UP);
   gpioDelay(1); /* 1 micro delay to let GPIO reach level reliably */
   v = gpioRead(GPIO);
   CHECK(1, 2, v, 1, 0, "set pull up down, read");

   gpioSetPullUpDown(GPIO, PI_PUD_DOWN);
   gpioDelay(1); /* 1 micro delay to let GPIO reach level reliably */
   v = gpioRead(GPIO);
   CHECK(1, 3, v, 0, 0, "set pull up down, read");

   gpioWrite(GPIO, PI_LOW);
   v = gpioGetMode(GPIO);
   CHECK(1, 4, v, 1, 0, "write, get mode");

   v = gpioRead(GPIO);
   CHECK(1, 5, v, 0, 0, "read");

   gpioWrite(GPIO, PI_HIGH);
   gpioDelay(1); /* 1 micro delay to let GPIO reach level reliably */
   v = gpioRead(GPIO);
   CHECK(1, 6, v, 1, 0, "write, read");
}

int t2_count;

void t2cb(int gpio, int level, uint32_t tick)
{
   t2_count++;
}

void t2() // PWM setting test, e.g GPIO_25
{
   int dc, f, r, rr, oc;

   printf("PWM dutycycle/range/frequency tests.\n");

   gpioSetPWMrange(GPIO, 255);
   gpioSetPWMfrequency(GPIO, 0);
   f = gpioGetPWMfrequency(GPIO);
   CHECK(2, 1, f, 10, 0, "set PWM range, set/get PWM frequency");

   t2_count = 0;

   gpioSetAlertFunc(GPIO, t2cb);

   gpioPWM(GPIO, 0);
   dc = gpioGetPWMdutycycle(GPIO);
   CHECK(2, 2, dc, 0, 0, "get PWM dutycycle");

   time_sleep(0.5); /* allow old notifications to flush */
   oc = t2_count;
   time_sleep(2);
   f = t2_count - oc;
   CHECK(2, 3, f, 0, 0, "set PWM dutycycle, callback");

   gpioPWM(GPIO, 128);
   dc = gpioGetPWMdutycycle(GPIO);
   CHECK(2, 4, dc, 128, 0, "get PWM dutycycle");

   oc = t2_count;
   time_sleep(2);
   f = t2_count - oc;
   CHECK(2, 5, f, 40, 5, "set PWM dutycycle, callback");

   gpioSetPWMfrequency(GPIO, 100);
   f = gpioGetPWMfrequency(GPIO);
   CHECK(2, 6, f, 100, 0, "set/get PWM frequency");

   oc = t2_count;
   time_sleep(2);
   f = t2_count - oc;
   CHECK(2, 7, f, 400, 1, "callback");

   gpioSetPWMfrequency(GPIO, 1000);
   f = gpioGetPWMfrequency(GPIO);
   CHECK(2, 8, f, 1000, 0, "set/get PWM frequency");

   oc = t2_count;
   time_sleep(2);
   f = t2_count - oc;
   CHECK(2, 9, f, 4000, 1, "callback");

   r = gpioGetPWMrange(GPIO);
   CHECK(2, 10, r, 255, 0, "get PWM range");

   rr = gpioGetPWMrealRange(GPIO);
   CHECK(2, 11, rr, 200, 0, "get PWM real range");

   gpioSetPWMrange(GPIO, 2000);
   r = gpioGetPWMrange(GPIO);
   CHECK(2, 12, r, 2000, 0, "set/get PWM range");

   rr = gpioGetPWMrealRange(GPIO);
   CHECK(2, 13, rr, 200, 0, "get PWM real range");

   gpioPWM(GPIO, 0);
}

int t3_val;
int t3_reset;
int t3_count;
uint32_t t3_tick;
float t3_on;
float t3_off;

void t3cbf(int gpio, int level, uint32_t tick, void *userdata)
{
   static int unreported = 1;

   uint32_t td;
   int *val;

   val = userdata;

   if (*val != USERDATA)
   {
      if (unreported)
      {
         fprintf(
             stderr,
             "unexpected userdata %d (expected %d)\n",
             *val, USERDATA);
      }
      unreported = 0;
   }

   if (t3_reset)
   {
      t3_count = 0;
      t3_on = 0.0;
      t3_off = 0.0;
      t3_reset = 0;
   }
   else
   {
      td = tick - t3_tick;

      if (level == 0)
         t3_on += td;
      else
         t3_off += td;
   }

   t3_count++;
   t3_tick = tick;
}

void t3() // PWM pulse accuracy test
{
   int f, rr;

   float on, off;

   int t, v;

   int pw[3] = {500, 1500, 2500};
   int dc[4] = {20, 40, 60, 80};

   printf("PWM/Servo pulse accuracy tests.\n");

   t3_val = USERDATA;
   t3_reset = 1;
   t3_count = 0;
   t3_tick = 0;
   t3_on = 0.0;
   t3_off = 0.0;

   gpioSetAlertFuncEx(GPIO, t3cbf, &t3_val); /* test extended alert */

   for (t = 0; t < 3; t++)
   {
      gpioServo(GPIO, pw[t]);
      v = gpioGetServoPulsewidth(GPIO);
      CHECK(3, t + t + 1, v, pw[t], 0, "get servo pulsewidth");

      time_sleep(1);
      t3_reset = 1;
      time_sleep(4);
      on = t3_on;
      off = t3_off;
      CHECK(3, t + t + 2, (1E3 * (on + off)) / on, 2E7 / pw[t], 1,
            "set servo pulsewidth");
   }

   gpioServo(GPIO, 0);
   gpioSetPWMfrequency(GPIO, 1000);
   f = gpioGetPWMfrequency(GPIO);
   CHECK(3, 7, f, 1000, 0, "set/get PWM frequency");

   rr = gpioSetPWMrange(GPIO, 100);
   CHECK(3, 8, rr, 200, 0, "set PWM range");

   for (t = 0; t < 4; t++)
   {
      gpioPWM(GPIO, dc[t]);
      v = gpioGetPWMdutycycle(GPIO);
      CHECK(3, t + t + 9, v, dc[t], 0, "get PWM dutycycle");

      time_sleep(1);
      t3_reset = 1;
      time_sleep(2);
      on = t3_on;
      off = t3_off;
      CHECK(3, t + t + 10, (1E3 * on) / (on + off), 1E1 * dc[t], 1,
            "set PWM dutycycle");
   }

   gpioPWM(GPIO, 0);
}

void t4() // pipe notification test (???)
{
   int h, e, f, n, s, b, l, seq_ok, toggle_ok;
   gpioReport_t r;
   char p[32];

   printf("Pipe notification tests.\n");

   gpioSetPWMfrequency(GPIO, 0);
   gpioPWM(GPIO, 0);
   gpioSetPWMrange(GPIO, 100);

   h = gpioNotifyOpen();

   sprintf(p, "/dev/pigpio%d", h);
   f = open(p, O_RDONLY);

   e = gpioNotifyBegin(h, (1 << GPIO));
   CHECK(4, 1, e, 0, 0, "notify open/begin");

   gpioPWM(GPIO, 50);
   time_sleep(4);
   gpioPWM(GPIO, 0);

   e = gpioNotifyPause(h);
   CHECK(4, 2, e, 0, 0, "notify pause");

   e = gpioNotifyClose(h);
   CHECK(4, 3, e, 0, 0, "notify close");

   n = 0;
   s = 0;
   l = 0;
   seq_ok = 1;
   toggle_ok = 1;

   while (1)
   {
      b = read(f, &r, 12);
      if (b == 12)
      {
         if (s != r.seqno)
            seq_ok = 0;

         if (n)
            if (l != (r.level & (1 << GPIO)))
               toggle_ok = 0;

         if (r.level & (1 << GPIO))
            l = 0;
         else
            l = (1 << GPIO);

         s++;
         n++;

         // printf("%d %d %d %X\n", r.seqno, r.flags, r.tick, r.level);
      }
      else
         break;
   }

   close(f);

   CHECK(4, 4, seq_ok, 1, 0, "sequence numbers ok");

   CHECK(4, 5, toggle_ok, 1, 0, "gpio toggled ok");

   CHECK(4, 6, n, 80, 10, "number of notifications");
}

int t5_count;

void t5cbf(int gpio, int level, uint32_t tick)
{
   if (level == 0)
      t5_count++; /* falling edges */
}

void t5() // waveform test & serial R/W test
{
   int BAUD = 4800;

   char *TEXT =
       "\n\
Now is the winter of our discontent\n\
Made glorious summer by this sun of York;\n\
And all the clouds that lour'd upon our house\n\
In the deep bosom of the ocean buried.\n\
Now are our brows bound with victorious wreaths;\n\
Our bruised arms hung up for monuments;\n\
Our stern alarums changed to merry meetings,\n\
Our dreadful marches to delightful measures.\n\
Grim-visaged war hath smooth'd his wrinkled front;\n\
And now, instead of mounting barded steeds\n\
To fright the souls of fearful adversaries,\n\
He capers nimbly in a lady's chamber\n\
To the lascivious pleasing of a lute.\n\
";

   gpioPulse_t wf[] =
       {
           {1 << GPIO, 0, 10000},
           {0, 1 << GPIO, 30000},
           {1 << GPIO, 0, 60000},
           {0, 1 << GPIO, 100000},
       };

   int e, oc, c, wid;

   char text[2048];

   printf("Waveforms & serial read/write tests.\n");

   t5_count = 0;

   gpioSetAlertFunc(GPIO, t5cbf);

   gpioSetMode(GPIO, PI_OUTPUT);

   e = gpioWaveClear();
   CHECK(5, 1, e, 0, 0, "callback, set mode, wave clear");

   e = gpioWaveAddGeneric(4, wf);
   CHECK(5, 2, e, 4, 0, "pulse, wave add generic");

   wid = gpioWaveCreate();
   e = gpioWaveTxSend(wid, PI_WAVE_MODE_REPEAT);
   if (e < 14)
      CHECK(5, 3, e, 9, 0, "wave tx repeat");
   else
      CHECK(5, 3, e, 19, 0, "wave tx repeat");

   oc = t5_count;
   time_sleep(5);
   c = t5_count - oc;
   CHECK(5, 4, c, 50, 1, "callback");

   e = gpioWaveTxStop();
   CHECK(5, 5, e, 0, 0, "wave tx stop");

   /* gpioSerialReadOpen changes the alert function */

   e = gpioSerialReadOpen(GPIO, BAUD, 8);
   CHECK(5, 6, e, 0, 0, "serial read open");

   gpioWaveClear();
   e = gpioWaveAddSerial(GPIO, BAUD, 8, 2, 5000000, strlen(TEXT), TEXT);
   CHECK(5, 7, e, 3405, 0, "wave clear, wave add serial");

   wid = gpioWaveCreate();
   e = gpioWaveTxSend(wid, PI_WAVE_MODE_ONE_SHOT);
   if (e < 6964)
      CHECK(5, 8, e, 6811, 0, "wave tx start");
   else
      CHECK(5, 8, e, 7116, 0, "wave tx start");

   CHECK(5, 9, 0, 0, 0, "NOT APPLICABLE");

   CHECK(5, 10, 0, 0, 0, "NOT APPLICABLE");

   while (gpioWaveTxBusy())
      time_sleep(0.1);
   time_sleep(0.1);
   c = gpioSerialRead(GPIO, text, sizeof(text) - 1);
   if (c > 0)
      text[c] = 0;
   CHECK(5, 11, strcmp(TEXT, text), 0, 0, "wave tx busy, serial read");

   e = gpioSerialReadClose(GPIO);
   CHECK(5, 12, e, 0, 0, "serial read close");

   c = gpioWaveGetMicros();
   CHECK(5, 13, c, 6158148, 0, "wave get micros");

   c = gpioWaveGetHighMicros();
   CHECK(5, 14, c, 6158148, 0, "wave get high micros");

   c = gpioWaveGetMaxMicros();
   CHECK(5, 15, c, 1800000000, 0, "wave get max micros");

   c = gpioWaveGetPulses();
   CHECK(5, 16, c, 3405, 0, "wave get pulses");

   c = gpioWaveGetHighPulses();
   CHECK(5, 17, c, 3405, 0, "wave get high pulses");

   c = gpioWaveGetMaxPulses();
   CHECK(5, 18, c, 12000, 0, "wave get max pulses");

   c = gpioWaveGetCbs();
   if (e < 6963)
      CHECK(5, 19, c, 6810, 0, "wave get cbs");
   else
      CHECK(5, 19, c, 7115, 0, "wave get cbs");

   c = gpioWaveGetHighCbs();
   if (e < 6963)
      CHECK(5, 20, c, 6810, 0, "wave get high cbs");
   else
      CHECK(5, 20, c, 7115, 0, "wave get high cbs");

   c = gpioWaveGetMaxCbs();
   CHECK(5, 21, c, 25016, 0, "wave get max cbs");

   /* waveCreatePad tests */
   gpioWaveTxStop();
   gpioWaveClear();
   gpioSetAlertFunc(GPIO, t5cbf);

   e = gpioWaveAddGeneric(2, (gpioPulse_t[]){{1 << GPIO, 0, 10000},
                                             {0, 1 << GPIO, 30000}});
   wid = gpioWaveCreatePad(50, 50, 0);
   CHECK(5, 22, wid, 0, 0, "wave create pad, count==1, wid==");

   e = gpioWaveAddGeneric(4, (gpioPulse_t[]){{1 << GPIO, 0, 10000},
                                             {0, 1 << GPIO, 30000},
                                             {1 << GPIO, 0, 60000},
                                             {0, 1 << GPIO, 100000}});
   wid = gpioWaveCreatePad(50, 50, 0);
   CHECK(5, 23, wid, 1, 0, "wave create pad, count==2, wid==");

   c = gpioWaveDelete(0);
   CHECK(5, 24, c, 0, 0, "delete wid==0 success");

   e = gpioWaveAddGeneric(6, (gpioPulse_t[]){{1 << GPIO, 0, 10000},
                                             {0, 1 << GPIO, 30000},
                                             {1 << GPIO, 0, 60000},
                                             {0, 1 << GPIO, 100000},
                                             {1 << GPIO, 0, 60000},
                                             {0, 1 << GPIO, 100000}});
   c = gpioWaveCreate();
   CHECK(5, 25, c, -67, 0, "No more CBs using wave create");
   wid = gpioWaveCreatePad(50, 50, 0);
   CHECK(5, 26, wid, 0, 0, "wave create pad, count==3, wid==");

   t5_count = 0;
   e = gpioWaveChain((char[]){1, 0}, 2);
   CHECK(5, 27, e, 0, 0, "wave chain [1,0]");
   while (gpioWaveTxBusy())
      time_sleep(0.1);
   CHECK(5, 28, t5_count, 5, 1, "callback count==");

   gpioSetAlertFunc(GPIO, NULL);
}

int t6_count;
int t6_on;
uint32_t t6_on_tick;

void t6cbf(int gpio, int level, uint32_t tick)
{
   if (level == 1)
   {
      t6_on_tick = tick;
      t6_count++;
   }
   else
   {
      if (t6_on_tick)
         t6_on += (tick - t6_on_tick);
   }
}

void t6() // trigger test
{
   int tp, t, p;

   printf("Trigger tests\n");

   gpioWrite(GPIO, PI_LOW);

   tp = 0;

   t6_count = 0;
   t6_on = 0;
   t6_on_tick = 0;

   gpioSetAlertFunc(GPIO, t6cbf);

   for (t = 0; t < 5; t++)
   {
      time_sleep(0.1);
      p = 10 + (t * 10);
      tp += p;
      gpioTrigger(GPIO, p, 1);
   }

   time_sleep(0.2);

   CHECK(6, 1, t6_count, 5, 0, "gpio trigger count");

   CHECK(6, 2, t6_on, tp, 25, "gpio trigger pulse length");
}

int t7_count;

void t7cbf(int gpio, int level, uint32_t tick)
{
   if (level == PI_TIMEOUT)
      t7_count++;
}

void t7() // watchdog test
{
   int c, oc;

   printf("Watchdog tests.\n");

   t7_count = 0;

   /* type of edge shouldn't matter for watchdogs */
   gpioSetAlertFunc(GPIO, t7cbf);

   gpioSetWatchdog(GPIO, 50); /* 50 ms, 20 per second */
   time_sleep(0.5);
   oc = t7_count;
   time_sleep(2);
   c = t7_count - oc;
   CHECK(7, 1, c, 39, 5, "set watchdog on count");

   gpioSetWatchdog(GPIO, 0); /* 0 switches watchdog off */
   time_sleep(0.5);
   oc = t7_count;
   time_sleep(2);
   c = t7_count - oc;
   CHECK(7, 2, c, 0, 1, "set watchdog off count");
}

void t8() // Bank R/W test
{
   int v;

   printf("Bank read/write tests.\n");

   gpioWrite(GPIO, 0);
   v = gpioRead_Bits_0_31() & (1 << GPIO);
   CHECK(8, 1, v, 0, 0, "read bank 1");

   gpioWrite(GPIO, 1);
   v = gpioRead_Bits_0_31() & (1 << GPIO);
   CHECK(8, 2, v, (1 << GPIO), 0, "read bank 1");

   gpioWrite_Bits_0_31_Clear(1 << GPIO);
   v = gpioRead(GPIO);
   CHECK(8, 3, v, 0, 0, "clear bank 1");

   gpioWrite_Bits_0_31_Set(1 << GPIO);
   v = gpioRead(GPIO);
   CHECK(8, 4, v, 1, 0, "set bank 1");

   v = gpioRead_Bits_32_53();

   if (v)
      v = 0;
   else
      v = 1;

   CHECK(8, 5, v, 0, 0, "read bank 2");

   v = gpioWrite_Bits_32_53_Clear(0);
   CHECK(8, 6, v, 0, 0, "clear bank 2");

   CHECK(8, 7, 0, 0, 0, "NOT APPLICABLE");

   v = gpioWrite_Bits_32_53_Set(0);
   CHECK(8, 8, v, 0, 0, "set bank 2");

   CHECK(8, 9, 0, 0, 0, "NOT APPLICABLE");
}

int t9_count;

void t9cbf(int gpio, int level, uint32_t tick)
{
   if (level == 1)
      t9_count++;
}

void t9() // Script store/run/status/stop/delete test
{
   int s, oc, c, e;
   uint32_t p[10];

   /*
   100 loops per second
   p0 number of loops
   p1 GPIO
   */
   char *script = "\
   ld p9 p0\
   tag 0\
   w p1 1\
   mils 5\
   w p1 0\
   mils 5\
   dcr p9\
   jp 0";

   printf("Script store/run/status/stop/delete tests.\n");

   gpioWrite(GPIO, 0); /* need known state */

   t9_count = 0;

   gpioSetAlertFunc(GPIO, t9cbf);

   s = gpioStoreScript(script);

   while (1)
   {
      /* loop until script initialised */
      time_sleep(0.1);
      e = gpioScriptStatus(s, p);
      if (e != PI_SCRIPT_INITING)
         break;
   }

   oc = t9_count;
   p[0] = 99;
   p[1] = GPIO;
   gpioRunScript(s, 2, p);
   time_sleep(2);
   c = t9_count - oc;
   CHECK(9, 1, c, 100, 0, "store/run script");

   oc = t9_count;
   p[0] = 200;
   p[1] = GPIO;
   gpioRunScript(s, 2, p);
   time_sleep(0.1);
   while (1)
   {
      e = gpioScriptStatus(s, p);
      if (e != PI_SCRIPT_RUNNING)
         break;
      time_sleep(0.5);
   }
   c = t9_count - oc;
   time_sleep(0.1);
   CHECK(9, 2, c, 201, 0, "run script/script status");

   oc = t9_count;
   p[0] = 2000;
   p[1] = GPIO;
   gpioRunScript(s, 2, p);
   time_sleep(0.1);
   while (1)
   {
      e = gpioScriptStatus(s, p);
      if (e != PI_SCRIPT_RUNNING)
         break;
      if (p[9] < 1900)
         gpioStopScript(s);
      time_sleep(0.1);
   }
   c = t9_count - oc;
   time_sleep(0.1);
   CHECK(9, 3, c, 110, 10, "run/stop script/script status");

   e = gpioDeleteScript(s);
   CHECK(9, 4, e, 0, 0, "delete script");
}
#if 0
//int32_t retrData(int SPIHandle, uint16_t opcode, int numOfbyte)
int16_t twave_gen(int SPIHandle)
{
   printf("waveform generation. \n");
   return 0;
}

int16_t twave_cls(int SPIHandle)
{
   printf("waveform close. \n");
   return 0;
}
#endif
void ta() // serial link (UART) test
{
   int h, b, e;
   char *TEXT;
   char text[2048];

   printf("Serial link tests.\n");

   /* this test needs RXD and TXD to be connected */

   h = serOpen("/dev/ttyAMA0", 57600, 0);

   CHECK(10, 1, h, 0, 0, "serial open");

   b = serRead(h, text, sizeof(text)); /* flush buffer */

   b = serDataAvailable(h);
   CHECK(10, 2, b, 0, 0, "serial data available");

   TEXT = "\
To be, or not to be, that is the question-\
Whether 'tis Nobler in the mind to suffer\
The Slings and Arrows of outrageous Fortune,\
Or to take Arms against a Sea of troubles,\
";
   e = serWrite(h, TEXT, strlen(TEXT));
   CHECK(10, 3, e, 0, 0, "serial write");

   e = serWriteByte(h, 0xAA);
   e = serWriteByte(h, 0x55);
   e = serWriteByte(h, 0x00);
   e = serWriteByte(h, 0xFF);

   CHECK(10, 4, e, 0, 0, "serial write byte");

   time_sleep(0.1); /* allow time for transmission */

   b = serDataAvailable(h);
   CHECK(10, 5, b, strlen(TEXT) + 4, 0, "serial data available");

   b = serRead(h, text, strlen(TEXT));
   CHECK(10, 6, b, strlen(TEXT), 0, "serial read");
   if (b >= 0)
      text[b] = 0;
   CHECK(10, 7, strcmp(TEXT, text), 0, 0, "serial read");

   b = serReadByte(h);
   CHECK(10, 8, b, 0xAA, 0, "serial read byte");

   b = serReadByte(h);
   CHECK(10, 9, b, 0x55, 0, "serial read byte");

   b = serReadByte(h);
   CHECK(10, 10, b, 0x00, 0, "serial read byte");

   b = serReadByte(h);
   CHECK(10, 11, b, 0xFF, 0, "serial read byte");

   b = serDataAvailable(h);
   CHECK(10, 12, b, 0, 0, "serial data availabe");

   e = serClose(h);
   CHECK(10, 13, e, 0, 0, "serial close");
}

void tb() // I2C test
{
   int h, e, b, len;
   char *exp;
   char buf[128];

   printf("SMBus / I2C tests.");

   /* this test requires an ADXL345 on I2C bus 1 addr 0x53 */

   h = i2cOpen(1, 0x53, 0);
   CHECK(11, 1, h, 0, 0, "i2cOpen");

   e = i2cWriteDevice(h, "\x00", 1); /* move to known register */
   CHECK(11, 2, e, 0, 0, "i2cWriteDevice");

   b = i2cReadDevice(h, buf, 1);
   CHECK(11, 3, b, 1, 0, "i2cReadDevice");
   CHECK(11, 4, buf[0], 0xE5, 0, "i2cReadDevice");

   b = i2cReadByte(h);
   CHECK(11, 5, b, 0xE5, 0, "i2cReadByte");

   b = i2cReadByteData(h, 0);
   CHECK(11, 6, b, 0xE5, 0, "i2cReadByteData");

   b = i2cReadByteData(h, 48);
   CHECK(11, 7, b, 2, 0, "i2cReadByteData");

   exp = "\x1D[aBcDeFgHjKM]";
   len = strlen(exp);

   e = i2cWriteDevice(h, exp, len);
   CHECK(11, 8, e, 0, 0, "i2cWriteDevice");

   e = i2cWriteDevice(h, "\x1D", 1);
   b = i2cReadDevice(h, buf, len - 1);
   CHECK(11, 9, b, len - 1, 0, "i2cReadDevice");
   CHECK(11, 10, strncmp(buf, exp + 1, len - 1), 0, 0, "i2cReadDevice");

   if (strncmp(buf, exp + 1, len - 1))
      printf("got [%.*s] expected [%.*s]\n", len - 1, buf, len - 1, exp + 1);

   e = i2cWriteByteData(h, 0x1d, 0xAA);
   CHECK(11, 11, e, 0, 0, "i2cWriteByteData");

   b = i2cReadByteData(h, 0x1d);
   CHECK(11, 12, b, 0xAA, 0, "i2cReadByteData");

   e = i2cWriteByteData(h, 0x1d, 0x55);
   CHECK(11, 13, e, 0, 0, "i2cWriteByteData");

   b = i2cReadByteData(h, 0x1d);
   CHECK(11, 14, b, 0x55, 0, "i2cReadByteData");

   exp = "[1234567890#]";
   len = strlen(exp);

   e = i2cWriteBlockData(h, 0x1C, exp, len);
   CHECK(11, 15, e, 0, 0, "i2c writeBlockData");

   e = i2cWriteDevice(h, "\x1D", 1);
   b = i2cReadDevice(h, buf, len);
   CHECK(11, 16, b, len, 0, "i2cReadDevice");
   CHECK(11, 17, strncmp(buf, exp, len), 0, 0, "i2cReadDevice");

   if (strncmp(buf, exp, len))
      printf("got [%.*s] expected [%.*s]\n", len, buf, len, exp);

   b = i2cReadI2CBlockData(h, 0x1D, buf, len);
   CHECK(11, 18, b, len, 0, "i2cReadI2CBlockData");
   CHECK(11, 19, strncmp(buf, exp, len), 0, 0, "i2cReadI2CBlockData");

   if (strncmp(buf, exp, len))
      printf("got [%.*s] expected [%.*s]\n", len, buf, len, exp);

   exp = "(-+=;:,<>!%)";
   len = strlen(exp);

   e = i2cWriteI2CBlockData(h, 0x1D, exp, len);
   CHECK(11, 20, e, 0, 0, "i2cWriteI2CBlockData");

   b = i2cReadI2CBlockData(h, 0x1D, buf, len);
   CHECK(11, 21, b, len, 0, "i2cReadI2CBlockData");
   CHECK(11, 22, strncmp(buf, exp, len), 0, 0, "i2cReadI2CBlockData");

   if (strncmp(buf, exp, len))
      printf("got [%.*s] expected [%.*s]\n", len, buf, len, exp);

   e = i2cClose(h);
   CHECK(11, 23, e, 0, 0, "i2cClose");
}
#endif // end of the other pigpio test routines.

/*
***************************************************************************
*  The SPI periphearl for MCP4822 DAC                                     *
*  \fn void tspi_mcp4822(int channel, int command, double value)          *
*                                                                         *
*  \param channel is the MCP4822 DAC channel ID 0-A, 1-B                  *
*         command indicates the operation to do:                          *
*                       0-disable output,                                 *
*                       1-enable the output,                              *
*                       2-load the new value                              *
*         value is the output value in volt.                              *
*                                                                         *
*  \return None                                                           *
*                                                                         *
***************************************************************************
*/

#define DAC_LDAC 19
#define MCP4822_DAB (1 << 15)
#define MCP4822_GA1 (1 << 13)  //0-set to 2x gain; 1-set to 1x gain.
#define MCP4822_ACT (1 << 12)  //0-disable the DAC output; 1-enable the DAC output.

void tspi_mcp4822(int channel, int command, double valu) // SPI channel test MCP4822
{
   /**********************************
    *  channel: 0-DAC_A; 1-DAC_B
    *  command: 0-DIS_ACT, 1-EN_ACT, 2-LDAC
    *  valu: 0 - 2
    *************************************/

   int h, b, e;
   char txBuf[8];

   printf("    MCP4822 Settings. %d %d %f\n", channel, command, valu);

   /* set the DAC_LDAC command 2 DAC_LDAC is high, otherwise it is low*/
   gpioSetMode(DAC_LDAC, PI_OUTPUT); //TODO - move to main()
   
   //if (command == 2)
   //   gpioWrite(DAC_LDAC, 1);
   //else
   //   gpioWrite(DAC_LDAC, 0);

   /* this test requires a MCP4822 on SPI channel 1 */
   /*
   *******************************************************************
   * the spiopen() has three parameters: spiChan, baud, and spiFlags *
   *   spiChan - two channels for main SPI, 0(gpio8) & 1(gpio7)      *
   *   baud - SPI speed, 1,250,000 Hz                                *
   *   spiFlags - the SPI module settings.                           *
   *   -------------------------------------------------------       *
   *   |21 |20 |19 |18 |17 |16 |15 |14 |13 |12 |11 |10 |9 |8 |       *
   *   |---|---|---|---|---|---|---|---|---|---|---|---|--|--|       *
   *   |b  |b  |b  |b  |b  |b  |R  |T  |n  |n  |n  |n  |W |A |       *
   *   |------------------------------------------------------       *
   *   |7  |6  |5  |4  |3  |2  |1  |0  |                             *
   *   |---|---|---|---|---|---|---|---|                             *
   *   |u2 |u1 |u0 |p2 |p1 |p0 |m  |m  |                             *
   *   ---------------------------------                             *
   *    A - 0 for main SPI, 1 for auciliart SPI                      *
   *    W - 0 the device is not 3-wire, 1 the device is 3-wire.      *
   *     e.g set to 0.                                               *
   *******************************************************************
   *
   */

   h = spiOpen(1, 1250000, 0); // open SPI decice "/dev/spidev0.1" with mode 0 for MCP4822
   //CHECK(12, 1, h, 4, 100, "spiOpenDAC");
   printf("    DAC - %d\n", h);

   /* set SPI device command e.g MCP4822 */
   /*
    *
    *******************************************************************
    * The commands send to SPI:                                       *
    *    channel: CHAN0 - 0, CHAN1 - 1                                *
    *    gain: GAIN2 - 0(2x), GAIN1 - 1(1x)
    *    outOn: OUT_ON - 1, OUT_OFF - 0
    *                                                                 *
    *    valuSet=voltSet/2.048*4096 ; referrence 2.048V & 12-bit      *
    *                                                                 *
    *******************************************************************
    *
    */
   unsigned int numSteps, remainVal, valuSet, ctrlData, setValue;
   char byte0, byte1;

   /* set MCP4822 value */
   if (valu < 0 && valu > 2.048)
      valu = 1.0;

   valuSet = (unsigned int)(valu / 2.048 * 4096);
   /* setting DAC_A value from 0 to set value with increacing 10 digital number per 20ms */
   numSteps = valuSet/10; remainVal = valuSet%10;

   /* set MCP4822 control bits */
   ctrlData = MCP4822_GA1;
   if (channel == 1)
      ctrlData |= MCP4822_DAB; // set DA-B
   else if(channel == 0)
      ctrlData &= (~MCP4822_DAB); // set DA-A

   if (command == 0)
      ctrlData &= (~MCP4822_ACT); // no DA activite
   else if (command == 1 || command == 2)
      ctrlData |= MCP4822_ACT; // set DA activite

   /* set DAC value */
   if(channel == 0) //DAC_A
   {
      printf("    MCP4822 CH_A steps. %d\n", numSteps);
      for(int n = 0; n < numSteps; n++)
      {
         setValue = ctrlData + n*10;
         //printf("    MCP4822 Data. %d\n", setValue);

         byte0 = setValue & 0xFF;
         byte1 = (setValue >> 8) & 0xFF;

         txBuf[1] = byte0;
         txBuf[0] = byte1;
         // sprintf(txBuf, "\x01\x80");
         //printf("MCP4822 Data. %x %x %x\n", setValue, txBuf[1], txBuf[0]);

         /* write data to SPI */
         b = spiWrite(h, txBuf, 2);
         //CHECK(12, 2, b, 2, 0, "spiWrie");

         /* latch data to DAC */
         gpioWrite(DAC_LDAC, 0);
         gpioDelay(400);
         gpioWrite(DAC_LDAC, 1);
         gpioDelay(20000); // delay 20ms
      }
   }
//   else{ //DAC_B
//
//   }
   setValue = ctrlData + valuSet;
   printf("    MCP4822 Data. 0x%x\n", setValue);

   byte0 = setValue & 0xFF;
   byte1 = (setValue >> 8) & 0xFF;

   txBuf[1] = byte0;
   txBuf[0] = byte1;
   // sprintf(txBuf, "\x01\x80");
   //printf("MCP4822 Data. %x %x %x\n", setValue, txBuf[1], txBuf[0]);

   /* write data to SPI */
   b = spiWrite(h, txBuf, 2);
   //CHECK(12, 2, b, 2, 0, "spiWrie");

   /* latch data to DAC */
   gpioWrite(DAC_LDAC, 0);
   gpioDelay(400);
   gpioWrite(DAC_LDAC, 1);

   /*
      for (x=0; x<5; x++)
      {
         b = spiXfer(h, txBuf, rxBuf, 3);
         CHECK(12, 2, b, 3, 0, "spiXfer");
         if (b == 3)
         {
            time_sleep(1.0);
            printf("%d ", ((rxBuf[1]&0x0F)*256)|rxBuf[2]);
         }
      }
   */

   e = spiClose(h);
   //CHECK(12, 3, e, 0, 0, "spiClose");
}

/*
***************************************************************************
*  The SPI periphearl for ADS131M04 ADC initialization                    *
*  \fn void tspi_ads131m04_init(int channel, int command, double value)   *
*                                                                         *
*  \param channel is the MCP4822 DAC channel ID 0-A, 1-B                  *
*         command indicates the operation to do:                          *
*                       0-disable output,                                 *
*                       1-enable the output,                              *
*                       2-load the new value                              *
*         value is the output value in volt.                              *
*                                                                         *
*  \return None                                                           *
*                                                                         *
***************************************************************************
*/

#define ADC_CLKIN_EN 21
#define ADC_SYNC_RST 20
#define ADC_DRDY 16

// SPI commands definitions
#define OPCODE_NULL ((uint16_t)0x0000)
#define OPCODE_RESET ((uint16_t)0x0011)
#define OPCODE_RREG ((uint16_t)0xA000)
#define OPCODE_WREG ((uint16_t)0x6000)
#define OPCODE_STANDBY ((uint16_t)0x0022)
#define OPCODE_WAKEUP ((uint16_t)0x0033)
#define OPCODE_LOCK ((uint16_t)0x0555)
#define OPCODE_UNLOCK ((uint16_t)0x0655)

#define ID_ADDRESS ((uint8_t)0x00)
#define ID_DEFAULT ((uint16_t)0x2000 | (CHANNEL_COUNT << 8)) // NOTE: May change with future device revisions!
#define STATUS_ADDRESS ((uint8_t)0x01)
#define STATUS_DEFAULT ((uint16_t)0x0500)
#define MODE_ADDRESS ((uint8_t)0x02)
#define MODE_DEFAULT ((uint16_t)0x0510)
#define CLOCK_ADDRESS ((uint8_t)0x03)
#define CLOCK_DEFAULT ((uint16_t)0x0F0E)
#define GAIN1_ADDRESS ((uint8_t)0x04)
#define GAIN1_DEFAULT ((uint16_t)0x0000)
#define GAIN2_ADDRESS ((uint8_t)0x05)
#define GAIN2_DEFAULT ((uint16_t)0x0000)
#define CFG_ADDRESS ((uint8_t)0x06)
#define CFG_DEFAULT ((uint16_t)0x0600)
#define THRSHLD_MSB_ADDRESS ((uint8_t)0x07)
#define THRSHLD_MSB_DEFAULT ((uint16_t)0x0000)
#define THRSHLD_LSB_ADDRESS ((uint8_t)0x08)
#define THRSHLD_LSB_DEFAULT ((uint16_t)0x0000)
#define CH0_CFG_ADDRESS ((uint8_t)0x09)
#define CH0_CFG_DEFAULT ((uint16_t)0x0000)
#define CH0_OCAL_MSB_ADDRESS ((uint8_t)0x0A)
#define CH0_OCAL_MSB_DEFAULT ((uint16_t)0x0000)
#define CH0_OCAL_LSB_ADDRESS ((uint8_t)0x0B)
#define CH0_OCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define CH0_GCAL_MSB_ADDRESS ((uint8_t)0x0C)
#define CH0_GCAL_MSB_DEFAULT ((uint16_t)0x0000)
#define CH0_GCAL_LSB_ADDRESS ((uint8_t)0x0D)
#define CH0_GCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define CH1_CFG_ADDRESS ((uint8_t)0x0E)
#define CH1_CFG_DEFAULT ((uint16_t)0x0000)
#define CH1_OCAL_MSB_ADDRESS ((uint8_t)0x0F)
#define CH1_OCAL_MSB_DEFAULT ((uint16_t)0x0000)
#define CH1_OCAL_LSB_ADDRESS ((uint8_t)0x10)
#define CH1_OCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define CH1_GCAL_MSB_ADDRESS ((uint8_t)0x11)
#define CH1_GCAL_MSB_DEFAULT ((uint16_t)0x8000)
#define CH1_GCAL_LSB_ADDRESS ((uint8_t)0x12)
#define CH1_GCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define CH2_CFG_ADDRESS ((uint8_t)0x13)
#define CH2_CFG_DEFAULT ((uint16_t)0x0000)
#define CH2_OCAL_MSB_ADDRESS ((uint8_t)0x14)
#define CH2_OCAL_MSB_DEFAULT ((uint16_t)0x0000)
#define CH2_OCAL_LSB_ADDRESS ((uint8_t)0x15)
#define CH2_OCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define CH2_GCAL_MSB_ADDRESS ((uint8_t)0x16)
#define CH2_GCAL_MSB_DEFAULT ((uint16_t)0x8000)
#define CH2_GCAL_LSB_ADDRESS ((uint8_t)0x17)
#define CH2_GCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define CH3_CFG_ADDRESS ((uint8_t)0x18)
#define CH3_CFG_DEFAULT ((uint16_t)0x0000)
#define CH3_OCAL_MSB_ADDRESS ((uint8_t)0x19)
#define CH3_OCAL_MSB_DEFAULT ((uint16_t)0x0000)
#define CH3_OCAL_LSB_ADDRESS ((uint8_t)0x1A)
#define CH3_OCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define CH3_GCAL_MSB_ADDRESS ((uint8_t)0x1B)
#define CH3_GCAL_MSB_DEFAULT ((uint16_t)0x8000)
#define CH3_GCAL_LSB_ADDRESS ((uint8_t)0x1C)
#define CH3_GCAL_LSB_DEFAULT ((uint16_t)0x0000)
#define REGMAP_CRC_ADDRESS ((uint8_t)0x3E)
#define REGMAP_CRC_DEFAULT ((uint16_t)0x0000)

#define NUM_REGISTERS ((uint8_t)64)

typedef struct
{
   uint16_t response;
   uint16_t crc;
   int32_t channel0;
   int32_t channel1;
   int32_t channel2;
   int32_t channel3;
} adc_channel_data;

typedef struct
{
   uint16_t regAddr;
   uint16_t setData;
   uint16_t numRegs;
} regInfor;

typedef struct
{
   float ratio;
   float squF1;
   float squF2;
}caliRlt;

typedef struct
{
   int dataCnt;
   caliRlt Rslt[30];
}manuCst;

adc_channel_data adcData;
regInfor regSetInf;

static uint16_t registerMap[NUM_REGISTERS];

char upperByte(uint16_t uint16_Word)
{
   char msByte;
   msByte = (char)((uint16_Word >> 8) & 0x00FF);

   return msByte;
}

char lowerByte(uint16_t uint16_Word)
{
   char lsByte;
   lsByte = (char)(uint16_Word & 0x00FF);

   return lsByte;
}

/*
*********************************************************************************************************
*                                                                                                       *
* Builds SPI TX data arrays to be tranferred.                                                           *
*                                                                                                       *
* \fn uint8_t setSPIarray(const uint16_t opcodeArray[], uint8_t numberOpcodes, uint8_t byteArray[])   *
*                                                                                                       *
* \param opcodeArray[] pointer to an array of 16-bit opcodes to use in the SPI command.                 *
* \param numberOpcodes the number of opcodes provided in opcodeArray[].                                 *
* \param byteArray[] pointer to an array of 8-bit SPI bytes to send to the device.                      *
*                                                                                                       *
* NOTE: The calling function must ensure it reserves sufficient memory for byteArray[]!                 *
*                                                                                                       *
* \return number of bytes added to byteArray[].                                                         *
*                                                                                                       *
*********************************************************************************************************
*
*/
uint8_t setSPIarray(const uint16_t opcodeArray[], uint8_t numberOpcodes, char byteArray[])
{
   /*
    * Frame size = opcode word(s) + optional CRC word
    * Number of bytes per word = 2, 3, or 4
    * Total bytes = bytes per word * number of words
    */
   uint8_t numberWords = numberOpcodes; // as SPI CRC disabled
   uint8_t bytesPerWord = 3;            // as 24-bit per word, getWordByteLength();
   uint8_t numberOfBytes = numberWords * bytesPerWord;

   int i;
   for (i = 0; i < numberWords; i++)
   {
      // NOTE: Be careful not to accidentally overflow the array here.
      // The array and opcodes are defined in the calling function, so
      // we are trusting that no mistakes were made in the calling function!
      byteArray[(i * bytesPerWord) + 0] = upperByte(opcodeArray[i]);
      byteArray[(i * bytesPerWord) + 1] = lowerByte(opcodeArray[i]);
      byteArray[(i * bytesPerWord) + 2] = 0; // lowerByte(opcodeArray[i]);
   }

   // set rest of the byteArray to 0
   for (i = numberOfBytes; i < 18; i++)
   {
      byteArray[i] = 0;
   }
   // #ifdef ENABLE_CRC_IN
   //  Calculate CRC and put it into TX array
   //    uint16_t crcWord = calculateCRC(&byteArray[0], numberOfBytes, 0xFFFF);
   //    byteArray[(i*bytesPerWord) + 0] = upperByte(crcWord);
   //    byteArray[(i*bytesPerWord) + 1] = lowerByte(crcWord);
   // #endif

   return numberOfBytes;
}

uint32_t combineBytes(const char dataBytes[], int numOfbyte)
{
   uint32_t combinedValue;

   if (numOfbyte == 2)
      combinedValue = ((uint32_t)dataBytes[0] << 8) | ((uint32_t)dataBytes[1]);
   if (numOfbyte == 3)
   {
      combinedValue = ((uint32_t)dataBytes[0] << 16) | ((uint32_t)dataBytes[1] << 8) | ((uint32_t)dataBytes[2]);
   }
   return combinedValue;
}

/*
****************************************************************************
*                                                                          *
* Retrieve data from the ADC with a NULL command.                          *
*                                                                          *
*                                                                          *
* \param opcode SPI command byte.                                          *
*                                                                          *
* NOTE: Other commands have their own dedicated functions to support       *
* additional functionality.                                                *
*                                                                          *
* \return ADC response byte (typically the STATUS byte).                   *
*                                                                          *
****************************************************************************
*/
int32_t retrData(int SPIHandle, uint16_t opcode, int numOfbyte)
{
   int esp; // h, x, b, e;

   // Build TX and RX byte array
   // uint8_t dataTx[8] = { 0 };      // 2 words, up to 4 bytes each = 8 bytes maximum
   // uint8_t dataRx[8] = { 0 };
   char txBuf[8] = {0};
   char rxBuf[8] = {0};
   uint8_t numberOfBytes;
   uint16_t lopcode = opcode; // OPCODE_NULL;

   /* prepare SPI command package */
   numberOfBytes = setSPIarray(&lopcode, 1, txBuf);

   // check txBuf
   // for(int i = 0; i < numberOfBytes; i++)
   {
      printf("txBuf0.0 numberOfBytes %d: x%x, x%x, x%x, x%x; \n", numberOfBytes, txBuf[0], txBuf[1], txBuf[2], txBuf[3]);
      printf("txBuf0.1: x%x, x%x, x%x, x%x; \n", txBuf[4], txBuf[5], txBuf[6], txBuf[7]);
   }

   // Send the opcode (and crc word, if enabled)
   esp = spiXfer(SPIHandle, txBuf, rxBuf, numberOfBytes);

   // check the rxBuf
   // for(int i = 0; i < numberOfBytes; i++)
   {
      printf("rxBuf0.0: x%x, x%x, x%x, x%x; \n", rxBuf[0], rxBuf[1], rxBuf[2], rxBuf[3]);
      printf("rxBuf0.1: x%x, x%x, x%x, x%x; \n", rxBuf[4], rxBuf[5], rxBuf[6], rxBuf[7]);
   }

   // Combine response bytes and return as a 16-bit word
   int32_t adcResponse = combineBytes(&rxBuf[0], numOfbyte);
   return adcResponse;
}

/*
****************************************************************************
*                                                                          *
* Sends the specified SPI command to the ADC (NULL, STANDBY, or WAKEUP).   *
*                                                                          *
*                                                                          *
* \param opcode SPI command byte.                                          *
*                                                                          *
* NOTE: Other commands have their own dedicated functions to support       *
* additional functionality.                                                *
*                                                                          *
* \return ADC response byte (typically the STATUS byte).                   *
*                                                                          *
****************************************************************************
*/

char txBuf[32] = {0};
char rxBuf[32] = {0};

uint16_t sendCommand(int SPIHandle, uint16_t opcode, regInfor *regData, adc_channel_data *DataStruct)
{
   int ret; // h, x, b, e;
   /* Assert if this function is used to send any of the following opcodes */
   // assert(OPCODE_RREG != opcode);      /* Use "readSingleRegister()"   */
   // assert(OPCODE_WREG != opcode);      /* Use "writeSingleRegister()"  */
   // assert(OPCODE_LOCK != opcode);      /* Use "lockRegisters()"        */
   // assert(OPCODE_UNLOCK != opcode);    /* Use "unlockRegisters()"      */
   // assert(OPCODE_RESET != opcode);     /* Use "resetDevice()"          */

   // Build TX and RX byte array

   char *ptxBuf = &txBuf; //[32] = {0};
   char *prxBuf = &rxBuf; //[32] = {0};

   uint8_t numberOfBytes;
   uint16_t lopcode[2] = {0};
   uint16_t lnumRegs = regData->numRegs;

   lopcode[0] = opcode;

   /* prepare SPI command package */
   if (OPCODE_RREG == lopcode[0] || OPCODE_NULL == lopcode[0]) // if it's read Reg or NULL.
   {
      lopcode[0] |= regData->regAddr << 7;
      lopcode[0] += lnumRegs;
      numberOfBytes = setSPIarray(&lopcode[0], 1, ptxBuf);
   }
   else if (OPCODE_WREG == lopcode[0]) // if it's write Reg.
   {
      lopcode[0] |= regData->regAddr << 7;
      lopcode[0] += lnumRegs;
      lopcode[1] = regData->setData;
      numberOfBytes = setSPIarray(&lopcode[0], 2, ptxBuf);
   }
   // printf("command: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x.\n", lopcode[0], lopcode[1], *(ptxBuf+0), *(ptxBuf+1), *(ptxBuf+2));

   // check the txBuf

   //printf("\ntxBuf0 numberOfBytes %d: 0x%x, 0x%x, 0x%x, 0x%x; \n", numberOfBytes, txBuf[0], txBuf[1], txBuf[2], txBuf[3]);

   //printf("txBuf1: 0x%x, 0x%x, 0x%x, 0x%x; \n", txBuf[4], txBuf[5], txBuf[6], txBuf[7]);

   // Send the opcode (and crc word, if enabled)
   numberOfBytes = 18; // 3bytes*6words.
   ret = spiXfer(SPIHandle, ptxBuf, prxBuf, numberOfBytes);

   // check the rxBuf
   // printf("rxBuf0: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x; \n", rxBuf[0], rxBuf[1], rxBuf[3], rxBuf[4], rxBuf[5]);

   // printf("rxBuf1: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x; \n", rxBuf[6], rxBuf[7], rxBuf[8], rxBuf[9], rxBuf[10], rxBuf[11]);

   // retrive data from the rxbuf[]
   DataStruct->response = (uint16_t)combineBytes(&rxBuf[0], 2);
   DataStruct->channel0 = combineBytes(&rxBuf[3], 3);
   DataStruct->channel1 = combineBytes(&rxBuf[6], 3);
   DataStruct->channel2 = combineBytes(&rxBuf[9], 3);
   DataStruct->channel3 = combineBytes(&rxBuf[12], 3);
   DataStruct->crc = (uint16_t)combineBytes(&rxBuf[15], 2);

   return ret; //DataStruct->response;
}

/*
****************************************************************************
*
* Reads ADC data.
*
* \fn int readData(adc_channel_data *DataStruct)
*
* \param *DataStruct points to an adc_channel_data type-defined structure/
*
* NOTE: Should be called after /DRDY goes low, and not during a /DRDY falling edge!
*
* \return Returns true if the CRC-OUT of the data read detects an error.
*
*****************************************************************************
*/
int readData(int SPIhandler, adc_channel_data *DataStruct)
{
   // int i;
   // uint8_t crcTx[4]                        = { 0 };
   // uint8_t dataRx[4]                       = { 0 };
   // uint8_t bytesPerWord                    = 3; //getWordByteLength();

#ifdef ENABLE_CRC_IN
   // Build CRC word (only if "RX_CRC_EN" register bit is enabled)
   uint16_t crcWordIn = calculateCRC(&DataTx[0], bytesPerWord * 2, 0xFFFF);
   crcTx[0] = upperByte(crcWordIn);
   crcTx[1] = lowerByte(crcWordIn);
#endif

   /* Set the nCS pin LOW */
   // setCS(LOW);

   // Send NULL word, receive response word
   DataStruct->response = (uint16_t)retrData(SPIhandler, OPCODE_NULL, 2); // sendCommand(SPIhandler, 0);
   // for (i = 0; i < bytesPerWord; i++)
   //{
   //     dataRx[i] = spiSendReceiveByte(0x00);
   // }
   // DataStruct->response = combineBytes(dataRx[0], dataRx[1]);

   // (OPTIONAL) Do something with the response (STATUS) word.
   // ...Here we only use the response for calculating the CRC-OUT
   // uint16_t crcWord = calculateCRC(&dataRx[0], bytesPerWord, 0xFFFF);

   // (OPTIONAL) Ignore CRC error checking
   uint16_t crcWord = 0;

   // Send 2nd word, receive channel 1 data
   DataStruct->channel0 = retrData(SPIhandler, OPCODE_NULL, 3); // TODO: return int32_t data!!!

   // for (i = 0; i < bytesPerWord; i++)
   //{
   //    dataRx[i] = spiSendReceiveByte(crcTx[i]);
   // }
   // DataStruct->channel0 = signExtend(&dataRx[0]);
   // crcWord = calculateCRC(&dataRx[0], bytesPerWord, crcWord);

   // Send 3rd word, receive channel 2 data
   DataStruct->channel1 = retrData(SPIhandler, OPCODE_NULL, 3); // TODO: return int32_t data!!!

   // for (i = 0; i < bytesPerWord; i++)
   //{
   //     dataRx[i] = spiSendReceiveByte(0x00);
   // }
   // DataStruct->channel1 = signExtend(&dataRx[0]);
   // crcWord = calculateCRC(&dataRx[0], bytesPerWord, crcWord);

   // Send 4th word, receive channel 3 data
   DataStruct->channel2 = retrData(SPIhandler, OPCODE_NULL, 3); // TODO: return int32_t data!!!

   // for (i = 0; i < bytesPerWord; i++)
   //{
   //     dataRx[i] = spiSendReceiveByte(0x00);
   // }
   // DataStruct->channel2 = signExtend(&dataRx[0]);
   // crcWord = calculateCRC(&dataRx[0], bytesPerWord, crcWord);

   // Send 5th word, receive channel 4 data
   DataStruct->channel3 = retrData(SPIhandler, OPCODE_NULL, 3); // TODO: return int32_t data!!!

   // for (i = 0; i < bytesPerWord; i++)
   //{
   //     dataRx[i] = spiSendReceiveByte(0x00);
   // }
   // DataStruct->channel3 = signExtend(&dataRx[0]);
   // crcWord = calculateCRC(&dataRx[0], bytesPerWord, crcWord);

   // Send the next word, receive CRC data if no CRC???
   // for (i = 0; i < bytesPerWord; i++)
   //{
   //    dataRx[i] = spiSendReceiveByte(0x00);
   //}
   // DataStruct->crc = combineBytes(dataRx[0], dataRx[1]);

   /* NOTE: If we continue calculating the CRC with a matching CRC, the result should be zero.
    * Any non-zero result will indicate a mismatch.
    */
   // crcWord = calculateCRC(&dataRx[0], bytesPerWord, crcWord);

   /* Set the nCS pin HIGH */
   // setCS(HIGH);

   // Returns true when a CRC error occurs
   return ((int)crcWord);
}

#if 0
/*
****************************************************************************
*
* Resets the device.
*
* \fn void resetDevice(void)
*
* NOTE: This function does not capture DOUT data, but it could be modified
* to do so.
*
* \return None.
*****************************************************************************
*/
void resetDevice(void)
{
    // Build TX and RX byte array
    uint8_t txBuf[8] = { 0 };      // 2 words, up to 4 bytes each = 8 bytes maximum
    uint16_t lopcode         = OPCODE_RESET;
    uint8_t numberOfBytes   = setSPIarray(&lopcode, 1, txBuf);

    uint8_t bytesPerWord    = 3;
    uint8_t wordsInFrame    = 4 + 2;

    // Set the nCS pin LOW
    //setCS(LOW);

    // Send the opcode (and CRC word, if enabled)
    int i;
    for (i = 0; i < numberOfBytes; i++)
    {
         spiSendReceiveByte(txBuf[i]);
    }

    // Finish sending remaining bytes
    for (i = numberOfBytes; i < (wordsInFrame * bytesPerWord); i++)
    {
        spiSendReceiveByte(0x00);
    }

    // NOTE: The ADS131M0x's next response word should be (0xFF20 | CHANCNT),
    // if the response is 0x0011 (acknowledge of RESET command), then the device
    // did not receive a full SPI frame and the reset did not occur!

    // Set the nCS pin HIGH
    //setCS(HIGH);

    // tSRLRST delay, ~1ms with 2.048 MHz fCLK
    delay_ms(1);

    // Update register setting array to keep software in sync with device
    //restoreRegisterDefaults();

    // Write to MODE register to enforce mode settings
    writeSingleRegister(MODE_ADDRESS, MODE_DEFAULT);
}
#endif

#if 0 //old ads131m04 start routines. TODO remove.
uint16_t tspi_ads131m04_init(int value) // SPI for ADS131M04
{
   int h, v, x, b, e;
   // char txBuf[8] = {0};
   // char rxBuf[8] = {0};
   // adc_channel_data Data;

   printf("SPI ads131 tests.\n");
   /* set the ADC_CLKIN_EN */
   gpioSetMode(ADC_CLKIN_EN, PI_OUTPUT);
   gpioWrite(ADC_CLKIN_EN, 1); // enable external clock 8.024MHz
   v = gpioRead(ADC_CLKIN_EN);
   CHECK(12, 0, v, 0, 0, "enable clock, get level");
   /* set the ADC_SYNC_RST */
   gpioSetMode(ADC_SYNC_RST, PI_OUTPUT);
   gpioWrite(ADC_SYNC_RST, 0); // set low to reset chip
   v = gpioRead(ADC_SYNC_RST);
   CHECK(12, 0, v, 0, 0, "set reset, get level");
   /* set the ADC_DRDY */
   gpioSetMode(ADC_DRDY, PI_INPUT); // set DRNY input
   v = gpioRead(ADC_DRDY);
   gpioDelay(1000);
   CHECK(12, 0, v, 0, 0, "set data ready, get level");

   /* this test requires a ADS131M04 on SPI channel 1 */
   /*
   *******************************************************************
   * the spiopen() has three parameters: spiChan, baud, and spiFlags *
   *   spiChan - two channels for main SPI, 0(gpio8) & 1(gpio7)      *
   *   baud - SPI speed, 1,250,000 Hz                                *
   *   spiFlags - the SPI module settings.                           *
   *   -------------------------------------------------------       *
   *   |21 |20 |19 |18 |17 |16 |15 |14 |13 |12 |11 |10 |9 |8 |       *
   *   |---|---|---|---|---|---|---|---|---|---|---|---|--|--|       *
   *   |b  |b  |b  |b  |b  |b  |R  |T  |n  |n  |n  |n  |W |A |       *
   *   |------------------------------------------------------       *
   *   |7  |6  |5  |4  |3  |2  |1  |0  |                             *
   *   |---|---|---|---|---|---|---|---|                             *
   *   |u2 |u1 |u0 |p2 |p1 |p0 |m  |m  |                             *
   *   ---------------------------------                             *
   *    A - 0 for main SPI, 1 for auciliart SPI                      *
   *    W - 0 the device is not 3-wire, 1 the device is 3-wire.      *
   *     e.g set to 0.                                               *
   *******************************************************************
   *
   */

   h = spiOpen(0, 1250000, 1); // open SPI decice "/dev/spidev0.0" with mode 1 for ads131m04
   CHECK(12, 1, h, 0, 0, "spiOpen");

   /* Initialize SPI device ADS131M04 */
   /*
    *
    *******************************************************************
    * The commands send to initialize ADS131M04:                      *
    *  # reset the chip with sending a low puls on RST pin            *
    *  # restore registers with defaults settings. internal records   *
    *  # validate first response word when beginning SPI              *
    *           (0xFF20 | CHANCNT)                                    *
    *  # configure MODE registers with defaults settings.             *
    *                                                                 *
    *******************************************************************
    *
    */

   // set reset pin high to finish the resetting chip
   gpioDelay(2000);
   gpioWrite(ADC_SYNC_RST, 1); // set high to end the reset chip

   // restore registers
   // add later

   // uint8_t RegAdd = 0;
   uint16_t rep0, rep;
   // write to Mode register to enforce mode settings
   // adc_channel_data adcData;
   // regInfor regSetInf;
   regSetInf.regAddr = 2;
   regSetInf.setData = 0x510;
   rep = sendCommand(h, OPCODE_WREG, &regSetInf, &adcData);

   // valiate first response word with (0xFF20 | CHANCNT)
   // rep0 = (uint16_t)retrData(h, OPCODE_NULL, 2); //it's done in th esendCommand()???
   rep0 = adcData.response;
   printf("Init SPI-ADC resp 0x%x. \n", rep0);
   gpioDelay(500);
   // set regiters value
#if 0  // check registers   
   for(int idx = 0; idx < 9; idx++)
   {
	   gpioDelay(100);
      rep = registerMap[RegAdd] = sendCommand(h, OPCODE_RREG, RegAdd, 0);
      printf("Reg %d 0x%x. \n", RegAdd, rep);
      gpioDelay(500);
	   RegAdd++;
   }
#endif // end of check registers

   // check STATUS if DRDY ready read data
   // rep = (uint16_t)retrData(h, OPCODE_NULL, 2);
   regSetInf.regAddr = 0;
   rep = sendCommand(h, OPCODE_NULL, &regSetInf, &adcData);

   rep &= 0x000f; // DRDY flags
   x = 0;
   while (x < 120)
   {
      // printf("Data: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", rep, adcData.channel0, adcData.channel1, adcData.channel2, adcData.channel3);
      // readData(h, &Data);
      // rep &= 0x000f;
      if ((rep & 0x000f) == 0)
      {
         printf("No Data\n");
         gpioDelay(160);

         regSetInf.regAddr = 0;
         rep = sendCommand(h, OPCODE_NULL, &regSetInf, &adcData);
      }
      else
      {
         printf("Data: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", rep, adcData.channel0, adcData.channel1, adcData.channel2, adcData.channel3);
         gpioDelay(150);
         regSetInf.regAddr = 0;
         rep = sendCommand(h, OPCODE_NULL, &regSetInf, &adcData);
      }
      x++;
      // gpioDelay(50);;
   }

   // rep = (uint16_t)retrData(h, OPCODE_NULL, 2);
   // printf("Status 0x%x\n", rep);

   e = spiClose(h);
   CHECK(12, 99, e, 0, 0, "spiClose");

   gpioWrite(ADC_CLKIN_EN, 0); // disable external clock 8.024MHz
   // v = gpioRead(ADC_CLKIN_EN);

   return rep0;
}

void tspi_ads131m04(char *command, uint16_t address, unsigned int value)
{
   char *Cmd = command;
   // char* Addr = address;

   uint16_t Addr = address;

   if (!strcmp("Init", Cmd))
   {
      uint16_t reps = tspi_ads131m04_init(Addr);
      return;
   }

   if (!strcmp("Set", Cmd))
   {
      printf("Command set: addr: 0x%x value 0x%x\n", Addr, value);
      return;
   }

   if (!strcmp("Get", Cmd))
   {
      printf("Command get: addr: 0x%x value 0x%x\n", Addr, value);
      return;
   }

   if (!strcmp("Reset", Cmd))
   {
      printf("Command Reset: addr: 0x%x value 0x%x\n", Addr, value);
      return;
   }

   if (!strcmp("Read", Cmd))
   {
      printf("Command Read Data: addr: 0x%x value 0x%x\n", Addr, value);
      return;
   }

   printf("ads131m04's commands, %s, %d, 0x%x\n", Cmd, Addr, value);

   return;
}
#endif // end of old ads131m04 start routines.

uint16_t tspi_ads131m04_rd(int SPIhandler, regInfor *getInf);

/* TODO - move to head file */
#define SAMPRAT (1000000/210)
#define ADCLNTH 32

typedef struct ADCRsults_t{
   uint32_t tick;
   double results0;
   double results1;
   double results2;
   double results3;
}adcRslts;

typedef struct UserData_t{
   int handle;
   int isRun;
   int datIdx;
   uint32_t preTick;
   adcRslts *pRslts;
}userData;

adcRslts adcRltData[ADCLNTH];
userData adcCapFuncData;

void adcCaptureFun(int gpio, int level, uint32_t tick, userData* padcCapFuncData) //a callback function for capture adc data.
{
   int reps, h, isRn, idx;
   uint32_t lpreTick;
   regInfor *pregInf = &regSetInf;
   
   lpreTick = padcCapFuncData->preTick;
   h = padcCapFuncData->handle;
   isRn = padcCapFuncData->isRun;
   idx = (padcCapFuncData->datIdx);

   if((level == 0) && (isRn == 1) && (tick > (lpreTick + SAMPRAT)))
   {
         pregInf->regAddr = 1;
         pregInf->numRegs = 0; //numRegs;
         reps = tspi_ads131m04_rd(h, pregInf);
         /*
         double step = 1200000.0 / 8388607.0;
         double v1, v2, v3, v4;

         if(adcData.channel0 > 0x7fffff)
         {
            v1 = (double)(~(adcData.channel0 | 0xff000000)+1);
            v1 = -v1;
         }
         else
         {
            v1 = (double)adcData.channel0;
         }

         if(adcData.channel1 > 0x7fffff)
         {
            v2 = (double)(~(adcData.channel1 | 0xff000000)+1);
            v2 = -v2;
         }
         else
         {
            v2 = (double)adcData.channel1;
         }
         
         if(adcData.channel2 > 0x7fffff)
         {
            v3 = (double)(~(adcData.channel2 | 0xff000000)+1);
            v3 = -v3;
         }
         else
         {
            v3 = (double)adcData.channel2;
         }
         
         if(adcData.channel3 > 0x7fffff)
         {
            v4 = (double)(~(adcData.channel3 | 0xff000000)+1);
            v4 = -v4;
         }
         else
         {
            v4 = (double)adcData.channel3;
         }
         */
         /* updata the adcCapFuncData */
         idx = (idx + 1)%ADCLNTH;
         padcCapFuncData->datIdx = idx;
         padcCapFuncData->preTick = tick;
         (padcCapFuncData->pRslts + idx)->tick = tick;


         //v1 *= step;
         //v2 *= step;
         //v3 *= step;
         //v4 *= step;
         
         //(padcCapFuncData->pRslts + idx)->results0 = v1;
         //(padcCapFuncData->pRslts + idx)->results1 = v2;
         //(padcCapFuncData->pRslts + idx)->results2 = v3;
         //(padcCapFuncData->pRslts + idx)->results3 = v4;
         
         //TODO - convert to int type
         int dataIn = adcData.channel0;
         if(dataIn > 0x7fffff)
         {
            dataIn = (adcData.channel0 | 0xff000000);
         }
         (padcCapFuncData->pRslts + idx)->results0 = (float)dataIn;
         
         dataIn = adcData.channel1;
         if(dataIn > 0x7fffff)
         {
            dataIn = (adcData.channel1 | 0xff000000);
         }
         (padcCapFuncData->pRslts + idx)->results1 = (float)dataIn;
         
         dataIn = adcData.channel2;
         if(dataIn > 0x7fffff)
         {
            dataIn = (adcData.channel2 | 0xff000000);
         }
         (padcCapFuncData->pRslts + idx)->results2 = (float)dataIn;
         
         dataIn = adcData.channel3;
         if(dataIn > 0x7fffff)
         {
            dataIn = (adcData.channel3 | 0xff000000);
         }
         (padcCapFuncData->pRslts + idx)->results3 = (float)dataIn;

         
         //printf("Data(%d, %u): 0x%x, %.02f, %.02f, %.2f, %.2f\n", gpio, tick-lpreTick, adcData.response, v1, v2, v3, v4);
               
   }
   
   return;
}

int tspi_ads131m04_start(regInfor *pregInf, adc_channel_data *padcData) // start ads131m04, return SPI handle
{
   int h, vclk, vrst0, vrst1, vdrdy;
   uint16_t rep0, rep;
   // char txBuf[8] = {0};
   // char rxBuf[8] = {0};
   // adc_channel_data Data;

   //printf("ads131 start up.\n");

   /* set the ADC_CLKIN_EN */
   gpioSetMode(ADC_CLKIN_EN, PI_OUTPUT);
   gpioWrite(ADC_CLKIN_EN, 1); // enable external clock 8.024MHz???
   vclk = gpioRead(ADC_CLKIN_EN);

   /* set the ADC_SYNC_RST */
   gpioSetMode(ADC_SYNC_RST, PI_OUTPUT);
   gpioWrite(ADC_SYNC_RST, 0); // set low to reset chip
   vrst0 = gpioRead(ADC_SYNC_RST);

   /* set the ADC_DRDY */
   gpioSetMode(ADC_DRDY, PI_INPUT); // set DRNY input
   vdrdy = gpioRead(ADC_DRDY);
   gpioDelay(1000);

   /* this test requires a ADS131M04 on SPI channel 1 */
   /*
   *******************************************************************
   * the spiopen() has three parameters: spiChan, baud, and spiFlags *
   *   spiChan - two channels for main SPI, 0(gpio8) & 1(gpio7)      *
   *   baud - SPI speed, 1,250,000 Hz                                *
   *   spiFlags - the SPI module settings.                           *
   *   -------------------------------------------------------       *
   *   |21 |20 |19 |18 |17 |16 |15 |14 |13 |12 |11 |10 |9 |8 |       *
   *   |---|---|---|---|---|---|---|---|---|---|---|---|--|--|       *
   *   |b  |b  |b  |b  |b  |b  |R  |T  |n  |n  |n  |n  |W |A |       *
   *   |------------------------------------------------------       *
   *   |7  |6  |5  |4  |3  |2  |1  |0  |                             *
   *   |---|---|---|---|---|---|---|---|                             *
   *   |u2 |u1 |u0 |p2 |p1 |p0 |m  |m  |                             *
   *   ---------------------------------                             *
   *    A - 0 for main SPI, 1 for auciliart SPI                      *
   *    W - 0 the device is not 3-wire, 1 the device is 3-wire.      *
   *     e.g set to 0.                                               *
   *******************************************************************
   *
   */

   // h = spiOpen(0, 1250000, 1); // open SPI decice "/dev/spidev0.0" with mode 1 for ads131m04
   h = spiOpen(0, 2500000, 1); // open SPI decice "/dev/spidev0.0" with mode 1 for ads131m04
   //CHECK(12, 1, h, 4, 100, "spiOpenADC");
   //printf("ADC - %d", h);

   /* Initialize SPI device ADS131M04 */
   /*
    *
    *******************************************************************
    * The commands send to initialize ADS131M04:                      *
    *  # reset the chip with sending a low puls on RST pin            *
    *  # restore registers with defaults settings. internal records   *
    *  # validate first response word when beginning SPI              *
    *           (0xFF20 | CHANCNT)                                    *
    *  # configure MODE registers with defaults settings.             *
    *                                                                 *
    *******************************************************************
    *
    */

   // set reset pin high to finish the resetting chip
   gpioDelay(2000);
   gpioWrite(ADC_SYNC_RST, 1); // set high to end the reset chip
   vrst1 = gpioRead(ADC_SYNC_RST);

   //printf("\nads131 ctrl signals: enclk-%d, rst0-%d, rst1-%d, vdrdy-%d.\n", vclk, vrst0, vrst1, vdrdy);

   // write to Mode register(0x2) to enforce mode settings
   pregInf->regAddr = 2;
   pregInf->setData = 0x510;
   rep = sendCommand(h, OPCODE_WREG, pregInf, padcData);

   // valiate first response word with (0xFF20 | CHANCNT)
   // rep0 = (uint16_t)retrData(h, OPCODE_NULL, 2); //it's done in th esendCommand()???
   rep0 = adcData.response;
   printf("start ads131m04 0x%x. \n", rep0);
   gpioDelay(500);
   
   /* set alert callback function */
   //userData* pfuncData = &adcCapFuncData;

   //pfuncData->handle = h; pfuncData->isRun = 0;
   //gpioSetAlertFuncEx(ADC_DRDY, adcCaptureFun, pfuncData);

   return h; // pigpio set, spi opened and return api handle, h.
}

int tspi_ads131m04_close(int SPIhandler) // close ads131m04, return SPI handle
{
   int ext;
   ext = spiClose(SPIhandler);
   //CHECK(12, 99, ext, 0, 0, "spiClose");

   gpioWrite(ADC_CLKIN_EN, 0); // disable external clock 8.024MHz

   return ext;
}

int tspi_ads131m04_rst(int SPIhandler)
{
   int ok = 0;
   /* reset ads131m04 */
   printf("reset ads131m04 handler - 0x%x. \n", SPIhandler);
   return ok;
}

uint16_t tspi_ads131m04_rd(int SPIhandler, regInfor *getInf)
{
   uint16_t ret = 0, rsp;
   regInfor *lpgetInf = getInf;
   adc_channel_data *lpadcData = &adcData;

   /* reead ads131m04 register */
   // regSetInf.regAddr = getInf->regAddr;
   // regSetInf.setData = 0;
   ret = sendCommand(SPIhandler, OPCODE_RREG, lpgetInf, lpadcData);
   rsp = lpadcData->response;
   
   //printf("read Reg. addr - 0x%x, data - 0x%x. \n", getInf->regAddr, rsp);
   return rsp;
}

uint16_t tspi_ads131m04_wt(int SPIhandler, regInfor *setInf)
{
   uint16_t ret = 0, rsp;
   /* write ads131m04 register */
   regInfor *lpgetInf = setInf;
   adc_channel_data *lpadcData = &adcData;

   /* reead ads131m04 register */
   // regSetInf.regAddr = setInf->regAddr;
   // regSetInf.setData = setInf->setData;
   ret = sendCommand(SPIhandler, OPCODE_WREG, lpgetInf, lpadcData);
   rsp = lpadcData->response;
   
   printf("write Reg. addr - 0x%x, data - 0x%x, rsp - 0x%x\n", setInf->regAddr, setInf->setData, rsp);
   return rsp;
}

char *inputStr(char *inStr)
{
   printf("Input Command: ");
   scanf("%s", inStr);
   return inStr;
}


#if 0  // old main & data reading
uint16_t tspi_ads131m04_rdData(int SPIhandler, adc_channel_data *inData)
{
   uint16_t ret = 0;char* inputString(*inputStr)
{
   pintf("Input Command: ");
   scanf("%s", inputStr);
   return inputStr;
}

   /* reset ads131m04 */
   printf("input data: rspn - 0x%0x ch0 - 0x%x ch1 - 0x%x ch2 - 0x%x ch3 - 0x%x. \n",
          inData->response, inData->channel0, inData->channel1, inData->channel2, inData->channel3);
   return ret;
}

int main(int argc, char *argv[])
{
   int i, t, c, status;
   double value = 0;
   int channel, command;

   printf("test command. %d %s %s %s %s\n", argc, argv[1], argv[2], argv[3], argv[4]);
   
   char *piPeriID=argv[1];
   
   status = gpioInitialise();

   if (status < 0)
   {
      fprintf(stderr, "pigpio initialisation failed.\n");
      return 1;
   }

   if(!strcmp("Init", piPeriID))
   {
      uint16_t reps = tspi_ads131m04_init(0);
      gpioTerminate();
      return 0;
   }

   if(!strcmp("Set", piPeriID))
   {
      //printf("Command set: addr: 0x%x value 0x%x\n", Addr, value);
      gpioTerminate();
      return 0;
   }

   if(!strcmp("Get", piPeriID))
   {
      //printf("Command get: addr: 0x%x value 0x%x\n", Addr, value);
      gpioTerminate();
      return 0;
   }

   if(!strcmp("Reset", piPeriID))
   {
      //printf("Command Reset: addr: 0x%x value 0x%x\n", Addr, value);
      gpioTerminate();
      return 0;
   }

   if(!strcmp("Read", piPeriID))
   {
      //printf("Command Read Data: addr: 0x%x value 0x%x\n", Addr, value);
      gpioTerminate();
      return 0;
   }

   printf("ads131m04's commands, %s, %d, 0x%f\n", piPeriID, 0, value);
   gpioTerminate();
   return 0;
   
   char test[64]={0,};

   if (argc > 1)
   {
      t = 0;

      for (i=0; i<strlen(argv[1]); i++)
      {
         c = tolower(argv[1][i]);

         if (!strchr(test, c))
         {
            test[t++] = c;
            test[t] = 0;
         }
      }

      if(argc == 5)
      {
         value = atof(argv[4]);
         channel = atoi(argv[2]); //address
         command = atoi(argv[3]); 
      }	  
   }
   else strcat(test, "0123456789");

   status = gpioInitialise();

   if (status < 0)
   {
      fprintf(stderr, "pigpio initialisation failed.\n");
      return 1;
   }

   if (strchr(test, '0')) t0();
   if (strchr(test, '1')) t1();
   if (strchr(test, '2')) t2();
   if (strchr(test, '3')) t3();
   if (strchr(test, '4')) t4();
   if (strchr(test, '5')) t5();
   if (strchr(test, '6')) t6();
   if (strchr(test, '7')) t7();
   if (strchr(test, '8')) t8();
   if (strchr(test, '9')) t9();
   if (strchr(test, 'a')) ta();
   if (strchr(test, 'b')) tb();
   if ((strchr(test, 'c')) && (value > 0))
   {
	   //tc(channel, command, value);
	   printf("the DAC code, %s \n", "DA command");
   }
   if (strchr(test, 'd'))
   {
	   //command = 2;
      if(argc == 5)
      {
         value = atof(argv[4]);
      }
      else{
         printf("the parameraters are wrong! need command, address, value.");
      }	  

      //tspi_ads131m04(argv[2], atoi(argv[3]), value);
      uint16_t reps = tspi_ads131m04_init(command);
	   printf("the ADC code, %s, %s, %s, 0x%x\n", "AD command", argv[2], argv[3], atoi(argv[4]));
   }

   gpioTerminate();

   return 0;
}
#else  // new main()

/* define the tec and laser pins */
#define TEC_CTRL_EN 23
#define LASER_DETECT_EN 25

char *mtd415 = "mtd415Set";
char *mtd415setFunc = "tecCmdset";
char *mtd415getFunc = "tecCmdget";

/**************************************************
 * file data: write/read data from a file.
 *   data including:
 *      - date: Sec:Minu:Hour:Mon:Day:Year
 *      - V1,V2,Ratio,Constant
 *      - GPS Data
 * ************************************************
*/
char inputData[64];
char outputData[64];

void	getDate(char *date)
{
   char buff1[100];
   time_t curTime;
   struct tm curDate = *localtime(&curTime);
 
	sprintf(date,"%d/%02d/%02d, %02d:%02d:%02d",
      curDate.tm_mon + 1, curDate.tm_mday, curDate.tm_year + 1900, 
      curDate.tm_hour, curDate.tm_min, curDate.tm_sec);

   return;
}

/*
void	readDate(char *date)
{
   char buff1[100];
   time_t curTime;
   struct tm curDate = *localtime(&curTime);
 
	sprintf(date,"%d/%02d/%02d, %02d:%02d:%02d\n",
      curDate.tm_mon + 1, curDate.tm_mday, curDate.tm_year + 1900, 
      curDate.tm_hour, curDate.tm_min, curDate.tm_sec);

   return;
}
*/

int main(int argc, char *argv[])
{
   int h, i, t, c, status;
   double value = 0;
   int channel; // command;

   float tecRet = 0;
   char *presult;

   //uint16_t reps;
   char command[16]; //*piPeriID=argv[1];
   int reps, addr, setData, numRegs;
   adc_channel_data *padcData = &adcData;
   regInfor *pregInf = &regSetInf;
   manuCst cnstRlts;

   // file
   FILE *fh;

   fh = fopen("./mCalib.txt", "ab"); //open/create a file. read or add data to file.

#if 0   /* for test file handler */
   char date[32];
   
   getDate(&date);
   printf(date);
   fprintf(fh, "%s\n", date);
   
   int ab;
   while(1){
      printf("\nend?");
      scanf("%s", command);
      printf("%s\n", command);
      
      if(!strcmp("Yes", command))
      {
         break;
      }  
      fprintf(fh, "%s\n", date);
   }



   ab = fclose(fh);

   printf("end code\n");
 
   return 0;

#endif   /* end of the test */

   status = gpioInitialise();

   if (status < 0)
   {
      printf("pigpio initialisation failed.\n");
      return 1;
   }
   printf("Hand-Helder Manufactory Calibration\n");

   /* disable the Laser and Tec */
   gpioWrite(TEC_CTRL_EN, 0);
   gpioWrite(LASER_DETECT_EN, 0);

   /* Set DAC_A to lowest value, such as 0.001mA */
   tspi_mcp4822(0, 2, 0.001);
   gpioDelay(400);

   /* enable the Tec-contrller */
   gpioWrite(TEC_CTRL_EN, 1);

   /* Ask Tec TempPoint set */
   char argu3[32] = "get temp point";
   //char *presult;
         //char argu2[32] = "tecSetPoint", argu3[32] = "null";
         //printf("    Input call name and argument: ");
         //scanf("%s %s", &argu2, &argu3);
   //printf("input func-name %s, %s\n\r", mtd415Func, argu3);
   presult = tempCtrll_py(3, mtd415, mtd415getFunc, &argu3);
   tecRet = atof(presult);
   //printf("     Temperature point is %s.\n\n", presult);
   
   printf("\n    Do you want set Temperature Point? (yes or no): ");
   scanf("%s", command);
   if(!strcmp("yes", command))
   {
      char argu3[32] = "set temp point";
      presult = tempCtrll_py(3, mtd415, mtd415setFunc, &argu3);
      tecRet = atof(presult);
      printf("     Temperature point is %s.\n\n", presult);

   }else{
      printf("     Temperture poin: %.4f. No re-set.\n\n", tecRet);
   }

   /* turn on the laser when current temperature is reached to Temp-Point */
   
   int count = 0; float tempRet = 0;
   while(fabs(tecRet - tempRet) > 0.01 && count < 30)
   {
      char argu3[32] = "get temperature";
      presult = tempCtrll_py(3, mtd415, mtd415getFunc, &argu3);
      tempRet = atof(presult);
      //printf("     Temperature point is %s.\n\n", presult);
      printf("     delta Temp is %.4f, %.4f, %.4f.\n\n", tecRet, tempRet, fabs(tecRet - tempRet));
      gpioDelay(100);
      count++;
   }

   /* turn on the laser detector */
   gpioWrite(LASER_DETECT_EN, 1);

   h = tspi_ads131m04_start(pregInf, padcData); // open the pigpio spi
   //printf("set GPIO = %d.\n", GPIO);
   
   /* use alert function */
   userData* pfuncData = &adcCapFuncData;

   pfuncData->pRslts = &adcRltData[0]; pfuncData->datIdx = 0;
   pfuncData->handle = h; pfuncData->isRun = 0;
   gpioSetAlertFuncEx(ADC_DRDY, NULL, pfuncData);

   //printf("pigpio started.\n    Input Command: ");
   printf("  Please input the following command:\n"
         "    setTempPoint,\n"
         "    setDAC,\n"
         "    startWaveforms,\n"
         "    readADC,\n"
         "    getCalibConstant,\n"
         "    closeCalib,\n\n"
            "Input Command: "         
         );

   scanf("%s", command);
   //*command = inputStr(command);

   while (1)
   {
      // printf("execute %s - %d.\n", command, addr);

      if (!strcmp("ResetClose", command))
      {
         // execute code;
         reps = tspi_ads131m04_rst(h);
         printf("    Input Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("ReadClose", command))
      {
         //printf("    Input Address and No. Regs: ");
         //scanf("%d %d", &addr, &numRegs);
         printf("    Input Address: ");
         scanf("%d", &addr);
         // execute code;
         // test mine int data to comment the following statment TODO --
         //pregInf->regAddr = addr;
         //pregInf->numRegs = 0; //numRegs;
         //reps = tspi_ads131m04_rd(h, pregInf);
         reps = addr + 0x80000000;
         printf("     input: %d 0x%x %.2f; reps: %d 0x%x %.2f\n    Input Command: ", addr, addr, (float)addr, reps, reps, (float)reps);
         scanf("%s", command);
      }
      else if (!strcmp("WriteClose", command))
      {
         printf("    Input Address and Setting: ");
         scanf("%x %x", &addr, &setData);
         // execute code;
         pregInf->regAddr = addr;
         pregInf->setData = setData;
         pregInf->numRegs = 0; // only set one register
         reps = tspi_ads131m04_wt(h, pregInf);
         printf("    reps: 0x%0x \n    Input Command: ", reps);
         scanf("%s", command);
      }
      else if (!strcmp("SetChxIntTestClose", command))
      {
         printf("    Input Setting: ");
         scanf("%x", &setData);
         // execute code;
         pregInf->regAddr = 0x09; //set channel0
         if((setData < 0) || (setData > 3))
            setData = 0;
         pregInf->setData = setData;
         pregInf->numRegs = 0; // only set one register
         reps = tspi_ads131m04_wt(h, pregInf);
         gpioDelay(20);

         pregInf->regAddr = 0x0e; //set channel1
         pregInf->setData = setData;
         pregInf->numRegs = 0; // only set one register
         reps = tspi_ads131m04_wt(h, pregInf);
         gpioDelay(20);

         pregInf->regAddr = 0x13; //set channel2
         pregInf->setData = setData;
         pregInf->numRegs = 0; // only set one register
         reps = tspi_ads131m04_wt(h, pregInf);
         gpioDelay(20);

         pregInf->regAddr = 0x18; //set channel3
         pregInf->setData = setData;
         pregInf->numRegs = 0; // only set one register
         reps = tspi_ads131m04_wt(h, pregInf);
         gpioDelay(20);

         printf("    reps: 0x%0x \n    Input Command: ", reps);
         scanf("%s", command);
      }
      /*       
      else if (!strcmp("Data", command)) // no this command
      {
         // execute code;
         reps = tspi_ads131m04_rdData(h, padcData);
         printf("Input Command: ");
         scanf("%s", command);
      }
      */
      else if (!strcmp("closeCalib", command))
      {
         // execute code;
         reps = tspi_ads131m04_close(h);
         //printf("Close Execution.\n");
         break;
      }
      else if (!strcmp("startWaveforms", command))
      {
         // execute code;
         //printf("     wave\n");
         reps = twave_gen(h); // generate waveforms
         printf("\nInput Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("ClsWave", command))
      {
         // execute code;
         reps = twave_cls(h); // close waveforms
         printf("\nInput Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("setDAC", command))
      {
         float value = 0;
         printf("    Input ChanID (0 or 1): ");
         scanf("%d", &addr);
         if(addr == 0)
         {
            printf("    Input Value (0 to 2.04 V): ");
            scanf("%f", &value);            
         }
         else if(addr == 1){
            printf("    Input Value (0 to 600 mV): ");            
            scanf("%f", &value);
            value /=1000;
         }else{
            printf("    ChanID is not correct!\n");
         }
         // execute code;
         tspi_mcp4822(addr, 2, value);
         printf("\nInput Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("LdisStartClose", command))
      {
         /* start Laser Distance main */
         UART_distMain(LASERDST);

         printf("\nInput Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("TctrlStartGPSClose", command))
      {
         /* start Laser Distance main */
         //UART_tempCMain(TEMPCTRL);

         /* for python GPS peaser test */
         char *parguGps1 = "nmeaParser";
         char arguGps2[32] = "parsMsg", arguGps3[32] = "The GPS parser calling test!";
         printf("    Input call name and argument: ");
         scanf("%s %s", &arguGps2, &arguGps3);


         tempCtrll_py(3, parguGps1, &arguGps2, &arguGps3);
         printf("Input Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("setTempPoint", command))
      {
         /* start Laser Distance main */
         //UART_tempCMain(TEMPCTRL);

         /* for python mtd415 tec-set */
         //char *pargu1 = "mtd415Set";
         //char argu2[32] = "tecCmd"; 
         char argu3[32] = "set temp point";
         //char *presult;
         //char argu2[32] = "tecSetPoint", argu3[32] = "null";
         //printf("    Input call name and argument: ");
         //scanf("%s %s", &argu2, &argu3);

         presult = tempCtrll_py(3, mtd415, mtd415setFunc, &argu3);
         printf("     set point is %s.\n\n", presult);

         printf("Input Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("getTemperature", command))
      {
         /* start Laser Distance main */
         //UART_tempCMain(TEMPCTRL);

         /* for python mtd415 tec-set */
         //char *pargu1 = "mtd415Set";
         //char argu2[32] = "tecSet", argu3[32] = "The GPS parser calling test!";
         //char argu2[32] = "tecCmd";
         char argu3[32] = "get temperature";
         //printf("    Input call name: ");
         //scanf("%s %s", &argu3);

         //if(!strcmp("get", argu3))
         //{
         //   argu3[32] = "get temp point";
         //}
         //else{
         //   argu3[32] = "set temp point";
         //}
         //char *presult;

         presult = tempCtrll_py(3, mtd415, mtd415getFunc, &argu3);
         printf("     get temperature is %s.\n\n", presult);
         
         printf("Input Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("getTempPt", command))
      {
         /* start Laser Distance main */
         //UART_tempCMain(TEMPCTRL);

         /* for python mtd415 tec-set */
         //char *pargu1 = "mtd415Set";
         //char argu2[32] = "tecSet", argu3[32] = "The GPS parser calling test!";
         //char argu2[32] = "tecCmd";
         char argu3[32] = "get temp point";
         //printf("    Input call name: ");
         //scanf("%s %s", &argu3);

         //if(!strcmp("get", argu3))
         //{
         //   argu3[32] = "get temp point";
         //}
         //else{
         //   argu3[32] = "set temp point";
         //}
         //char *presult;

         presult = tempCtrll_py(3, mtd415, mtd415getFunc, &argu3);
         printf("     get temp point is %s.\n\n", presult);
         
         printf("Input Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("DbgData", command))
      {
         printf("Tx0:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", txBuf[0], txBuf[1], txBuf[2], txBuf[3], txBuf[4], txBuf[5], txBuf[6], txBuf[7], txBuf[8]);
         printf("Tx1:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", txBuf[9], txBuf[10], txBuf[11], txBuf[12], txBuf[13], txBuf[14], txBuf[15], txBuf[16], txBuf[17]);

         printf("Rx0:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", rxBuf[0], rxBuf[1], rxBuf[2], rxBuf[3], rxBuf[4], rxBuf[5], rxBuf[6], rxBuf[7], rxBuf[8]);
         printf("Rx1:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", rxBuf[9], rxBuf[10], rxBuf[11], rxBuf[12], rxBuf[13], rxBuf[14], rxBuf[15], rxBuf[16], rxBuf[17]);

         // printf("Data: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", adcData.response, adcData.channel0, adcData.channel1, adcData.channel2, adcData.channel3);
         double step = 1200000.0 / 8388607.0;
         double v1, v2, v3, v4; 
         
         if(adcData.channel0 > 0x7fffff)
         {
            v1 = (double)(~(adcData.channel0 | 0xff000000)+1);
            v1 = -v1;
         }
         else
         {
            v1 = (double)adcData.channel0;
         }

         if(adcData.channel1 > 0x7fffff)
         {
            v2 = (double)(~(adcData.channel1 | 0xff000000)+1);
            v2 = -v2;
         }
         else
         {
            v2 = (double)adcData.channel1;
         }
         
         if(adcData.channel2 > 0x7fffff)
         {
            v3 = (double)(~(adcData.channel2 | 0xff000000)+1);
            v3 = -v3;
         }
         else
         {
            v3 = (double)adcData.channel2;
         }
         
         if(adcData.channel3 > 0x7fffff)
         {
            v4 = (double)(~(adcData.channel3 | 0xff000000)+1);
            v4 = -v4;
         }
         else
         {
            v4 = (double)adcData.channel3;
         }

         v1 *= step;
         v2 *= step;
         v3 *= step;
         v4 *= step;
         printf("Data: 0x%x, %.02f, %.02f, %.02f, %.02f\n", adcData.response, v1, v2, v3, v4);

         printf("Input Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("readADC", command))
      {
         #if 1
         int count = 0;
         /* initialize the ncurses lib */
         initscr();
         /* use alert function */
         pfuncData->datIdx = 0;
         pfuncData->isRun = 1;
         
         gpioSetAlertFuncEx(ADC_DRDY, adcCaptureFun, pfuncData);

         /* while loop for checking temp and adc each 100 ms */
         uint32_t curTick, preTick;
         uint32_t preDatTick = 0;
         int ratioIdx = 0, capIdx = 0;
         float ratioArray[20];
         preTick = curTick = gpioTick(); 
         while(/*!kbhit()*/pfuncData->isRun == 1 && count < 24)
         {
            //if((curTick - preTick) >= 100000)
            //cnstRlts.dataCnt;
            if(pfuncData->datIdx > 23)
            {
               pfuncData->isRun = 0;
               /* get temperature */              
               //char *pargu1 = "mtd415Set";
               //char argu2[32] = "tecCmd"; 
               char argu3[32] = "get temperature";
               //char *presult;
            
               presult = tempCtrll_py(3, mtd415, mtd415getFunc, &argu3);
               printf("\r     set temperature is %s.\r\n", presult);

               /* calculate data */
               int32_t fx1 = 0,fy1 = 0, fx2 = 0, fy2 = 0, maxDtick = 0, curTick;
               double df1, df2, v1, v2, v3, v4, step;
               
               for(int n = 2; n < 22; n++)
               {
                  fx1 += (pfuncData->pRslts + n)->results0;
                  fy1 += (pfuncData->pRslts + n)->results1;
                  fx2 += (pfuncData->pRslts + n)->results2;
                  fy2 += (pfuncData->pRslts + n)->results3;
                  if(n > 3)
                     curTick = (pfuncData->pRslts + n)->tick - (pfuncData->pRslts + (n - 1))->tick;
                  maxDtick = (maxDtick > curTick) ? maxDtick:curTick;
               }

               fx1 /= 20; fy1 /= 20; fx2 /= 20; fy2 /= 20;
               
               /* convert to voltage */
               //int count = 0, countf = 0;
                
               #if 1  //TODO - rm following code 
               uint32_t ticksSum = 0;

               if(fx1 > 0x7fffff) //adcData.channel0
               {
                  v1 = (double)(~(fx1 | 0xff000000)+1);
                  v1 = -v1;
               }
               else
               {
                  v1 = (double)fx1;
               }

               if(fy1 > 0x7fffff) //adcData.channel1
               {
                  v2 = (double)(~(fy1 | 0xff000000)+1);
                  v2 = -v2;
               }
               else
               {
                  v2 = (double)fy1;
               }
               
               if(fx2 > 0x7fffff) //adcData.channel2
               {
                  v3 = (double)(~(fx2 | 0xff000000)+1);
                  v3 = -v3;
               }
               else
               {
                  v3 = (double)fx2;
               }
               
               if(fy2 > 0x7fffff) //adcData.channel3
               {
                  v4 = (double)(~(fy2 | 0xff000000)+1);
                  v4 = -v4;
               }
               else
               {
                  v4 = (double)fy2;
               }
               #endif //TODO end

               step = 1200000.0 / 8388607.0;

               v1 *= step;
               v2 *= step;
               v3 *= step;
               v4 *= step;
               
               df1 = v1*v1 + v2*v2;
               df2 = v3*v3 + v4*v4;

               df1 = sqrt(df1);
               df2 = sqrt(df2);

               curTick = gpioTick();

               capIdx = cnstRlts.dataCnt = count;
               
               capIdx = capIdx%20;
               cnstRlts.dataCnt = capIdx;
               if(df2 != 0.0)
               {
                  //n = (n+1)%20;
                  //cnstRlts.dataCnt = n;

                  cnstRlts.Rslt[capIdx].squF1 = df1;
                  cnstRlts.Rslt[capIdx].squF2 = df2;
                  ratioArray[capIdx] = cnstRlts.Rslt[capIdx].ratio = df1/df2;
                  printf("     lasting %d max-dalt %d; result %.4f, %.4f and ratio %.4f\n\r", (curTick - preTick), maxDtick, df1, df2, df1/df2);
               }
               else
               {
                  printf("    dF2 is zero. %.4f, %.4f", df1, df2);
               }
               printf("    (%d): %.02f, %.02f, %.2f, %.2f\n\r", count, v1, v2, v3, v4);
            
               /* renew the data buffer */
               pfuncData->datIdx = 0;
               pfuncData->isRun = 1;
               curTick = preTick = gpioTick();
               count++;
               //}

               /* end of convert */
               
               /* output data */
               printf("\r    %d - %d -- %d\n\r", (pfuncData->pRslts + 2)->tick,
               (pfuncData->pRslts + 21)->tick,
               (pfuncData->pRslts + 21)->tick - (pfuncData->pRslts + 2)->tick);

               //curTick = gpioTick();
            }
            //if(kbhit())
            //{
            //   pfuncData->isRun = 0;
            //   break;
            //}
         }

         /* any keyboard press ends the loop */
         //printf(" ");
         //scanf("%s", command);
  
         pfuncData->isRun = 0;
         
         endwin(); //end ncurses

         /* check the capture data */
         printf("\n\n    Calculating Constant.\n     Input the gas concentration: ");
         float cnstRlt = 0, samplePercent, avgRatio;
         scanf("%f", &samplePercent);

         for(ratioIdx = 0; ratioIdx < 20; ratioIdx++)
         {
            avgRatio += ratioArray[ratioIdx];
            //printf("%.4f, %.4f, %d\n", ratioArray[ratioIdx], avgRatio, ratioIdx);
         }
         
         if(avgRatio == 0)
         {
            printf("the ratio is zero!\n");
         }
         else{
            printf("totalRatio %.4f; numbers %d\n", avgRatio, ratioIdx);
            avgRatio /= ratioIdx;
            cnstRlt = samplePercent/avgRatio;
            printf("\n     Average Ration %.4f; concentration %.4f The contant is %.4f. \n", 
                  avgRatio, samplePercent, cnstRlt);
         }


         /* save data in a file */
         // save date and time in file

         // save current constant in file
         fprintf(fh, "Gas Concentration: %.4f, Constant: %.4f, AvgRatio: %.4f\n", samplePercent, cnstRlt, avgRatio);

   #if 0 //debug print
         int dataIdx = pfuncData->datIdx;
         for(int n = 0; n < ADCLNTH; n++)
         {
            printf("     Data1(%d, %d, %u): %.2f, %.2f, %.2f, %.2f\n", n,
               pfuncData->datIdx, 
               (pfuncData->pRslts + n + 1)->tick - (pfuncData->pRslts + n)->tick,
               (pfuncData->pRslts + n)->results0,
               (pfuncData->pRslts + n)->results1,
               (pfuncData->pRslts + n)->results2,
               (pfuncData->pRslts + n)->results3);
         }
         printf("    %d - %d -- %d\n", (pfuncData->pRslts + 2)->tick,
               (pfuncData->pRslts + 21)->tick,
               (pfuncData->pRslts + 21)->tick - (pfuncData->pRslts + 2)->tick);

         /* end of using alert function */
   #endif //end of debug print
         #endif
      
         #if 0 //read data number of data
         printf("    Input No. to capture: ");
         scanf("%d", &setData);

         pregInf->regAddr = 1;
         pregInf->numRegs = 0; //numRegs;%[^\n]%*c
         reps = tspi_ads131m04_rd(h, pregInf);

         //printf("Tx0:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", txBuf[0], txBuf[1], txBuf[2], txBuf[3], txBuf[4], txBuf[5], txBuf[6], txBuf[7], txBuf[8]);
         //printf("Tx1:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", txBuf[9], txBuf[10], txBuf[11], txBuf[12], txBuf[13], txBuf[14], txBuf[15], txBuf[16], txBuf[17]);

         //printf("Rx0:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", rxBuf[0], rxBuf[1], rxBuf[2], rxBuf[3], rxBuf[4], rxBuf[5], rxBuf[6], rxBuf[7], rxBuf[8]);
         //printf("Rx1:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", rxBuf[9], rxBuf[10], rxBuf[11], rxBuf[12], rxBuf[13], rxBuf[14], rxBuf[15], rxBuf[16], rxBuf[17]);

         // printf("Data: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", adcData.response, adcData.channel0, adcData.channel1, adcData.channel2, adcData.channel3);
         //double step = 1200000.0 / 8388607.0;
         //double v1, v2, v3, v4;

            //v1 = (double)(~(0xe00000));
            //printf("v1 %f.\n", v1);
            //v1 = 0.0 - v1;
            //printf("0-v1 %f.\n", v1);

         //printf("Data: 0x%x, %f, %f, %f, %f\n", adcData.response, v1, v2, v3, v4);
         #endif

         printf("\nInput Command: ");
         scanf("%s", command);
      }
      else if (!strcmp("CaptureADCClose", command))
      {
         #if 0
         /* use alert function */
         pfuncData->isRun = 1;
         
         //gpioSetAlertFuncEx(ADC_DRDY, adcCaptureFun, pfuncData);
         printf(" ");
         scanf("%s", command);
             
         pfuncData->isRun = 0;
            
         /* check the capture data */
         printf("\n ");
         for(int n = 0; n < ADCLNTH; n++)
         {
            //(padcCapFuncData->pRslts + idx)->results0 = v1;
            //(padcCapFuncData->pRslts + idx)->results1 = v2;
            //(padcCapFuncData->pRslts + idx)->results2 = v3;
            //(padcCapFuncData->pRslts + idx)->results3 = v4;
         
            printf("Data(%d, %u): %.2f, %.2f, %.2f, %.2f\n", n, 
               (pfuncData->pRslts + n)->tick,
               (pfuncData->pRslts + n)->results0,
               (pfuncData->pRslts + n)->results1,
               (pfuncData->pRslts + n)->results2,
               (pfuncData->pRslts + n)->results3);
         }

         /* end of using alert function */
         #endif
      
         #if 1 //read data number of data
         printf("    Input No. to capture: ");
         scanf("%d", &setData);

         pregInf->regAddr = 1;
         pregInf->numRegs = 0; //numRegs;
         reps = tspi_ads131m04_rd(h, pregInf);
         
         int count = 0, countf = 0;
         double step = 1200000.0 / 8388607.0;
         double v1, v2, v3, v4;
         uint32_t ticksSum = 0;

         while((count < setData) && (countf < 200*setData))
         {
            if(0x50f == adcData.response)
            {

               if(adcData.channel0 > 0x7fffff)
               {
                  v1 = (double)(~(adcData.channel0 | 0xff000000)+1);
                  v1 = -v1;
               }
               else
               {
                  v1 = (double)adcData.channel0;
               }

               if(adcData.channel1 > 0x7fffff)
               {
                  v2 = (double)(~(adcData.channel1 | 0xff000000)+1);
                  v2 = -v2;
               }
               else
               {
                  v2 = (double)adcData.channel1;
               }
               
               if(adcData.channel2 > 0x7fffff)
               {
                  v3 = (double)(~(adcData.channel2 | 0xff000000)+1);
                  v3 = -v3;
               }
               else
               {
                  v3 = (double)adcData.channel2;
               }
               
               if(adcData.channel3 > 0x7fffff)
               {
                  v4 = (double)(~(adcData.channel3 | 0xff000000)+1);
                  v4 = -v4;
               }
               else
               {
                  v4 = (double)adcData.channel3;
               }
               
               v1 *= step;
               v2 *= step;
               v3 *= step;
               v4 *= step;
               
               printf("Data(%d): %.02f, %.02f, %.2f, %.2f\n", count, v1, v2, v3, v4);
               count++;
               ticksSum = 0;
            }

            ticksSum += gpioDelay(5000); //1000000/200Hz
            reps = tspi_ads131m04_rd(h, pregInf);
            countf++;
         }
         //printf("Tx0:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", txBuf[0], txBuf[1], txBuf[2], txBuf[3], txBuf[4], txBuf[5], txBuf[6], txBuf[7], txBuf[8]);
         //printf("Tx1:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", txBuf[9], txBuf[10], txBuf[11], txBuf[12], txBuf[13], txBuf[14], txBuf[15], txBuf[16], txBuf[17]);

         //printf("Rx0:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", rxBuf[0], rxBuf[1], rxBuf[2], rxBuf[3], rxBuf[4], rxBuf[5], rxBuf[6], rxBuf[7], rxBuf[8]);
         //printf("Rx1:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", rxBuf[9], rxBuf[10], rxBuf[11], rxBuf[12], rxBuf[13], rxBuf[14], rxBuf[15], rxBuf[16], rxBuf[17]);

         // printf("Data: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", adcData.response, adcData.channel0, adcData.channel1, adcData.channel2, adcData.channel3);
         //double step = 1200000.0 / 8388607.0;
         //double v1, v2, v3, v4;

            //v1 = (double)(~(0xe00000));
            //printf("v1 %f.\n", v1);
            //v1 = 0.0 - v1;
            //printf("0-v1 %f.\n", v1);

         //printf("Data: 0x%x, %f, %f, %f, %f\n", adcData.response, v1, v2, v3, v4);
         #endif

         printf("Input Command: ");
         scanf("%s", command);
      }
      else
      {

         printf("\nIncorrect command! %s\n\n", command);
#if 0    
         printf("The commands should be:\n");
         printf("    \"GenWave\" to generate waveforms. \"ClsWave\" to stop waveforms.\n\n");

         printf("    \"SetDAC\" to set DAC output value.\n");
         printf("    \"SetChxIntTest\" to set ADC internal capture test. 0-nomal; 1-0;\n");
         printf("                      2-160mv; 3--160mv.\n");
         printf("    \"CaptureADC\" to capture ADC input.\n");
         printf("    \"Close\" to stop the application.\n\n");

         printf("    \"LdisStart\" to start the Laser Distence Sencor.\n\n");
         printf("    \"TecSetStart\" to start the Temperature Contoller.\n\n");
         printf("Input Command: ");
#else
      printf("  Please input the following command:\n"
            "    setTempPoint,\n"
            "    setDAC,\n"
            "    startWaveforms,\n"
            "    readADC,\n"
            "    getCalibConstant,\n"
            "    closeCalib,\n\n"
            "Input Command: ");
#endif
         scanf("%s", command);
         //break;
      }
   }

   //printf("close the pigpio. \n");
   printf("close the Manufactory Calibration. \n");

   gpioTerminate();

   fclose(fh); //close the file
   return 0;
}
#endif // end of main()

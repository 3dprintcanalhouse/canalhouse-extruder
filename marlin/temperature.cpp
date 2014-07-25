/*
  temperature.c - temperature control
  Part of Marlin
  
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)
 
 It has preliminary support for Matthew Roberts advance algorithm 
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html

 */


#include "Marlin.h"
#include "ultralcd.h"
#include "temperature.h"
#include "watchdog.h"

//===========================================================================
//=============================public variables============================
//===========================================================================
int target_temperature[EXTRUDERS] = { 0 };
int current_temperature_raw[EXTRUDERS] = { 0 };
unsigned long current_pressure_raw = 0;
float current_temperature[EXTRUDERS] = { 0 };
float current_pressure = 0;

#ifdef PIDTEMP
  float Kp=DEFAULT_Kp;
  float Ki=(DEFAULT_Ki*PID_dT);
  float Kd=(DEFAULT_Kd/PID_dT);
  #ifdef PID_ADD_EXTRUSION_RATE
    float Kc=DEFAULT_Kc;
  #endif
#endif //PIDTEMP
  
  
//===========================================================================
//=============================private variables============================
//===========================================================================
static volatile bool temp_meas_ready = false;

  static unsigned char soft_pwm[EXTRUDERS];
  
#if EXTRUDERS > 4
# error Unsupported number of extruders
#elif EXTRUDERS > 3
# define ARRAY_BY_EXTRUDERS(v1, v2, v3, v4) { v1, v2, v3, v4 }
#elif EXTRUDERS > 2
# define ARRAY_BY_EXTRUDERS(v1, v2, v3, v4) { v1, v2, v3 }
#elif EXTRUDERS > 1
# define ARRAY_BY_EXTRUDERS(v1, v2, v3, v4) { v1, v2 }
#else
# define ARRAY_BY_EXTRUDERS(v1, v2, v3, v4) { v1 }
#endif

// Init min and max temp with extreme values to prevent false errors during startup
static int minttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP , HEATER_3_RAW_LO_TEMP );
static int maxttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP , HEATER_3_RAW_HI_TEMP );
static int minttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 0, 0, 0 , 0 );
static int maxttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 16383, 16383, 16383 , 16383 );

static void *heater_ttbl_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS( (void *)HEATER_0_TEMPTABLE, (void *)HEATER_1_TEMPTABLE, (void *)HEATER_2_TEMPTABLE, (void *)HEATER_3_TEMPTABLE );
static uint8_t heater_ttbllen_map[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN , HEATER_3_TEMPTABLE_LEN );

static float analog2temp(int raw, uint8_t e);
static void updateTemperaturesFromRawValues();

#ifdef WATCH_TEMP_PERIOD
int watch_start_temp[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0,0,0,0);
unsigned long watchmillis[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0,0,0,0);
#endif //WATCH_TEMP_PERIOD

//===========================================================================
//=============================   functions      ============================
//===========================================================================
 
int getHeaterPower(int heater) {
  return soft_pwm[heater];
}

void manage_heater()
{
  float pid_output;
  uint8_t coolOutput;

  if(temp_meas_ready != true)   //better readability
    return; 

  updateTemperaturesFromRawValues();
    
  if (IsStopped() || millis() < 1000)
  {
    //On a critical error disable all relays.
    WRITE(EXTRUDER_ENABLE, 0);
    WRITE(MAIN_RELAY_0, 0);
    WRITE(MAIN_RELAY_1, 0);
  }else{
/* Moved to M80 for remote turn on */
//    WRITE(MAIN_RELAY_0, 1);
//    WRITE(MAIN_RELAY_1, 1);

// max pressure values, for original tube 190 bar, for new 2014-02-02 tube 100 bar
//    if (current_pressure > 190)
    if (current_pressure > 95)
    {
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM("Extruder switched off. Over Pressure triggered!!");
        LCD_ALERTMESSAGEPGM("Err: PRESSURE");
        Stop();
    }
    
    if (current_temperature[0] > 150 &&
        current_temperature[1] > 170 &&
        current_temperature[2] > 180 &&
        current_temperature[3] > 180)
    {
        WRITE(EXTRUDER_ENABLE, 1);
    }else{
        WRITE(EXTRUDER_ENABLE, 0);
    }
  }

  for(int e = 0; e < EXTRUDERS; e++) 
  {

    pid_output = 0;
    coolOutput = 0;
    if(current_temperature[e] < target_temperature[e] - 1) {
      pid_output = PID_MAX;
    }
    if(current_temperature[e] > target_temperature[e] + 1) {
      coolOutput = 1;
    }

    // Check if temperature is within the correct range
    if((current_temperature[e] > minttemp[e]) && (current_temperature[e] < maxttemp[e])) 
    {
      soft_pwm[e] = (int)pid_output >> 1;
    }
    else {
      soft_pwm[e] = 0;
      coolOutput = 0;
    }
    if (current_temperature[e] < 35)
        coolOutput = 0;
    if (coolOutput)
    {
        if (e == 0) WRITE(HEATER_0_COOL_PIN, 1);
        if (e == 1) WRITE(HEATER_1_COOL_PIN, 1);
        if (e == 2) WRITE(HEATER_2_COOL_PIN, 1);
        #if HEATER_3_COOL_PIN > -1
        if (e == 3) WRITE(HEATER_3_COOL_PIN, 1);
        #endif
    }else{
        if (e == 0) WRITE(HEATER_0_COOL_PIN, 0);
        if (e == 1) WRITE(HEATER_1_COOL_PIN, 0);
        if (e == 2) WRITE(HEATER_2_COOL_PIN, 0);
        #if HEATER_3_COOL_PIN > -1
        if (e == 3) WRITE(HEATER_3_COOL_PIN, 0);
        #endif
    }

    #ifdef WATCH_TEMP_PERIOD
    if(watchmillis[e] && millis() - watchmillis[e] > WATCH_TEMP_PERIOD)
    {
        if(degHotend(e) < watch_start_temp[e] + WATCH_TEMP_INCREASE)
        {
            setTargetHotend(0, e);
            LCD_MESSAGEPGM("Heating failed");
            SERIAL_ECHO_START;
            SERIAL_ECHOLN("Heating failed");
        }else{
            watchmillis[e] = 0;
        }
    }
    #endif

  } // End extruder for loop
}

#define PGM_RD_W(x)   (short)pgm_read_word(&x)
// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
static float analog2temp(int raw, uint8_t e) {
  if(e >= EXTRUDERS)
  {
      SERIAL_ERROR_START;
      SERIAL_ERROR((int)e);
      SERIAL_ERRORLNPGM(" - Invalid extruder number !");
      kill();
  } 
  #ifdef HEATER_0_USES_MAX6675
    if (e == 0)
    {
      return 0.25 * raw;
    }
  #endif

  if(heater_ttbl_map[e] != NULL)
  {
    float celsius = 0;
    uint8_t i;
    short (*tt)[][2] = (short (*)[][2])(heater_ttbl_map[e]);

    for (i=1; i<heater_ttbllen_map[e]; i++)
    {
      if (PGM_RD_W((*tt)[i][0]) > raw)
      {
        celsius = PGM_RD_W((*tt)[i-1][1]) + 
          (raw - PGM_RD_W((*tt)[i-1][0])) * 
          (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i-1][1])) /
          (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[e]) celsius = PGM_RD_W((*tt)[i-1][1]);

    return celsius;
  }
  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
}

/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues()
{
    for(uint8_t e=0;e<EXTRUDERS;e++)
    {
        current_temperature[e] = analog2temp(current_temperature_raw[e], e);
    }
    //Raw      = 0-1023*16/2 = 0-5V
    //Pressure = 0-(12/1000*2.5)*500 Bar
    current_pressure = (((float(current_pressure_raw) * 2 / OVERSAMPLENR / 1024) * 500.0) + PRESSURE_OFFSET) * PRESSURE_GAIN;

    //Reset the watchdog after we know we have a temperature measurement.
    watchdog_reset();

    CRITICAL_SECTION_START;
    temp_meas_ready = false;
    CRITICAL_SECTION_END;
}

void tp_init()
{
  // Finish init of mult extruder arrays 
  for(int e = 0; e < EXTRUDERS; e++) {
    // populate with the first value 
    maxttemp[e] = maxttemp[0];
  }
  SET_OUTPUT(EXTRUDER_ENABLE);
  SET_OUTPUT(MAIN_RELAY_0);
  SET_OUTPUT(MAIN_RELAY_1);
  WRITE(EXTRUDER_ENABLE, 0);
  WRITE(MAIN_RELAY_0, 0);
  WRITE(MAIN_RELAY_1, 0);

  #if (HEATER_0_PIN > -1) 
    SET_OUTPUT(HEATER_0_PIN);
  #endif  
  #if (HEATER_1_PIN > -1) 
    SET_OUTPUT(HEATER_1_PIN);
  #endif  
  #if (HEATER_2_PIN > -1) 
    SET_OUTPUT(HEATER_2_PIN);
  #endif  
  #if (HEATER_3_PIN > -1) 
    SET_OUTPUT(HEATER_3_PIN);
  #endif  

  #ifdef HEATER_0_USES_MAX6675
    #ifndef SDSUPPORT
      SET_OUTPUT(MAX_SCK_PIN);
      WRITE(MAX_SCK_PIN,0);
    
      SET_OUTPUT(MAX_MOSI_PIN);
      WRITE(MAX_MOSI_PIN,1);
    
      SET_INPUT(MAX_MISO_PIN);
      WRITE(MAX_MISO_PIN,1);
    #endif
    
    SET_OUTPUT(MAX6675_SS);
    WRITE(MAX6675_SS,1);
  #endif

  // Set analog inputs
  ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADIF | 0x07;
  DIDR0 = 0;
  #ifdef DIDR2
    DIDR2 = 0;
  #endif
  #if (TEMP_0_PIN > -1)
    #if TEMP_0_PIN < 8
       DIDR0 |= 1 << TEMP_0_PIN; 
    #else
       DIDR2 |= 1<<(TEMP_0_PIN - 8); 
    #endif
  #endif
  #if (TEMP_1_PIN > -1)
    #if TEMP_1_PIN < 8
       DIDR0 |= 1<<TEMP_1_PIN; 
    #else
       DIDR2 |= 1<<(TEMP_1_PIN - 8); 
    #endif
  #endif
  #if (TEMP_2_PIN > -1)
    #if TEMP_2_PIN < 8
       DIDR0 |= 1 << TEMP_2_PIN; 
    #else
       DIDR2 = 1<<(TEMP_2_PIN - 8); 
    #endif
  #endif
  #if (TEMP_3_PIN > -1)
    #if TEMP_3_PIN < 8
       DIDR0 |= 1<<TEMP_3_PIN; 
    #else
       DIDR2 |= 1<<(TEMP_3_PIN - 8); 
    #endif
  #endif
  
  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  OCR0B = 128;
  TIMSK0 |= (1<<OCIE0B);  
  
  // Wait for temperature measurement to settle
  delay(250);

#ifdef HEATER_0_MINTEMP
  minttemp[0] = HEATER_0_MINTEMP;
  while(analog2temp(minttemp_raw[0], 0) < HEATER_0_MINTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    minttemp_raw[0] += OVERSAMPLENR;
#else
    minttemp_raw[0] -= OVERSAMPLENR;
#endif
  }
#endif //MINTEMP
#ifdef HEATER_0_MAXTEMP
  maxttemp[0] = HEATER_0_MAXTEMP;
  while(analog2temp(maxttemp_raw[0], 0) > HEATER_0_MAXTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    maxttemp_raw[0] -= OVERSAMPLENR;
#else
    maxttemp_raw[0] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP

#if (EXTRUDERS > 1) && defined(HEATER_1_MINTEMP)
  minttemp[1] = HEATER_1_MINTEMP;
  while(analog2temp(minttemp_raw[1], 1) > HEATER_1_MINTEMP) {
#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
    minttemp_raw[1] += OVERSAMPLENR;
#else
    minttemp_raw[1] -= OVERSAMPLENR;
#endif
  }
#endif // MINTEMP 1
#if (EXTRUDERS > 1) && defined(HEATER_1_MAXTEMP)
  maxttemp[1] = HEATER_1_MAXTEMP;
  while(analog2temp(maxttemp_raw[1], 1) > HEATER_1_MAXTEMP) {
#if HEATER_1_RAW_LO_TEMP < HEATER_1_RAW_HI_TEMP
    maxttemp_raw[1] -= OVERSAMPLENR;
#else
    maxttemp_raw[1] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP 1

#if (EXTRUDERS > 2) && defined(HEATER_2_MINTEMP)
  minttemp[2] = HEATER_2_MINTEMP;
  while(analog2temp(minttemp_raw[2], 2) > HEATER_2_MINTEMP) {
#if HEATER_2_RAW_LO_TEMP < HEATER_2_RAW_HI_TEMP
    minttemp_raw[2] += OVERSAMPLENR;
#else
    minttemp_raw[2] -= OVERSAMPLENR;
#endif
  }
#endif //MINTEMP 2
#if (EXTRUDERS > 2) && defined(HEATER_2_MAXTEMP)
  maxttemp[2] = HEATER_2_MAXTEMP;
  while(analog2temp(maxttemp_raw[2], 2) > HEATER_2_MAXTEMP) {
#if HEATER_2_RAW_LO_TEMP < HEATER_2_RAW_HI_TEMP
    maxttemp_raw[2] -= OVERSAMPLENR;
#else
    maxttemp_raw[2] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP 2

#if (EXTRUDERS > 3) && defined(HEATER_3_MAXTEMP)
  maxttemp[3] = HEATER_3_MAXTEMP;
  while(analog2temp(maxttemp_raw[3], 3) > HEATER_3_MAXTEMP) {
#if HEATER_3_RAW_LO_TEMP < HEATER_3_RAW_HI_TEMP
    maxttemp_raw[3] -= OVERSAMPLENR;
#else
    maxttemp_raw[3] += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP 3
}

void setWatch() 
{  
#ifdef WATCH_TEMP_PERIOD
  for (int e = 0; e < EXTRUDERS; e++)
  {
    if(degHotend(e) < degTargetHotend(e) - (WATCH_TEMP_INCREASE * 2))
    {
      watch_start_temp[e] = degHotend(e);
      watchmillis[e] = millis();
    } 
  }
#endif 
}


void disable_heater()
{
  for(int i=0;i<EXTRUDERS;i++)
    setTargetHotend(0,i);
  #if TEMP_0_PIN > -1
  target_temperature[0]=0;
  soft_pwm[0]=0;
   #if HEATER_0_PIN > -1  
     WRITE(HEATER_0_PIN,LOW);
   #endif
  #endif
     
  #if TEMP_1_PIN > -1
    target_temperature[1]=0;
    soft_pwm[1]=0;
    #if HEATER_1_PIN > -1 
      WRITE(HEATER_1_PIN,LOW);
    #endif
  #endif
      
  #if TEMP_2_PIN > -1
    target_temperature[2]=0;
    soft_pwm[2]=0;
    #if HEATER_2_PIN > -1  
      WRITE(HEATER_2_PIN,LOW);
    #endif
  #endif 

  #if TEMP_3_PIN > -1
    target_temperature[3]=0;
    soft_pwm[3]=0;
    #if HEATER_3_PIN > -1  
      WRITE(HEATER_3_PIN,LOW);
    #endif
  #endif 
}

void max_temp_error(uint8_t e) {
  disable_heater();
  if(IsStopped() == false) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN((int)e);
    SERIAL_ERRORLNPGM(": Extruder switched off. MAXTEMP triggered !");
    LCD_ALERTMESSAGEPGM("Err: MAXTEMP");
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop();
  #endif
}

void min_temp_error(uint8_t e) {
  disable_heater();
  if(IsStopped() == false) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN((int)e);
    SERIAL_ERRORLNPGM(": Extruder switched off. MINTEMP triggered !");
    LCD_ALERTMESSAGEPGM("Err: MINTEMP");
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop();
  #endif
}

#ifdef HEATER_0_USES_MAX6675
#define MAX6675_HEAT_INTERVAL 250
long max6675_previous_millis = -HEAT_INTERVAL;
int max6675_temp = 2000;

int read_max6675()
{
  if (millis() - max6675_previous_millis < MAX6675_HEAT_INTERVAL) 
    return max6675_temp;
  
  max6675_previous_millis = millis();
  max6675_temp = 0;
    
  #ifdef	PRR
    PRR &= ~(1<<PRSPI);
  #elif defined PRR0
    PRR0 &= ~(1<<PRSPI);
  #endif
  
  SPCR = (1<<MSTR) | (1<<SPE) | (1<<SPR0);
  
  // enable TT_MAX6675
  WRITE(MAX6675_SS, 0);
  
  // ensure 100ns delay - a bit extra is fine
  asm("nop");//50ns on 20Mhz, 62.5ns on 16Mhz
  asm("nop");//50ns on 20Mhz, 62.5ns on 16Mhz
  
  // read MSB
  SPDR = 0;
  for (;(SPSR & (1<<SPIF)) == 0;);
  max6675_temp = SPDR;
  max6675_temp <<= 8;
  
  // read LSB
  SPDR = 0;
  for (;(SPSR & (1<<SPIF)) == 0;);
  max6675_temp |= SPDR;
  
  // disable TT_MAX6675
  WRITE(MAX6675_SS, 1);

  if (max6675_temp & 4) 
  {
    // thermocouple open
    max6675_temp = 2000;
  }
  else 
  {
    max6675_temp = max6675_temp >> 3;
  }

  return max6675_temp;
}
#endif


// Timer 0 is shared with millies
ISR(TIMER0_COMPB_vect)
{
  //these variables are only accesible from the ISR, but static, so they don't loose their value
  static unsigned char temp_count = 0;
  static unsigned long raw_temp_0_value = 0;
  static unsigned long raw_temp_1_value = 0;
  static unsigned long raw_temp_2_value = 0;
  static unsigned long raw_temp_3_value = 0;
  static unsigned long raw_pressure_value = 0;
  static unsigned char temp_state = 0;
  
    if(soft_pwm[0] > 0) WRITE(HEATER_0_PIN,1); else WRITE(HEATER_0_PIN,0);
    if(soft_pwm[1] > 0) WRITE(HEATER_1_PIN,1); else WRITE(HEATER_1_PIN,0);
    if(soft_pwm[2] > 0) WRITE(HEATER_2_PIN,1); else WRITE(HEATER_2_PIN,0);
    if(soft_pwm[3] > 0) WRITE(HEATER_3_PIN,1); else WRITE(HEATER_3_PIN,0);
  
  switch(temp_state) {
    case 0: // Prepare TEMP_0
      #if (TEMP_0_PIN > -1)
        #if TEMP_0_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_0_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 1;
      break;
    case 1: // Measure TEMP_0
      #if (TEMP_0_PIN > -1)
        raw_temp_0_value += ADC;
      #endif
      #ifdef HEATER_0_USES_MAX6675 // TODO remove the blocking
        raw_temp_0_value = read_max6675();
      #endif
      temp_state = 2;
      break;
    case 2: // Prepare TEMP_3_PIN
      #if (TEMP_3_PIN > -1)
        #if TEMP_3_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_3_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 3;
      break;
    case 3: // Measure TEMP_BED
      #if (TEMP_3_PIN > -1)
        raw_temp_3_value += ADC;
      #endif
      temp_state = 4;
      break;
    case 4: // Prepare TEMP_1
      #if (TEMP_1_PIN > -1)
        #if TEMP_1_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_1_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 5;
      break;
    case 5: // Measure TEMP_1
      #if (TEMP_1_PIN > -1)
        raw_temp_1_value += ADC;
      #endif
      temp_state = 6;
      break;
    case 6: // Prepare TEMP_2
      #if (TEMP_2_PIN > -1)
        #if TEMP_2_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_2_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 7;
      break;
    case 7: // Measure TEMP_2
      #if (TEMP_2_PIN > -1)
        raw_temp_2_value += ADC;
      #endif
      temp_state = 8;
      break;
    case 8: // Prepare pressure
      ADCSRB = 0;
      ADMUX = ((1 << REFS0) | (PRESSURE_PIN & 0x07));
      ADCSRA |= 1<<ADSC; // Start conversion
      
      lcd_buttons_update();
      temp_state = 9;
      break;
    case 9: // Measure pressure
      raw_pressure_value += ADC / 2;
      temp_state = 0;
      temp_count++;
      break;
//    default:
//      SERIAL_ERROR_START;
//      SERIAL_ERRORLNPGM("Temp measurement error!");
//      break;
  }
    
  if(temp_count >= 16) // 8 ms * 16 = 128ms.
  {
    if (!temp_meas_ready) //Only update the raw values if they have been read. Else we could be updating them during reading.
    {
      current_temperature_raw[0] = raw_temp_0_value;
#if EXTRUDERS > 1
      current_temperature_raw[1] = raw_temp_1_value;
#endif
#if EXTRUDERS > 2
      current_temperature_raw[2] = raw_temp_2_value;
#endif
#if EXTRUDERS > 3
      current_temperature_raw[3] = raw_temp_3_value;
#endif
      current_pressure_raw = raw_pressure_value;
    }
    
    temp_meas_ready = true;
    temp_count = 0;
    raw_temp_0_value = 0;
    raw_temp_1_value = 0;
    raw_temp_2_value = 0;
    raw_temp_3_value = 0;
    raw_pressure_value = 0;
    
    if (millis() < 1000) return;

#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
    if(current_temperature_raw[0] <= maxttemp_raw[0]) {
#else
    if(current_temperature_raw[0] >= maxttemp_raw[0]) {
#endif
        max_temp_error(0);
    }
#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
    if(current_temperature_raw[0] >= minttemp_raw[0]) {
#else
    if(current_temperature_raw[0] <= minttemp_raw[0]) {
#endif
        min_temp_error(0);
    }
#if EXTRUDERS > 1
#if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
    if(current_temperature_raw[1] <= maxttemp_raw[1]) {
#else
    if(current_temperature_raw[1] >= maxttemp_raw[1]) {
#endif
        max_temp_error(1);
    }
#if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
    if(current_temperature_raw[1] >= minttemp_raw[1]) {
#else
    if(current_temperature_raw[1] <= minttemp_raw[1]) {
#endif
        min_temp_error(1);
    }
#endif
#if EXTRUDERS > 2
#if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
    if(current_temperature_raw[2] <= maxttemp_raw[2]) {
#else
    if(current_temperature_raw[2] >= maxttemp_raw[2]) {
#endif
        max_temp_error(2);
    }
#if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
    if(current_temperature_raw[2] >= minttemp_raw[2]) {
#else
    if(current_temperature_raw[2] <= minttemp_raw[2]) {
#endif
        min_temp_error(2);
    }
#endif
#if EXTRUDERS > 3
#if HEATER_3_RAW_LO_TEMP > HEATER_3_RAW_HI_TEMP
    if(current_temperature_raw[3] <= maxttemp_raw[3]) {
#else
    if(current_temperature_raw[3] >= maxttemp_raw[3]) {
#endif
        max_temp_error(3);
    }
#if HEATER_3_RAW_LO_TEMP > HEATER_3_RAW_HI_TEMP
    if(current_temperature_raw[3] >= minttemp_raw[3]) {
#else
    if(current_temperature_raw[3] <= minttemp_raw[3]) {
#endif
        min_temp_error(3);
    }
#endif
  
  }  
}

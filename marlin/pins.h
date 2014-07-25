#ifndef PINS_H
#define PINS_H

#define KNOWN_BOARD
/*****************************************************************
* Ultimaker pin assignment
******************************************************************/

#ifndef __AVR_ATmega1280__
 #ifndef __AVR_ATmega2560__
 #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
 #endif
#endif

#define EXTRUDER_ENABLE 4
#define MAIN_RELAY_0    2
#define MAIN_RELAY_1    3

#define HEATER_0_PIN  35
#define HEATER_0_COOL_PIN  27
#define TEMP_0_PIN 8 /* analog pin */

#define HEATER_1_PIN 37
#define HEATER_1_COOL_PIN  25
#define TEMP_1_PIN 9 /* analog pin */

#define HEATER_2_PIN 39
#define HEATER_2_COOL_PIN  23
#define TEMP_2_PIN 10 /* analog pin */

#define HEATER_3_PIN 7
#define HEATER_3_COOL_PIN  -1
#define TEMP_3_PIN 1 /* analog pin */

#define PRESSURE_PIN 0 /* analog pin */

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            -1
#define PS_ON_PIN          -1
#define KILL_PIN           -1

#ifdef ULTRA_LCD

  //arduino pin witch triggers an piezzo beeper
    #define BEEPER 18

    #define LCD_PINS_RS 20 
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 16
    #define LCD_PINS_D5 21 
    #define LCD_PINS_D6 5
    #define LCD_PINS_D7 6
    
    //buttons are directly attached
    #define BTN_EN1 40
    #define BTN_EN2 42
    #define BTN_ENC 19  //the click
    
    #define BLEN_C 2
    #define BLEN_B 1
    #define BLEN_A 0
    
    #define SDCARDDETECT 38
    
      //encoder rotation values
    #define encrot0 0
    #define encrot1 2
    #define encrot2 3
    #define encrot3 1
#endif //ULTRA_LCD

#define SENSITIVE_PINS {}
#endif

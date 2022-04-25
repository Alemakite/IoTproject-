////////////////////////////////////////////////////////////////
// Advanced Lights controller
// Arduino Nano
// Name / Date
// Piotr Kosek / 08.04.2022 
////////////////////////////////////////////////////////////////

//RESOURCE MANAGER MODULE
#define GRANTED A2
#define DEMAND A6

#define GRANTED_HI pinMode(GRANTED,INPUT);

#define GRANTED_LO digitalWrite(GRANTED, LOW); \
                   pinMode(GRANTED, OUTPUT);

static char ardRMstate;
static bool ardTrigger;
#define noTriggerNoDemand 0 // idle state
#define triggerMaster 1     // trigger detected, there was no demand 
#define noTriggerSlave 2    // there was a demand, it is granted 

enum button
{
  notPressed,
  partialPressed,
  normalPressed
};

button _B1_state; //variable of the button type
button _B2_state;

#define _B1 B00000001 //button 1
#define _B2 B00000010 //button 2
#define notPress 0
#define partPress 1
#define normalPress 2

//3-colour led 
#define TRI_RED 9
#define TRI_BLUE 10
#define TRI_GREEN 11
#define rampUp 0
#define dimDown 1
#define Off 0
#define Amber 1
#define Blue 2
#define Green 3
#define Red 4

//HEARTBEAT, COUNTER AND ACCELEROMETER MODULE

#define char_t  B01111000
#define char_E  B01111001 
#define char_1  B00000110
#define char_2  B01011011
#define char_B  B01001111
#define char_F  B01100110

#define LATCH B00100000
#define DATA B00000001
#define CLOCK B00010000

#include <Wire.h>
#define PWR_MGMT_1 0x6B //power menegment register on the accelerometer
#define MPU_6050 0x68   //accelerometer's proccesor
#define ACCEL_XOUT_HI 0x3B  //X axis register
#define ACCEL_YOUT_HI 0x3D  //Y axis register
#define ACCEL_ZOUT_HI 0x3F  //Z axis register
#define TEMP_OUT_H 0x41


//TRAFFIC LIGTHS MODULE
#define led_config B11111100
#define set_reset B00000000

#define light_set0 B01001000
#define light_set1 B00100100
#define light_set2 B00101100
#define light_set3 B00110000
#define light_set4 B00101000
#define light_set5 B00100100
#define light_set6 B01100100
#define light_set7 B10000100
#define light_set8 B01000100

//MAINTENANCE MODULE 
#define maintenance B01001000

//BARRIER CROSSING MODULE
#define barrier_cross_set0 B00000000
#define barrier_cross_set1 B01001000 
#define barrier_cross_set2 B00100100
#define barrier_cross_set3 B00000100
#define barrier_cross_set4 B00100000

//MOTOR RACE MODULE
#define motor_race_reset B00000000
#define motor_race_set1 B00000100
#define motor_race_set2 B00001100
#define motor_race_set3 B00011100
#define motor_race_set4 B00111100
#define motor_race_set5 B01111100
#define motor_race_set7 B10010000
#define motor_race_red_flag B00100100
#define motor_race_abort1 B01001000

enum priority
{
    equal,
    set1,
    set2,
    turnOff
};

priority priorityMode;

static byte displayState, mask;
static byte beat_byte = B10000000;

static char traffic_state;
static char triLED_state;

enum triLEDcombo
{ 
    off,
    amber,
    blue,
    green,
    red,
};

triLEDcombo lights;

bool init_triLights_clock;
bool init_Traffic_Lights_clock;
bool init_scheduler_clock;
bool init_Heartbeat_clock;
bool init_counter_clock;
bool init_barrier_cross_clock;
bool init_B1_debounce_clock;
bool init_B2_debounce_clock;
bool init_acc_clock;
bool init_motor_race_clock;
bool init_triLED_clock;

bool demandRequest()
{
  return (!(analogRead(DEMAND) >> 8)); 
}

void setup()
{
  Serial.begin(9600);

  DDRB |= B00110001;  //DATA, CLOCK and LATCH pin config

  PORTB |= B00110000;
  PORTB = B00000000; //toggling the LATCH and CLOCK

  displayState = B00000000;

  DDRB |= B00001110; //triLED setup
  
  DDRC |= B00000011;  //button setup
  
  PORTC |= _B1; //apply internal pullup
  PORTC |= _B2;

  _B1_state = notPressed;
  _B2_state = notPressed;

  DDRD |= led_config; //initialise all led pins to be outputs
  PORTD &= set_reset; //turn the led pins off

  ardTrigger = false;
  Wire.begin(8); //Accelerometer I2c - setup
  Wire.onReceive(receiveEvent);

   /*
  Wire.beginTransmission(MPU_6050); //MPU mode - setup
  Wire.write(PWR_MGMT_1);
  Wire.write(0); //awaken the MPU
  Wire.endTransmission(true);
  */
  
  init_Heartbeat_clock = true;
  init_scheduler_clock = true;
  init_Traffic_Lights_clock = true;
  init_barrier_cross_clock = true;
  init_motor_race_clock = true;
  init_counter_clock = true;
  init_B1_debounce_clock = true;
  init_B2_debounce_clock = true;
  init_acc_clock = true;
  init_triLED_clock = true;
  init_triLights_clock = true;
}

void loop()
{
    //RESOURCE MANAGER MODULE
      switch(ardRMstate)
      {
        case noTriggerNoDemand:
          GRANTED_HI;
          if(ardTrigger) 
            ardRMstate = triggerMaster;
          else if(demandRequest()) 
            ardRMstate = noTriggerSlave;
          else 
            ardRMstate = noTriggerNoDemand;
        break;

        case triggerMaster:
         GRANTED_LO;
           if(ardTrigger)
            ardRMstate = triggerMaster;    
          else 
            ardRMstate = noTriggerSlave;
        break;

        case noTriggerSlave:
          GRANTED_LO;
          if(demandRequest())
            ardRMstate = noTriggerSlave;
          else 
            ardRMstate = noTriggerNoDemand;
        break;
      }
      
    //module 0 - SCHEDULER
    {
        static unsigned long scheduler_time;

        static unsigned int  scheduler_delay; //this may be converted to long nevertheless

        static bool scheduler_execute; //while true, executethe module
        if (init_scheduler_clock)
        {
            scheduler_delay = 10;
            scheduler_time = millis();
            scheduler_execute = false;
            init_scheduler_clock = false;
         
        }
        else if ((long)(millis() - scheduler_time) > scheduler_delay)  //if delay time exceeded, begin execution with safe roll_over
        {
            scheduler_time = millis();
            scheduler_execute = true;
            init_Heartbeat_clock = false;
        }
        else
            scheduler_execute = false;

        if (scheduler_execute)
        {
            switch (traffic_state)
            {
            case 97:
                init_Traffic_Lights_clock = false;
                init_barrier_cross_clock = true;
                init_motor_race_clock = true;
                displayState = (B10000000 & displayState) | char_E; //display heartbeat along with the t
                break;
            case 98:
                init_Traffic_Lights_clock = false;
                init_barrier_cross_clock = true;
                init_motor_race_clock = true;
                displayState = (B10000000 & displayState) | char_1;
                break;
            case 99:
                init_Traffic_Lights_clock = false;
                init_barrier_cross_clock = true;
                init_motor_race_clock = true;
                displayState = (B10000000 & displayState) | char_2; 
                break;
            case 100:
                init_Traffic_Lights_clock = false;
                init_barrier_cross_clock = true;
                init_motor_race_clock = true;
                displayState = (B10000000 & displayState) | char_t;
                break;
            case 101:
                init_Traffic_Lights_clock = true;
                init_barrier_cross_clock = false;
                init_motor_race_clock = true;
                displayState = (B10000000 & displayState) | char_B;
                break;
            case 102:
                init_Traffic_Lights_clock = true;
                init_barrier_cross_clock = true;
                init_motor_race_clock = false;
                displayState = (B10000000 & displayState) | char_F;
                break;
            //default: traffic_state = 'a';
            }
        }
    }

    //3-colour LED scheduler
    {
      static bool triLED_execute;
      static unsigned long triLED_delay, triLED_time;
  
      if (init_triLED_clock)
        {
          triLED_time = millis();
          triLED_delay = 995;
          triLED_execute = true;
        }
      else if (((long)(millis() - triLED_time)) > triLED_delay)
        {
            triLED_time = millis();
            triLED_execute = true;
            init_triLights_clock = false;
        }
      else 
          triLED_execute = false;
  
      if (triLED_execute)
      {
        switch(triLED_state)
        { 
          case 103:
              lights = off;
              break;
          case 104:
              lights = amber;
              break;
          case 105:
              lights = blue;
              break;
          case 106:
              lights = green;
              break;
          case 107:
              lights = red;
              break;
          default: lights = off; break;
        }
      }
    }
    
    // module 1 - HEARTBEAT
    {
        static unsigned long heartbeat_time, heartbeat_switch;

        static unsigned int  heartbeat_duration;;

        static bool heartbeat_execute; //while true, execute module

        static unsigned char heartbeat_state;
        if (init_Heartbeat_clock)
        {
            heartbeat_duration = 500;
            heartbeat_time = millis();
            heartbeat_execute = false;
        }
        else if ((long)(millis() - heartbeat_time) > heartbeat_duration)
            {
                heartbeat_time = millis();
                heartbeat_execute = true;
            }
        else
                heartbeat_execute = false;

        if (heartbeat_execute)
        {
            switch (heartbeat_state)
            {
            case 0:
                displayState |= beat_byte;
                heartbeat_state = 1;
                break;

            case 1:
                displayState &= ~beat_byte;
                heartbeat_state = 0;
                break;

            default: heartbeat_state = 0; break;
            }
        }
    }

    //module 2 - MULTI PURPOSE TRAFFIC LIGHTS CONTROLLER
    {
        static unsigned long traffic_lights_time, maitenance_switch;

        static unsigned int traffic_lights_duration, module1_delay;

        static bool traffic_lights_execute, set1Active, set2Active;

        static unsigned char traffic_lights_state; //state variable for a module
        
        if (init_Traffic_Lights_clock)
        {
            module1_delay = 10;
            traffic_lights_time = millis();
            traffic_lights_execute = false;
            traffic_lights_state = 0;
            set1Active = false;
            set2Active = false;
        }
        
        else if ((long)(millis() - traffic_lights_time) > module1_delay)  //if delay time exceeded, begin execution with safe roll_over
            {
                traffic_lights_time = millis(); //time update, not sure
                traffic_lights_execute = true;
            }
        else
                traffic_lights_execute = false;
                
        if (traffic_lights_execute)
        {
                  switch (traffic_lights_state)
                  {
                  case 0:
                    set1Active = false;
                    set2Active = false;
                    PORTD &= set_reset;
                    PORTD |= light_set0;
                    module1_delay = 2000;
                    traffic_lights_state = 1;
                  break;
  
                  case 1:
                    if(traffic_state == 'a')
                      priorityMode = equal;
                    else if(traffic_state == 'b')
                      priorityMode = set1;
                    else if(traffic_state == 'c')
                      priorityMode = set2;
                    else if(traffic_state == 'd') 
                    {
                      priorityMode = turnOff;
                      traffic_lights_state = 9;
                      break;
                    }
                    set1Active = true;
                    set2Active = false;
                    PORTD &= set_reset;
                    module1_delay = 1000;
                    PORTD |= light_set1;
                    traffic_lights_state = 2;
                    break;
  
                  case 2:
                    set1Active = true;
                    set2Active = false;
                    PORTD &= set_reset;       
                    module1_delay = 1000;
                    PORTD |= light_set2;
                    traffic_lights_state = 3;
                    break;
  
                  case 3:
                    set1Active = true;
                    set2Active = false;
                    PORTD &= set_reset;
                    if (priorityMode == equal)
                        module1_delay = 5000;
                    else if (priorityMode == set1)
                        module1_delay = 7000;
                    else
                        module1_delay = 3000;
                        
                    PORTD |= light_set3;
                    traffic_lights_state = 4;
                    break;
  
                  case 4:
                    set1Active = true;
                    set2Active = false;
                    PORTD &= set_reset;
                    module1_delay = 2000;
                    PORTD |= light_set4;
                    traffic_lights_state = 5;
                    break;
  
                  case 5:
                    set1Active = false;
                    set2Active = true;
                    PORTD &= set_reset;
                    module1_delay = 1000;
                    PORTD |= light_set5;
                    traffic_lights_state = 6;
                    break;
  
                  case 6:
                    set1Active = false;
                    set2Active = true;
                    PORTD &= set_reset;
                    module1_delay = 1000;    
                    PORTD |= light_set6;
                    traffic_lights_state = 7;
                    break;
  
                  case 7:
                    set1Active = false;
                    set2Active = true;
                    PORTD &= set_reset;
                    if (priorityMode == equal)
                      module1_delay = 5000;
                    else if (priorityMode == set1)
                      module1_delay = 3000;
                    else
                      module1_delay = 7000;
                      
                    PORTD |= light_set7;
                    traffic_lights_state = 8;
                    break;
  
                  case 8:
                    set1Active = false;
                    set2Active = true;
                    PORTD &= set_reset;
                    module1_delay = 1000;    
                    PORTD |= light_set8;
                    traffic_lights_state = 0;
                    break;
                  case 9:
                    set1Active = false;
                    set2Active = false;
                    PORTD &= set_reset;
                    module1_delay = 500;
                    PORTD |= maintenance;
                    traffic_lights_state = 10;
                    break;

                  case 10:
                    set1Active = false;
                    set2Active = false;
                    PORTD &= set_reset;
                    if(traffic_state == 'a' || traffic_state == 'b' || traffic_state == 'c')
                      { 
                        traffic_lights_state = 1;
                        break;
                      }
                    else
                    {
                    module1_delay = 500; 
                    traffic_lights_state = 9;
                    break;
                    }
                  }
         }
    }
        
    //module 3 - BARRIER CROSSING
    {
        static unsigned long barrier_cross_time;

        static unsigned int barrier_cross_duration, barrier_cross_delay, _SW1_PressCount, _SW2_PressCount;

        static bool barrier_cross_execute;

        static unsigned char barrier_cross_state, _SW1_PressCount_state, _SW2_PressCount_state;
        if (init_barrier_cross_clock)
        {
            barrier_cross_delay = 10;
            barrier_cross_time = millis();
            barrier_cross_execute = false;
            barrier_cross_state = 0;
            //PORTD &= ~led_config;  //set LEDs off
        }

        else if ((long)(millis() - barrier_cross_time) > barrier_cross_delay)  //if delay time exceeded, begin execution with safe roll_over
            {
                barrier_cross_time = millis(); //time update, not sure
                barrier_cross_execute = true;
            }
        else
                barrier_cross_execute = false;
                
        if(barrier_cross_execute)
        {
            switch(barrier_cross_state)
            {
              case 0:
                PORTD &= set_reset;
                PORTD |= barrier_cross_set0;
                if(_SW1_PressCount > 0)
                {
                  barrier_cross_state = 1;
                }
                else
                {
                  barrier_cross_state = 0;
                }
                break;

              case 1:
                PORTD &= set_reset;
                PORTD |= barrier_cross_set1;
                barrier_cross_delay = 5000; 
                barrier_cross_state = 2;
                break;

              case 2:
                PORTD &= set_reset; 
                PORTD |= barrier_cross_set2;
                barrier_cross_delay = 500;  
                barrier_cross_state = 3;
                break;

              case 3:
                PORTD &= set_reset;
                PORTD |= barrier_cross_set3;
                barrier_cross_delay = 500;  
                barrier_cross_state = 4;
                break;

              case 4:
                PORTD &= set_reset;
                PORTD |= barrier_cross_set4;
                if(_SW2_PressCount == _SW1_PressCount)
                {
                  _SW2_PressCount = 0;
                  _SW1_PressCount = 0;
                  barrier_cross_delay = 10;  
                  barrier_cross_state = 0;
                }
                else
                {
                  barrier_cross_delay = 500;  
                  barrier_cross_state = 3;
                }
              break; 
            }
        }
            switch(_SW1_PressCount_state)
            {
              case 0:
                if(_B1_state == normalPressed)
                  {
                      _SW1_PressCount++;
                      _SW1_PressCount_state = 1;
                  }
                else
                      _SW1_PressCount_state = 0;
              break;

              case 1:
                if(_B1_state == notPressed)
                   {
                      _SW1_PressCount_state = 0;
                   }
                else
                      _SW1_PressCount_state = 1;
              break;
              default: _SW1_PressCount_state = 0;
            }

             switch(_SW2_PressCount_state)
            {
              case 0:
              if(barrier_cross_state > 2)
              {
                if(_B2_state == normalPressed)
                  {
                      _SW2_PressCount++;
                      _SW2_PressCount_state = 1;
                  }
              }
              else
                _SW2_PressCount_state = 0;
              break;

              case 1:
                if(_B2_state == notPressed)
                   {
                      _SW2_PressCount_state = 0;
                   }
                else
                      _SW2_PressCount_state = 1;
              break;
              default: _SW2_PressCount_state = 0;
                  
            }
    }

    //module 3 - MOTOR RACE
    {
        static unsigned long motor_race_time;

        static unsigned int motor_race_duration, motor_race_delay, _SW1_PressCount, _SW2_PressCount;

        static bool motor_race_execute, _SW1_isPressed,  _SW2_isPressed; //while true, execute module

        static unsigned char motor_race_state, _SW1_PressCount_state, _SW2_PressCount_state;
        if (init_motor_race_clock)
        {
            motor_race_delay = 10;
            motor_race_time = millis();
            motor_race_execute = false;
            _SW1_isPressed = false;
            _SW2_isPressed = false;
            motor_race_state = 0;
            //PORTD &= ~led_config;  //set LEDs off
        }

        else if ((long)(millis() - motor_race_time) > motor_race_delay)  //if delay time exceeded, begin execution with safe roll_over
            {
                motor_race_time = millis(); //time update, not sure
                motor_race_execute = true;
            }
        else
                motor_race_execute = false;
                
        if(motor_race_execute)
        {
            switch(motor_race_state)
            {
              case 0:
                PORTD &= set_reset;
                if(_SW1_isPressed)
                {
                  _SW1_isPressed = false;
                  motor_race_state = 1;
                }
                else
                {
                  motor_race_state = 0;
                }
              break;

              case 1:
                PORTD &= set_reset;
                PORTD |= motor_race_set1;
                if(_SW1_isPressed)
                {
                  _SW1_isPressed = false;
                   motor_race_state = 9;
                }
                else
                {
                  motor_race_delay = 1000;
                  motor_race_state = 2;
                }
              break;

              case 2:
                PORTD &= set_reset; 
                PORTD |= motor_race_set2;
                if(_SW1_isPressed)
                {
                  _SW1_isPressed = false;
                   motor_race_state = 9;
                }
                else
                {
                  motor_race_delay = 1000;
                  motor_race_state = 3;
                }
              break;

              case 3:
                PORTD &= set_reset;
                PORTD |= motor_race_set3;
                if(_SW1_isPressed)
                {
                  _SW1_isPressed = false;
                   motor_race_state = 9;
                }
                else
                {
                  motor_race_delay = 1000;
                  motor_race_state = 4;
                }
              break;

              case 4:
                PORTD &= set_reset;
                PORTD |= motor_race_set4;
                if(_SW1_isPressed)
                {
                   _SW1_isPressed = false;
                   motor_race_state = 9;
                }
                else
                {
                  motor_race_delay = 1000;
                  motor_race_state = 5;
                }
              break;
              
              case 5:
                PORTD &= set_reset;
                PORTD |= motor_race_set5;
                if(_SW1_isPressed)
                {
                   _SW1_isPressed = false;
                   motor_race_state = 9;
                }
                else
                {
                   motor_race_delay = random(2000, 5000);  
                   motor_race_state = 6;
                }
              break;
              
              case 6:
                PORTD &= set_reset;
                if(_SW1_isPressed)
                {
                   _SW1_isPressed = false;
                   motor_race_state = 8;
                }
                else
                {
                  motor_race_delay = 2000;  
                  motor_race_state = 7;
                }
              break;

              case 7:
                PORTD &= set_reset;
                PORTD |= motor_race_set7;
                motor_race_delay = 1000;
                if(_SW2_isPressed)
                {
                  _SW2_isPressed = false;
                  motor_race_state = 0;
                }
                if(_SW1_isPressed)
                {
                  _SW1_isPressed = false;
                  motor_race_state = 8;
                }
                break;
               case 8:  //Red Flag
                PORTD &= set_reset;
                PORTD |= motor_race_red_flag;     
                if(_SW2_isPressed)
                {
                  _SW2_isPressed = false;
                  motor_race_state = 0;
                }
              break;
               case 9:  //Abort 1
                PORTD &= set_reset;
                PORTD |= motor_race_abort1;
                motor_race_delay = 500;  
                motor_race_state = 10;
              break;
              case 10:  //Abort 2
                PORTD &= set_reset;
                 if(_SW1_isPressed)
                 {
                  _SW1_isPressed = false;
                  motor_race_delay = 10;  
                  motor_race_state = 0;
                 }
                 else
                 {
                  motor_race_delay = 500;  
                  motor_race_state = 9;
                 }
              break;
            }
        }
            
            switch(_SW1_PressCount_state)
            {
              case 0:
                if(_B1_state == normalPressed)
                  {
                      _SW1_isPressed = true;
                      _SW1_PressCount_state = 1;
                  }
                else
                      _SW1_PressCount_state = 0;
              break;

              case 1:
                if(_B1_state == notPressed)
                   {
                      _SW1_PressCount_state = 0;
                   }
                else
                      _SW1_PressCount_state = 1;
              break;
              default: _SW1_PressCount_state = 0;
            }

             switch(_SW2_PressCount_state)
            {
              case 0:
                if(_B2_state == normalPressed)
                  {
                       _SW2_isPressed = true;
                      _SW2_PressCount_state = 1;
                  }
              else
                _SW2_PressCount_state = 0;
              break;

              case 1:
                if(_B2_state == notPressed)
                   {
                      _SW2_PressCount_state = 0;
                   }
                else
                      _SW2_PressCount_state = 1;
              break;
              default: _SW2_PressCount_state = 0;
                  
            }
        }
   
    //module 4 - SW1 DEBOUNCE MODULE
    { 
        static unsigned long _B1_debounce_time, _B1_debounce_switch;

        static unsigned int debounce, module3_delay; //debounce duration

        static bool _B1_debounce_execute; //while true, execute module

        static unsigned char _B1_debounce_state;

        if (init_B1_debounce_clock)
        {
            debounce = 300;
            module3_delay = 17;
            _B1_debounce_time = millis();
            _B1_debounce_execute = false;
            init_B1_debounce_clock = false;
            _B1_debounce_state = notPress;
        }
        
        else if (((long)(millis() - _B1_debounce_time)) > module3_delay)
        {
            _B1_debounce_time = millis();
            _B1_debounce_execute = true;
        }
        else 
            _B1_debounce_execute = false;

        if (_B1_debounce_execute)
        {
            switch (_B1_debounce_state)
            {
            case notPress:
                _B1_state = notPressed;
  
                if (PINC & _B1)  //if HIGH
                    _B1_debounce_state = notPress;
                else
                {
                    _B1_debounce_state = partPress;
                    _B1_debounce_switch = _B1_debounce_time;
                }
  
                break;
  
            case partPress:
                _B1_state = partialPressed;
                if (PINC & _B1)  //if HIGH
                    _B1_debounce_state = notPress;
                else if ((long)(millis() - _B1_debounce_switch) >= debounce)
                    _B1_debounce_state = normalPress;
                else
                    _B1_debounce_state = partPress;
                break;
  
            case normalPress:
                _B1_state = normalPressed;
                if (PINC & _B1)  //if HIGH
                    _B1_debounce_state = notPress;
                else
                    _B1_debounce_state = normalPress;
                break;
  
            default: _B1_debounce_state = notPress; break;
            }
        }
    }

    //module 5 - SW2 DEBOUNCE MODULE
    {
        static unsigned long _B2_debounce_time, _B2_debounce_switch;

        static unsigned int debounce, module4_delay; //debounce duration

        static bool _B2_debounce_execute; //while true, execute module

        static unsigned char _B2_debounce_state;

        if (init_B2_debounce_clock)
        {
            debounce = 300;
            module4_delay = 17;
            _B2_debounce_time = millis();
            _B2_debounce_execute = false;
            init_B2_debounce_clock = false;
            _B2_debounce_state = notPress;
        }    
        else if (((long)(millis() - _B2_debounce_time)) > module4_delay)
        {
            _B2_debounce_time = millis();
            _B2_debounce_execute = true;
        }
        else 
            _B2_debounce_execute = false;

        if (_B2_debounce_execute)
        {
            switch (_B2_debounce_state)
            {
            case notPress:
                _B2_state = notPressed;
                if (PINC & _B2)  //if HIGH
                    _B2_debounce_state = notPress;
                else
                {
                    _B2_debounce_state = partPress;
                    _B2_debounce_switch = _B2_debounce_time;
                }

                break;

            case partPress:
                _B2_state = partialPressed;
                if (PINC & _B2)  //if HIGH
                    _B2_debounce_state = notPress;
                else if ((long)(millis() - _B2_debounce_switch) >= debounce)
                    _B2_debounce_state = normalPress;
                else
                    _B2_debounce_state = partPress;
                break;

            case normalPress:
                _B2_state = normalPressed;
                if (PINC & _B2)  //if HIGH
                    _B2_debounce_state = notPress;
                else
                    _B2_debounce_state = normalPress;
                break;

            default: _B2_debounce_state = notPress;
            }
        }
    }

    //triLEDred driver module
    {
        static bool  triLights_execute;
        static unsigned long triLights_delay, triLights_time;
        static int triLights_state;
        unsigned long difference;

        if(init_triLights_clock)
        {
          triLights_time = millis();
          triLights_delay = 995;
          triLights_execute = true;
          init_triLights_clock = false;
        }
          
      if(triLights_execute)
      {
         difference = millis() - triLights_time;
        switch(triLights_state)
        { 
          case rampUp:

            if(lights == off)
              {
                analogWrite(TRI_BLUE,0);
                analogWrite(TRI_RED,0);
                analogWrite(TRI_GREEN,0);
              }
            if(lights == amber)
            {
               analogWrite(TRI_RED,difference * 0.255);
               analogWrite(TRI_GREEN,difference * 0.03);
            }
            else if(lights == blue)
            {
                analogWrite(TRI_BLUE,difference * 0.255);
            }
            else if(lights == green)
            {
                analogWrite(TRI_GREEN,difference * 0.255);
            }
            else if(lights == red)
              {
                analogWrite(TRI_RED,difference * 0.255);
              }
                
            if(difference >= triLights_delay)
            {
              triLights_state = dimDown;
              triLights_time = millis();
              difference = 0;
            }
            break;
          case dimDown:
            if(lights == off)
            {
                analogWrite(TRI_BLUE,0);
                analogWrite(TRI_RED,0);
                analogWrite(TRI_GREEN,0);
            }
            if(lights == amber)
            {
               analogWrite(TRI_RED, (1000-difference) * 0.255);
               analogWrite(TRI_GREEN,(1000-difference) * 0.03);
            }
            else if(lights == blue)
            {
               analogWrite(TRI_BLUE,(1000-difference) * 0.255);
            }
            else if(lights == green)
            {
              analogWrite(TRI_GREEN,(1000-difference) * 0.255);
            }
            else if(lights == red)
            {
              analogWrite(TRI_RED,(1000-difference) * 0.255);
            }
            if(difference >= triLights_delay)
            {
              triLights_state = rampUp;
              triLights_time = millis();
              difference = 0;
            }
            break;
            default: triLights_state = rampUp;
    }
    }
    }
    //module 6 - ACCELEROMETER
    {
        static unsigned long acc_time, acc_switch;

        static unsigned int debounce, acc_delay; //debounce duration

        static bool acc_execute; //while true, execute module

        static unsigned char acc_state;

        static int16_t accX, accY, accZ, TEMP_OUT; // accelerometer axes variables

        static float T;

        if (init_acc_clock)
        {
            acc_delay = 333;
            acc_time = millis();
            acc_execute = false;
        }
        else if ((long)(millis() - acc_time) > acc_delay)
        {
            acc_time = millis();
            acc_execute = true;
        }

        else
            acc_execute = false;

        if (acc_execute)
        {
            Wire.beginTransmission(MPU_6050);
            Wire.write(ACCEL_ZOUT_HI);
            Wire.endTransmission(false);
            Wire.requestFrom(MPU_6050, 4, true);  //request acces to 6 registers, 2 for each axis

            accZ = Wire.read() << 8 | Wire.read();
            TEMP_OUT = Wire.read() <<8 | Wire.read();
            
            if (accZ > 13000)  //if the board is lying flat (checking the Z axis value)
            {
              //safe
                acc_state = 0;
            }

            //dangerous
            if (accZ < -13000) //if the board  is lying upside_down(checking the Z axis)
            {
                acc_state = 1;
            }

            if (accX > 13000) //if the board is held in landscape (checking the X axis)
            {
                acc_state = 2;
            }

            if (accX < -13000) //if the board is held upside-down in landspace (checking the X axis)
            {
                acc_state = 3;
            }

            if (accY < -16000) //if the board is held in portrait(on the left side)(checking the Y axis)
            {
                acc_state = 4;
            }

            if (accY > 16000) //if the board is held in portrait(on the right side)(checking the Y axis)
            {
                acc_state = 5;
            }

            T = (TEMP_OUT / 340) + 36.53;
            static unsigned msb = B10000000 & displayState;
            switch (acc_state)
            {
           // case 0:
            //    displayState = msb | char_F;  //flat
             //   priorityMode = equal;
              //  break;

           
            }
        }
    }
    Write_to_shift_register(displayState);
    Serial.println(traffic_state);
 }

 void Write_to_shift_register(byte data)
{
  mask = B10000000;
  for(int i = 0; i < 8; i++)
  {
    if(data & mask)
      PORTB |= DATA; //DATA HIGH 
    else
      PORTB &= ~DATA;  //DATA LOW

    PORTB |= CLOCK; //CLOCK HIGH
    mask >>= 1;
    PORTB &= ~CLOCK;  //CLOCK LOW
  }
  
  PORTB |= LATCH; //LATCH HIGH
  PORTB &= ~LATCH;  //LATCH LOW
}

void receiveEvent(int howMany) 
{ int temp;
 while (0 < Wire.available()) 
 {
  temp = Wire.read();
 }

  if(temp > 0x60 && temp < 0x67)
  {
      Wire.write('A');
      traffic_state = temp;
  }
  if(temp > 0x66 && temp < 0x6C)
    {
      Wire.write('A');
      triLED_state = temp;
    }
  if(temp < 0x61 || temp > 0x6C)
      Wire.write('N');

    Serial.println(temp);
}

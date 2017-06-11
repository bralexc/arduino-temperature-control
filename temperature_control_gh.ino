/*
- DS18B20 sensor for temperature reading
- two buttons for adjusting target temperature (+ and -)

pin out:

 button up   -> gnd
 button up   -> pin A2
 button down -> gnd
 button down -> pin A1
 button sel  -> gnd
 button sel  -> pin A0
 (buttons will use internal pull-up, so no need for resistors)
 
 (looking at the pot)
 pot left  -> gnd
 pot right -> +5V
 pot mid   -> pin A3
 
 sensor red wire (vcc)     -> +5V
 sensor yellow wire (data) -> pin 4 -> 4.7 kOhm resistor -> +5V
 sensor green wire (gnd)   -> gnd

 (this means there is a pull-up in the data wire)
 (btw, green SHOULD be data and yellow SHOULD be gnd, but in mine it's like above)
 
 * if it keeps reading -1000, check gnd and data
 * if it keeps reading +85, check vcc

 heater relay on + -> pin 5
 heater relay on - -> gnd
 
 buzzer+ -> pin 3
 buzzer- -> 220 Ohm resistor -> gnd

 
 (this is using the 3pin lcd library, so pins are only needed for power and controling a shift register)

 lcd pcb-      -> gnd          LCD info:      01234567890123456789               
 lcd pcb+      -> +5V                        |--------------------|            
 lcd pcb clock -> pin 8                      |time:  00:00 > 00:00| 0
 lcd pcb latch -> pin 9                   1  |temp:  00:00 > 00.00| 1
 lcd pcb data  -> pin 10                  2  |                    | 2
                                          3  |HP     00%  00:00:00| 3
                                             |--------------------|
                                              01234567890123456789
                             
                             
TODO:
 - Switch heater to PID library
 - Pump control
 - Boil control (heater power, hop additions, etc)
 - Save data to EEPROM?
 - Auto/Manual heater/pump control

*/


#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal595.h> 

// pins --- start

#define BUTTON_UP A2
#define BUTTON_DOWN A1
#define BUTTON_SEL A0
#define DS18S20 4

#define HEATER_RELAY 5
#define HEATER_POT A3
#define HEATER_WINDOW 4000 // ms

#define BUZZER 3

// pins --- end


volatile unsigned long window_start_time; // the time the heater window starts
volatile unsigned long heater_control;    // the amount of time within the window the heater should be on
int heater_percentage;

volatile bool heater_active;  // tells if the heater is enabled
volatile bool heater_on;      // tells if the heater is currently on

// delay between temperature measurements
#define MEASURE_DELAY 2000  // 2 s

// delay between data log
#define DATA_LOG_DELAY 10000 // 10 s

// here is where we define the buttons that we'll use
byte buttons[] = {BUTTON_UP, BUTTON_DOWN, BUTTON_SEL};
#define NUMBUTTONS sizeof(buttons)

// temperature sensor definitions
OneWire ds(DS18S20);
DallasTemperature sensors(&ds);
DeviceAddress insideThermometer;

// LCD
LiquidCrystal595 lcd(10,9,8);     // datapin, latchpin, clockpin

float target_temperature;
float current_temperature;
unsigned long last_measure;
unsigned long last_data_log;

boolean start_timer = 0;
unsigned long mash_time_start;
unsigned long mash_step_time = (unsigned long)2*60*1000; // 2 min -> millis
boolean in_mash_step = 1;

void print_formated_time(unsigned long, byte, byte, boolean);
char * formatted_time(unsigned long ms, boolean hour=false);
void ring_buzzer(int ntimes, int buzz_time, int frequency=1500, bool interrupt=false);


byte mode;

void setup()
{
  // init heater control variables
  heater_control = HEATER_WINDOW / 4;  // starts at 25%
  heater_percentage = 25;
  window_start_time = 0;
  heater_active = true;
  heater_on = false;
  
  target_temperature = 25.0; // Celsius
  last_measure = millis();
  last_data_log = millis();
  
  // start lcd (20 cols, 4 rows)
  lcd.begin(20, 4);
  
  byte heat_off[8] = {B00000, B01010, B01010, B01110, B01010, B01010, B01010, B00000};  // [1] HEAT symbol
  byte heat_on[8]  = {B11111, B10101, B10101, B10001, B10101, B10101, B10101, B11111};  // [2] reverse HEAT symbol
  byte pump_off[8] = {B00000, B01100, B01010, B01010, B01100, B01000, B01000, B00000};  // [3] Pump Symbol 
  byte pump_on[8]  = {B11111, B10011, B10101, B10101, B10011, B10111, B10111, B11111};  // [4] Reverse PUMP Symbol
  
  lcd.createChar(1, heat_off);
  lcd.createChar(2, heat_on);
  lcd.createChar(3, pump_off);
  lcd.createChar(4, pump_on);

  // start sensors, get adress and set resolution to 0.25 (10 bit)
  sensors.begin();
  sensors.getAddress(insideThermometer, 0);
  sensors.setResolution(insideThermometer, 10);
  
  // Setup buttons with an internal pull-up
  for(byte i=0; i<NUMBUTTONS; i++) {
    pinMode(buttons[i], INPUT_PULLUP);
  }

  // setup led and relay
  pinMode(HEATER_RELAY, OUTPUT);

  // for talking to python - meant to log temperature data
  Serial.begin(9600);
  
  //print_base_menu();
  set_new_mash_step();
  
  // set timer interrupt for heater control after everything has been set
  // heater will be polled at 20 Hz (50 ms)
  // interrupt config ---- start
  noInterrupts();                           // disable all interrupts
  TCCR1A = 0;                               // set entire TCCR1A register to 0
  TCCR1B = 0;                               // same for TCCR1B
  TCNT1  = 0;                               // initialize counter value to 0

  // set compare match register for 20 Hz increments
  OCR1A = 3124;                             // = (16E6) / (20*256) - 1 (must be <65536)

  TCCR1B |= (1 << WGM12);                   // turn on CTC mode
  TCCR1B |= (1 << CS12);                    // Set CS12 bits for 256 prescaler
  TIMSK1 |= (1 << OCIE1A);                  // enable timer compare interrupt
  interrupts(); // enable all interrupts
  // interrupt config ---- end
}

void loop()
{
  // update temperature as needed  
  updateTemperature();
  
  // Check button press
  checkButtonPress();
  
  // turn the relay OFF if temperature higher then target  
  setRelayState();
  
  // update timers on the lcd
  updateTimes();
  
  // log data to serial
  logData();
  
  delay(50);
}




// update temperature reading
void updateTemperature()
{
  // only perform measurement if enough time has passed
  if((millis() - last_measure) >= MEASURE_DELAY)
  {
    last_measure = millis();
    sensors.requestTemperatures();
    current_temperature = sensors.getTempC(insideThermometer);
    if(current_temperature)
    {      
      lcd.setCursor(7,1);
      lcd.print(current_temperature);
    }
  }
}



// write python to ser
void logData()
{
  if((millis() - last_data_log) >= DATA_LOG_DELAY)
  {
    last_data_log = millis();
   
    // print to serial for python routine - time, temperature, power
    Serial.print(last_data_log/1000);
    Serial.print(" ");
    Serial.print(current_temperature);
    Serial.print(" ");
    Serial.print(heater_active);
    Serial.print(" ");
    Serial.println(heater_percentage);
  }
}



// check if SEL button was pressed
void checkButtonPress()
{
  if(btn_press(BUTTON_SEL, 500))
  {
    set_new_mash_step();
  }
}




// set relay state based on temperature
void setRelayState()
{
  heater_control = map(analogRead(HEATER_POT), 0, 1023, 0, HEATER_WINDOW);
  heater_percentage = map(analogRead(HEATER_POT), 0, 1023, 0, 100);
  
  lcd.setCursor(7,3);
  if(heater_percentage < 10)  lcd.print("0");
  lcd.print(heater_percentage);   
  
  lcd.setCursor(0,3);
  
  // if temperature below target, turn heat/pump on
  if(current_temperature < target_temperature)
  {
    heater_active = true; // activate the heater
  
    lcd.write(2);
    //lcd.setCursor(1,3); lcd.write(4);
  }
  
  // if not, turn both off
  else
  {
    digitalWrite(HEATER_RELAY, LOW);
    heater_active = false;
    
    lcd.write(1);
    //lcd.setCursor(1,3); lcd.write(3);
    
    if(!start_timer)
    {
      start_timer = 1;
      mash_time_start = millis();
	  
      // play buzzer to inform the timer has started (f [Hz], duration [ms])
      tone(BUZZER, 2000, 100);
      delay(100);
      tone(BUZZER, 2000, 500);
    }
  }  
}




// update times printed on the LCD
void updateTimes()
{
  if (in_mash_step)
  {
    if (start_timer)
    {
      // current mash step time
      print_formated_time(millis() - mash_time_start, 7, 0, 0);
      
      if((millis() - mash_time_start) > mash_step_time){
        lcd.setCursor(0,2);
        lcd.print("-> step finished! <-");
        tone(BUZZER, 2500);
      }
    }
  }
  // total elapsed time
  print_formated_time(millis(), 13, 3, 1);  
}




// get data and set new mash step temperature and time
#define BUTTON_DELAY 50
void set_new_mash_step()
{
  
  // next lines draw the 'new mash step' menu:
  //
  //       01234567890123456789 
  //      |--------------------| 
  //      | new mash step data | 0
  //   1  |        ----        | 1
  //   2  |step time: >00 min  | 2
  //   3  |step temp:  00.0  *C| 3
  //      |--------------------|
  //       01234567890123456789
  
  
  lcd.clear();
  
  lcd.setCursor(0,0);
  lcd.print(" new mash step data ");
  
  lcd.setCursor(0,1);
  lcd.print("        ----        ");
  
  lcd.setCursor(0,2); lcd.print("step time: >"); 
  // if time < 10 min, print a leading '0'
  if(mash_step_time/60000 < 10)
    lcd.print("0");
  lcd.print(mash_step_time/60000);
  lcd.print(" min");
  
  lcd.setCursor(0,3);
  lcd.print("step temp:  ");
  lcd.print(target_temperature);
  lcd.print(" *C");

  tone(BUZZER, 1000, 500); // f (Hz), length (ms)
  
  unsigned long new_mash_step_time = mash_step_time;
  boolean new_data;
  // first loop is for time data; keep inside it until SEL is pressed
  while(!btn_press(BUTTON_SEL, 50))
  {
    new_data = false;
    
    if(digitalRead(BUTTON_UP) == LOW)
    {
      delay(BUTTON_DELAY);
      if(digitalRead(BUTTON_UP) == LOW)
      {
        new_mash_step_time += 60000UL;
        new_data = true;
        delay(BUTTON_DELAY);
      }
    }

    if(digitalRead(BUTTON_DOWN) == LOW)
    {
      delay(BUTTON_DELAY);
      if(digitalRead(BUTTON_DOWN) == LOW)
      {
        if((long)new_mash_step_time - 60000L < 60000L)
          new_mash_step_time = 00000UL;
        else
          new_mash_step_time -= 60000UL;
        new_data = true;
        delay(BUTTON_DELAY);
      }
    }    
    
    // only update the LCD if we have new data
    if (new_data) {
      lcd.setCursor(12,2);
      if(new_mash_step_time/60000 < 10)
        lcd.print("0");
      lcd.print(new_mash_step_time/60000);
      lcd.print(" min");
    }
    delay(10);
  }
  
  lcd.setCursor(11,2);
  lcd.print(" ");
  lcd.setCursor(11,3);
  lcd.print(">");

  tone(BUZZER, 1000, 100); // f (Hz), length (ms)
  
  // second loop is for the temperature
  float new_target_temperature = target_temperature;
  while(!btn_press(BUTTON_SEL, 50))
  {
    new_data = false;

    if(digitalRead(BUTTON_UP) == LOW)
    {
      delay(BUTTON_DELAY);
      if(digitalRead(BUTTON_UP) == LOW)
      {
        new_target_temperature += 0.5;
        if(new_target_temperature > 99.0)
          new_target_temperature = 99.0;
        new_data = true;
        delay(BUTTON_DELAY);
      }
    }

    if(digitalRead(BUTTON_DOWN) == LOW)
    {
      delay(BUTTON_DELAY);
      if(digitalRead(BUTTON_DOWN) == LOW)
      {
        new_target_temperature -= 0.5;
        new_data = true;
        delay(BUTTON_DELAY);
      }
    }


    if (new_data) {
      lcd.setCursor(12,3);
      if(new_target_temperature < 10.0)
        lcd.print("0");
      lcd.print(new_target_temperature);
      lcd.print(" *C");
    }
    delay(10);
  }

  tone(BUZZER, 1000, 100); // f (Hz), length (ms)
  
  // once time and temperature are set, let's confirm those values were ok
  lcd.setCursor(0,1);  lcd.print("  00.0 C for 00 min ");
  lcd.setCursor(0,2);  lcd.print("      confirm?      ");
  lcd.setCursor(0,3);  lcd.print("      yes   >no     ");
  
  lcd.setCursor(2,1);
  if(new_target_temperature < 10.0) lcd.print("0");
  lcd.print(new_target_temperature);
  
  lcd.setCursor(13,1);
  if(new_mash_step_time/60000 < 10) lcd.print("0");
  lcd.print(new_mash_step_time/60000);
  lcd.print(" min  ");

  new_data = false;
  while(!btn_press(BUTTON_SEL, 50))
  {
    
    if (btn_press(BUTTON_DOWN, 50)) {
      lcd.setCursor(5,3);
      lcd.print(" yes   >no");
      new_data = false;
    }
      
    if (btn_press(BUTTON_UP, 50)){
      lcd.setCursor(5,3);
      lcd.print(">yes    no");
      new_data = true;
    }

    delay(10);
  }  

  // if we're ok with the new values, go ahead and overwrite the old ones  
  if(new_data)
  {
    target_temperature = new_target_temperature;
    mash_step_time = new_mash_step_time;
    start_timer = 0;

    tone(BUZZER, 1000, 100);
    delay(100);
    tone(BUZZER, 1500, 100);
    delay(100);
    tone(BUZZER, 2000, 500);    
  }
  else
  {
    tone(BUZZER, 2000, 100);
    delay(100);
    tone(BUZZER, 1500, 100);
    delay(100);
    tone(BUZZER, 1000, 500);    
  }
  
  print_base_menu();
}



// print base menu
void print_base_menu()
{
  lcd.setCursor(0,0); lcd.print("time:  00:00 > "); lcd.print(formatted_time(mash_step_time));
  lcd.setCursor(0,1); lcd.print("temp:  00.00 > "); lcd.print(target_temperature);
  lcd.setCursor(0,2); lcd.print("                    ");
  lcd.setCursor(0,3); lcd.print("HP     00%  00:00:00");
  
  lcd.setCursor(0,3); lcd.write(1);
  lcd.setCursor(1,3); lcd.write(3);
}



// timer compare interrupt service routine
ISR(TIMER1_COMPA_vect) 
{
  long now = millis();

  // only runs if the heater is active
  if(heater_active)
  { 
    // check if we've advanced to the next window
    if(now > window_start_time + HEATER_WINDOW)
    {
      window_start_time = now;
    }
  
    if(heater_control > (now - window_start_time))
    {
      // only turn heater on if it's not already on
      if(!heater_on)
      {
        digitalWrite(HEATER_RELAY, HIGH);
        heater_on = true;
      }
    }
    else
    {
      // only turn heater off if it's not already off
      if(heater_on)
      {
        digitalWrite(HEATER_RELAY, LOW);
        heater_on = false;
      }
    }
  }
}


// print formated time from milis in int
char * formatted_time(unsigned long ms, boolean hour)
{
  static char time[8];
  unsigned long s;
  int m;
  byte h;
  
  s = ms / 1000;
  m = s / 60;
  s = s % 60;
  
  if(hour) {
    h = m / 60;
    m = m % 60;
    sprintf(time, "%i:%02i:%02i", h, m, s);
  }
  else {
    sprintf(time, "%02i:%02i", m, s);
  }
  
  return time;
}



// ring buzzer by specified amount of times, length and sound frequency
void ring_buzzer(int ntimes, int buzz_time, int frequency, bool interrupt)
{
  // first disable the buzzer just in case it's in use
  noTone(BUZZER);
  
  while(ntimes--)
  {
    tone(BUZZER, frequency);
    delay(buzz_time);
    
    noTone(BUZZER);
    delay(buzz_time/3);
  }
}




// print formated time on a given LCD place
void print_formated_time(unsigned long ms, byte col, byte row, boolean hour)
{
  lcd.setCursor(col, row);
  lcd.print(formatted_time(ms, hour));
}




// check if a given button was pressed for some given time
boolean btn_press(byte button_press, int ms)
{
  if (digitalRead(button_press) == LOW)
  {
    delay(ms);
    if (digitalRead(button_press) == LOW)
    {
      while(digitalRead(button_press) == LOW) {} // avoid repeated presses
      return true;
    }
  }
  return false;
}

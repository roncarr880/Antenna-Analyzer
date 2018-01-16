
/*
 *  Antenna Analyzer   by  K1URC
 *    Uses the QRP-LABS arduino shield, QRP-LABS SI5351 synth, an arduino UNO and a LCD display.
 *    Measures SWR but not R and X.  
 *    Navigate menu's using a tap, double tap or long press of the single button switch.
 *      Generally Tap increased value, Double Tap decreased value, and Long Press moves to the next menu.
 *      If you have trouble getting started, try some long presses of about 1 second each.
 *    Two modes:  sweep and frequency generator.
 *      In sweep mode, switch tap or double tap will move the sweep range up or down 500 khz.  Also  
 *      the tap switch press will trigger an output of a CSV file( one export for each press ).
 *      Long press to exit.
 *      
 *      In frequency generation mode, tap and double tap will move the generation frequency up or down 10khz.
 *      Long press to exit.
 *      
 *  Calculation:  SWR = (Vf + Vr)/(Vf - Vr)
 *  
 *  Circuit Mods:
 *    The shield was built stock except for:
 *    The reset button was installed, but the etch run to the headers was cut and the switch was jumpered
 *    to the closest pin on the GPS header.  The signal is 1pps, and it goes to pin 9 on the UNO.
 *    
 *    The jumper W2 W3 was left open,  W2 is wired to the SWR bridge, and the output of the bridge is
 *    wired back to W3 to pick up the antenna connector as the unknown port of the bridge.
 *    
 *    The bridge circuit is wired on the proto section of the arduino shield.
 *    
 *    TBD - how the bridge works with vs without a low pass filter.
 *    At this point, I have just a jumper in place, no low pass filter.
 *    
 *    I hooked arduino pin 3 to the LCD contrast pin via a 1K resistor and 10uf cap to ground.  
 *    The contrast adjustment is made via PWM.  A regular pot could be used instead as is standard 
 *    with most displays.  The display is wired to arduino pins 6,7,8,10,11,12.  You can use different
 *    pins and just change the program if you wish.
 *    
 * Notes: 
 * 
 *    The bias setting for Q1 WORKS IN REVERSE on the arduino shield.  I used the zero bias method.
 *    
 *    Resetting the SI5351 when a new frequency is loaded apparently causes wideband noise.  The SWR 
 *    readings were not stable.  This issue was mitigated with a flag for when resets would be desired,
 *    and by changing the pinmode of the analog channels to digital and writing the lines low to discharge 
 *    the caps in the bridge circuit.
 *    
 *    I used 1N914 type diodes in the bridge.  Some schottky diodes would probably work better.  
 *    The FUDGE factor is adjusted in the program to produce correct SWR readings in the 2:1 range( when 
 *    the load resistance is 25 or 100 ohms ).
 *    
 *    The 10 ohm resistor in the bridge could possibly be left out and the increased signal in the bridge
 *    may produce more accurate readings.
 *
 */


#include <LiquidCrystal.h>
// #include <Wire.h>   // using Han's I2C code instead of the wire library

// SI5351 routines from Hans Summers demo code
#define I2C_START 0x08                  // I2C interface constants
#define I2C_START_RPT 0x10
#define I2C_SLA_W_ACK 0x18
#define I2C_SLA_R_ACK 0x40
#define I2C_DATA_ACK 0x28
#define I2C_WRITE 0b11000000
#define I2C_READ 0b11000001

#define SI_CLK0_CONTROL 16               // Si5351A Register definitions
#define SI_CLK1_CONTROL 17
#define SI_CLK2_CONTROL 18
#define SI_SYNTH_PLL_A 26
#define SI_SYNTH_PLL_B 34
#define SI_SYNTH_MS_0 42
#define SI_SYNTH_MS_1 50
#define SI_SYNTH_MS_2 58
#define SI_PLL_RESET 177

#define SI_R_DIV_1 0b00000000           // R-division ratio definitions
#define SI_R_DIV_2 0b00010000
#define SI_R_DIV_4 0b00100000
#define SI_R_DIV_8 0b00110000
#define SI_R_DIV_16 0b01000000
#define SI_R_DIV_32 0b01010000
#define SI_R_DIV_64 0b01100000
#define SI_R_DIV_128 0b01110000

#define SI_CLK_SRC_PLL_A 0b00000000
#define SI_CLK_SRC_PLL_B 0b00100000

#define XTAL_FREQ 27003741 
// Crystal frequency for Si5351A board - needs to be adjusted specific to your board's crystal


void si5351aOutputOff(uint8_t clk);
void si5351aSetFrequency(uint32_t frequency);


#define SW  9     // command switch ( reset switch rewired to 1pps with etch cut and wire jumper )
#define FWD  A1   // Vfwd
#define REV  A2   // Vrev
#define FUDGE  30 // about .15 volts diode drop.  Adjust for reasonable readings in the 2:1 range.

// switch states for command switch wired to arduino pin 9
// the switch responds to taps, double tap, and long press.
#define IDLE_ 0
#define ARMED 1
#define DTDELAY 2
#define TAP 3
#define DTAP 4
#define LONGPRESS 5
#define DONE 6

byte sw;     // current switch state  ( unsigned char )
byte contrast = 20;  // contrast is set with PWM virtual pin 3
byte mstate;  // current main function
byte mode;    // sweep mode or frequency gen mode
unsigned long timer;

unsigned long freq = 14123456L;        // defaults, change as you wish.
unsigned long gen_freq = 14123456L;
unsigned long start_freq = 5000000L;
unsigned long end_freq = 10000000L;

byte si5351_reset = 1;    // reset needed or ok to set the frequency without a reset

// custom fonts
byte bar1[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111
};
byte bar2[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111
};
byte bar3[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111
};
byte bar4[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111
};
byte bar5[8] = {
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
byte bar6[8] = {
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
byte bar7[8] = {
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
 
LiquidCrystal lcd(6, 7,   8, 10, 11, 12);

  
void setup() {

  analogWrite(3,contrast);        // LCD contrast via pwm
  Serial.begin(19200);            // CSV file export
  // set up the LCD's number of rows and columns: 
  lcd.begin(16, 2);
  lcd.createChar(1, bar1);
  lcd.createChar(2, bar2);
  lcd.createChar(3, bar3);
  lcd.createChar(4, bar4);
  lcd.createChar(5, bar5);
  lcd.createChar(6, bar6);
  lcd.createChar(7, bar7);
  lcd.clear();
  // Print a message to the LCD.
  lcd.print(" K1URC Antenna  ");
  lcd.setCursor(0,1);
  lcd.print("   Analyzer     ");
  delay(2000);
  lcd.setCursor(0,0);
  lcd.print("Cmds: Long Press");
  lcd.setCursor(0,1);
  lcd.print(" Tap,  Dbl Tap  ");
  delay(3000);
  lcd.clear();
  lcd.print("Contrast");
  lcd.setCursor(2,1);
  lcd.print(contrast);
  
  pinMode(SW,INPUT_PULLUP);
  i2cInit();                             // Initialise I2C interface
  
}

/***************    menu editing routines,  setting parameters ********************/
void set_contrast(){

   switch(sw){
     case TAP:  contrast += 2;
     case DTAP: --contrast;  lcd.setCursor(2,1); lcd.print(contrast); lcd.write(' ');
          sw = DONE;
          analogWrite(3,contrast); 
          break;
     case LONGPRESS: ++mstate; sw = DONE; break;
   }
}

void set_mode(){

  if( mstate == 1 ){                 // 1st time entry
    lcd.clear();  lcd.print("Mode");
    mode ^= 1; sw = TAP;             // force a print of the mode
    ++mstate;
  }

  switch( sw ){
    case TAP:
    case DTAP:
       mode ^= 1;
       lcd.setCursor(2,1);
       if( mode ) lcd.print("Freq Gen");
       else       lcd.print("Sweep   ");
       sw = DONE;
    break;
    case LONGPRESS: ++mstate; sw = DONE;
       if( mode ) mstate = 9;             // set up the freq gen mode
    break;  
  }  
}

void sweep_low(){

  if( mstate == 3 ){
    lcd.clear();  lcd.print("Start Freq");
    sw = TAP;             // force a print
    start_freq -= 500000L;
    ++mstate;
  }

  switch(sw){
    case TAP: start_freq += 1000000L;
    case DTAP: start_freq -= 500000L;
       lcd.setCursor(2,1);
       lcd.print(start_freq/1000000L);
       lcd.write('.');
       if(start_freq % 1000000) lcd.write('5');
       else lcd.write('0');
       lcd.write(' ');  lcd.print("Mhz ");
       sw = DONE;
       break;
    case LONGPRESS: ++mstate;  sw = DONE; break;     
  }
  
}

void sweep_high(){

  if( mstate == 5 ){
    lcd.clear();  lcd.print("End Freq");
    sw = TAP;             // force a print
    end_freq -= 500000L;
    ++mstate;
  }

  switch(sw){
    case TAP: end_freq += 1000000L;
    case DTAP: end_freq -= 500000L;
       if( end_freq <= start_freq ) end_freq = start_freq + 500000L;
       lcd.setCursor(2,1);
       lcd.print(end_freq/1000000L);
       lcd.write('.');
       if(end_freq % 1000000) lcd.write('5');
       else lcd.write('0');
       lcd.write(' ');  lcd.print("Mhz ");
       sw = DONE;
       break;
    case LONGPRESS: ++mstate;  sw = DONE;
       lcd.clear(); // lcd.print("Running Sweep"); delay(300);
       break;     
  }
}

void set_gen_freq(){
static unsigned long mult;
unsigned long temp;
byte count;

  if(mstate == 9 ){
    lcd.clear();  lcd.print("Frequency Gen");
    sw = TAP;             // force a print
    mult = 1000000L;
    ++mstate;
    freq = gen_freq;
    freq -= mult;    
  }

  switch(sw){
    case TAP:  freq +=  2*mult;
    case DTAP: freq -= mult;
      lcd.setCursor(2,1);
      if( freq < 10000000L ) lcd.write(' ');
      lcd.print(freq);  // lcd.write(' ');
      curs_pos( mult );
      sw = DONE;
    break;
    case LONGPRESS:
      if( mult == 1 ){
        ++mstate;
        gen_freq = freq;
        lcd.noCursor();   lcd.noBlink();
      }
      else{
        mult /= 10;
        curs_pos(mult);
      }
    sw = DONE;  
    break;
  }
}

void curs_pos(unsigned long mult){    // display a blinking cursor at digit to edit
byte count;

      count = 9;                // set cursor at digit to change
      while( mult > 1 ) --count, mult /= 10;
      lcd.setCursor(count,1);   lcd.cursor();  lcd.blink();  
}
/****************************************************************/

void loop() {
static int count = 0;   

  switch(mstate){
    case 0:  set_contrast();          break;
    case 1: case 2:  set_mode();      break;
    case 3: case 4:  sweep_low();     break;    // set sweep start
    case 5: case 6:  sweep_high();    break;    // set sweep end
    case 7: case 8:  sweep();         break;    // sweeping
    case 9: case 10: set_gen_freq();  break;    // set the gen frequency
    case 11: case 12: freq_gen();     break;    // generating a single frequency, display swr                      
  }

  // run this code each millisecond
  if( timer != millis() ){
    timer = millis();
    read_switch();
    // we are getting tired of pressing the switch for the contrast adjustment.
    // time out this one menu.  If you are using a pot for contrast, change this to timeout
    // in 1 msec.
    if( sw >= TAP ) count = 0;
    else ++count;
    if( mstate == 0 && count > 7000 ) ++mstate;   // 7 seconds     
  }

}

//  read the switch and advance through the states
//  call once per millisecond
void read_switch(){
static byte pressed;
static byte not_pressed;

  if( digitalRead(SW) == LOW ) ++pressed, not_pressed = 0;
  else ++not_pressed, pressed = 0;

  switch(sw){
    case IDLE_:  if( pressed > 20 ) sw = ARMED;
    break;
    case ARMED:  
       if( pressed > 250 ) sw = LONGPRESS;
       else if( not_pressed > 20) sw = DTDELAY;
    break;
    case DTDELAY:
       if( not_pressed > 150 ) sw = TAP;
       else if( pressed > 20 ) sw = DTAP;
    break;
    case DONE:  if(not_pressed > 20 ) sw = IDLE_;
    break;
       
  }
  
}


void sweep(){

static float swr;
static unsigned long min1,min2,add_val,bar_val,bar_add;
float new_swr;
byte c;
static byte csv_active = 0;

  if( sw == LONGPRESS ){    // check if user cancel 
    mstate = 1;   sw = DONE;
    si5351aOutputOff(SI_CLK0_CONTROL);
    return;
  }
  else if( sw == TAP ){     // the switch here does two functions, moves the range and starts the CSV export
                            // since we only have one switch perhaps this is ok.
    start_freq += 500000L;   end_freq += 500000L;
    csv_active = 1;   mstate = 7;      // force a restart of the sweep for the CSV
    sw = DONE;  
  }
  else if( sw == DTAP ){
    start_freq -= 500000L;  end_freq -= 500000L;
    // csv_active = 1;      // only export on tap, so to get an export of the original range: dtap then tap.
    mstate = 7;
    sw = DONE;
  }

  if( mstate == 7 ){     // start sweep
    add_val = (end_freq - start_freq) / 1000;
    bar_add = (end_freq - start_freq) / 17;
    bar_val = start_freq + bar_add;
    freq = start_freq - add_val;    // adjust as it will be set back to start below
    swr = 99;  min1 = start_freq;  min2 = end_freq;
    ++mstate;
    lcd.setCursor(0,1);
    si5351_reset = 1;
    if( csv_active ){    // print the header
      Serial.println("Frequency,SWR");
    }
    return;    
  }
    // else continue sweep
  freq += add_val;
  if( freq > end_freq ){     // sweep is finised, display result
    mstate = 7;              // start over next time
    // display values
    freq = min1 + min2;    // get average value
    freq >>= 1;
    lcd.setCursor(0,0);
    print_formatted(freq);
    lcd.setCursor(12,0);
    lcd.print(swr,1);     lcd.write(' ');
    gen_freq = freq;       // default the freq gen to the best scan freq 
    csv_active = 0;        // just one export for each key press, so turn off when done 
    return; 
  }
    // still sweeping
  si5351aSetFrequency(freq);
  new_swr = calc_swr();
  if( new_swr == swr ) min2 = freq;    // move upper value
  else if( new_swr < swr ){            // store better values
    min1 = min2 = freq;
    swr = new_swr;
  } 
  if( freq > bar_val ){                // swr plot on lower line of LCD
    bar_val += bar_add;
    c = new_swr - 1.0;
    if( new_swr < 2.0 ) c = 0;
    if( c == 0 ) lcd.write(' ');
    else if( c > 7 ) lcd.write(255);   
    else lcd.write(c);                 // using the special LCD characters
  }
  if( csv_active ){
    Serial.print(freq);   Serial.write(',');  Serial.println(new_swr,2);
  }
}

void print_formatted( unsigned long val ){   // print freq as Mhz
float fval;

  if( val < 10000000L ) lcd.write(' ');
  // if( val < 1000000L ) lcd.write(' ');   not needed as a leading zero is displayed by print float
                                         // for frequencies less than 1 meg
  fval = (float)val/1000000.0f;
  lcd.print(fval,3);
  lcd.print(" Mhz");
}



void freq_gen(){        // run once per ms, update swr 10 times a second
static byte counter;
static unsigned long last_time;
float swr;
byte b,c;

  if( timer == last_time ) return;   // run once per ms
  last_time = timer;
  
  if( sw == LONGPRESS ){
    mstate = 1;  sw = DONE;
    si5351aOutputOff(SI_CLK0_CONTROL);
    return;
  }
  else if( sw == TAP ){
     sw = DONE;
     freq += 10000L;
     mstate = 11;       // force to setup the new frequency
  }
  else if( sw == DTAP ){
    sw = DONE;
    freq -= 10000L;
    mstate = 11;
  }

  if( mstate == 11 ){      // first time, set up the si5351
    ++mstate;
    lcd.clear();
    lcd.setCursor(8,0);
    lcd.print(freq);
    lcd.setCursor(0,0);
    lcd.print("swr");
    si5351aSetFrequency(freq);
  }

  if( ++counter < 100 ) return;   // update swr display 10 times a second
  counter = 0;
  // update swr reading
  swr = calc_swr();
  // lcd.setCursor(5,1);
  lcd.setCursor( 0,1 );
  if( swr >= 10.0 ) lcd.print(swr,0);
  else lcd.print(swr,1);
  lcd.write(' ');
              // show a bar graph
  lcd.setCursor(4,1);
  c = swr - 1;
  if( c > 12 ) c = 12;
  b = 12 - c;
  while( c--) lcd.write(255);
  c = swr * 10.0;
  c = c % 10;                    // show fraction part of swr using the special LCD characters
  if(swr < 13){                  // perhaps some vertical bars would look better but we also want
    c = map(c,0,9,0,8);          // to display a swr plot vs frequency using the same characters.
    if( c == 0 ) lcd.write(' ');
    else if( c > 7 ) lcd.write(255);
    else lcd.write(c);
  }
  while( b--) lcd.write(' ');    // clear to end of line
}



float calc_swr(){    // get a swr reading

float fwd,rev;
int val; byte i;
float volts;

   if( mode == 0 ){    // slow down scan so bridge has time to discharge
     delay(1);
     read_switch();   // key press timing correction because of the delay
   }

  val = 0;
  for(i = 0; i < 4; ++i){     // average a few readings
     val += analogRead(FWD);   
  }
  val >>= 2;
  fwd = (float)val;
  
    // if this message shows up, the signal is too high.  Turn down the bias to reduce output.
    // my signals seem to be in the 1 volt or so range with the zero current bias method.
    if( val > 1000 ){
      volts = 5.0 * ( fwd / 1024.0 );
      lcd.setCursor(0,0);     // do not exceed 5 volts into the UNO A/D converters
      lcd.print(volts,2);  lcd.write(" too large");
      // turn things off
      si5351aOutputOff(SI_CLK0_CONTROL);
      while(1);    // HANG HERE !
    } 

  val = 0;
  for(i = 0; i < 4; ++i){
    val += analogRead(REV);
  }
  val >>= 2;  
  rev = (float)val;

  if( rev > 5.0 ){    // adjust for diode drop
    rev += FUDGE;
    fwd += FUDGE;
  }
  if(( fwd - rev ) < 1.0 ) return 99.0;
  return ( (fwd + rev)/(fwd - rev) );
  
}


/*****************************************************************/

//  Hans Summers  QRP-LABS code for SI5351 and I2C
uint8_t i2cStart()
{
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

  while (!(TWCR & (1<<TWINT))) ;

  return (TWSR & 0xF8);
}

void i2cStop()
{
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

  while ((TWCR & (1<<TWSTO))) ;
}

uint8_t i2cByteSend(uint8_t data)
{
  TWDR = data;

  TWCR = (1<<TWINT) | (1<<TWEN);

  while (!(TWCR & (1<<TWINT))) ;

  return (TWSR & 0xF8);
}

uint8_t i2cByteRead()
{
  TWCR = (1<<TWINT) | (1<<TWEN);

  while (!(TWCR & (1<<TWINT))) ;

  return (TWDR);
}

uint8_t i2cSendRegister(uint8_t reg, uint8_t data)
{
  uint8_t stts;

  stts = i2cStart();
  if (stts != I2C_START) return 1;

  stts = i2cByteSend(I2C_WRITE);
  if (stts != I2C_SLA_W_ACK) return 2;

  stts = i2cByteSend(reg);
  if (stts != I2C_DATA_ACK) return 3;

  stts = i2cByteSend(data);
  if (stts != I2C_DATA_ACK) return 4;
  
  i2cStop();

  return 0;
}

uint8_t i2cReadRegister(uint8_t reg, uint8_t *data)
{
  uint8_t stts;

  stts = i2cStart();
  if (stts != I2C_START) return 1;

  stts = i2cByteSend(I2C_WRITE);
  if (stts != I2C_SLA_W_ACK) return 2;

  stts = i2cByteSend(reg);
  if (stts != I2C_DATA_ACK) return 3;

  stts = i2cStart();
  if (stts != I2C_START_RPT) return 4;

  stts = i2cByteSend(I2C_READ);
  if (stts != I2C_SLA_R_ACK) return 5;
  
  *data = i2cByteRead();

  i2cStop();

  return 0;
}

// Init TWI (I2C)
//
void i2cInit()
{
  TWBR = 12;   // 12 is 400k for 16 meg clock. Original here was 92, perhaps that is 100k for a 20mhz crystal
  TWSR = 0;    // anyway it was too slow and the key presses were not timed correctly when scanning
  TWDR = 0xFF;
  PRR = 0;
}

//
// Set up specified PLL with mult, num and denom
// mult is 15..90
// num is 0..1,048,575 (0xFFFFF)
// denom is 0..1,048,575 (0xFFFFF)
//
void setupPLL(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom)
{
  uint32_t P1;     // PLL config register P1
  uint32_t P2;     // PLL config register P2
  uint32_t P3;     // PLL config register P3

  P1 = (uint32_t)(128 * ((float)num / (float)denom));
  P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
  P2 = (uint32_t)(128 * ((float)num / (float)denom));
  P2 = (uint32_t)(128 * num - denom * P2);
  P3 = denom;

  i2cSendRegister(pll + 0, (P3 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 1, (P3 & 0x000000FF));
  i2cSendRegister(pll + 2, (P1 & 0x00030000) >> 16);
  i2cSendRegister(pll + 3, (P1 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 4, (P1 & 0x000000FF));
  i2cSendRegister(pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
  i2cSendRegister(pll + 6, (P2 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 7, (P2 & 0x000000FF));
}

//
// Set up MultiSynth with integer divider and R divider
// R divider is the bit value which is OR'ed onto the appropriate 
// register, it is a #define in si5351a.h
//
void setupMultisynth(uint8_t synth, uint32_t divider, uint8_t rDiv)
{
  uint32_t P1;     // Synth config register P1
  uint32_t P2;     // Synth config register P2
  uint32_t P3;     // Synth config register P3

  P1 = 128 * divider - 512;
  P2 = 0;          // P2 = 0, P3 = 1 forces an integer value for the divider - for best jitter performance
  P3 = 1;

  i2cSendRegister(synth + 0, (P3 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 1, (P3 & 0x000000FF));
  i2cSendRegister(synth + 2, ((P1 & 0x00030000) >> 16) | rDiv);
  i2cSendRegister(synth + 3, (P1 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 4, (P1 & 0x000000FF));
  i2cSendRegister(synth + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
  i2cSendRegister(synth + 6, (P2 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 7, (P2 & 0x000000FF));
}

//
// Switches off Si5351a output
// Example: si5351aOutputOff(SI_CLK0_CONTROL);
// will switch off output CLK0
//
void si5351aOutputOff(uint8_t clk)
{
  i2cSendRegister(clk, 0x80);         // Refer to SiLabs AN619 to see bit values - 0x80 turns off the output stage
  si5351_reset = 1;                   // reset needed when we turn the clock back on
}

//
// Set CLK0 output ON and to the specified frequency
// Frequency is in the range 1MHz to 150MHz
// Example: si5351aSetFrequency(10000000);
// will set output CLK0 to 10MHz
//
// This example sets up PLL A
// and MultiSynth 0
// and produces the output on CLK0
//
void si5351aSetFrequency(uint32_t frequency)
{
  uint32_t pllFreq;
  uint32_t xtalFreq = XTAL_FREQ;
  uint32_t l;
  float f;
  uint8_t mult;
  uint32_t num;
  uint32_t denom;
  uint32_t divider;

  divider = 900000000 / frequency;    // Calculate the division ratio. 900,000,000 is the maximum internal PLL frequency: 900MHz
  if (divider % 2) divider--;         // Ensure an even integer division ratio

  pllFreq = divider * frequency;      // Calculate the pllFrequency: the divider * desired output frequency

  mult = pllFreq / xtalFreq;          // Determine the multiplier to get to the required pllFrequency
  l = pllFreq % xtalFreq;             // It has three parts:
  f = l;                              // mult is an integer that must be in the range 15..90
  f *= 1048575;                       // num and denom are the fractional parts, the numerator and denominator
  f /= xtalFreq;                      // each is 20 bits (range 0..1048575)
  num = f;                            // the actual multiplier is mult + num / denom
  denom = 1048575;                    // For simplicity we set the denominator to the maximum 1048575

                                      // Set up PLL A with the calculated  multiplication ratio
  setupPLL(SI_SYNTH_PLL_A, mult, num, denom);
                                      // Set up MultiSynth divider 0, with the calculated divider.
                                      // The final R division stage can divide by a power of two, from 1..128.
                                      // reprented by constants SI_R_DIV1 to SI_R_DIV128 (see si5351a.h header file)
                                      // If you want to output frequencies below 1MHz, you have to use the
                                      // final R division stage
  setupMultisynth(SI_SYNTH_MS_0, divider, SI_R_DIV_1);
                                      // Reset the PLL. This causes a glitch in the output. For small changes to
                                      // the parameters, you don't need to reset the PLL, and there is no glitch
  if( si5351_reset ){
    i2cSendRegister(SI_PLL_RESET, 0xA0);
    si5351_reset = 0;
    i2cSendRegister(SI_CLK0_CONTROL, 0x4F | SI_CLK_SRC_PLL_A);
    pinMode(FWD,OUTPUT);
    pinMode(REV,OUTPUT);
    digitalWrite(FWD,LOW);            // discharge the bridge
    digitalWrite(REV,LOW);
    delay(1);
    pinMode(FWD,INPUT);
    pinMode(REV,INPUT);
    delay(20);   
  }
 
                                      // Finally switch on the CLK0 output (0x4F)
                                      // and set the MultiSynth0 input to be PLL A 
 // i2cSendRegister(SI_CLK0_CONTROL, 0x4F | SI_CLK_SRC_PLL_A);  // moved to above code block
}




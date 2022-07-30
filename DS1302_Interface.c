/*  EEET2505 Assignment - Version 2
    Author: Minh Tran (s3818343) - Khoa Tran (s3847766)
    Date created: Dec 30, 2021
    Last modified date: Jan 8, 2022
*/


#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>


#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// Define connections between RTC module pins and GPIO ports
#define RST PORTC5
#define DATA PORTC4
#define CLK PORTC3

// Define DS1302 addresses for read and write
#define WR_SEC_REG 0x80
#define RD_SEC_REG 0x81
#define WR_MIN_REG 0x82
#define RD_MIN_REG 0x83
#define WR_HR_REG 0x84
#define RD_HR_REG 0x85
#define WR_WP_REG 0x8E  // Write Protection Register
#define WR_TCS_REG 0x90  // Trickle Charge Control Register


enum PORTB_ASSIGNMENT {
  SEG_G = PORTB0,
  DIGIT_1 = PORTB1,
  DIGIT_2 = PORTB2,
  DIGIT_3 = PORTB3,
  DIGIT_4 = PORTB4,
  SET_INDICATOR = PORTB5
};


enum PORTD_ASSIGNMENT {
  SEG_A = PORTD0,
  SEG_B = PORTD1,
  SEG_C = PORTD4,
  SEG_D = PORTD5,
  SEG_E = PORTD6,
  SEG_F = PORTD7
};


enum STATE {
  SET_HOUR,
  SET_MINUTE,
  RUN_TIME
};


volatile uint8_t hour = 0, minute = 0, second = 0;
volatile enum STATE state = RUN_TIME;
volatile int blink_count = 0;
volatile bool digit_on = true;
volatile int triggered_time = 0;
volatile int last_triggered_time = 0;
volatile const int BOUNCE_TIME = 175; // Button bouncing time (in milliseconds), minimum time difference between consecutive button press to be registered
volatile int delay_count;
volatile boolean display_mode = true;  // This variable dictates the display mode in RUN_TIME, true means hour-minute, false means minute-second


// Function Declarations
void select_digit(int digit);
void display_num(int num);
void display_clock(boolean mode);
void set_hour(void);
void set_minute(void);
void wait_us(int us);
uint8_t dec_to_bcd(uint8_t dec);
uint8_t bcd_to_dec(uint8_t bcd);
void start_transmit(void);
void end_transmit(void);
uint8_t read_byte(void);
void write_byte(uint8_t content);
void RTC_init(void);
void read_time(void);
void write_time(uint8_t addr, uint8_t content);
void trickle_charge(uint8_t charge_mode);


int main(void) {
  // Port B, C, and D as outputs, D2 and D3 as inputs
  DDRB = 0xFF;
  DDRC = 0xFF;
  DDRD = 0xFF;
  DDRD &= ~((1 << DDD2) | (1 << DDD3));
  // Clear ext. interrupt flags
  EIFR |= (1 << INTF0) | (1 << INTF1);

  init(); // This function is called to allow the use of the millis() function

  // Turn on ext. interrupt 0 and 1, trigger requests on falling edge
  EIMSK |= (1 << INT0) | (1 << INT1);
  EICRA |= (1 << ISC01) | (1 << ISC11);

  // Reset Timer 1 and 2 control registers to default since init() changes some of TC1 and 2 settings
  TCCR1A = 0x00;
  TCCR1B = 0x00;
  TCCR2A = 0x00;
  TCCR2B = 0x00;
  TIFR1 |= (1 << OCF1A);
  sei();  // Enable global interrupt bit

  RTC_init();  // Initialize DS1302 RTC module
  trickle_charge(0b10100101);  // Select 1 diode, 2k ohms for trickle charging
  update_time();

  while (1) {
    switch (state) {
      case SET_HOUR:
        PORTB |= (1 << SET_INDICATOR);  // Turn on indicator LED to indicate system in Set mode
        set_hour();
        break;
      case SET_MINUTE:
        PORTB |= (1 << SET_INDICATOR);  // Turn on indicator LED to indicate system in Set mode
        set_minute();
        break;
      case RUN_TIME:
        PORTB &= ~(1 << SET_INDICATOR);  // Turn off indicator LED during Time mode
        update_time();
        display_clock(display_mode);
        break;
      default:
        state = RUN_TIME;
    }
  }
}


ISR (TIMER1_COMPA_vect) {
  delay_count -= 1;
}


ISR (INT0_vect) {
  /* Button pressed -> change state
     |-----------------------------------------|
     |-> SET HOUR --> SET MINUTE --> RUN TIME -|
  */
  sei();
  triggered_time = millis();  // Get the time when button is pressed
  // If the time difference between current and last press is less than the bouncing time, assume it's a bounce and ignore, otherwise register the press
  if (triggered_time - last_triggered_time > BOUNCE_TIME) {
    // Determine next state based on current state
    if (state == SET_HOUR) {
      state = SET_MINUTE;
    }
    else if (state == SET_MINUTE) {
      write_time(WR_HR_REG, dec_to_bcd(hour));
      write_time(WR_MIN_REG, dec_to_bcd(minute));
      write_time(WR_SEC_REG, 0);
      state = RUN_TIME;
    }
    else {
      display_mode = true;  // When exit set mode, default display mode is hour-minute
      state = SET_HOUR;
    }
  }
  last_triggered_time = triggered_time;  // Save last time button was pressed
}


ISR (INT1_vect) {
  // Button pressed -> increment hour or minute (depends on the current state) by 1
  triggered_time = millis();  // Get the time when button is pressed
  // If the time difference between current and last press is less than the bouncing time, assume it's a bounce and ignore, otherwise register the press
  if (triggered_time - last_triggered_time > BOUNCE_TIME) {
    if (state == SET_MINUTE) {
      minute += 1;
      // If reaches 60th minute, increment hour by 1 and clear minute to 0
      if (minute == 60) minute = 0;
    } else if (state == SET_HOUR) {
      hour += 1;
      if (hour == 24) hour = 0;  // If reaches 24th hour, clear hour to 0
    } else {
      // If in RUN_TIME state, increment button becomes display mode button that switch between hour-minute mode and minute-second mode
      display_mode = !display_mode;
    }
  }
  last_triggered_time = triggered_time;
}


void display_clock(boolean mode) {
  /* This function uses multiplexing technique to display the 4-digit 7-segment LEDs display during RUN_TIME state
     @param (boolean) mode: True - display hour + minute or False - minute + second
     @return: None
  */

  // Turning on each digit (left -> right) for a very short amount of time, the human eye cannot tell if each digit is on one at a time or all on.
  select_digit(1);
  // Display the tenth digit of current hour or minute on 1st digit (left most)
  if (mode) display_num(hour / 10);
  else display_num(minute / 10);
  _delay_ms(5);

  select_digit(2);
  // Display the unit digit of current hour or minute on 2nd digit
  if (mode) display_num(hour % 10);
  else display_num(minute % 10);
  _delay_ms(5);

  select_digit(3);
  // Display the tenth digit of current minute or second on 3rd digit
  if (mode) display_num(minute / 10);
  else display_num(second / 10);
  _delay_ms(5);

  select_digit(4);
  // Display the tenth digit of current minute or second on 4th digit (right most)
  if (mode) display_num(minute % 10);
  else display_num(second % 10);
  _delay_ms(5);
}


void select_digit(int digit) {
  /* This function select which digit in the 4-digit 7-segment LEDs display to turn on
     @param (int) digit: The digit to turn on (1 being the left-most digit and 4 being the right-most one)
     @return: None
  */
  // 4-digit 7-segment LEDs is common anode
  // Turning on the selected digit and turning off others
  switch (digit) {
    case 1:
      PORTB |= (1 << DIGIT_1);
      PORTB &= ~((1 << DIGIT_2) | (1 << DIGIT_3) | (1 << DIGIT_4));
      break;
    case 2:
      PORTB |= (1 << DIGIT_2);
      PORTB &= ~((1 << DIGIT_1) | (1 << DIGIT_3) | (1 << DIGIT_4));
      break;
    case 3:
      PORTB |= (1 << DIGIT_3);
      PORTB &= ~((1 << DIGIT_1) | (1 << DIGIT_2) | (1 << DIGIT_4));
      break;
    case 4:
      PORTB |= (1 << DIGIT_4);
      PORTB &= ~((1 << DIGIT_1) | (1 << DIGIT_2) | (1 << DIGIT_3));
      break;
    default:
      PORTB &= ~((1 << DIGIT_1) | (1 << DIGIT_2) | (1 << DIGIT_3) | (1 << DIGIT_4));
  }
}


void display_num(int num) {
  /* This function controls the pattern being display on 1 digit of the 7-segment LEDs
     @param (int) num: Decimal number between 0 - 9 to display on the 7-segment LEDs
     @return: None
  */
  // 7-seg LED is common anode
  // Turn on corresponding segment and turn off others
  switch (num) {
    case 0:
      PORTD &= ~((1 << SEG_F) | (1 << SEG_E) | (1 << SEG_D) | (1 << SEG_C) | (1 << SEG_B) | (1 << SEG_A));
      PORTB |= (1 << SEG_G);
      break;
    case 1:
      PORTD &= ~((1 << SEG_B) | (1 << SEG_C));
      PORTB |= (1 << SEG_G);
      PORTD |= (1 << SEG_F) | (1 << SEG_E) | (1 << SEG_D) | (1 << SEG_A);
      break;
    case 2:
      PORTB &= ~(1 << SEG_G);
      PORTD &= ~((1 << SEG_E) | (1 << SEG_D) | (1 << SEG_B) | (1 << SEG_A));
      PORTD |= (1 << SEG_F) | (1 << SEG_C);
      break;
    case 3:
      PORTB &= ~(1 << SEG_G);
      PORTD &= ~((1 << SEG_D) | (1 << SEG_C) | (1 << SEG_B) | (1 << SEG_A));
      PORTD |= (1 << SEG_F) | (1 << SEG_E);
      break;
    case 4:
      PORTB &= ~(1 << SEG_G);
      PORTD &= ~((1 << SEG_F) | (1 << SEG_C) | (1 << SEG_B));
      PORTD |= (1 << SEG_E) | (1 << SEG_D) | (1 << SEG_A);
      break;
    case 5:
      PORTB &= ~(1 << SEG_G);
      PORTD &= ~((1 << SEG_F) | (1 << SEG_D) | (1 << SEG_C) | (1 << SEG_A));
      PORTD |= (1 << SEG_E) | (1 << SEG_B);
      break;
    case 6:
      PORTB &= ~(1 << SEG_G);
      PORTD &= ~((1 << SEG_F) | (1 << SEG_E) | (1 << SEG_D) | (1 << SEG_C) | (1 << SEG_A));
      PORTD |= (1 << SEG_B);
      break;
    case 7:
      PORTD &= ~((1 << SEG_C) | (1 << SEG_B) | (1 << SEG_A));
      PORTB |= (1 << SEG_G);
      PORTD |= (1 << SEG_F) | (1 << SEG_E) | (1 << SEG_D);
      break;
    case 8:
      PORTB &= ~(1 << SEG_G);
      PORTD &= ~((1 << SEG_F) | (1 << SEG_E) | (1 << SEG_D) | (1 << SEG_C) | (1 << SEG_B) | (1 << SEG_A));
      break;
    case 9:
      PORTB &= ~(1 << SEG_G);
      PORTD &= ~((1 << SEG_F) | (1 << SEG_D) | (1 << SEG_C) | (1 << SEG_B) | (1 << SEG_A));
      PORTD |= (1 << SEG_E);
      break;
    default:
      // All LEDs off
      PORTB |= (1 << SEG_G);
      PORTD |= (1 << SEG_F) | (1 << SEG_E) | (1 << SEG_D) | (1 << SEG_C) | (1 << SEG_B) | (1 << SEG_A);
  }
}


void set_hour(void) {
  /* This function displays the hour and minute digits using 4-digit 7-segment LEDs during SET_HOUR state
     @param: None
     @return: None
  */
  // The hour digits blink to indicate hour is being set
  blink_count += 1;
  // The value being compared to blink_count dictates the frequency at which the digits are blinking, greater -> blink faster
  if (blink_count == 25) {
    digit_on = !digit_on;
    blink_count = 0;
  }

  // 2 hour digits toggle between on and off (blinking), while 2 minute digits stay on
  // display(-1) turns off the digit
  select_digit(1);
  if (!digit_on) {
    display_num(-1);
  } else {
    display_num(hour / 10);
  }
  _delay_ms(5);

  select_digit(2);
  if (!digit_on) {
    display_num(-1);
  } else {
    display_num(hour % 10);
  }
  _delay_ms(5);

  select_digit(3);
  display_num(minute / 10);
  _delay_ms(5);

  select_digit(4);
  display_num(minute % 10);
  _delay_ms(5);
}


void set_minute(void) {
  /* This function displays the hour and minute digits using 4-digit 7-segment LEDs during SET_MINUTE state
     @param: None
     @return: None
  */
  // The minute digits blink to indicate minute is being set
  blink_count += 1;
  if (blink_count == 25) {
    digit_on = !digit_on;
    blink_count = 0;
  }
  // 2 minute digits toggle between on and off (blinking), while 2 hour digits stay on
  // display(-1) turns off the digit
  select_digit(1);
  display_num(hour / 10);
  _delay_ms(5);

  select_digit(2);
  display_num(hour % 10);
  _delay_ms(5);

  select_digit(3);
  if (!digit_on) {
    display_num(-1);
  } else {
    display_num(minute / 10);
  }
  _delay_ms(5);

  select_digit(4);
  if (!digit_on) {
    display_num(-1);
  } else {
    display_num(minute % 10);
  }
  _delay_ms(5);
}


void wait_us(int us) {
  /* This function generates a microsecond delay
     @param (int) us: Total time (in microseconds) to delay
     @return: None
  */
  delay_count = us;
  // Turn on TC1 CTC mode and no prescale
  TCCR1B |= (1 << WGM12) | (1 << CS10);
  OCR1A = 15;  // TC2 compare match value for 1us
  TIMSK1 |= (1 << OCIE1A);  // Enable TC1 output compare interrupt
  while (delay_count > 0) {}  // When delay_count has not reached 0, keep system idle
  // Turn off and reset TC1
  TIMSK1 &= ~(1 << OCIE1A);  // Turn off Timer 1 Output Compare interrupt
  TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));  // Turn off Timer 1
  TCNT1 = 0;  // Reset Timer/Counter 1
  return; // When sec seconds has elapsed, return to main program
}


uint8_t dec_to_bcd(uint8_t dec) {
  /* This function converts a decimal number to its Binary Coded Decimal (BCD) equivalent
     @param (uint8_t) dec: Decimal number to convert
     @return: (uint8_t) equivalent BCD number of dec
  */
  // The 10th digit is shifted left 4 bits -> x16
  return (dec / 10 * 16) + (dec % 10);
}


uint8_t bcd_to_dec(uint8_t bcd) {
  /* This function converts a BCD number to its decimal equivalent
     @param (uint8_t) bcd: BCD number to convert
     @return: (uint8_t): equivalent decimal number of bcd
  */
  return (bcd / 16 * 10) + (bcd % 16);
}


void start_transmit(void) {
  /* This function starts the serial communication of data with the DS1302 RTC
     @param: None
     @return: None
  */
  // Reset RST and CLK signal to LOW
  PORTC &= ~(1 << CLK);
  PORTC &= ~(1 << RST);

  // Raise RST signal to HIGH to start transmitting data
  PORTC |= (1 << RST); // RST High
}


void end_transmit(void) {
  /* This function terminates the serial communication of data with the DS1302 RTC
     @param: None
     @return: None
  */
  // Reset RST and CLK signal to LOW
  PORTC &= ~(1 << RST); // RST Low
  PORTC &= ~(1 << CLK); // Clock Low
}

uint8_t read_byte(void) {
  /* This function reads one byte (8-bit) of data from the DS1302 RTC
     @param: None
     @return: (uint8_t) data from RTC, in decimal format
  */
  uint8_t content = 0;

  DDRC &= ~(1 << DATA); // Change Data pin to input
  PORTC &= ~(1 << DATA); // Disable Internal Pull Up
  // Start reading 8 bits from Bit 0 (LSB)
  for (int i = 0; i < 8; i++) {
    PORTC &= ~(1 << CLK); // Drop CLK to receive data
    wait_us(1);  // CLK LOW
    // If RTC sends 1 to data pin, write 1 to the corresponding bit position of content
    if (PINC & (1 << DATA)) content |= (1 << i);
    PORTC |= (1 << CLK); // Raise CLK
    wait_us(1);  // CLK HIGH
  }
  return content;
}

void write_byte(uint8_t content) {
  /* This function writes one byte (8-bit) of data to the DS1302 RTC
     @param: (uint8_t) content: content to write to RTC, in BCD format
     @return: None
  */
  DDRC |= (1 << DATA); // Set Data pin as output
  // Start writing each of the 8 bits to RTC, starting with Bit 0 (LSB) of the data
  for (int i = 0; i < 8; i++) {
    PORTC &= ~(1 << CLK); // Drop CLK
    wait_us(1);  // CLK LOW
    // If the current bit of the data is 1, write 1 to the Data port, otherwise, write 0
    if (content & (1 << i)) PORTC |= (1 << DATA);
    else PORTC &= ~(1 << DATA);
    wait_us(1);  // Ensure the correct timing according to the datasheet before raising the CLK
    PORTC |= (1 << CLK); // Raise CLK to send data bit to RTC
    wait_us(1);  // CLK HIGH
  }
}

void RTC_init(void) {
  /* This function initializes the DS1302 RTC module on startup
     @param: None
     @return: None
  */
  uint8_t temp_sec, temp_hr;  // These variables are used to store the current data in the RTC

  // Clear Clock Halt bit - bit 7 of the second register
  start_transmit();
  write_byte(RD_SEC_REG);
  temp_sec = read_byte();  // Read current value in Second Register
  end_transmit();

  start_transmit();
  write_byte(WR_SEC_REG);
  write_byte(temp_sec & 0x7f);  // Only clear bit 7 of the second register to 0
  end_transmit();

  // Set 24 hour mode - bit 7 of the hour register
  start_transmit();
  write_byte(RD_HR_REG);
  temp_hr = read_byte();  // Read current value in Hour Register
  end_transmit();

  start_transmit();
  write_byte(WR_HR_REG);
  write_byte(temp_hr & 0x7f);  // Clear only bit 7 of the hour register to 0 to select 24-hour mode
  end_transmit();

  // Clear Write Protection bit - bit 7 of the control register
  start_transmit();
  write_byte(WR_WP_REG);
  write_byte(0x00);  // Clear the control register to 0 to disable write protect
  end_transmit();
}

void update_time(void) {
  /* This function reads from the DS1302 and updates the current time
     @param: None
     @return: None
  */
  uint8_t hr_bcd, mn_bcd, sec_bcd;
  start_transmit();
  write_byte(RD_HR_REG);
  hr_bcd = read_byte();  // Read current hour from DS1302 in BCD format
  end_transmit();

  start_transmit();
  write_byte(RD_MIN_REG);
  mn_bcd = read_byte();  // Read current minute from DS1302 in BCD format
  end_transmit();

  start_transmit();
  write_byte(RD_SEC_REG);
  sec_bcd = read_byte();  // Read current second from DS1302 in BCD format
  end_transmit;

  // Convert BCD to decimal format and update the current hour and minute variables
  hour = bcd_to_dec(hr_bcd);
  minute = bcd_to_dec(mn_bcd);
  second = bcd_to_dec(sec_bcd);
}

void write_time(uint8_t addr, uint8_t content) {
  /* This function writes time data to the DS1302 RTC
     @param (uint8_t) addr: address of the DS1302 time register to write to
     @param (uint8_t) content: data to write to the DS1302, in bcd format
     @return: None
  */
  start_transmit();
  write_byte(addr);
  write_byte(content);
  end_transmit();
}


void trickle_charge(uint8_t charge_mode) {
  /* This function sets up the trickle charger for the DS1302 RTC module
     @param (uint8_t) selection: Select resistor and diode for trickle charging (datasheet pg.7)
     @return: None
  */
  start_transmit();
  write_byte(WR_TCS_REG);
  write_byte(charge_mode);
  end_transmit();
}

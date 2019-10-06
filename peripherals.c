#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

// Pins for LEDs and button:
#define RED_LED 5
#define YELLOW_LED 13
#define BUTTON 19

#define DELAY 700 // Delay for loop iterations (mainly), in ms

#ifndef	TRUE
#define	TRUE (1==1)
#define	FALSE (1==2)
#endif

#define	PAGE_SIZE (4*1024)
#define	BLOCK_SIZE (4*1024)

#define	INPUT 0
#define	OUTPUT 1

#define	LOW 0
#define	HIGH 1

// Pins for LCD:
#define STRB_PIN 24
#define RS_PIN 25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22

// Data structure representation of the LCD:
struct lcdDataStruct {
  int bits, rows, cols ;
  int rsPin, strbPin ;
  int dataPins [8] ;
  int cx, cy ;
};

struct lcdDataStruct *display; // A global instance of the LCD data structure is required for linking purposes.

static int lcdControl;

// HD44780U commands (see Fig 11, p28 of the Hitachi HD44780U datasheet):
#define	LCD_CLEAR	0x01
#define	LCD_HOME	0x02
#define	LCD_ENTRY	0x04
#define	LCD_CTRL 0x08
#define	LCD_CDSHIFT	0x10
#define	LCD_FUNC	0x20
#define	LCD_CGRAM	0x40
#define	LCD_DGRAM	0x80

// Bits in the entry register:
#define	LCD_ENTRY_SH 0x01
#define	LCD_ENTRY_ID 0x02

// Bits in the control register:
#define	LCD_BLINK_CTRL 0x01
#define	LCD_CURSOR_CTRL 0x02
#define	LCD_DISPLAY_CTRL 0x04

// Bits in the function register:
#define	LCD_FUNC_F 0x04
#define	LCD_FUNC_N 0x08
#define	LCD_FUNC_DL	0x10
#define	LCD_CDSHIFT_RL 0x04

static volatile unsigned int gpiobase ;
static volatile uint32_t *gpio ;

int failure (int fatal, const char *message, ...);

#define	PI_GPIO_MASK	(0xFFFFFFC0) // Mask for the bottom 64 pins which belong to the Raspberry Pi.

// Wait for some number of milliseconds:
void delay (unsigned int howLong) {
  struct timespec sleeper, dummy;
  sleeper.tv_sec = (time_t)(howLong / 1000);
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000;
  nanosleep (&sleeper, &dummy);
}

int failure (int fatal, const char *message, ...) {
  va_list argp;
  char buffer [1024];
  if (!fatal) // && wiringPiReturnCodes
    return -1 ;
  va_start (argp, message);
  vsnprintf (buffer, 1023, message, argp);
  va_end (argp) ;
  fprintf (stderr, "%s", buffer);
  exit (EXIT_FAILURE);
  return 0;
}

void delayMicroseconds (unsigned int howLong) {
  struct timespec sleeper;
  unsigned int uSecs = howLong % 1000000;
  unsigned int wSecs = howLong / 1000000;
  if (howLong == 0)
    return;
    #if 0
    else if (howLong < 100)
    delayMicrosecondsHard(howLong);
    #endif
  else {
    sleeper.tv_sec = wSecs;
    sleeper.tv_nsec = (long) (uSecs * 1000L);
    nanosleep (&sleeper, NULL);
  }
}

/*	
 * Toggle the strobe (Really the "E") pin to the device.
 * According to the documentation, data is latched on the falling edge. 
 */
void strobe (const struct lcdDataStruct *lcd) {
  // Note timing changes for new version of delayMicroseconds.
  digitalWrite (gpio, lcd->strbPin, 1); 
  delayMicroseconds (50);
  digitalWrite (gpio, lcd->strbPin, 0); 
  delayMicroseconds (50);
}

// Send a data or command byte to the display:
void sendDataCmd (const struct lcdDataStruct *lcd, unsigned char data) {
  register unsigned char myData = data;
  unsigned char i, d4 ;
  if (lcd->bits == 4) {
    d4 = (myData >> 4) & 0x0F;
    for (i=0; i<4; i++) {
      digitalWrite (gpio, lcd->dataPins [i], (d4 & 1));
      d4 >>= 1 ;
    }
    strobe(lcd) ;
    d4 = myData & 0x0F;
    for (i=0; i<4; i++) {
      digitalWrite(gpio, lcd->dataPins[i], (d4 & 1));
      d4 >>= 1;
    }
  } else {
    for (i = 0 ; i < 8 ; i++) {
      digitalWrite(gpio, lcd->dataPins[i], (myData & 1));
      myData >>= 1;
    }
  }
  strobe(lcd);
}

// Send a command byte to the display:
void lcdPutCommand (const struct lcdDataStruct *lcd, unsigned char command) {
  #ifdef DEBUG
    fprintf(stderr, "lcdPutCommand: digitalWrite(%d,%d) and sendDataCmd(%d,%d)\n", lcd->rsPin, 0, lcd, command);
  #endif
  digitalWrite(gpio, lcd->rsPin, 0);
  sendDataCmd(lcd, command);
  delay(2);
}

void lcdPut4Command (const struct lcdDataStruct *lcd, unsigned char command) {
  register unsigned char myCommand = command ;
  register unsigned char i ;
  digitalWrite (gpio, lcd->rsPin, 0) ;
  for (i=0; i<4; i++) {
    digitalWrite(gpio, lcd->dataPins [i], (myCommand & 1));
    myCommand >>= 1;
  }
  strobe(lcd) ;
}

// Home the LCD cursor:
void lcdHome (struct lcdDataStruct *lcd) {
  #ifdef DEBUG
    fprintf(stderr, "lcdHome: lcdPutCommand(%d,%d)\n", lcd, LCD_HOME);
  #endif
  lcdPutCommand(lcd, LCD_HOME);
  lcd->cx = lcd->cy = 0;
  delay(5);
}

// Clear the LCD:
void lcdClear (struct lcdDataStruct *lcd) {
  #ifdef DEBUG
    fprintf(stderr, "lcdClear: lcdPutCommand(%d,%d) and lcdPutCommand(%d,%d)\n", lcd, LCD_CLEAR, lcd, LCD_HOME);
  #endif
  lcdPutCommand(lcd, LCD_CLEAR);
  lcdPutCommand(lcd, LCD_HOME);
  lcd->cx = lcd->cy = 0;
  delay(5);
}

/* 
 * Update the position of the cursor on the display.
 * Ignore invalid locations. 
 */
void lcdPosition (struct lcdDataStruct *lcd, int x, int y) {
  if ((x > lcd->cols) || (x < 0))
    return;
  if ((y > lcd->rows) || (y < 0))
    return;
  lcdPutCommand (lcd, x + (LCD_DGRAM | (y>0 ? 0x40 : 0x00)));
  lcd->cx = x;
  lcd->cy = y;
}

/*
 * lcdDisplay: lcdCursor: lcdCursorBlink:
 * Turn the display, cursor, cursor blinking on/off.
 */
void lcdDisplay (struct lcdDataStruct *lcd, int state) {
  if (state)
    lcdControl |=  LCD_DISPLAY_CTRL;
  else
    lcdControl &= ~LCD_DISPLAY_CTRL;
  lcdPutCommand (lcd, LCD_CTRL | lcdControl); 
}

void lcdCursor (struct lcdDataStruct *lcd, int state) {
  if (state)
    lcdControl |=  LCD_CURSOR_CTRL;
  else
    lcdControl &= ~LCD_CURSOR_CTRL;
  lcdPutCommand (lcd, LCD_CTRL | lcdControl); 
}

void lcdCursorBlink (struct lcdDataStruct *lcd, int state) {
  if (state)
    lcdControl |=  LCD_BLINK_CTRL;
  else
    lcdControl &= ~LCD_BLINK_CTRL;
  lcdPutCommand (lcd, LCD_CTRL | lcdControl); 
}

// Send a data byte to be displayed on the display. We implement a very simple terminal here - with line wrapping, but no scrolling.
void lcdPutchar (struct lcdDataStruct *lcd, unsigned char data) {
  digitalWrite(gpio, lcd->rsPin, 1);
  sendDataCmd(lcd, data) ;
  if (++lcd->cx == lcd->cols) {
    lcd->cx = 0;
    if (++lcd->cy == lcd->rows)
      lcd->cy = 0 ;
    lcdPutCommand (lcd, lcd->cx + (LCD_DGRAM | (lcd->cy>0 ? 0x40 : 0x00) /* rowOff [lcd->cy] */ )) ;
  }
}

// Send a string to be displayed on the LCD:
void lcdPuts (struct lcdDataStruct *lcd, const char *string) {
  while (*string)
    lcdPutchar (lcd, *string++);
}

int digitalWrite (volatile uint32_t *gpio, int pin, int value) { 
  asm (
    "MOV R0, #28\n"			// r0 = 28
    "ADD R0, %[base]\n"		// r0 = base address + 28   *(gpio + 7)
    "MOV R1, %[value]\n" 	// r1 = on or off 
    "CMP R1, #1\n"			// check if r1 == 1
    "BEQ store\n"           // if true go to store
    "ADD R0, #12\n"         // r0 += 12 *(gpio + 8)
    "store: MOV R1, %[pin]\n"  // r1 = pin number - store is going to act on this 
    "MOV R2, #1\n" // r2  = 1 
    "LSL R2, R1\n" //1<<pin
    "STR R2, [R0]\n"  //*(gpio +7) = 1 << (pin& 31)
    :[pin] "+r" (pin) // pin = pin (output read) outputting what the pin should do 
    :[base] "r" (gpio), [value] "r" (value) // get   
    :"r0","r1","r2","r3","cc" // registers used
  ); 
}

int setup(void) {
  unsigned char func;
  int pinRed = RED_LED, pinYellow = YELLOW_LED, pinButton = BUTTON; 
  int fSel, shift, pin,  clrOff, setOff, off, fd, j, theValue, thePin;
  unsigned int howLong = DELAY;
  uint32_t res; /* testing only */

  if (geteuid() != 0)
    fprintf (stderr, "Setup: must be root.\n(Did you forget sudo?)\n");

  // constants for RPi2
  gpiobase = 0x3F200000;

  // memory mapping - open the master /dev/memory device
  if ((fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0)
    return failure (FALSE, "Setup: unable to open /dev/mem - %s\n", strerror (errno)) ;

  // GPIO:
  gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, gpiobase) ;
  if ((int32_t)gpio == -1)
    return failure (FALSE, "Setup: mmap(GPIO) failed - %s\n", strerror (errno)) ;
 

  /*
   * Setting the mode for LED pin 5 in C:
   * *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (1 << shift); 
   * Instead, ARM Assembler is used:
   */ 
  pinMode(gpio, 5, 1);
  
  /*
   * Setting the mode for LED pin 13 in C:
   * *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (1 << shift); 
   * Again, this is performed in ARM Assembler.
   */ 
  pinMode(gpio, 13, 1);
  
  struct lcdDataStruct * lcd = (struct lcdDataStruct *)malloc(sizeof (struct lcdDataStruct));
  if (lcd == NULL)
    return -1;
 
  digitalWrite(gpio,23,1);
  digitalWrite(gpio,10,1);
  digitalWrite(gpio,27,1); 
  digitalWrite(gpio,22,1);

  // hard-wired GPIO pins:
  lcd->rsPin = RS_PIN;
  lcd->strbPin = STRB_PIN;
  lcd->bits = 4;
  lcd->rows = 2; // # of rows on the display
  lcd->cols = 16; // # of cols on the display
  lcd->cx = 0; // x-pos of cursor
  lcd->cy = 0; // y-pos of curosr

  lcd->dataPins [0] = DATA0_PIN;
  lcd->dataPins [1] = DATA1_PIN;
  lcd->dataPins [2] = DATA2_PIN;
  lcd->dataPins [3] = DATA3_PIN;

  int i; 
  int bits = 4; 
  digitalWrite (gpio, lcd->rsPin,   0) ; pinMode (gpio, lcd->rsPin, OUTPUT);
  digitalWrite (gpio, lcd->strbPin, 0) ; pinMode (gpio, lcd->strbPin, OUTPUT);

  for (i=0 ; i<bits; i++) {
    digitalWrite(gpio, lcd->dataPins[i], 0);
    pinMode(gpio, lcd->dataPins[i], OUTPUT); 
  }
  delay(35); // mS

  if (bits == 4) {
    func = LCD_FUNC | LCD_FUNC_DL; // Set 8-bit mode 3 times
    lcdPut4Command(lcd, func >> 4); 
    delay(35);
    lcdPut4Command(lcd, func >> 4); 
    delay(35);
    lcdPut4Command(lcd, func >> 4); 
    delay(35);
    func = LCD_FUNC; // 4th set: 4-bit mode
    lcdPut4Command(lcd, func >> 4); 
    delay(35);
    lcd->bits = 4;
  } else {
    failure(TRUE, "Setup: only 4-bit connection supported.\n");
    func = LCD_FUNC | LCD_FUNC_DL;
    lcdPutCommand(lcd, func); 
    delay(35);
    lcdPutCommand(lcd, func);
    delay(35);
    lcdPutCommand(lcd, func);
    delay(35);
  }

  if (lcd->rows > 1) {
    func |= LCD_FUNC_N;
    lcdPutCommand (lcd, func); 
    delay(35);
  }

  // Rest of the initialisation sequence
  lcdDisplay(lcd, TRUE);
  lcdCursor(lcd, FALSE);
  lcdCursorBlink(lcd, FALSE);
  lcdClear(lcd);

  lcdPutCommand (lcd, LCD_ENTRY | LCD_ENTRY_ID); // Set entry mode to increment address counter after write.
  lcdPutCommand (lcd, LCD_CDSHIFT | LCD_CDSHIFT_RL); // Set display shift to right-to-left.
 
  memcpy(&display,&lcd,sizeof(lcd)); // Copy the configured LCD representation to the global instance. 
}

say(char line1[16], char line2[16]) { 
  lcdClear(display);
  lcdPosition(display, 0, 0); 
  lcdPuts(display, line1) ;
  lcdPosition(display, 0, 1); 
  lcdPuts (display, line2);
}


wink(int LED) { 
	digitalWrite(gpio, LED, 1); 
	sleep(1); 
	digitalWrite(gpio, LED, 0); 
	sleep(1); 
}

blink() {
	digitalWrite(gpio, RED_LED, 1); 
	digitalWrite(gpio, YELLOW_LED, 1); 
	sleep(1);
	digitalWrite(gpio, RED_LED, 0); 
	digitalWrite(gpio, YELLOW_LED, 0); 
	sleep(1); 
}
checkLED(int correct, int close){
	int i;
	for(i = 0; i < correct; i++){
		wink(YELLOW_LED);
	}
	wink(RED_LED);
	for(i =0; i < close; i++){
		wink(YELLOW_LED);
	}
}
	

int pinMode(volatile uint32_t *gpio, int pin, int value) { 
  int shift = (pin%10)*3;
  int gpfsel = (pin/10)*4;
  asm(
   	/* BASICALLY, JUST TRANSLATE THIS IS INTO ASSEMBLER.
   int fSel = pin/10; // GPIO pin ab lives in register a.
   int shift = (pin%10)*3; // GPIO pin ab sits in slot b of register a, thus shift b*3. 
  *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (1 << shift) ;  // Sets bits to one = output
  *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift));              // Sets bits to zero = input
  *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (value << shift) ;  // Sets bits to one = output
  * */
	"MOV R0, %[gpio]\n" // r0 = base address 
	"MOV R1, %[shift]\n" // r1 = how many bits to shift base address by 
	"MOV R2, %[value]\n" // r2 = input or output device?
	"ADD R0, %[gpfsel]\n" // get the gpfsel register before setting pin
	"LDR R3, [R0]\n" // r3 = memory address of r0 - prepare it for the and statement 
	"MVN R4, R4, LSL R1\n" // negate(R4), left shift it by the bit shift value
	"AND R3, R3, R4\n" // (*(gpio + fSel) & ~(7 << shift))
	"ORR R2, R3, R2, LSL R1\n" // (*(gpio + fSel) & ~(7 << shift)) | (value << shift) 
	"STR R2, [R0]\n"
	:[value] "+r" (value)
	:[gpio] "r" (gpio), [shift] "r" (shift), [gpfsel] "r" (gpfsel)
	:"r0", "r1", "r2", "r3", "r4", "cc"
  );
}

int button() {	
	int value = 0;	// 0 = not clicked,    1 = clicked!
	asm(
		"MOV R0, %[GPIO]\n" // r0 = base address.
		"MOV R1, #52\n" // GPLEV0 register = 13*4 = *(gpio+13);  
		"ADD R0, R1\n" // *(gpio+13);
		"LDR R4, [R0]\n" // loading it into memory "[]"
		"MOV R5, #1\n" // r5 = 1
		"MOV R6, #19\n" // r6 = 19th button pin; 
		"LSL R5, R6\n" // r5 << r6 = 1<<(button&31); 
		"AND %[value], R5, R4\n" // (r5 & r4)
		:[value] "=r" (value)
		:[GPIO] "r" (gpio)
		:"r0","r1","r4","r5","r6","cc"
	);
	
	value = (value!=0) ? 1:0; // if number recieved > 0 then a click is registered, else 0 (no click). 

	return value;
}

int clickCount() {
	int number = 0;
	int i, j, code[2];
	unsigned int howLong = 200;
	for (i=0; i<25; i++) {
		if (button() == HIGH)
		  j = 1; 
		else
		  j = 0; 
		number += j; 
		{ 
			struct timespec sleeper, dummy; 
			sleeper.tv_sec = (time_t)(howLong/1500); 
			sleeper.tv_nsec = (long)(howLong%1500)*1000000; 
			nanosleep(&sleeper, &dummy);
		}
	}
	return number;
}

//// WHOAAAAA BUDDY THIS IS A TEST
test() { 
	wink(RED_LED); 
	wink(YELLOW_LED); 
	blink();
	say("LCD Test","Successful");
}

#include <avr/io.h>
//---------------------- Pin Mapping ------------------------------------- Pin Mapping -----------

#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))
#define SETONLY(ADDRESS,BIT) (ADDRESS = (1<<BIT))

#define SETBITMASK(x,y) (x |= (y))
#define CLEARBITMASK(x,y) (x &= (~y))
#define FLIPBITMASK(x,y) (x ^= (y))
#define CHECKBITMASK(x,y) (x & (y))

#define VARFROMCOMB(x, y) x
#define BITFROMCOMB(x, y) y

#define C_SETBIT(comb) SETBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))
#define C_CLEARBIT(comb) CLEARBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))
#define C_FLIPBIT(comb) FLIPBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))
#define C_CHECKBIT(comb) CHECKBIT(VARFROMCOMB(comb), BITFROMCOMB(comb))



#define LCD_LINES           4     /**< number of visible lines of the display */
#define LCD_DISP_LENGTH    20     /**< visible characters per line of the display */


#define LCD_DATA0_PORT   PORTB     /**< port for 4bit data bit 0 */
#define LCD_DATA1_PORT   PORTD     /**< port for 4bit data bit 1 */
#define LCD_DATA2_PORT   PORTD     /**< port for 4bit data bit 2 */
#define LCD_DATA3_PORT   PORTD     /**< port for 4bit data bit 3 */
#define LCD_DATA0_PIN    0            /**< pin for 4bit data bit 0  */
#define LCD_DATA1_PIN    7            /**< pin for 4bit data bit 1  */
#define LCD_DATA2_PIN    6            /**< pin for 4bit data bit 2  */
#define LCD_DATA3_PIN    5            /**< pin for 4bit data bit 3  */
#define LCD_RS_PORT      PORTC     /**< port for RS line         */
#define LCD_RS_PIN       0            /**< pin  for RS line         */
#define LCD_RW_PORT      PORTC     /**< port for RW line         */
#define LCD_RW_PIN       1            /**< pin  for RW line         */
#define LCD_E_PORT       PORTC     /**< port for Enable line     */
#define LCD_E_PIN        2            /**< pin  for Enable line     */


//---------------------I/O Pin Define--------------------------------------I/O Pin Define-------------------
//Inputs

#define KEY_SET			PIND, 4			// Key_SET in Schematic
#define KEY_ENT			PIND, 3			// Key_EN in Schematic
#define KEY_UP			PINB, 7			// Key_UP in Schematic
#define KEY_DN			PINB, 6			// Key_DN in Schematic

#define DIR_KEY_SET		DDRD, 4			// Key_SET in Schematic
#define DIR_KEY_ENT		DDRD, 3			// Key_EN in Schematic
#define DIR_KEY_UP		DDRB, 7			// Key_UP in Schematic
#define DIR_KEY_DN		DDRB, 6			// Key_DN in Schematic

#define PORT_KEY_SET	PORTD, 4		// Key_SET in Schematic
#define PORT_KEY_ENT	PORTD, 3		// Key_EN in Schematic
#define PORT_KEY_UP		PORTB, 7		// Key_UP in Schematic
#define PORT_KEY_DN		PORTB, 6		// Key_DN in Schematic


//Port for LED
#define LED_HEATER	PORTC, 3		//LED for Heater Status Indication (HTR_ON)
#define LED_PUMP	PORTB, 1		//LED for Pump Status Indication (PUMP_ON)

//Direction for LED
#define DIR_LED_HEATER	DDRC, 3		//LED for Heater Status Indication (HTR_ON)
#define DIR_LED_PUMP	DDRB, 1		//LED for Pump Status Indication (PUMP_ON)

	
/*
// Control Line for Sampling Pump
#define PUMP		PORTB, 2

// Direction for for Sampling Pump I/O Line
#define PUMP		DDRB, 2
*/
// PWM OUT for Sampling Pump
#define	PWM		PORTB, 2

// Direction for PWM / Sampling Pump
#define	DIR_PWM	DDRB, 2

// Transmit Enable for MAX487 (RS485 Driver)
#define TX_EN		PORTD, 2
// Direction for Transmit Enable for MAX487 (RS485 Driver)
#define DIR_TX_EN	DDRD, 2

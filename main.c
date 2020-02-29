/**
  ******************************************************************************
  * @file    main.c
  * @author  Kevon Brooks
  * @version V1.0
  * @brief   16x2 LCD (4-bit mode) interfacing with STM32F767ZI(ARM Cortex M7)
  ******************************************************************************
*/


#include <stdio.h>
#include <stdint.h>
#include <string.h>

//initialize macro for GPIO
#define AHB1_BUS                 ((uint32_t*)(0x40023830))     // Bus for GPIO
#define GPIO_PORT                (0x01)                        //GPIO port A
#define GPIO_MODE                ((uint32_t*)(0x40020000))     // GPIO MODE register
#define GPIO_DATA                ((uint32_t*)(0x40020014))     // GPIO A DATA register
#define GPIO_MODE_CLEAR          (~(0xFFFF))                   // clear mode pin
#define GPIO_MODE_SET            (0x5555)                      //IO pin 0 to 7 as output

//control macro for LCD
#define  LCD_RS                  (0x0001)                     // pin 0
#define  LCD_RW                  (0x0002)                     // pin 1
#define  LCD_E                   (0x0008)                     // pin 3

//Data macro for LCD
#define GPIO_PIN4                (0x0010)                     // Data pin 4
#define GPIO_PIN5                (0x0020)					  // Data pin 5
#define GPIO_PIN6                (0x0040)                     // Data pin 6
#define GPIO_PIN7                (0x0080)                     // Data pin 7
#define LCD_DATA_PINS            (0x00F0)                     // Data pin 4 to 7
#define SHIFT_LEFT               (4)                          // shift mask value
#define UPPER_MASK               (0xF0)
#define LOWER_MASK               (0x0F)

/*-----function signature------*/
extern void initialise_monitor_handles(void);
void LCD_INIT(void);
void LCD_CMD(unsigned char cmd);
void LCD_print_char(char msg);
void LCD_print_string(char* msg);
int  word_size(char* msg);

/*------------Delay function---------*/
void delay_ms(uint32_t ms)
{
	for(uint32_t i =0; i < ms;i++)
	{
	 for(uint32_t j = 0; j <10000; j++); // delay for 1 millisecond
	}

}

/*
 * initialize pointers for GPIOx Bus, GPIO port D mode register & input data register,
 * 16x2 LCD input variables
 */

uint32_t volatile * const pClkCtrlReg = AHB1_BUS;
uint32_t volatile * const pGPIO_modeReg = GPIO_MODE;
uint32_t volatile * const pLCD_data = GPIO_DATA;

int main(void)
{
     /* Enable GPIO and set pin(s) (mode register uses 2 bits for each pin configure).*/
	*pClkCtrlReg |= GPIO_PORT;                         // Enable port A on AHB1 bus.
	*pGPIO_modeReg &= GPIO_MODE_CLEAR;                 //clear bits 0 to 7 data in the mode register.
	*pGPIO_modeReg |= GPIO_MODE_SET;                   // Set mode register as output mode.

	initialise_monitor_handles();
	LCD_INIT();
	LCD_print_string("");  //use double quotes to print single character in the LCd_print_string function

 return 0;
}

/*---------function initializes LCD-------*/
void LCD_INIT(void)
{
	delay_ms(10);
	LCD_CMD(0x01);    //Display clear
	LCD_CMD(0x02);    // Initialize lcd in 4-bit mode
	LCD_CMD(0x28);    //4-bit, 5x7 dots 2 lines
	LCD_CMD(0x0C);    //Display on, cursor off
	LCD_CMD(0x06);    //Entry mode, auto increment with no shift
	LCD_CMD(0x01);    //Display clear
	LCD_CMD(0x80);    // First line first position
}

/*------Function sends commands to the LCD-------*/
void LCD_CMD(unsigned char cmd)
{
	*pLCD_data &= ~(LCD_DATA_PINS);     			//Clear LCD pin D4:D7
	*pLCD_data &= ~(LCD_RS | LCD_RW);               //Set RS & RW  LOW
	*pLCD_data |= LCD_E;							//Enable High
	*pLCD_data |= (UPPER_MASK & cmd);               // send command upper nibble
	delay_ms(5);
	*pLCD_data &= ~LCD_E;                   		 //Enable LOW

	delay_ms(5);
	*pLCD_data &= ~(LCD_DATA_PINS);     		     //Clear LCD pin D4:D7
	*pLCD_data |= LCD_E;                    		 //Enable High
	*pLCD_data |= ((LOWER_MASK & cmd) << SHIFT_LEFT);      // send command upper nibble
	delay_ms(5);
	*pLCD_data &= ~LCD_E;                                  //Set Enable LOW
	delay_ms(5);
}


/*----------function print character on LCD---------*/
void LCD_print_string(char* msg)
{
   uint8_t flag = 0, line1 = 1, line_len = 16, line2 = 0, line_pos = 0;
   uint32_t i = 0, word_len = 0;

   while(i < strlen(msg))
	{
		// check length of the word(s) in the string
		if(msg[i] != ' ' && !flag)
		{
			for(int j = i; j < strlen(msg); j++)
			{
				if(msg[j] == ' ' || msg[j] == '\n')
				{
					flag = 1;
					break;
				}
				word_len++;
			}
		}

		//check if there is enough space to print word on the current line
		if((word_len > (line_len - line_pos) && line1) || (line_pos == 16 && line1))
		{
			LCD_CMD(0xC0);                             //force cursor to line2 first pos.
			line1 = 0;
			line2 = 1;
			line_pos = 0;
		}else if ((word_len > (line_len - line_pos) && line2) || (line_pos == 16 && line2))
		{
			LCD_CMD(0x01);                            //clear screen.
			LCD_CMD(0x80);                            //force cursor to line1 first pos.
			line1 = 1;
			line2 = 0;
			line_pos = 0;

		}

		//print each word in the string
		for(int j = i; j < strlen(msg); j++)
		{

			if((msg[j] == ' ' || msg[j] == '\n') && line_pos == 16)
			{
				flag = 0;
				i++;
				break;
			}

			*pLCD_data &= ~(LCD_DATA_PINS);
			*pLCD_data |= (LCD_E | LCD_RS); 				      // Set Enable HIGH, Set RS HIGH->Data mode
			*pLCD_data &= ~LCD_RW;                                // Set RW LOW for Write mode
			*pLCD_data |= (UPPER_MASK & msg[j]);                  //Send upper nibble
			delay_ms(5);
			*pLCD_data &= ~ LCD_E;                                // Set Enable LOW

			delay_ms(5);
			*pLCD_data |=  LCD_E;
			*pLCD_data &= ~(LCD_DATA_PINS);
			*pLCD_data |= ((LOWER_MASK & msg[j]) << SHIFT_LEFT);     //Send lower nibble
			delay_ms(5);
			*pLCD_data &= ~ LCD_E;                                   // Set Enable LOW
            line_pos++;

			if(msg[j] == ' ' || msg[j] == '\n')
			{
				i++;
				flag = 0;
				break;
			}

			i++;
		}
		word_len = 0;
    }
}



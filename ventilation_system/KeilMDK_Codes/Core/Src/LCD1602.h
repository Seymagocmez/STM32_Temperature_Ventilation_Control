#ifndef INC_LCD1602_H_
#define INC_LCD1602_H_

#include "main.h"
#include "stm32f1xx_hal.h"

// -----------------------------
// LCD Pin Definitions
// -----------------------------
#define RS_Pin     GPIO_PIN_2   // Register Select (PA0) – Command/Data selection
#define RW_Pin     GPIO_PIN_3   // Read/Write (PA1) – Read or write mode
#define E_Pin      GPIO_PIN_4   // Enable (PA2) – Data latch trigger

// Data pins connected to LCD D0-D7 (8-bit mode)
#define D0_Pin     GPIO_PIN_0   // PC0
#define D1_Pin     GPIO_PIN_1   // PC1
#define D2_Pin     GPIO_PIN_2   // PC2
#define D3_Pin     GPIO_PIN_3   // PC3
#define D4_Pin     GPIO_PIN_4   // PC4
#define D5_Pin     GPIO_PIN_5   // PC5
#define D6_Pin     GPIO_PIN_6   // PC6
#define D7_Pin     GPIO_PIN_10  // PC10

// GPIO port assignments
#define RS_Port    GPIOA
#define RW_Port    GPIOA
#define E_Port     GPIOA
#define D_Port     GPIOC

// -----------------------------
// Control Macros for LCD Pins
// -----------------------------
// These macros simplify GPIO control for LCD signal lines

#define RS_DataR()         HAL_GPIO_WritePin(RS_Port, RS_Pin, GPIO_PIN_SET)     // RS = 1 (Data mode)
#define RS_InstructionR()  HAL_GPIO_WritePin(RS_Port, RS_Pin, GPIO_PIN_RESET)   // RS = 0 (Instruction mode)

#define RW_Read()          HAL_GPIO_WritePin(RW_Port, RW_Pin, GPIO_PIN_SET)     // RW = 1 (Read mode)
#define RW_Write()         HAL_GPIO_WritePin(RW_Port, RW_Pin, GPIO_PIN_RESET)   // RW = 0 (Write mode)

#define E_Set()            HAL_GPIO_WritePin(E_Port, E_Pin, GPIO_PIN_SET)       // E = 1 (Enable high)
#define E_Rst()            HAL_GPIO_WritePin(E_Port, E_Pin, GPIO_PIN_RESET)     // E = 0 (Enable low)


// -----------------------------
// Set GPIO Direction (Input or Output)
// dir = 'I' for Input (Read mode), 'O' for Output (Write mode)
// Used during data reading and writing
// -----------------------------
void DataDir(char dir)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	HAL_GPIO_WritePin(D_Port, D0_Pin|D1_Pin|D2_Pin|D3_Pin|D4_Pin|D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D2_Pin|D3_Pin|D4_Pin|D5_Pin|D6_Pin|D7_Pin;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	
	if(dir == 'I')  // Input mode for reading from LCD
	{
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	}
	else if(dir == 'O') // Output mode for writing to LCD
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	}
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}


// -----------------------------
// Read 8-bit data from LCD data pins
// -----------------------------
uint8_t ReadData()
{
	uint8_t dat = 0;
	// Read each pin and set corresponding bit if high
	if(HAL_GPIO_ReadPin(GPIOC, D0_Pin)==GPIO_PIN_SET) dat |= 0x01;
	if(HAL_GPIO_ReadPin(GPIOC, D1_Pin)==GPIO_PIN_SET) dat |= 0x02;
	if(HAL_GPIO_ReadPin(GPIOC, D2_Pin)==GPIO_PIN_SET) dat |= 0x04;
	if(HAL_GPIO_ReadPin(GPIOC, D3_Pin)==GPIO_PIN_SET) dat |= 0x08;
	if(HAL_GPIO_ReadPin(GPIOC, D4_Pin)==GPIO_PIN_SET) dat |= 0x10;
	if(HAL_GPIO_ReadPin(GPIOC, D5_Pin)==GPIO_PIN_SET) dat |= 0x20;
	if(HAL_GPIO_ReadPin(GPIOC, D6_Pin)==GPIO_PIN_SET) dat |= 0x40;
	if(HAL_GPIO_ReadPin(GPIOC, D7_Pin)==GPIO_PIN_SET) dat |= 0x80;
	return dat;
}


// -----------------------------
// Write 8-bit data to LCD data pins
// dat: 8-bit value to be written to LCD
// -----------------------------
void WriteData(uint8_t dat)
{
	uint16_t Set_Pins = 0, Rst_Pins = 0;

	// Determine which pins need to be set or reset based on bit values
	if(dat & 0x01) Set_Pins |= D0_Pin; else Rst_Pins |= D0_Pin;
	if(dat & 0x02) Set_Pins |= D1_Pin; else Rst_Pins |= D1_Pin;
	if(dat & 0x04) Set_Pins |= D2_Pin; else Rst_Pins |= D2_Pin;
	if(dat & 0x08) Set_Pins |= D3_Pin; else Rst_Pins |= D3_Pin;
	if(dat & 0x10) Set_Pins |= D4_Pin; else Rst_Pins |= D4_Pin;
	if(dat & 0x20) Set_Pins |= D5_Pin; else Rst_Pins |= D5_Pin;
	if(dat & 0x40) Set_Pins |= D6_Pin; else Rst_Pins |= D6_Pin;
	if(dat & 0x80) Set_Pins |= D7_Pin; else Rst_Pins |= D7_Pin;

	// Apply values to data pins
	HAL_GPIO_WritePin(GPIOC, Set_Pins, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, Rst_Pins, GPIO_PIN_RESET);
}


// -----------------------------
// Wait until LCD is not busy
// Checks the busy flag (bit 7 of status)
// -----------------------------

void LCD_Busy_Wait()
{
	uint8_t status;
	DataDir('I');           // Set data pins as input
	RS_InstructionR();      // RS = 0: Instruction mode
	RW_Read();              // RW = 1: Read mode

	do
	{
		E_Set();            // Enable pulse
		__NOP();            // Small delay
		status = ReadData();
		E_Rst();            // End pulse
	}
	while(status & 0x80);   // Wait until busy flag is cleared
}


// -----------------------------
// Send a command to LCD
// cmd: LCD instruction byte
// -----------------------------
void LCD_Write_Cmd(uint8_t cmd)
{
	DataDir('O');           // Set data pins as output
	WriteData(cmd);         // Write command value
	RS_InstructionR();      
	RW_Write();             
	E_Rst();                // Ensure E low before pulse
	E_Set();                // Latch the command
	__NOP();                
	E_Rst();                
	LCD_Busy_Wait();        // Wait for execution to finish
}


// -----------------------------
// Write data (character) to LCD
// dat: ASCII value to be displayed
// -----------------------------

void LCD_Write_Data(uint8_t dat)
{
	DataDir('O');
	WriteData(dat);         // Send character
	RS_DataR();             // RS = 1 (Data mode)
	RW_Write();             // RW = 0 (Write)
	E_Set();
	__NOP();
	E_Rst();
	LCD_Busy_Wait();        // Wait until LCD is ready
}


// -----------------------------
// Initialize LCD in 8-bit mode
// -----------------------------

void LCD_Init()
{
	LCD_Write_Cmd(0x38);    // Function Set: 8-bit, 2 lines, 5x8 dots
	HAL_Delay(2);
	LCD_Write_Cmd(0x01);    // Clear display
	HAL_Delay(2);
	LCD_Write_Cmd(0x06);    // Entry mode: move cursor right
	HAL_Delay(2);
	LCD_Write_Cmd(0x0C);    // Display ON, cursor OFF
	HAL_Delay(2);
}


// -----------------------------
// Display a string on LCD
// x: 0 = first line, 1 = second line
// y: column position (0–15)
// str: null-terminated string
// -----------------------------

void LCD_ShowString(uint8_t x, uint8_t y, char *str)
{
	uint8_t i = 0;

	// Set cursor position
	if(x == 0)
		LCD_Write_Cmd(0x80 | y);   // First line
	else if(x == 1)
		LCD_Write_Cmd(0xC0 | y);   // Second line

	// Write each character until null or max 16 chars
	for(i = 0; i < 16 && str[i] != '\0'; i++)
	{
		LCD_Write_Data(str[i]);
		HAL_Delay(2);  // Short delay for stability
	}
}

#endif // INC_LCD1602_H_

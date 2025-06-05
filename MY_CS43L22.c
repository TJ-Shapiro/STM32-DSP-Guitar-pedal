#include "i2c.h"
#include "MY_CS43L22.h"


static uint8_t iData[2];



//(1): Functions definitions
//-------------- Static Functions ---------------//
// Function(1): Write to register
static void write_register(uint8_t reg, uint8_t *data)
{
	iData[0] = reg;
	iData[1] = data[0];
	i2c_writeByte(DAC_I2C_ADDR, (char)iData[0], (char)iData[1]);

}
// Function(2): Read from register
static void read_register(uint8_t reg, uint8_t *data)
{
	char tempData=0;
	i2c_readByte(DAC_I2C_ADDR,(char)reg, &tempData);
	*data=tempData;
}


void CS43L22_Pin_RST_Init(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;

	GPIOD->MODER|=GPIO_MODER_MODE4_0;
	GPIOD->MODER&=~GPIO_MODER_MODE4_1;
}

void CS43L22_RST_Pin_Low(void)
{
	GPIOD->BSRR=GPIO_BSRR_BR4;
}

void CS43L22_RST_Pin_High(void)
{
	GPIOD->BSRR=GPIO_BSRR_BS4;
}


//-------------- Public Functions ----------------//
// Function(1): Initialisation

void CS43_Init(CS43_MODE outputMode)
{

	CS43L22_RST_Pin_High();

	//(2): Power down
	iData[1] = 0x01;
	write_register(POWER_CONTROL1,iData);

	//(3): Enable Right and Left headphones
	iData[1] =  (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
	iData[1] |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
	iData[1] |= (3 << 2);  // PDN_SPKB[0:1] = 11 (Speaker B always off)
	iData[1] |= (3 << 0);  // PDN_SPKA[0:1] = 11 (Speaker A always off)
	write_register(POWER_CONTROL2,&iData[1]);

	//(4): Automatic clock detection
	iData[1] = (1 << 7);
	write_register(CLOCKING_CONTROL,&iData[1]);

	//(5): Interface control 1
	read_register(INTERFACE_CONTROL1, iData);
	iData[1] &= (1 << 5); // Clear all bits except bit 5 which is reserved
	iData[1] &= ~(1 << 7);  // Slave
	iData[1] &= ~(1 << 6);  // Clock polarity: Not inverted
	iData[1] &= ~(1 << 4);  // No DSP mode
	iData[1] &= ~(1 << 2);  // Left justified, up to 24 bit (default)
	iData[1] |= (1 << 2);
	iData[1] |=  (3 << 0);  // 16-bit audio word length for I2S interface
	write_register(INTERFACE_CONTROL1,&iData[1]);


	//(6): Passthrough A settings
	read_register(PASSTHROUGH_A, &iData[1]);
	iData[1] &= 0xF0;      // Bits [4-7] are reserved
	iData[1] |=  (1 << 0); // Use AIN1A as source for passthrough
	write_register(PASSTHROUGH_A,&iData[1]);


	//(7): Passthrough B settings
	read_register(PASSTHROUGH_B, &iData[1]);
	iData[1] &= 0xF0;      // Bits [4-7] are reserved
	iData[1] |=  (1 << 0); // Use AIN1B as source for passthrough
	write_register(PASSTHROUGH_B,&iData[1]);


	//(8): Miscellaneous register settings
	read_register(MISCELLANEOUS_CONTRLS, &iData[1]);
	if(outputMode == MODE_ANALOG)
	{
		iData[1] |=  (1 << 7);   // Enable passthrough for AIN-A
		iData[1] |=  (1 << 6);   // Enable passthrough for AIN-B
		iData[1] &= ~(1 << 5);   // Unmute passthrough on AIN-A
		iData[1] &= ~(1 << 4);   // Unmute passthrough on AIN-B
		iData[1] &= ~(1 << 3);   // Changed settings take affect immediately
	}
	else if(outputMode == CS43_MODE_I2S)
	{
		iData[1] = 0x02;
	}
	write_register(MISCELLANEOUS_CONTRLS,&iData[1]);
	//(9): Unmute headphone and speaker

	read_register(PLAYBACK_CONTROL, &iData[1]);
	iData[1] = 0x00;
	write_register(PLAYBACK_CONTROL,&iData[1]);

	//(10): Set volume to default (0dB)
	iData[1] = 0x00;
	write_register(PASSTHROUGH_VOLUME_A,&iData[1]);
	write_register(PASSTHROUGH_VOLUME_B,&iData[1]);
	write_register(PCM_VOLUME_A,&iData[1]);
	write_register(PCM_VOLUME_B,&iData[1]);
}

// Function(2): Enable Right and Left headphones
void CS43_Enable_RightLeft(uint8_t side)
{
	switch (side)
	{
		case 0:
			iData[1] =  (3 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			iData[1] |= (3 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			break;
		case 1:
			iData[1] =  (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			iData[1] |= (3 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			break;
		case 2:
			iData[1] =  (3 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			iData[1] |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			break;
		case 3:
			iData[1] =  (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			iData[1] |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			break;
		default:
			break;
	}
	iData[1] |= (3 << 2);  // PDN_SPKB[0:1] = 11 (Speaker B always off)
	iData[1] |= (3 << 0);  // PDN_SPKA[0:1] = 11 (Speaker A always off)
	write_register(POWER_CONTROL2,&iData[1]);
}

// Function(3): Set Volume Level
void CS43_SetVolume(uint8_t volume)
{
	int8_t tempVol = volume - 50;
	tempVol = tempVol*(127/50);
	uint8_t myVolume =  (uint8_t )tempVol;
	iData[1] = myVolume;
	write_register(PASSTHROUGH_VOLUME_A,&iData[1]);
	write_register(PASSTHROUGH_VOLUME_B,&iData[1]);

	iData[1] = VOLUME_CONVERT_D(volume);

	/* Set the Master volume */
	write_register(CS43L22_REG_MASTER_A_VOL,&iData[1]);
	write_register(CS43L22_REG_MASTER_B_VOL,&iData[1]);
}

// Function(4): Start the Audio DAC
void CS43_Start(void)
{
	// Write 0x99 to register 0x00.
	iData[1] = 0x99;
	write_register(CONFIG_00,&iData[1]);
	// Write 0x80 to register 0x47.
	iData[1] = 0x80;
	write_register(CONFIG_47,&iData[1]);
	// Write '1'b to bit 7 in register 0x32.
	read_register(CONFIG_32, &iData[1]);
	iData[1] |= 0x80;
	write_register(CONFIG_32,&iData[1]);
	// Write '0'b to bit 7 in register 0x32.
	read_register(CONFIG_32, &iData[1]);
	iData[1] &= ~(0x80);
	write_register(CONFIG_32,&iData[1]);
	// Write 0x00 to register 0x00.
	iData[1] = 0x00;
	write_register(CONFIG_00,&iData[1]);
	//Set the "Power Ctl 1" register (0x02) to 0x9E
	iData[1] = 0x9E;
	write_register(POWER_CONTROL1,&iData[1]);
}

void CS43_Stop(void)
{
	iData[1] = 0x01;
	write_register(POWER_CONTROL1,&iData[1]);
}

#pragma once

static const uint8_t I2C_ADDR = 0x18;
static const uint16_t CHIP_ID = 0xE26A;
static const uint8_t CHIP_VERSION = 2;

static const uint8_t REG_CHIP_ID_L = 0xfa;
static const uint8_t REG_CHIP_ID_H = 0xfb;
static const uint8_t REG_VERSION = 0xfc;

// Rotary encoder
static const uint8_t REG_ENC_EN = 0x04;
static const uint8_t BIT_ENC_EN_1 = 0;
static const uint8_t BIT_ENC_MICROSTEP_1 = 1;
static const uint8_t BIT_ENC_EN_2 = 2;
static const uint8_t BIT_ENC_MICROSTEP_2 = 3;
static const uint8_t BIT_ENC_EN_3 = 4;
static const uint8_t BIT_ENC_MICROSTEP_3 = 5;
static const uint8_t BIT_ENC_EN_4 = 6;
static const uint8_t BIT_ENC_MICROSTEP_4 = 7;

static const uint8_t REG_ENC_1_CFG = 0x05;
static const uint8_t REG_ENC_1_COUNT = 0x06;
static const uint8_t REG_ENC_2_CFG = 0x07;
static const uint8_t REG_ENC_2_COUNT = 0x08;
static const uint8_t REG_ENC_3_CFG = 0x09;
static const uint8_t REG_ENC_3_COUNT = 0x0A;
static const uint8_t REG_ENC_4_CFG = 0x0B;
static const uint8_t REG_ENC_4_COUNT = 0x0C;

// Cap touch
static const uint8_t REG_CAPTOUCH_EN = 0x0D;
static const uint8_t REG_CAPTOUCH_CFG = 0x0E;
static const uint8_t REG_CAPTOUCH_0 = 0x0F;   // First of 8 bytes from 15-22

// Switch counters
static const uint8_t REG_SWITCH_EN_P0 = 0x17;
static const uint8_t REG_SWITCH_EN_P1 = 0x18;
static const uint8_t REG_SWITCH_P00 = 0x19;   // First of 8 bytes from 25-40
static const uint8_t REG_SWITCH_P10 = 0x21;   // First of 8 bytes from 33-49

static const uint8_t REG_P0 = 0x40;       // protect_bits 2 // Bit addressing
static const uint8_t REG_SP = 0x41;       // Read only
static const uint8_t REG_DPL = 0x42;      // Read only
static const uint8_t REG_DPH = 0x43;      // Read only
static const uint8_t REG_RCTRIM0 = 0x44;  // Read only
static const uint8_t REG_RCTRIM1 = 0x45;  // Read only
static const uint8_t REG_RWK = 0x46;
static const uint8_t REG_PCON = 0x47;     // Read only
static const uint8_t REG_TCON = 0x48;
static const uint8_t REG_TMOD = 0x49;
static const uint8_t REG_TL0 = 0x4a;
static const uint8_t REG_TL1 = 0x4b;
static const uint8_t REG_TH0 = 0x4c;
static const uint8_t REG_TH1 = 0x4d;
static const uint8_t REG_CKCON = 0x4e;
static const uint8_t REG_WKCON = 0x4f;    // Read only
static const uint8_t REG_P1 = 0x50;       // protect_bits 3 6 // Bit addressing
static const uint8_t REG_SFRS = 0x51;     // TA protected // Read only
static const uint8_t REG_CAPCON0 = 0x52;
static const uint8_t REG_CAPCON1 = 0x53;
static const uint8_t REG_CAPCON2 = 0x54;
static const uint8_t REG_CKDIV = 0x55;
static const uint8_t REG_CKSWT = 0x56;    // TA protected // Read only
static const uint8_t REG_CKEN = 0x57;     // TA protected // Read only
static const uint8_t REG_SCON = 0x58;
static const uint8_t REG_SBUF = 0x59;
static const uint8_t REG_SBUF_1 = 0x5a;
static const uint8_t REG_EIE = 0x5b;      // Read only
static const uint8_t REG_EIE1 = 0x5c;     // Read only
static const uint8_t REG_CHPCON = 0x5f;   // TA protected // Read only
static const uint8_t REG_P2 = 0x60;       // Bit addressing
static const uint8_t REG_AUXR1 = 0x62;
static const uint8_t REG_BODCON0 = 0x63;  // TA protected
static const uint8_t REG_IAPTRG = 0x64;   // TA protected // Read only
static const uint8_t REG_IAPUEN = 0x65;   // TA protected // Read only
static const uint8_t REG_IAPAL = 0x66;    // Read only
static const uint8_t REG_IAPAH = 0x67;    // Read only
static const uint8_t REG_IE = 0x68;       // Read only
static const uint8_t REG_SADDR = 0x69;
static const uint8_t REG_WDCON = 0x6a;    // TA protected
static const uint8_t REG_BODCON1 = 0x6b;  // TA protected
static const uint8_t REG_P3M1 = 0x6c;
static const uint8_t REG_P3S = 0xc0;      // Page 1 // Reassigned from 0x6c to avoid collision
static const uint8_t REG_P3M2 = 0x6d;
static const uint8_t REG_P3SR = 0xc1;     // Page 1 // Reassigned from 0x6d to avoid collision
static const uint8_t REG_IAPFD = 0x6e;    // Read only
static const uint8_t REG_IAPCN = 0x6f;    // Read only
static const uint8_t REG_P3 = 0x70;       // Bit addressing
static const uint8_t REG_P0M1 = 0x71;     // protect_bits  2
static const uint8_t REG_P0S = 0xc2;      // Page 1 // Reassigned from 0x71 to avoid collision
static const uint8_t REG_P0M2 = 0x72;     // protect_bits  2
static const uint8_t REG_P0SR = 0xc3;     // Page 1 // Reassigned from 0x72 to avoid collision
static const uint8_t REG_P1M1 = 0x73;     // protect_bits  3 6
static const uint8_t REG_P1S = 0xc4 ;     // Page 1 // Reassigned from 0x73 to avoid collision
static const uint8_t REG_P1M2 = 0x74;     // protect_bits  3 6
static const uint8_t REG_P1SR = 0xc5;     // Page 1 // Reassigned from 0x74 to avoid collision
static const uint8_t REG_P2S = 0x75;
static const uint8_t REG_IPH = 0x77;      // Read only
static const uint8_t REG_PWMINTC = 0xc6;  // Page 1 // Read only // Reassigned from 0x77 to avoid collision
static const uint8_t REG_IP = 0x78;       // Read only
static const uint8_t REG_SADEN = 0x79;
static const uint8_t REG_SADEN_1 = 0x7a;
static const uint8_t REG_SADDR_1 = 0x7b;
static const uint8_t REG_I2DAT = 0x7c;    // Read only
static const uint8_t REG_I2STAT = 0x7d;   // Read only
static const uint8_t REG_I2CLK = 0x7e;    // Read only
static const uint8_t REG_I2TOC = 0x7f;    // Read only
static const uint8_t REG_I2CON = 0x80;    // Read only
static const uint8_t REG_I2ADDR = 0x81;   // Read only
static const uint8_t REG_ADCRL = 0x82;
static const uint8_t REG_ADCRH = 0x83;
static const uint8_t REG_T3CON = 0x84;
static const uint8_t REG_PWM4H = 0xc7;    // Page 1 // Reassigned from 0x84 to avoid collision
static const uint8_t REG_RL3 = 0x85;
static const uint8_t REG_PWM5H = 0xc8;    // Page 1 // Reassigned from 0x85 to avoid collision
static const uint8_t REG_RH3 = 0x86;
static const uint8_t REG_PIOCON1 = 0xc9;  // Page 1 // Reassigned from 0x86 to avoid collision
static const uint8_t REG_TA = 0x87;       // Read only
static const uint8_t REG_T2CON = 0x88;
static const uint8_t REG_T2MOD = 0x89;
static const uint8_t REG_RCMP2L = 0x8a;
static const uint8_t REG_RCMP2H = 0x8b;
static const uint8_t REG_TL2 = 0x8c;
static const uint8_t REG_PWM4L = 0xca;    // Page 1 // Reassigned from 0x8c to avoid collision
static const uint8_t REG_TH2 = 0x8d;
static const uint8_t REG_PWM5L = 0xcb;    // Page 1 // Reassigned from 0x8d to avoid collision
static const uint8_t REG_ADCMPL = 0x8e;
static const uint8_t REG_ADCMPH = 0x8f;
static const uint8_t REG_PSW = 0x90;      // Read only
static const uint8_t REG_PWMPH = 0x91;
static const uint8_t REG_PWM0H = 0x92;
static const uint8_t REG_PWM1H = 0x93;
static const uint8_t REG_PWM2H = 0x94;
static const uint8_t REG_PWM3H = 0x95;
static const uint8_t REG_PNP = 0x96;
static const uint8_t REG_FBD = 0x97;
static const uint8_t REG_PWMCON0 = 0x98;
static const uint8_t REG_PWMPL = 0x99;
static const uint8_t REG_PWM0L = 0x9a;
static const uint8_t REG_PWM1L = 0x9b;
static const uint8_t REG_PWM2L = 0x9c;
static const uint8_t REG_PWM3L = 0x9d;
static const uint8_t REG_PIOCON0 = 0x9e;
static const uint8_t REG_PWMCON1 = 0x9f;
static const uint8_t REG_ACC = 0xa0;      // Read only
static const uint8_t REG_ADCCON1 = 0xa1;
static const uint8_t REG_ADCCON2 = 0xa2;
static const uint8_t REG_ADCDLY = 0xa3;
static const uint8_t REG_C0L = 0xa4;
static const uint8_t REG_C0H = 0xa5;
static const uint8_t REG_C1L = 0xa6;
static const uint8_t REG_C1H = 0xa7;
static const uint8_t REG_ADCCON0 = 0xa8;
static const uint8_t REG_PICON = 0xa9;    // Read only
static const uint8_t REG_PINEN = 0xaa;    // Read only
static const uint8_t REG_PIPEN = 0xab;    // Read only
static const uint8_t REG_PIF = 0xac;      // Read only
static const uint8_t REG_C2L = 0xad;
static const uint8_t REG_C2H = 0xae;
static const uint8_t REG_EIP = 0xaf;      // Read only
static const uint8_t REG_B = 0xb0 ;       // Read only
static const uint8_t REG_CAPCON3 = 0xb1;
static const uint8_t REG_CAPCON4 = 0xb2;
static const uint8_t REG_SPCR = 0xb3;
static const uint8_t REG_SPCR2 = 0xcc;    // Page 1 // Reassigned from 0xb3 to avoid collision
static const uint8_t REG_SPSR = 0xb4;
static const uint8_t REG_SPDR = 0xb5;
static const uint8_t REG_AINDIDS0 = 0xb6;
static const uint8_t REG_AINDIDS1 = 0; // Added to have common code with SuperIO
static const uint8_t REG_EIPH = 0xb7;     // Read only
static const uint8_t REG_SCON_1 = 0xb8;
static const uint8_t REG_PDTEN = 0xb9;    // TA protected
static const uint8_t REG_PDTCNT = 0xba;   // TA protected
static const uint8_t REG_PMEN = 0xbb;
static const uint8_t REG_PMD = 0xbc;
static const uint8_t REG_EIP1 = 0xbe;     // Read only
static const uint8_t REG_EIPH1 = 0xbf;    // Read only


static const uint8_t REG_USER_FLASH = 0xD0;
static const uint8_t REG_FLASH_PAGE = 0xF0;

static const uint8_t REG_INT = 0xf9;
static const uint8_t MASK_INT_TRIG = 0x1;
static const uint8_t MASK_INT_OUT = 0x2;
static const uint8_t BIT_INT_TRIGD = 0;
static const uint8_t BIT_INT_OUT_EN = 1;
static const uint8_t BIT_INT_PIN_SWAP = 2;    // 0 = P1.3, 1 = P0.0

static const uint8_t REG_INT_MASK_P0 = 0x00;
static const uint8_t REG_INT_MASK_P1 = 0x01;
static const uint8_t REG_INT_MASK_P3 = 0x03;

static const uint8_t REG_ADDR = 0xfd;

static const uint8_t REG_CTRL = 0xfe;         // 0 = Sleep, 1 = Reset, 2 = Read Flash, 3 = Write Flash, 4 = Addr Unlock
static const uint8_t MASK_CTRL_SLEEP = 0x1;
static const uint8_t MASK_CTRL_RESET = 0x2;
static const uint8_t MASK_CTRL_FREAD = 0x4;
static const uint8_t MASK_CTRL_FWRITE = 0x8;
static const uint8_t MASK_CTRL_ADDRWR = 0x10;

static const int BIT_ADDRESSED_REGS_SIZE = 4;
static uint8_t BIT_ADDRESSED_REGS[BIT_ADDRESSED_REGS_SIZE] = {REG_P0, REG_P1, REG_P2, REG_P3};

// from sioe_regs:
static const uint8_t REG_PWM0CON0 = 0xaa;
static const uint8_t REG_PWM0CON1 = 0xab;

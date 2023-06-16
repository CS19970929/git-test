#ifndef I2C_H
#define I2C_H

#define AT24C02

#define sEE_MAX_TRIALS_NUMBER     300

#define sEE_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define sEE_LONG_TIMEOUT         ((uint32_t)(10 * sEE_FLAG_TIMEOUT))
#define sEE_HW_ADDRESS           0xA0   /* E0 = E1 = E2 = 0 */ 
#define sEE_PAGESIZE   			 64

#define sEE_OK         0
#define sEE_FAIL       1   
#define sEE_I2C        I2C2

extern uint8_t sEE_I2CFaultcnt;

#ifdef __EEPROM_TEST
extern UINT16 a;
extern UINT16 b;
extern UINT16 c;
extern UINT16 d;
#endif

void PwrMag_WrDataToE2prom(void);
void I2C_EE_Init(void);
void EEPROM_test(void);

uint16_t ReadEEPROM_Word_NoZone(uint16_t addr);
int8_t WriteEEPROM_Word_NoZone(uint16_t addr, uint16_t data);


#endif	/* I2C_H */



#include <Arduino.h>
#include <Wire.h>
#include <mkigor_std.h>

#ifndef mkigor_BMI160_h
#define mkigor_BMI160_h

// BMI160 Command definitions
#define START_FOC	   0x03
#define ACC_SET_PMU_MODE  0x10	// | 0b0001 00nn
#define GYR_SET_PMU_MODE  0x14	// | 0b0001 01nn
#define MAG_SET_PMU_MODE  0x18	// | 0b0001 10nn
#define PROG_NVM       0xA0
#define FIFO_FLUSH     0xB0
#define INT_RESET      0xB1
#define SOFTRESET      0xB6
#define STEP_CNT_CLR   0xB2

//================================================



//================================================

class cl_BMI160 
{
private:
	uint8_t clv_i2cAddr;
	uint8_t clv_codeChip;

public:
	cl_BMI160() {				///	default class constructor
		clv_i2cAddr = 0x69;		///	default BMI160 i2c address
		clv_codeChip = 0;		///	default code chip 0 => not found.
	}

	uint8_t check(uint8_t lp_i2cAddr);
	uint8_t readReg(uint8_t reg_addr);
	void	readRegs(uint8_t reg_addr, uint8_t* lp_buf, uint8_t lp_len);
	uint8_t writeReg(uint8_t reg_addr, uint8_t reg_data);
	uint8_t doCMD(uint8_t lp_cmd);
	void	readGyrAccRaw(int16_t *ptrOut_GyrAccRaw);
	void	readGyrAccAvrN(int16_t *ptrOut_GyrAccRaw, uint8_t lp_n);
	void	readAll();
	void	beginStep();
	void	init();
};

#endif
//=================================================================================
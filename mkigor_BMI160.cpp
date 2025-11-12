/**
 * @file mkigor_BMI160.cpp
 * @author your name (you@domain.com)
 * @brief Short Atdruino library for BMI160 6-axis IMU sensor with minimum functionality to read Gyro and Accel data
 * @version 1.0
 * @date 2025-11-12
 * 
 * @remarks	Glossary, abbreviations used in the module. Name has small or capital letters ("camelCase"),
 * 	and consist only 2 or 1 symbol '_', that divide it in => prefix + name + suffix.
 *	prefix: 
 *		gv_*	- Global Variable;
 *		lv_*	- Local Variable (live inside statement);
 *		cl_*	- CLass;
 *		cd_*	- Class Definition;
 *		cgv_*	- Class public (Global) member (Variable);
 *		clv_*	- Class private (Local) member (Variable);
 *		cgf_*	- Class public (Global) metod (Function), not need, no usefull, becouse we see parenthesis => ();
 *		clf_*	- Class private (Local) metod (Function);
 *		lp_*	- in function, local parameter.
 *	suffix:
 *		like ending *_t, as usual, point to the type, informative, but not mandatory to use.
 *		possible is: _i8, _i16, _i32, _i64, _u8, _u16, _u32, _u64, _f, _df, _c, _b, _stru, etc.
 *	example:	- prefix_nameOfFuncOrVar_suffix, gv_tphg_stru => global var (tphg) structure.
 */
#include <mkigor_BMI160.h>

//==========================================================================

/**
 * @brief Check for presence BMI160 on I2C bus by addr lp_i2cAddr
 * 
 * @param lp_i2cAddr 
 * @return uint8_t chip Code if Ok, else 0x00
 */
uint8_t cl_BMI160::check(uint8_t lp_i2cAddr) {
	clv_i2cAddr = lp_i2cAddr;
	uint8_t lv_error;
	clv_codeChip = 0;
	Wire.begin();
    Wire.beginTransmission(clv_i2cAddr);
    lv_error = Wire.endTransmission();
    if (lv_error == 0) {
        Wire.beginTransmission(clv_i2cAddr);
        Wire.write(0x00);   // Select register address = 0xD0 of chip_id
        Wire.endTransmission();
		if (Wire.requestFrom(clv_i2cAddr, 1U) == 1) clv_codeChip = Wire.read();
		Wire.endTransmission(true);
		}
    return clv_codeChip;
}

uint8_t cl_BMI160::readReg(uint8_t reg_addr) {
	uint8_t lv_ret = 0xFF;
	Wire.beginTransmission(clv_i2cAddr);
	Wire.write(reg_addr);
	Wire.endTransmission();
	if (Wire.requestFrom(clv_i2cAddr, 1U) == 1) lv_ret = Wire.read();
	Wire.endTransmission(true);
	return lv_ret;
}

void	cl_BMI160::readRegs(uint8_t reg_addr, uint8_t* lp_buf, uint8_t lp_len) {
	Wire.beginTransmission(clv_i2cAddr);
	Wire.write(reg_addr);
	Wire.endTransmission();
	if (Wire.requestFrom(clv_i2cAddr, lp_len) == lp_len)
		for (uint8_t i = 0; i < lp_len; i++)
			lp_buf[i] = Wire.read();
}

uint8_t cl_BMI160::writeReg(uint8_t reg_addr, uint8_t reg_data) {
	Wire.beginTransmission(clv_i2cAddr);
	Wire.write(reg_addr);
	Wire.endTransmission();
	Wire.write(reg_data);
	uint8_t r = Wire.endTransmission(true);
	return r;
}

uint8_t cl_BMI160::doCMD(uint8_t lp_cmd) {
	/*
CMD Register (0x7E) => Command register triggers operations like softreset, NVM programming, etc.
cmd:
start_foc: 0x03
	Starts Fast Offset Calibration for the accel and gyro as configured in Register (0x69)
	FOC_CONF and stores the result into the Register (0x71-0x77) OFFSET register.
acc_set_pmu_mode: 0b0001 00nn
	Sets the PMU mode for the accelerometer. The encoding for ‘nn’ is identical to
	acc_pmu_status in Register (0x03) PMU_STATUS.
gyr_set_pmu_mode: 0b0001 01nn
	Sets the PMU mode for the gyroscope. The encoding for ‘nn’ is identical to
	gyr_pmu_status in Register (0x03) PMU_STATUS
mag_set_pmu_mode: 0b0001 10nn
	Sets the PMU mode for the mag interface. The encoding for ‘nn’ is identical to
	mag_pmu_status in Register (0x03) PMU_STATUS.
prog_nvm: 0xA0
	Writes the NVM backed registers into NVM.
fifo_flush: 0xB0
	clears all data in the FIFO, does not change the Register (0x46-0x47) FIFO_CONFIG and
	Register (0x45) FIFO_DOWNS registers.
int_reset: 0xB1
	resets the interrupt engine, the Register (0x1C-0x1F) INT_STATUS and the interrupt pin.
softreset: 0xB6
	triggers a reset including a reboot. Other values are ignored. Following a delay, all user
	configuration settings are overwritten with their default state or the setting stored in the
	NVM, wherever applicable. This register is functional in all operation modes.
step_cnt_clr: 0xB2
	triggers a reset of the step counter. This register is functional in all operation modes.

PMU_STATUS Register (0x03) => Shows the current power mode of the sensor.
<5:4> acc_pmu_status	0b00 Suspend	0b01 Normal	0b10 Low Power
<3:2> acc_pmu_status	0b00 Suspend	0b01 Normal	0b10 Reserved	0b11 Fast Start-Up
<1:0> mag_pmu_status	0b00 Suspend	0b01 Normal	0b10 Low Power

	*/
	Wire.beginTransmission(clv_i2cAddr);
	Wire.write(0x7E);
	Wire.endTransmission();
	Wire.write(lp_cmd);
	return Wire.endTransmission(true);
}

void cl_BMI160::readGyrAccRaw(int16_t *ptrOut_GyrAccRaw) {
	uint8_t n = 12;
	uint8_t lv_buf[n];
	readRegs(0x0C, lv_buf, n);
	ptrOut_GyrAccRaw[0] = (int16_t(lv_buf[1]) << 8) | lv_buf[0]; //gx
	ptrOut_GyrAccRaw[1] = (int16_t(lv_buf[3]) << 8) | lv_buf[2]; //gy
	ptrOut_GyrAccRaw[2] = (int16_t(lv_buf[5]) << 8) | lv_buf[4]; //gz
	ptrOut_GyrAccRaw[3] = (int16_t(lv_buf[7]) << 8) | lv_buf[6]; //ax
	ptrOut_GyrAccRaw[4] = (int16_t(lv_buf[9]) << 8) | lv_buf[8]; //ay
	ptrOut_GyrAccRaw[5] = (int16_t(lv_buf[11]) << 8) | lv_buf[10]; //az
	// gv_senTime = (uint32_t)((lv_buf[12]) | (lv_buf[13] << 8) | (lv_buf[14] << 16));
}

/**
 * @brief Read Gyr & Acc n times and calc average result
 * 
 * @param ptr_GyrAccRaw array[6] of int16_t to output GyrX, GyrY, GyrZ, AccX, AccY, AccZ
 * @param lp_n times to read and average
 */
void cl_BMI160::readGyrAccAvrN(int16_t *ptrOut_GyrAccRaw, uint8_t lp_n) {
	int32_t lv_GyrAccSum[6] = { 0,0,0,0,0,0 };
	int16_t	lv_GyrAccRaw[6];
	for (uint16_t i = 0; i < lp_n; i++) {
		readGyrAccRaw(lv_GyrAccRaw);
		lv_GyrAccSum[0] += lv_GyrAccRaw[0];
		lv_GyrAccSum[1] += lv_GyrAccRaw[1];
		lv_GyrAccSum[2] += lv_GyrAccRaw[2];
		lv_GyrAccSum[3] += lv_GyrAccRaw[3];
		lv_GyrAccSum[4] += lv_GyrAccRaw[4];
		lv_GyrAccSum[5] += lv_GyrAccRaw[5];
	}
	ptrOut_GyrAccRaw[0] = lv_GyrAccSum[0] / lp_n;
	ptrOut_GyrAccRaw[1] = lv_GyrAccSum[1] / lp_n;
	ptrOut_GyrAccRaw[2] = lv_GyrAccSum[2] / lp_n;
	ptrOut_GyrAccRaw[3] = lv_GyrAccSum[3] / lp_n;
	ptrOut_GyrAccRaw[4] = lv_GyrAccSum[4] / lp_n;
	ptrOut_GyrAccRaw[5] = lv_GyrAccSum[5] / lp_n;
}

void cl_BMI160::readAll() {
	int8_t lv_len = 127;
	uint8_t lv_buf[lv_len];
	for (uint8_t i = 0; i < lv_len; i++) lv_buf[i] = readReg(i);

	mkistdf_prnBuf(lv_buf, lv_len);
}

void cl_BMI160::beginStep() {
	/*
STEP_CONF Register (0x7A-0x7B)=> Step detector and step counter configuration

The step detector and step counter are disabled by default. To enable the step counter,
the bit step_cnt_en <3> in Register (0x7B) STEP_CONF_1 must be set to ‘1’.
The step detector generates an interrupt on each detected step. The step counter
counts the number of detected steps and stores the value in the Register (0x78-0x79)
STEP_CNT.

The step counter can be configured in different modes by setting the Register (0x7A) STEP_CONF[1]
as described in the following table.

Normal mode:
	Recommended for most applications. Well balanced between false positives and false negatives.
STEP_CONF[0, 1]: 0x15 (0b0001 0101) 0x03 (0b0000 0011) (the step_cnt_en bit is set to 0)
Sensitive mode:
	Recommended for light weighted persons. Will give few false negatives but eventually more false positives.
STEP_CONF[0, 1]: 0x2D (0b0010 1101)  0x00 (0b0000 0000) (the step_cnt_en bit is set to 0)
Robust mode:
	Will give few false positives but eventually more false negatives.
STEP_CONF[0, 1]: 0x1D (0b0001 1101) 0x07 (0b0000 0111) (the step_cnt_en bit is set to 0)

The step counter register can be read out at Register (0x78-0x79) STEP_CNT. The step counter
can be reset by sending the command 0xB2 to the Register (0x7E) CMD.
	*/

	writeReg(0x7A, 0x15); //step config normal mode
	writeReg(0x7B, 0x0B); //enable step counter
	//	should make delay(10);
}

/**
 * @brief	Initializes the sensor BMI160 to max sensetivty. All interrupts ON. FIFO is disabled.
 * 			A little correct offset values in the mine copy bmi160.
 */,
void cl_BMI160::init() {
	doCMD(SOFTRESET); 				delay(100);
	doCMD(ACC_SET_PMU_MODE | 0x01);	delay(100); //set acc to normal mode
	doCMD(GYR_SET_PMU_MODE | 0x01);	delay(100);	//set gyro to normal mode
	doCMD(MAG_SET_PMU_MODE | 0x00);	delay(100);	//set mag to Suspend mode
	doCMD(STEP_CNT_CLR);			delay(100);
	beginStep(); 					delay(100);

	writeReg(0x40, 0x2C);	// Accelerometer configuration (reg 0x40) = 0x2C: ODR 100Hz, BW 32Hz, normal mode
	writeReg(0x41, 0x03);	// Accelerometer range (reg 0x41) = 0x03: +/-2g
	writeReg(0x42, 0x2C);	// Gyroscope configuration (reg 0x42) = 0x2C: ODR 100Hz, BW 32Hz, normal mode
	writeReg(0x43, 0x00);	// Gyroscope range (reg 0x43) = 0x00: +/-2000dps

	writeReg(0x50, 0xff);	//	Register (0x50-0x52) INT_EN => Enable All interrupt 
	writeReg(0x51, 0xff);
	writeReg(0x52, 0xff);
	writeReg(0x54, 0x0F);	// INT_LATCH => Latch until INT_STATUS read (reg 0x54)

	writeReg(0x71, 0);		//	Ax offset
	writeReg(0x72, 23);		//	Ax offset
	writeReg(0x73, 0);		//	Az offset
	writeReg(0x74, 0);		//	Gx offset
	writeReg(0x75, 6);		//	Gy offset
	writeReg(0x76, 0);		//	Gz offset
	writeReg(0x77, 0xC0);	//	offset enable
}

//==========================================================================
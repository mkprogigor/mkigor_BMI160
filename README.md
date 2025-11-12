# mkigor_BMI160
Short Arduino library for Bosch sensor BMI160.
It is minimum to start work with BMI160.

Function (method) => `uint8_t check(uint8_t lp_i2cAddr)`<BR>
It must be call 1st, because it checks presence of sensor!<BR>
Fn check connection with sensor by I2C address `lp_i2cAddr` (usually 0x69).<BR>
Fn return byte: 0 = if sensor does not present or CHIP CODE (usually 0xD1).<BR>
Example of use:<BR>
```c++
if	(cl_bmi.check(0x69) != 0xD1) {
		Serial.println("BMI160 not found at address 0x69. Check connections.");
		delay(3000);	return;
}
```
<BR>
Function => `void init()`<BR>
Min initializes the sensor BMI160 to max sensetivty. All interrupts ON. FIFO is disabled.<BR>

Function => `void	readGyrAccAvrN(int16_t *ptrOut_GyrAccRaw, uint8_t lp_n);`<BR>
Read Gyr & Acc n times and calc average result:<BR>
`ptrOut_GyrAccRaw	= array[6] of int16_t to output average values of GyrX, GyrY, GyrZ, AccX, AccY, AccZ`<BR>
`uint8_t lp_n	= times to read and average.`<BR>
<BR>
...
<BR>
I used oficial Bosch datasheet BMI160 and Github.
I thanks authors for help in coding:<BR>
https://github.com/hanyazou/BMI160-Arduino<BR>
https://github.com/DFRobot/DFRobot_BMI160<BR>

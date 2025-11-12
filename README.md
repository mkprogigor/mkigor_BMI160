# mkigor_BMI160
Short Arduino library for Bosch sensor BMI160.
It is minimum to start work with BMI160.

Function (method) => `uint8_t check(uint8_t lp_i2cAddr)`<BR>
It must be call 1st, because it checks presence of sensor!<BR>
Fn check connection with sensor by I2C address `lp_i2cAddr` (usually 0x69).<BR>
Fn return byte: 0 = if sensor does not present or CHIP CODE in otherwise.<BR>
Possible chip codes are: 0x58=>BMP280, 0x60=>BME280, 0x61=>BME680.<BR>
Note: i2c address 0x76, 0x77 possible for BMP280 or BME280 or BME680 - pl, check it.<BR>

Function => `void init()`<BR>
Min initializes the sensor BMI160 to max sensetivty. All interrupts ON. FIFO is disabled.<BR>

Function => `void	readGyrAccAvrN(int16_t *ptrOut_GyrAccRaw, uint8_t lp_n);`<BR>
Read Gyr & Acc n times and calc average result:<BR>
`ptrOut_GyrAccRaw	= array[6] of int16_t to output average values of GyrX, GyrY, GyrZ, AccX, AccY, AccZ`<BR>
`	uint8_t lp_n	= times to read and average,`<BR>
<BR>
...
<BR>
I used oficial Bosch datasheet BMI160 and Github.
I thanks authors for help in coding:<BR>
https://github.com/hanyazou/BMI160-Arduino<BR>
https://github.com/DFRobot/DFRobot_BMI160<BR>

Example of use this lib in my project of "Easy DIY weather station"<BR>
https://github.com/mkprogigor/mkigor_esp32c3_ws<BR>

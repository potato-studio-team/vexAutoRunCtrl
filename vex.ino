/*
 Name:    vex.ino
 Created: 2020/8/14 18:19:26
 Author:  chen1
*/
#include<Wire.h>
#include <AFMotor.h>

const float g = 9.79666; // 重力加速度的值
const int averageTime = 5; // 传感器结果计算平均值时元素的个数
const int nCalibTimes = 50; // 校准mpu时读数的次数，!注意!填入的值应<=100
const int MPU = 0x68; //MPU-6050的I2C地址

int calibResultA[5] = { 0, 0, 0, 0, 0 }; // mpu校准与测量差值，及校准返回值，基准值为0：AccX， AccY，GyroX，GyroY，GyroZ
int calibResultB = 0; // mpu校准与测量差值，及校准返回值，基准为g：AccZ
int AccTime = 2; // 默认Acc倍数
int GyroTime = 255; // 默认Gyro倍数

//----------------------------------------------------------------------------------------------MPU6050-----------------------------------------------------------------------------------------//

// 配置倍率 Acc
void settingAccTime(int AccF) {
	float AccTime = 2.0;

	// 配置倍率 Acc
	if (AccF <= 3 & AccF > 0) { // 验证是否更改了配置，验证配置是否输入正确
		Wire.beginTransmission(MPU); //开启MPU-6050的传输
		Wire.write(0x1C); // 加速度倍率寄存器的地址
		Wire.requestFrom(0x68, 1, true); // 请求读出原配置

		unsigned char acc_conf = Wire.read();// 原配置读出
		acc_conf = ((acc_conf & 0xE7) | (AccF << 3));// 倍率配置更新

		Wire.write(acc_conf); // 更新倍率写入
		Wire.endTransmission(true); // 结束传输，释放总线
	}

	// 重新设置倍率 Acc
	if (AccF == 0) AccTime = 2.0;
	else if (AccF == 1) AccTime = 4.0;
	else if (AccF == 2) AccTime = 8.0;
	else if (AccF == 3) AccTime = 16.0;
	else AccTime = 2;
}

// 配置倍率 Gyro
void settingGyroTime(int GyroF) {
	float AccTime = 2.0;

	// 配置倍率 Gyro
	if (GyroF <= 3 & GyroF > 0) { // 验证是否更改了配置，验证配置是否输入正确
		Wire.beginTransmission(MPU); // 开启MPU-6050的传输
		Wire.write(0x1B); // 角速度倍率寄存器的地址
		Wire.requestFrom(0x68, 1, true); // 请求读出原配置

		unsigned char acc_conf = Wire.read();// 原配置读出
		acc_conf = ((acc_conf & 0xE7) | (GyroF << 3));// 倍率配置更新

		Wire.write(acc_conf); // 更新倍率写入
		Wire.endTransmission(true); //结束传输，释放总线
	}

	// 重新设置倍率 Gyro
	if (GyroF == 0) GyroTime = 250.0;
	else if (GyroF == 1) GyroTime = 500.0;
	else if (GyroF == 2) GyroTime = 1000.0;
	else if (GyroF == 3) GyroTime = 2000.0;
	else GyroTime = 2;
}

// 校准mpu6050
void calibrationMpu(String type) {
	int16_t AccX, AccY, AccZ, Temp, GyroX, GyroY, GyroZ;
	long data[nCalibTimes];
	float sum = 0.0;
	int result = 0;

	// 先测量次数个值
	for (int i = 0; i < nCalibTimes; i++) {
		Wire.beginTransmission(0x68);
		Wire.write(0x3B);
		Wire.endTransmission(false);
		Wire.requestFrom(0x68, 14, true);

		AccX = Wire.read() << 8 | Wire.read();
		AccY = Wire.read() << 8 | Wire.read();
		AccZ = Wire.read() << 8 | Wire.read();
		Temp = Wire.read() << 8 | Wire.read();
		GyroX = Wire.read() << 8 | Wire.read();
		GyroY = Wire.read() << 8 | Wire.read();
		GyroZ = Wire.read() << 8 | Wire.read();

		// 数组赋值
		if (type == "AccX") data[i] = AccX;
		else if (type == "AccY") data[i] = AccY;
		else if (type == "AccZ") data[i] = AccZ;
		else if (type == "Temp") data[i] = Temp;
		else if (type == "GyroX") data[i] = GyroX;
		else if (type == "GyroY") data[i] = GyroY;
		else if (type == "GyroZ") data[i] = GyroZ;
		else data[i] = 0;
	}

	// 判断基准值符合0的数据种类
	if (type != "Temp" & type != "AccZ") {
		for (int i = 0; i < nCalibTimes; i++) {
			sum = data[i] - 0 + sum; // 0为默认基准值
		}
		result = round(sum / nCalibTimes);

		if (type == "AccX") calibResultA[0] = result;
		else if (type == "AccY") calibResultA[1] = result;
		else if (type == "GyroX") calibResultA[2] = result;
		else if (type == "GyroY") calibResultA[3] = result;
		else if (type == "GyroZ") calibResultA[4] = result;
	}
	// 校准值为Z轴
	else if (type == "AccZ") {
		for (int i = 0; i < nCalibTimes; i++) {
			sum = data[i] - 32768 / g + sum; //g为默认基准值
		}
		calibResultB = sum / nCalibTimes;
	}
	else {
		for (int i = 0; i < 5; i++) {
			calibResultA[i] = 0;
		}
		calibResultB = 0;
	}
}

// 获取mpu6050信息
float getAccGyro(String type) {

	int16_t AccX, AccY, AccZ, Temp, GyroX, GyroY, GyroZ;

	float GyroTime = 250.0;
	int data[averageTime];
	int time = 0;
	float result = 0.0;

	for (int i = 0; i < averageTime; i++) {
		Wire.beginTransmission(0x68);
		Wire.write(0x3B);
		Wire.endTransmission(false);
		Wire.requestFrom(0x68, 14, true);
		AccX = Wire.read() << 8 | Wire.read();
		AccY = Wire.read() << 8 | Wire.read();
		AccZ = Wire.read() << 8 | Wire.read();
		Temp = Wire.read() << 8 | Wire.read();
		GyroX = Wire.read() << 8 | Wire.read();
		GyroY = Wire.read() << 8 | Wire.read();
		GyroZ = Wire.read() << 8 | Wire.read();

		if (type == "AccX") data[i] = AccX;
		else if (type == "AccY") data[i] = AccY;
		else if (type == "AccZ") data[i] = AccZ;
		else if (type == "Temp") data[i] = Temp;
		else if (type == "GyroX") data[i] = GyroX;
		else if (type == "GyroY") data[i] = GyroY;
		else if (type == "GyroZ") data[i] = GyroZ;
		else data[i] = 0;
	}


	// 计算总和
	for (int i = 0; i < averageTime; i++) {
		if (data[i] == 0) {// 排除无效数据
			continue;
		}
		else
		{
			time++;
			result = result + data[i];
		}
	}

	// 计算
	if (result != 0) {
		// 计算平均数
		result = result / time;

		//引入校准差值
		if (type != "Temp" & type != "AccZ") {
			if (type == "AccX")  result = result - calibResultA[0];
			else if (type == "AccY") result = result - calibResultA[1];
			else if (type == "GyroX") result = result - calibResultA[2];
			else if (type == "GyroY") result = result - calibResultA[3];
			else if (type == "GyroZ") result = result - calibResultA[4];
		}

		// 计算物理量
		if (type == "AccX" || type == "AccY" || type == "AccZ") {
			result = AccTime * g * result / 32768;
		}
		else if (type == "GyroX" || type == "GyroY" || type == "GyroZ") {
			result = GyroTime * result / 32768;
		}
		return result;
	}
	else
	{
		return 0;
	}
}

//----------------------------------------------------------------------------------------------HC-SR04-----------------------------------------------------------------------------------------//

// 获取距离
long getDistance(void) {
	long data[averageTime];
	long duration, mm;
	double result = 0L;
	int time = 0;

	// 测量距离averageTime次
	for (int i = 0; i < averageTime; i++) {
		// 发出声波
		digitalWrite(A1, LOW);
		delayMicroseconds(5);
		digitalWrite(A1, HIGH);
		delayMicroseconds(10);
		digitalWrite(A1, LOW);

		duration = pulseIn(A0, HIGH);// pulsein 读取波的持续时间

		// 计算
		mm = (duration / 2) * 0.34;

		/*
		Serial.print(mm);
		Serial.print("mm");
		Serial.println();

		delay(100);
		*/
		data[i] = mm;
	}

	// 计算总和
	for (int i = 0; i < averageTime; i++) {
		if (data[i] == 0) {// 排除无效数据
			continue;
		}
		else
		{
			time++;
			result = result + data[i];
		}
	}

	// 计算平均数
	if (result != 0) {
		result = result / time;
		return round(result);
	}
	else
	{
		return 0;
	}

}

//----------------------------------------------------------------------------------------------AFMotor-----------------------------------------------------------------------------------------//

//建立电机对象
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

void LmotorGo(int speed) {
	// 设置电机速度
	motor2.setSpeed(abs(speed));

	// 电机在启动时停止转动
	motor2.run(RELEASE);

	// 判断旋转方向
	if (speed <= 0) {
		motor2.run(BACKWARD);
	}
	else
	{
		motor2.run(FORWARD);
	}
}

void RmotorGo(int speed) {
	// 设置电机速度
	motor1.setSpeed(abs(speed));

	// 电机在启动时停止转动
	motor1.run(RELEASE);

	// 判断旋转方向
	if (speed <= 0) {
		motor1.run(FORWARD);
	}
	else
	{
		motor1.run(BACKWARD);
	}
}

//===================================================Main===================================================//

//===================================================GoGo===================================================//

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);// 设置串口监视器波特率
	// mpu setup
	Wire.begin();//默认作为master设备打开
	Serial.begin(9600);//串口通讯的波特率设置为9600
	Wire.beginTransmission(0x68);//MPU6050对应的I2C slave设备地址是0x68
	Wire.write(0x6B);//写入下一步要写的寄存器地址0x6b
	Wire.write(0);//向0x6b寄存器写入0，使MPU6050开始工作
	Wire.endTransmission(true);

	settingAccTime(0); // 设置Acc倍数
	settingGyroTime(1); // 设置Gyro倍数
	calibrationMpu("GyroY"); //校准mpu

	// 测距 setup
	pinMode(A1, OUTPUT);
	pinMode(A0, INPUT);
}

// the loop function runs over and over again until power down or reset
void loop() {
	RmotorGo(255);
	LmotorGo(255);
}
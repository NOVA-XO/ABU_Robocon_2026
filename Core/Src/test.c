/* =============================================================================
 *  test.c — Тоног төхөөрөмжийн тест функцууд
 *           (encoder, servo, sensor, joystick, motor, PWM)
 *
 *  Төсөл : STM32F407 Robot Firmware
 *  Огноо : Jul 1, 2026
 *  Автор : nova
 * =============================================================================
 */
#include "test.h"
#include "default.h"

extern I2C_HandleTypeDef hi2c2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;

extern int counter[];             // encoder тоолуурууд
extern int timer;                 // зөөлөн тоолуур
extern int control_data[5][4];    // джойстик/товчлуурын өгөгдөл

/* -----------------------------------------------------------------------------
 *  showEncoderStatus — 4 encoder + timer утгыг OLED дээр харуулах
 * -----------------------------------------------------------------------------
 */
void showEncoderStatus(void) {
    colorFill(Black);

    for (int i = 0; i < 4; ++i) {
        setCursor(4, i * 12 + 2);
        printStr("Encoder %d = %d", i + 1, counter[i]);
    }
    setCursor(4, 48);
    printStr("Timer = %d", timer);
    setScreen();
}

/* -----------------------------------------------------------------------------
 *  testOptocoupler — Соленоид 1..8-г control_data товчлууруудаар шалгах
 * -----------------------------------------------------------------------------
 */
void testOptocoupler(void){
	for (int i = 0; i < 4; i++) {
		controlSolenoid(i + 1, control_data[1][i]);
		controlSolenoid(i + 5, control_data[2][i]);
	}
}

/* -----------------------------------------------------------------------------
 *  testServo — Серво + brush-ийг джойстикоор турших, хурдыг OLED дээр харуулах
 * -----------------------------------------------------------------------------
 */
void testServo(void) {
    static int angle = 0;
   // static bool incrementFlag = true;
   // static bool decrementFlag = true;

    if(control_data[3][1]==1){
    	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 48000);
    	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 48000);
    }
    else{
    	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 24000);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 24000);
    }


    angle=control_data[0][3]*240;
/*
    if (control_data[1][2] == 1 && incrementFlag) {
        angle += 5;
        incrementFlag = false;
    } else if (control_data[1][2] == 0) {
        incrementFlag = true;
    }

    if (control_data[1][0] == 1 && decrementFlag) {
        angle -= 5;
        decrementFlag = false;
    } else if (control_data[1][0] == 0) {
        decrementFlag = true;
    }
 */
    if (angle > 24000) {
        angle = 24000;
    } else if (angle < 0) {
        angle = 0;
    }


    controlServo(true, angle);
    controlServo(false, angle);

	colorFill(Black);
    setCursor(2, 2);
    printStr("Speed = %d", angle);
    setScreen();

}

/* -----------------------------------------------------------------------------
 *  test_sensor — val1..val8 сенсорын утгыг OLED дээр 2 баганаар харуулах
 * -----------------------------------------------------------------------------
 */
void test_sensor(void){
	colorFill(Black);

    setCursor(2, 2);
    printStr("val2 = %d", val2);

    setCursor(2, 12);
    printStr("val4 = %d", val4);

    setCursor(2, 24);
    printStr("val6 = %d", val6);

    setCursor(2, 36);
    printStr("val8 = %d", val8);

    setCursor(64, 2);
    printStr("val1 = %d", val1);

    setCursor(64, 12);
    printStr("val3 = %d", val3);

    setCursor(64, 24);
    printStr("val5 = %d", val5);

    setCursor(64, 36);
    printStr("val7 = %d", val7);

    setScreen();
}

/* -----------------------------------------------------------------------------
 *  test_joyStick — Джойстик болон товчлууруудын бүх утгыг OLED дээр харуулах
 * -----------------------------------------------------------------------------
 */
void test_joyStick(void){
	colorFill(Black);

	for (int i = 0; i < 4; i++) {
	    setCursor(0, i * 12);
	    printStr("%d", control_data[0][i]);
	    //setCursor(12, i * 12);
	    //printStr("%d", control_data[1][i]);
	}
	for (int i = 1; i < 5; i++) {
	    for (int j = 0; j < 4; j++) {
	        setCursor(10 * (i-1) + 32, j * 12);
	        printStr("%d", control_data[i][j]);
	    }
	}

	setScreen();
}

/* -----------------------------------------------------------------------------
 *  test_motor — Мотор 1..6-г джойстикийн тэнхлэгүүдээр турших
 * -----------------------------------------------------------------------------
 */
void test_motor(void)
{
	    motor_control(1, control_data[0][0] * 10);
		motor_control(2, control_data[0][0] * 10);
		motor_control(3, control_data[0][1] * 10);
	    motor_control(4, control_data[0][1] * 10);
		motor_control(5, control_data[0][2] * 10);
		motor_control(6, control_data[0][2] * 10);
}

/* -----------------------------------------------------------------------------
 *  test_pwm — Бүх PWM сувгийг 0..1000 хүртэл аажим гүйлгэж турших (blocking)
 * -----------------------------------------------------------------------------
 */
void test_pwm(void)
{
	  for(int i = 0; i < 1000; i++)
	  {

		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, i); //M1
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, i); //M2
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, i); //M3
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, i); //M4
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i); //M5
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, i); //M6

		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, i);
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, i);
		  HAL_Delay(10);
	  }
}


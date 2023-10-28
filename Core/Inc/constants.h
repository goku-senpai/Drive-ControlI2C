#ifndef CONSTANTS_H
#define CONSTANTS_H


#define UART_RX_BUFFER_SIZE 100
#define SENDDELAY 10


#define SPEED_KF                        0.01f   ///< Speed konstant depending on motor
#define DUTYCYCLE_MAX                   90      ///< Output max limiter 100-90 for DUTY-Cycle
#define DUTYCYCLE_MIN                   20      ///< Output min limiter 100-0 for DUTY-Cycle
#define MAX_OUT                         0.1     ///< Output limiter 0-1 for PID-Controller
#define MAX_OUTPUT                      100     ///< Output limiter 0-100 for MOTOR-Controller


/**
 * @defgroup MOTOR setup
 * @{
 */

#define MOTOR_TIM_A                     TIM_CHANNEL_1   ///< Motor Timer
#define MOTOR_ENA1_PORT                 GPIOD           ///< Motor PWM Port
#define MOTOR_ENA1_PIN                  GPIO_PIN_13     ///< Motor PWM Pin

#define IN1_PIN                         GPIO_PIN_2      ///< Motor dir1 Port
#define IN1_PORT                        GPIOB           ///< Motor dir1 Pin
#define IN2_PIN                         GPIO_PIN_6      ///< Motor dir2 Port
#define IN2_PORT                        GPIOB           ///< Moroe dir2 Pin

#define POSITION_TOLERANCE              0.05            ///< Motor constant to adjust the output offset for control


/** @} */
/**
 * @defgroup ENCODER Analogencoder
 * @{
 */
/** 131(Übersetzung) *16 (Encoderauflösung) *4( 2*Rise and Fall flag) */
#define ENCODER_RESOLUTION              8384.0f  ///< analog Encoder Resolution
 
//Encoder Motor 1
#define ENCODER_M1_A_PIN                GPIO_PIN_0      ///< analog Encoder A PIN
#define ENCODER_M1_A_PORT               GPIOB           ///< analog Encoder A PORT
#define ENCODER_M1_A_ALTERNATE          GPIO_AF2_TIM4   ///< analog Encoder A TIM

#define ENCODER_M1_B_PIN                GPIO_PIN_1      ///< analog Encoder B PIN
#define ENCODER_M1_B_PORT               GPIOB           ///< analog Encoder B Port
#define ENCODER_M1_B_ALTERNATE          GPIO_AF2_TIM4   ///< analog Encoder B TIM

// PID Controller constants
/**
 * @defgroup PID pidcontroller
 * 
 */
#define POS_KP 1        ///< PID Kp value
#define POS_KI 0        ///< PID Kp value
#define POS_KD 0        ///< PID Kp value

#define DEFAULT_TARGET_START 0 ///< PID Start
#define DEFAULT_MAX_INTEGRAL 0 ///< Max Integral
 

/**
 * @defgroup LED Led
 * Onbooard LED defines
 *
 * @{
 */
/** pin of leds */
#define LED_RED_PIN                   GPIO_PIN_14
#define LED_BLUE_PIN                  GPIO_PIN_7
#define LED_GREEN_PIN                 GPIO_PIN_0
/** port of leds */
#define LED_GREEN_PORT                GPIOB
#define LED_RED_PORT                  GPIOB
#define LED_BLUE_PORT                 GPIOB


/**
 * @defgroup CONSTANTS
 * @{
 */
/** sampling time of controller in ms */
#define EXP_DT                              (50) //TODO: check diffs TO 50 trim params!!!!


#define M_PI                            3.14159265358979323846
#define RAD2DEG                         57.295779513082
#define DEG2RAD                         0.017453292519943


#endif
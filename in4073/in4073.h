/*------------------------------------------------------------------
 *  in4073.h -- defines, globals, function prototypes
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#ifndef IN4073_H__
#define IN4073_H__

#include <inttypes.h>
#include <stdio.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml.h"
#include "kalman.h"
#include "protocol.h"

#define RED					22
#define YELLOW				24
#define GREEN				28
#define BLUE				30
#define INT_PIN				5

#define MOTOR_0_PIN			21
#define MOTOR_1_PIN			23
#define MOTOR_2_PIN			25
#define MOTOR_3_PIN			29

 // additional
enum control_mode_t control_mode;
struct msg_telemetry_t msg_tele;
struct msg_profile_t msg_profile;
bool lost_flag;
bool bat_flag;
bool init_raw;
bool pc_link;
bool raw_status;
bool log_status;

uint16_t c1phi;
uint16_t c1theta;
uint16_t c2phi;
uint16_t c2theta;
int16_t estimated_p;
int16_t estimated_q;
int16_t bp;
int16_t bq;

int16_t r_butter;
int16_t isay;
int16_t isax;

uint8_t mmode;           			///< mode message from pc
uint16_t mthrust;           		///< thrust message from pc
int16_t mroll, mpitch, myaw;        ///< attitude message from pc

int16_t cphi, ctheta, cpsi;         ///< Calibration values of phi, theta, psi
int16_t cp, cq, cr;                ///< Calibration valies of p, q and r
int16_t csax, csay;                ///< Calibration values of sax, say
uint8_t P, P1, P2; 
extern uint8_t log_flag;               
       
int16_t ae[4];
void set_control_mode(enum control_mode_t mode);
//void set_control_gains(uint8_t yaw_d);
void set_control_gains(uint8_t yaw_d, uint8_t g_angle, uint8_t g_rate);
void set_control_command(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw);
void run_filters_and_control(void);

void calibration(void);

// Timers
#define TIMER_PERIOD	50000 //50000us=50ms=20Hz (MAX 16bit, 65ms)
void timers_init(void);
uint32_t get_time_us(void);
bool check_timer_flag(void);
void clear_timer_flag(void);

// GPIO
void gpio_init(void);
bool check_sensor_int_flag(void);
void clear_sensor_int_flag(void);

// Queue
#define QUEUE_SIZE 128
typedef struct {
	uint8_t Data[QUEUE_SIZE];
	uint8_t first,last;
  	uint8_t count;
} queue;
void init_queue(queue *q);
void enqueue(queue *q, char x);
char dequeue(queue *q);

// UART
#define RX_PIN_NUMBER  16
#define TX_PIN_NUMBER  14
queue rx_queue;
queue tx_queue;
void uart_init(void);
void uart_put(uint8_t);

// TWI
#define TWI_SCL	4
#define TWI_SDA	2
void twi_init(void);
bool i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data);
bool i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);

// MPU wrapper
int16_t phi, theta, psi;
int16_t sp, sq, sr; ///< 131 LSB / (degrees / s)
int16_t sax, say, saz;
uint8_t sensor_fifo_count;
void imu_init(bool dmp, uint16_t interrupt_frequency); // if dmp is true, the interrupt frequency is 100Hz - otherwise 32Hz-8kHz
void get_dmp_data(void);
int8_t get_dmp_data_encode(void);
void get_raw_sensor_data(void);
int8_t get_raw_sensor_data_encode(void);

// Barometer
int32_t pressure;
int32_t temperature;
void read_baro(void);
void baro_init(void);

// ADC
uint16_t bat_volt;
void adc_init(void);
void adc_request_sample(void);

// Flash
bool spi_flash_init(void);
bool flash_chip_erase(void);
bool flash_write_byte(uint32_t address, uint8_t data);
bool flash_write_bytes(uint32_t address, uint8_t *data, uint32_t count);
bool flash_read_byte(uint32_t address, uint8_t *buffer);
bool flash_read_bytes(uint32_t address, uint8_t *buffer, uint32_t count);

// BLE
queue ble_rx_queue;
queue ble_tx_queue;
void ble_init(void);
void ble_send(void);

//communication check;
void comm_check(uint16_t comm_duration, uint32_t *total_dur, bool *update_flag);

#endif // IN4073_H__

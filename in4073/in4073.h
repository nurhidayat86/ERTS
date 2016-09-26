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

#define RED				22
#define YELLOW				24
#define GREEN				28
#define BLUE				30
#define INT_PIN				5

#define MOTOR_0_PIN			21
#define MOTOR_1_PIN			23
#define MOTOR_2_PIN			25
#define MOTOR_3_PIN			29

// Fractions
#define CONTROL_FRAC 6            ///< The control gains fraction in powers of 2

// Control
enum control_mode_t {
  MODE_SAFE,
  MODE_PANIC,
  MODE_MANUAL,
  MODE_CALIBRATION,
  MODE_YAW,
  MODE_FULL,
  MODE_RAW,
  MODE_HEIGHT,
  ESCAPE
};

//log messages
struct log_t {
  int16_t phi, theta, psi, sp, sq, sr, sax, say, saz, roll, pitch, yaw, bat_volt, ae[4];
  uint16_t thrust;
  uint8_t mode;
  uint32_t temperature, pressure;
}__attribute__((packed, aligned(1)));

enum control_mode_t control_mode;
int16_t ae[4];
struct log_t log_write, log_read;

void set_control_mode(enum control_mode_t mode);
void set_control_gains(uint16_t yaw_d);
void set_control_from_js(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw);
void run_filters_and_control(void);

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
void get_raw_sensor_data(void);

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
//additional flash
bool write_log(void);
bool read_log (void);
bool flash_data (void);
bool status_log;
struct log_t log_msg;
uint8_t index_logging;

// BLE
queue ble_rx_queue;
queue ble_tx_queue;
void ble_init(void);
void ble_send(void);

#endif // IN4073_H__

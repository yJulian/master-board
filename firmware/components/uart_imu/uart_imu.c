#include "uart_imu.h"
#include <stdlib.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#define UART_NUM UART_NUM_1
#define BUF_SIZE 128
#define PIN_TXD 32
#define PIN_RXD 35

// IMU response definitions
#define IMU_RESPONSE_MSG_LEN_INCL_HEADER 40
#define IMU_RESPONSE_MSG_LEN_EXCL_HEADER 38
#define IMU_RESPONSE_MSG_HEADER_LEN 2
#define IMU_RESPONSE_SUCCESS 0xBB
#define IMU_RESPONSE_ERR 0xEE

//position of the data in the IMU message
#define IMU_RESPONSE_MSG_STATUS_POS 0
#define IMU_RESPONSE_MSG_LEN_POS 1
#define ACCX_POS 2
#define ACCY_POS 4
#define ACCZ_POS 6
#define MAGX_POS 8
#define MAGY_POS 10
#define MAGZ_POS 12
#define GYRX_POS 14
#define GYRY_POS 16
#define GYRZ_POS 18
#define EUL_YAW_POS 20
#define EUL_ROLL_POS 22
#define EUL_PITCH_POS 24
#define QUAT_W_POS 26
#define QUAT_X_POS 28
#define QUAT_Y_POS 30
#define QUAT_Z_POS 32
#define LINACC_X_POS 34
#define LINACC_Y_POS 36
#define LINACC_Z_POS 38

#define FLOAT_TO_D16QN(a, n) ((int16_t)((a) * (1 << (n))))
#define IMU_QN_MAG 4
#define IMU_QN_GYR 4
#define IMU_QN_EUL 4
#define IMU_QN_QUAT 14

typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
    float mag_x;
    float mag_y;
    float mag_z;
    float gyr_x;
    float gyr_y;
    float gyr_z;
    float roll;
    float pitch;
    float yaw;
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    float linacc_x;
    float linacc_y;
    float linacc_z;
} ImuData_t;

typedef struct
{
    uint8_t data[IMU_RESPONSE_MSG_LEN_EXCL_HEADER]; // containing [ACC_X_LSB, ACC_X_MSB, ACC_Y_LSB,...]
} ImuRawData_t;

static intr_handle_t handle_console;

// Receive buffer to collect incoming data from the ISR
ImuRawData_t latest_frame;

// Mailbox for safe message exchange between main thread and ISR
QueueHandle_t mailbox;

ImuData_t current_values;

int intr_cpt = 0;
uint8_t read_index_imu = 0; //where to read the latest updated imu data

/*
 * Define UART interrupt subroutine to ackowledge interrupt
 */
static void IRAM_ATTR uart_intr_handle(void *arg)
{
    uint16_t rx_fifo_len, status;
    status = UART1.int_st.val;             // read UART interrupt Status
    rx_fifo_len = UART1.status.rxfifo_cnt; // read number of bytes in UART buffer
    intr_cpt++;
    ImuRawData_t frame;

    if (rx_fifo_len >= IMU_RESPONSE_MSG_LEN_INCL_HEADER)
    {
        if (UART1.fifo.rw_byte == IMU_RESPONSE_SUCCESS) // Check status field
        {
            if (UART1.fifo.rw_byte == IMU_RESPONSE_MSG_LEN_EXCL_HEADER) // Check length field
            {
                for (uint8_t i = 0; i < IMU_RESPONSE_MSG_LEN_EXCL_HEADER; i++)
                {
                    frame.data[i] = UART1.fifo.rw_byte;
                }
                xQueueOverwriteFromISR(mailbox, &frame, NULL);
            }
            else
            {
                // Unexpected length
            }
        }
        else
        {
            // Non-success response start byte
        }
    }
    else
    {
        // Not enough in rx_fifo_len
    }

    // Fix of esp32 hardware bug as in https://github.com/espressif/arduino-esp32/pull/1849
    while (UART1.status.rxfifo_cnt || (UART1.mem_rx_status.wr_addr != UART1.mem_rx_status.rd_addr))
    {
        UART1.fifo.rw_byte;
    }
    
    // clear UART interrupt status
    uart_clear_intr_status(UART_NUM, status);
}

float get_accel_from_bytes(uint8_t lsb, uint8_t msb) {
    int16_t raw = (int16_t)((msb << 8) | lsb);
    return raw * 0.00981f; // convert mg to m/s²
}

float get_gyro_from_bytes(uint8_t lsb, uint8_t msb) {
    int16_t raw = (int16_t)((msb << 8) | lsb);
    return raw / 16.0f; // convert to °/s
}

float get_mag_from_bytes(uint8_t lsb, uint8_t msb) {
    int16_t raw = (int16_t)((msb << 8) | lsb);
    return (float)raw; // already in µT
}

float get_euler_from_bytes(uint8_t lsb, uint8_t msb) {
    int16_t raw = (int16_t)((msb << 8) | lsb);
    return raw / 16.0f; // convert to degrees
}

float get_quaternion_from_bytes(uint8_t lsb, uint8_t msb) {
    int16_t raw = (int16_t)((msb << 8) | lsb);
    return raw / 16384.0f; // convert to unit quaternion
}

float get_linacc_from_bytes(uint8_t lsb, uint8_t msb) {
    int16_t raw = (int16_t)((msb << 8) | lsb);
    return raw * 0.00981f; // convert mg to m/s²
}

inline int parse_IMU_data()
{
    xQueuePeek(mailbox, &latest_frame, 0);

    current_values.acc_x = get_accel_from_bytes(latest_frame.data[ACCX_POS], latest_frame.data[ACCX_POS + 1]);
    current_values.acc_y = get_accel_from_bytes(latest_frame.data[ACCY_POS], latest_frame.data[ACCY_POS + 1]);
    current_values.acc_z = get_accel_from_bytes(latest_frame.data[ACCZ_POS], latest_frame.data[ACCZ_POS + 1]);

    current_values.mag_x = get_mag_from_bytes(latest_frame.data[MAGX_POS], latest_frame.data[MAGX_POS + 1]);
    current_values.mag_y = get_mag_from_bytes(latest_frame.data[MAGY_POS], latest_frame.data[MAGY_POS + 1]);
    current_values.mag_z = get_mag_from_bytes(latest_frame.data[MAGZ_POS], latest_frame.data[MAGZ_POS + 1]);

    current_values.gyr_x = get_gyro_from_bytes(latest_frame.data[GYRX_POS], latest_frame.data[GYRX_POS + 1]);
    current_values.gyr_y = get_gyro_from_bytes(latest_frame.data[GYRY_POS], latest_frame.data[GYRY_POS + 1]);
    current_values.gyr_z = get_gyro_from_bytes(latest_frame.data[GYRZ_POS], latest_frame.data[GYRZ_POS + 1]);

    current_values.yaw  = get_euler_from_bytes(latest_frame.data[EUL_YAW_POS],  latest_frame.data[EUL_YAW_POS + 1]);
    current_values.roll = get_euler_from_bytes(latest_frame.data[EUL_ROLL_POS], latest_frame.data[EUL_ROLL_POS + 1]);
    current_values.pitch = get_euler_from_bytes(latest_frame.data[EUL_PITCH_POS], latest_frame.data[EUL_PITCH_POS + 1]);

    current_values.quat_w = get_quaternion_from_bytes(latest_frame.data[QUAT_W_POS], latest_frame.data[QUAT_W_POS + 1]);
    current_values.quat_x = get_quaternion_from_bytes(latest_frame.data[QUAT_X_POS], latest_frame.data[QUAT_X_POS + 1]);
    current_values.quat_y = get_quaternion_from_bytes(latest_frame.data[QUAT_Y_POS], latest_frame.data[QUAT_Y_POS + 1]);
    current_values.quat_z = get_quaternion_from_bytes(latest_frame.data[QUAT_Z_POS], latest_frame.data[QUAT_Z_POS + 1]);

    current_values.linacc_x = get_linacc_from_bytes(latest_frame.data[LINACC_X_POS], latest_frame.data[LINACC_X_POS + 1]);
    current_values.linacc_y = get_linacc_from_bytes(latest_frame.data[LINACC_Y_POS], latest_frame.data[LINACC_Y_POS + 1]);
    current_values.linacc_z = get_linacc_from_bytes(latest_frame.data[LINACC_Z_POS], latest_frame.data[LINACC_Z_POS + 1]);

    return 0;
}

uint16_t times_100_in_D16QN(float val) { return (int16_t)(val * 100); }

uint16_t get_acc_x_in_D16QN() { return times_100_in_D16QN(current_values.acc_x); }
uint16_t get_acc_y_in_D16QN() { return times_100_in_D16QN(current_values.acc_y); }
uint16_t get_acc_z_in_D16QN() { return times_100_in_D16QN(current_values.acc_z); }

uint16_t get_mag_x_in_D16QN() { return FLOAT_TO_D16QN(current_values.mag_x, IMU_QN_MAG); }
uint16_t get_mag_y_in_D16QN() { return FLOAT_TO_D16QN(current_values.mag_y, IMU_QN_MAG); }
uint16_t get_mag_z_in_D16QN() { return FLOAT_TO_D16QN(current_values.mag_z, IMU_QN_MAG); }

uint16_t get_gyr_x_in_D16QN() { return FLOAT_TO_D16QN(current_values.gyr_x, IMU_QN_GYR); }
uint16_t get_gyr_y_in_D16QN() { return FLOAT_TO_D16QN(current_values.gyr_y, IMU_QN_GYR); }
uint16_t get_gyr_z_in_D16QN() { return FLOAT_TO_D16QN(current_values.gyr_z, IMU_QN_GYR); }

uint16_t get_roll_in_D16QN() { return FLOAT_TO_D16QN(current_values.roll, IMU_QN_EUL); }
uint16_t get_pitch_in_D16QN() { return FLOAT_TO_D16QN(current_values.pitch, IMU_QN_EUL); }
uint16_t get_yaw_in_D16QN() { return FLOAT_TO_D16QN(current_values.yaw, IMU_QN_EUL); }

uint16_t get_quat_w_in_D16QN() { return FLOAT_TO_D16QN(current_values.quat_w, IMU_QN_QUAT); }
uint16_t get_quat_x_in_D16QN() { return FLOAT_TO_D16QN(current_values.quat_x, IMU_QN_QUAT); }
uint16_t get_quat_y_in_D16QN() { return FLOAT_TO_D16QN(current_values.quat_y, IMU_QN_QUAT); }
uint16_t get_quat_z_in_D16QN() { return FLOAT_TO_D16QN(current_values.quat_z, IMU_QN_QUAT); }

uint16_t get_linacc_x_in_D16QN() { return times_100_in_D16QN(current_values.linacc_x); }
uint16_t get_linacc_y_in_D16QN() { return times_100_in_D16QN(current_values.linacc_y); }
uint16_t get_linacc_z_in_D16QN() { return times_100_in_D16QN(current_values.linacc_z); }

void print_imu()
{
    printf("\n(%f %f %f) \t (%f %f %f) \t (%f %f %f) \t (%f %f %f) \t (%f %f %f %f) \t (%f %f %f)",
           current_values.acc_x,
           current_values.acc_y,
           current_values.acc_z,
           current_values.mag_x,
           current_values.mag_y,
           current_values.mag_z,
           current_values.gyr_x,
           current_values.gyr_y,
           current_values.gyr_z,
           current_values.roll,
           current_values.pitch,
           current_values.yaw,
           current_values.quat_w,
           current_values.quat_x,
           current_values.quat_y,
           current_values.quat_z,
           current_values.linacc_x,
           current_values.linacc_y,
           current_values.linacc_z
        );
}

void print_table(uint8_t *ptr, int len)
{
    for (int i = 0; i < len; i++)
    {
        printf("%02x ", ptr[i]);
    }
    printf("\n");
}

void custom_write_uart(unsigned char *buff, int size)
{
    int i = 0;
    for (i = 0; i < size; i++)
    {
        UART1.fifo.rw_byte = buff[i];
    }
}

void request_UART_read_periodically(void *params)
{
    const char read_command[4] = {0xAA, 0x01, 0x08, IMU_RESPONSE_MSG_LEN_EXCL_HEADER};
    while (1)
    {
        // Request the register read periodically
        custom_write_uart(read_command, sizeof(read_command));
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Debug output
        if (false) {
            //parse_IMU_data(); // Is called by the masterboard_main
            printf(" intr_cpt:%d\n", intr_cpt);
            print_imu();
        }
    }
}

// Initializes the UART IMU. THIS METHOD MUST NOT BE BLOCKING/INFITE LOOPING! (as other logic comes after the method call in the main)
int imu_init()
{
    printf("Initialising uart for IMU...\n");

    mailbox = xQueueCreate(1, sizeof(ImuRawData_t));

    // Configure UART 115200 bauds
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_rx_timeout(UART_NUM, 3); // timeout in symbols, this will wait to trigger the interrupt until no more data received for 3 symbol times
    uart_set_pin(UART_NUM, PIN_TXD, PIN_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    // Set IMU in NDOF mode
    const char mode_set_command[5] = {0xAA, 0x00, 0x3D, 0x01, 0x0C}; // 0x0C = NDOF mode
    vTaskDelay(100 / portTICK_PERIOD_MS); // Let the IMU some time to boot
    custom_write_uart(mode_set_command, sizeof(mode_set_command));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_disable_tx_intr(UART_NUM);
    uart_disable_rx_intr(UART_NUM);
    uart_isr_free(UART_NUM);
    uart_isr_register(UART_NUM, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console);
    uart_enable_rx_intr(UART_NUM);

    xTaskCreatePinnedToCore(
        request_UART_read_periodically,
        "UART periodic read",
        2048,           // Stack size
        NULL,           // Parameters
        5,              // Priority
        NULL,           // Task Handle
        tskNO_AFFINITY  // Mapping to core 0/1 or none
    );

    return 0;
}

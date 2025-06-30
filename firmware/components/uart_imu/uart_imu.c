#include "imu.h"

#if CONFIG_IMU_DRIVER_UART
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define FLOAT_TO_D16QN(a, n) ((int16_t)((a) * (1 << (n))))

#define UART_NUM UART_NUM_1
#define BUF_SIZE 128
#define PIN_TXD 32
#define PIN_RXD 35

/* BNO055 UART protocol definitions */
#define BNO_START_BYTE 0xAA
#define BNO_WRITE      0x00
#define BNO_READ       0x01
#define BNO_RESP_OK    0xBB
#define BNO_RESP_ERR   0xEE
#define BNO_ACK_OK     0x01

/* BNO055 register addresses (page 0) */
#define BNO_ACCEL_DATA_X_LSB      0x08
#define BNO_GYRO_DATA_X_LSB       0x14
#define BNO_EULER_H_LSB           0x1A
#define BNO_LINEAR_ACCEL_DATA_X_LSB 0x28
#define BNO_OPR_MODE              0x3D

/* Output scaling factors */
#define ACC_SCALE   (1.0f / 100.0f)
#define GYR_SCALE   (1.0f / 16.0f)
#define EULER_SCALE (1.0f / 16.0f)

#define IMU_QN_ACC 11
#define IMU_QN_GYR 11
#define IMU_QN_EF  13

struct imu_data {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyr_x;
    float gyr_y;
    float gyr_z;
    float roll;
    float pitch;
    float yaw;
    float linacc_x;
    float linacc_y;
    float linacc_z;
};

static struct imu_data imu;

static esp_err_t bno_write(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t buf[4 + len];
    buf[0] = BNO_START_BYTE;
    buf[1] = BNO_WRITE;
    buf[2] = reg;
    buf[3] = len;
    memcpy(&buf[4], data, len);
    uart_write_bytes(UART_NUM, (const char *)buf, 4 + len);

    uint8_t resp[2] = {0};
    int r = uart_read_bytes(UART_NUM, resp, 2, pdMS_TO_TICKS(100));
    if (r == 2 && resp[0] == BNO_RESP_ERR && resp[1] == BNO_ACK_OK)
        return ESP_OK;
    return ESP_FAIL;
}

static esp_err_t bno_read(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t cmd[4] = {BNO_START_BYTE, BNO_READ, reg, len};
    uart_write_bytes(UART_NUM, (const char *)cmd, 4);

    uint8_t header[2] = {0};
    int r = uart_read_bytes(UART_NUM, header, 2, pdMS_TO_TICKS(100));
    if (r != 2)
        return ESP_FAIL;
    if (header[0] == BNO_RESP_ERR)
        return ESP_FAIL;
    if (header[0] != BNO_RESP_OK)
        return ESP_FAIL;
    int to_read = header[1];
    if (to_read > len)
        to_read = len;
    r = uart_read_bytes(UART_NUM, data, to_read, pdMS_TO_TICKS(100));
    return (r == to_read) ? ESP_OK : ESP_FAIL;
}

static void update_sensor(void)
{
    uint8_t buf[6];

    if (bno_read(BNO_ACCEL_DATA_X_LSB, buf, 6) == ESP_OK) {
        int16_t ax = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t ay = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t az = (int16_t)((buf[5] << 8) | buf[4]);
        imu.acc_x = ax * ACC_SCALE;
        imu.acc_y = ay * ACC_SCALE;
        imu.acc_z = az * ACC_SCALE;
    }

    if (bno_read(BNO_GYRO_DATA_X_LSB, buf, 6) == ESP_OK) {
        int16_t gx = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t gy = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t gz = (int16_t)((buf[5] << 8) | buf[4]);
        imu.gyr_x = gx * GYR_SCALE;
        imu.gyr_y = gy * GYR_SCALE;
        imu.gyr_z = gz * GYR_SCALE;
    }

    if (bno_read(BNO_EULER_H_LSB, buf, 6) == ESP_OK) {
        int16_t yaw = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t roll = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t pitch = (int16_t)((buf[5] << 8) | buf[4]);
        imu.yaw = yaw * EULER_SCALE;
        imu.roll = roll * EULER_SCALE;
        imu.pitch = pitch * EULER_SCALE;
    }

    if (bno_read(BNO_LINEAR_ACCEL_DATA_X_LSB, buf, 6) == ESP_OK) {
        int16_t lax = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t lay = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t laz = (int16_t)((buf[5] << 8) | buf[4]);
        imu.linacc_x = lax * ACC_SCALE;
        imu.linacc_y = lay * ACC_SCALE;
        imu.linacc_z = laz * ACC_SCALE;
    }
}

int imu_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, PIN_TXD, PIN_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);

    uint8_t mode = 0x00; /* CONFIGMODE */
    bno_write(BNO_OPR_MODE, &mode, 1);
    vTaskDelay(pdMS_TO_TICKS(30));

    mode = 0x0C; /* NDOF fusion mode */
    bno_write(BNO_OPR_MODE, &mode, 1);
    vTaskDelay(pdMS_TO_TICKS(30));

    memset(&imu, 0, sizeof(imu));
    return 0;
}

int parse_IMU_data(void)
{
    update_sensor();
    return 0;
}

void print_imu(void)
{
    printf("\n%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t",
           imu.acc_x,
           imu.acc_y,
           imu.acc_z,
           imu.gyr_x,
           imu.gyr_y,
           imu.gyr_z,
           imu.roll,
           imu.pitch,
           imu.yaw,
           imu.linacc_x,
           imu.linacc_y,
           imu.linacc_z);
}

uint16_t get_acc_x_in_D16QN(void) { return FLOAT_TO_D16QN(imu.acc_x, IMU_QN_ACC); }
uint16_t get_acc_y_in_D16QN(void) { return FLOAT_TO_D16QN(imu.acc_y, IMU_QN_ACC); }
uint16_t get_acc_z_in_D16QN(void) { return FLOAT_TO_D16QN(imu.acc_z, IMU_QN_ACC); }

uint16_t get_gyr_x_in_D16QN(void) { return FLOAT_TO_D16QN(imu.gyr_x, IMU_QN_GYR); }
uint16_t get_gyr_y_in_D16QN(void) { return FLOAT_TO_D16QN(imu.gyr_y, IMU_QN_GYR); }
uint16_t get_gyr_z_in_D16QN(void) { return FLOAT_TO_D16QN(imu.gyr_z, IMU_QN_GYR); }

uint16_t get_roll_in_D16QN(void) { return FLOAT_TO_D16QN(imu.roll, IMU_QN_EF); }
uint16_t get_pitch_in_D16QN(void) { return FLOAT_TO_D16QN(imu.pitch, IMU_QN_EF); }
uint16_t get_yaw_in_D16QN(void) { return FLOAT_TO_D16QN(imu.yaw, IMU_QN_EF); }

uint16_t get_linacc_x_in_D16QN(void) { return FLOAT_TO_D16QN(imu.linacc_x, IMU_QN_ACC); }
uint16_t get_linacc_y_in_D16QN(void) { return FLOAT_TO_D16QN(imu.linacc_y, IMU_QN_ACC); }
uint16_t get_linacc_z_in_D16QN(void) { return FLOAT_TO_D16QN(imu.linacc_z, IMU_QN_ACC); }

#endif /* CONFIG_IMU_DRIVER_UART */

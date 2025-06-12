#include "imu.h"
#include "driver/i2c.h"
#include <stdio.h>
#include <stdint.h>

#define FLOAT_TO_D16QN(a, n) ((int16_t)((a) * (1 << (n))))

#define IMU_QN_ACC 11
#define IMU_QN_GYR 11
#define IMU_QN_EF 13

#define I2C_PORT I2C_NUM_0
#define I2C_SCL_PIN 22
#define I2C_SDA_PIN 21
#define I2C_FREQ_HZ 400000
#define MPU6050_ADDR 0x68

static struct {
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
} imu;

int imu_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);

    uint8_t data[2] = {0x6B, 0};
    i2c_master_write_to_device(I2C_PORT, MPU6050_ADDR, data, 2, pdMS_TO_TICKS(100));
    return 0;
}

static void update_sensor(void)
{
    uint8_t reg = 0x3B;
    uint8_t buf[14];
    i2c_master_write_read_device(I2C_PORT, MPU6050_ADDR, &reg, 1, buf, sizeof(buf), pdMS_TO_TICKS(100));

    int16_t ax = (buf[0] << 8) | buf[1];
    int16_t ay = (buf[2] << 8) | buf[3];
    int16_t az = (buf[4] << 8) | buf[5];
    int16_t gx = (buf[8] << 8) | buf[9];
    int16_t gy = (buf[10] << 8) | buf[11];
    int16_t gz = (buf[12] << 8) | buf[13];

    imu.acc_x = ax / 16384.0f;
    imu.acc_y = ay / 16384.0f;
    imu.acc_z = az / 16384.0f;
    imu.gyr_x = gx / 131.0f;
    imu.gyr_y = gy / 131.0f;
    imu.gyr_z = gz / 131.0f;

    imu.roll = 0.0f;
    imu.pitch = 0.0f;
    imu.yaw = 0.0f;
    imu.linacc_x = imu.acc_x;
    imu.linacc_y = imu.acc_y;
    imu.linacc_z = imu.acc_z;
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

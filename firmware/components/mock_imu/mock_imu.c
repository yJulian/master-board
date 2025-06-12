#include "imu.h"
#include <stdio.h>
#include <stdint.h>

#define FLOAT_TO_D16QN(a, n) ((int16_t)((a) * (1 << (n))))

#define IMU_QN_ACC 11
#define IMU_QN_GYR 11
#define IMU_QN_EF 13

static const float mock_acc = 1.0f;  // 1 g
static const float mock_gyr = 0.0f;
static const float mock_attitude = 0.0f;

int imu_init(void) { return 0; }
int parse_IMU_data(void) { return 0; }

void print_imu(void)
{
    printf("\n%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t",
           mock_acc,
           mock_acc,
           mock_acc,
           mock_gyr,
           mock_gyr,
           mock_gyr,
           mock_attitude,
           mock_attitude,
           mock_attitude,
           mock_acc,
           mock_acc,
           mock_acc);
}

uint16_t get_acc_x_in_D16QN(void) { return FLOAT_TO_D16QN(mock_acc, IMU_QN_ACC); }
uint16_t get_acc_y_in_D16QN(void) { return FLOAT_TO_D16QN(mock_acc, IMU_QN_ACC); }
uint16_t get_acc_z_in_D16QN(void) { return FLOAT_TO_D16QN(mock_acc, IMU_QN_ACC); }

uint16_t get_gyr_x_in_D16QN(void) { return FLOAT_TO_D16QN(mock_gyr, IMU_QN_GYR); }
uint16_t get_gyr_y_in_D16QN(void) { return FLOAT_TO_D16QN(mock_gyr, IMU_QN_GYR); }
uint16_t get_gyr_z_in_D16QN(void) { return FLOAT_TO_D16QN(mock_gyr, IMU_QN_GYR); }

uint16_t get_roll_in_D16QN(void) { return FLOAT_TO_D16QN(mock_attitude, IMU_QN_EF); }
uint16_t get_pitch_in_D16QN(void) { return FLOAT_TO_D16QN(mock_attitude, IMU_QN_EF); }
uint16_t get_yaw_in_D16QN(void) { return FLOAT_TO_D16QN(mock_attitude, IMU_QN_EF); }

uint16_t get_linacc_x_in_D16QN(void) { return FLOAT_TO_D16QN(mock_acc, IMU_QN_ACC); }
uint16_t get_linacc_y_in_D16QN(void) { return FLOAT_TO_D16QN(mock_acc, IMU_QN_ACC); }
uint16_t get_linacc_z_in_D16QN(void) { return FLOAT_TO_D16QN(mock_acc, IMU_QN_ACC); }

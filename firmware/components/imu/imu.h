#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "sdkconfig.h"

int imu_init(void);
int parse_IMU_data(void);
void print_imu(void);

uint16_t get_acc_x_in_D16QN(void);
uint16_t get_acc_y_in_D16QN(void);
uint16_t get_acc_z_in_D16QN(void);

uint16_t get_gyr_x_in_D16QN(void);
uint16_t get_gyr_y_in_D16QN(void);
uint16_t get_gyr_z_in_D16QN(void);

uint16_t get_roll_in_D16QN(void);
uint16_t get_pitch_in_D16QN(void);
uint16_t get_yaw_in_D16QN(void);

uint16_t get_linacc_x_in_D16QN(void);
uint16_t get_linacc_y_in_D16QN(void);
uint16_t get_linacc_z_in_D16QN(void);

#endif // IMU_H

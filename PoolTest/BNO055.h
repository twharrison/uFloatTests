/***************************************************************************
  This is a library for the BNO055 orientation sensor

  Designed specifically to work with the Adafruit BNO055 Breakout.

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by KTOWN for Adafruit Industries.

  MIT license, all text above must be included in any redistribution
 ***************************************************************************/

#ifndef __BNO055_H__
#define __BNO055_H__

#include "Adafruit_Sensor.h"
#include "utility/imumaths.h"
#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>

#define BNO055_ADDRESS_A (0x28)
#define BNO055_ADDRESS_B (0x29)
#define BNO055_ID        (0xA0)

      /* Page id register definition */
      #define BNO055_PAGE_ID_ADDR                               0X07

      /* PAGE0 REGISTER DEFINITION START*/
      #define BNO055_CHIP_ID_ADDR                                      0x00
      #define BNO055_ACCEL_REV_ID_ADDR                                 0x01
      #define BNO055_MAG_REV_ID_ADDR                                   0x02
      #define BNO055_GYRO_REV_ID_ADDR                                  0x03
      #define BNO055_SW_REV_ID_LSB_ADDR                                0x04
      #define BNO055_SW_REV_ID_MSB_ADDR                                0x05
      #define BNO055_BL_REV_ID_ADDR                                    0X06

      /* Accel data register */
      #define BNO055_ACCEL_DATA_X_LSB_ADDR                             0X08
      #define BNO055_ACCEL_DATA_X_MSB_ADDR                             0X09
      #define BNO055_ACCEL_DATA_Y_LSB_ADDR                             0X0A
      #define BNO055_ACCEL_DATA_Y_MSB_ADDR                             0X0B
      #define BNO055_ACCEL_DATA_Z_LSB_ADDR                             0X0C
      #define BNO055_ACCEL_DATA_Z_MSB_ADDR                             0X0D

      /* Mag data register */
      #define BNO055_MAG_DATA_X_LSB_ADDR                               0X0E
      #define BNO055_MAG_DATA_X_MSB_ADDR                               0X0F
      #define BNO055_MAG_DATA_Y_LSB_ADDR                               0X10
      #define BNO055_MAG_DATA_Y_MSB_ADDR                               0X11
      #define BNO055_MAG_DATA_Z_LSB_ADDR                               0X12
      #define BNO055_MAG_DATA_Z_MSB_ADDR                               0X13

      /* Gyro data registers */
      #define BNO055_GYRO_DATA_X_LSB_ADDR                              0X14
      #define BNO055_GYRO_DATA_X_MSB_ADDR                              0X15
      #define BNO055_GYRO_DATA_Y_LSB_ADDR                              0X16
      #define BNO055_GYRO_DATA_Y_MSB_ADDR                              0X17
      #define BNO055_GYRO_DATA_Z_LSB_ADDR                              0X18
      #define BNO055_GYRO_DATA_Z_MSB_ADDR                              0X19

      /* Euler data registers */
      #define BNO055_EULER_H_LSB_ADDR                                  0X1A
      #define BNO055_EULER_H_MSB_ADDR                                  0X1B
      #define BNO055_EULER_R_LSB_ADDR                                  0X1C
      #define BNO055_EULER_R_MSB_ADDR                                  0X1D
      #define BNO055_EULER_P_LSB_ADDR                                  0X1E
      #define BNO055_EULER_P_MSB_ADDR                                  0X1F

      /* Quaternion data registers */
      #define BNO055_QUATERNION_DATA_W_LSB_ADDR                        0X20
      #define BNO055_QUATERNION_DATA_W_MSB_ADDR                        0X21
      #define BNO055_QUATERNION_DATA_X_LSB_ADDR                        0X22
      #define BNO055_QUATERNION_DATA_X_MSB_ADDR                        0X23
      #define BNO055_QUATERNION_DATA_Y_LSB_ADDR                        0X24
      #define BNO055_QUATERNION_DATA_Y_MSB_ADDR                        0X25
      #define BNO055_QUATERNION_DATA_Z_LSB_ADDR                        0X26
      #define BNO055_QUATERNION_DATA_Z_MSB_ADDR                        0X27

      /* Linear acceleration data registers */
      #define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                      0X28
      #define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                      0X29
      #define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                      0X2A
      #define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                      0X2B
      #define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                      0X2C
      #define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                      0X2D

      /* Gravity data registers */
      #define BNO055_GRAVITY_DATA_X_LSB_ADDR                           0X2E
      #define BNO055_GRAVITY_DATA_X_MSB_ADDR                           0X2F
      #define BNO055_GRAVITY_DATA_Y_LSB_ADDR                           0X30
      #define BNO055_GRAVITY_DATA_Y_MSB_ADDR                           0X31
      #define BNO055_GRAVITY_DATA_Z_LSB_ADDR                           0X32
      #define BNO055_GRAVITY_DATA_Z_MSB_ADDR                           0X33

      /* Temperature data register */
      #define BNO055_TEMP_ADDR                                         0X34

      /* Status registers */
      #define BNO055_CALIB_STAT_ADDR                                   0X35
      #define BNO055_SELFTEST_RESULT_ADDR                              0X36
      #define BNO055_INTR_STAT_ADDR                                    0X37

      #define BNO055_SYS_CLK_STAT_ADDR                                 0X38
      #define BNO055_SYS_STAT_ADDR                                     0X39
      #define BNO055_SYS_ERR_ADDR                                      0X3A

      /* Unit selection register */
      #define BNO055_UNIT_SEL_ADDR                                     0X3B
      #define BNO055_DATA_SELECT_ADDR                                  0X3C

      /* Mode registers */
      #define BNO055_OPR_MODE_ADDR                                     0X3D
      #define BNO055_PWR_MODE_ADDR                                     0X3E

      #define BNO055_SYS_TRIGGER_ADDR                                  0X3F
      #define BNO055_TEMP_SOURCE_ADDR                                  0X40

      /* Axis remap registers */
      #define BNO055_AXIS_MAP_CONFIG_ADDR                              0X41
      #define BNO055_AXIS_MAP_SIGN_ADDR                                0X42

      /* SIC registers */
      #define BNO055_SIC_MATRIX_0_LSB_ADDR                             0X43
      #define BNO055_SIC_MATRIX_0_MSB_ADDR                             0X44
      #define BNO055_SIC_MATRIX_1_LSB_ADDR                             0X45
      #define BNO055_SIC_MATRIX_1_MSB_ADDR                             0X46
      #define BNO055_SIC_MATRIX_2_LSB_ADDR                             0X47
      #define BNO055_SIC_MATRIX_2_MSB_ADDR                             0X48
      #define BNO055_SIC_MATRIX_3_LSB_ADDR                             0X49
      #define BNO055_SIC_MATRIX_3_MSB_ADDR                             0X4A
      #define BNO055_SIC_MATRIX_4_LSB_ADDR                             0X4B
      #define BNO055_SIC_MATRIX_4_MSB_ADDR                             0X4C
      #define BNO055_SIC_MATRIX_5_LSB_ADDR                             0X4D
      #define BNO055_SIC_MATRIX_5_MSB_ADDR                             0X4E
      #define BNO055_SIC_MATRIX_6_LSB_ADDR                             0X4F
      #define BNO055_SIC_MATRIX_6_MSB_ADDR                             0X50
      #define BNO055_SIC_MATRIX_7_LSB_ADDR                             0X51
      #define BNO055_SIC_MATRIX_7_MSB_ADDR                             0X52
      #define BNO055_SIC_MATRIX_8_LSB_ADDR                             0X53
      #define BNO055_SIC_MATRIX_8_MSB_ADDR                             0X54

      /* Accelerometer Offset registers */
      #define ACCEL_OFFSET_X_LSB_ADDR                                  0X55
      #define ACCEL_OFFSET_X_MSB_ADDR                                  0X56
      #define ACCEL_OFFSET_Y_LSB_ADDR                                  0X57
      #define ACCEL_OFFSET_Y_MSB_ADDR                                  0X58
      #define ACCEL_OFFSET_Z_LSB_ADDR                                  0X59
      #define ACCEL_OFFSET_Z_MSB_ADDR                                  0X5A

      /* Magnetometer Offset registers */
      #define MAG_OFFSET_X_LSB_ADDR                                    0X5B
      #define MAG_OFFSET_X_MSB_ADDR                                    0X5C
      #define MAG_OFFSET_Y_LSB_ADDR                                    0X5D
      #define MAG_OFFSET_Y_MSB_ADDR                                    0X5E
      #define MAG_OFFSET_Z_LSB_ADDR                                    0X5F
      #define MAG_OFFSET_Z_MSB_ADDR                                    0X60

      /* Gyroscope Offset register s*/
      #define GYRO_OFFSET_X_LSB_ADDR                                   0X61
      #define GYRO_OFFSET_X_MSB_ADDR                                   0X62
      #define GYRO_OFFSET_Y_LSB_ADDR                                   0X63
      #define GYRO_OFFSET_Y_MSB_ADDR                                   0X64
      #define GYRO_OFFSET_Z_LSB_ADDR                                   0X65
      #define GYRO_OFFSET_Z_MSB_ADDR                                   0X66

      /* Radius registers */
      #define ACCEL_RADIUS_LSB_ADDR                                    0X67
      #define ACCEL_RADIUS_MSB_ADDR                                    0X68
      #define MAG_RADIUS_LSB_ADDR                                      0X69
      #define MAG_RADIUS_MSB_ADDR                                      0X6A

#define NUM_BNO055_OFFSET_REGISTERS (22)

typedef struct
{
    uint16_t accel_offset_x;
    uint16_t accel_offset_y;
    uint16_t accel_offset_z;
    uint16_t gyro_offset_x;
    uint16_t gyro_offset_y;
    uint16_t gyro_offset_z;
    uint16_t mag_offset_x;
    uint16_t mag_offset_y;
    uint16_t mag_offset_z;

    uint16_t accel_radius;
    uint16_t mag_radius;
} adafruit_bno055_offsets_t;

class BNO055 : public Adafruit_Sensor
{
  public:


	/*
    typedef enum
    {
      // Page id register definition
      #define BNO055_PAGE_ID_ADDR                               0X07

      // PAGE0 REGISTER DEFINITION START
      #define BNO055_CHIP_ID_ADDR                                      0x00
      #define BNO055_ACCEL_REV_ID_ADDR                                 0x01
      #define BNO055_MAG_REV_ID_ADDR                                   0x02
      #define BNO055_GYRO_REV_ID_ADDR                                  0x03
      #define BNO055_SW_REV_ID_LSB_ADDR                                0x04
      #define BNO055_SW_REV_ID_MSB_ADDR                                0x05
      #define BNO055_BL_REV_ID_ADDR                                    0X06

      // Accel data register 
      #define BNO055_ACCEL_DATA_X_LSB_ADDR                             0X08
      #define BNO055_ACCEL_DATA_X_MSB_ADDR                             0X09
      #define BNO055_ACCEL_DATA_Y_LSB_ADDR                             0X0A
      #define BNO055_ACCEL_DATA_Y_MSB_ADDR                             0X0B
      #define BNO055_ACCEL_DATA_Z_LSB_ADDR                             0X0C
      #define BNO055_ACCEL_DATA_Z_MSB_ADDR                             0X0D

      // Mag data register
      #define BNO055_MAG_DATA_X_LSB_ADDR                               0X0E
      #define BNO055_MAG_DATA_X_MSB_ADDR                               0X0F
      #define BNO055_MAG_DATA_Y_LSB_ADDR                               0X10
      #define BNO055_MAG_DATA_Y_MSB_ADDR                               0X11
      #define BNO055_MAG_DATA_Z_LSB_ADDR                               0X12
      #define BNO055_MAG_DATA_Z_MSB_ADDR                               0X13

      // Gyro data registers
      #define BNO055_GYRO_DATA_X_LSB_ADDR                              0X14
      #define BNO055_GYRO_DATA_X_MSB_ADDR                              0X15
      #define BNO055_GYRO_DATA_Y_LSB_ADDR                              0X16
      #define BNO055_GYRO_DATA_Y_MSB_ADDR                              0X17
      #define BNO055_GYRO_DATA_Z_LSB_ADDR                              0X18
      #define BNO055_GYRO_DATA_Z_MSB_ADDR                              0X19

      // Euler data registers
      #define BNO055_EULER_H_LSB_ADDR                                  0X1A
      #define BNO055_EULER_H_MSB_ADDR                                  0X1B
      #define BNO055_EULER_R_LSB_ADDR                                  0X1C
      #define BNO055_EULER_R_MSB_ADDR                                  0X1D
      #define BNO055_EULER_P_LSB_ADDR                                  0X1E
      #define BNO055_EULER_P_MSB_ADDR                                  0X1F

      // Quaternion data registers
      #define BNO055_QUATERNION_DATA_W_LSB_ADDR                        0X20
      #define BNO055_QUATERNION_DATA_W_MSB_ADDR                        0X21
      #define BNO055_QUATERNION_DATA_X_LSB_ADDR                        0X22
      #define BNO055_QUATERNION_DATA_X_MSB_ADDR                        0X23
      #define BNO055_QUATERNION_DATA_Y_LSB_ADDR                        0X24
      #define BNO055_QUATERNION_DATA_Y_MSB_ADDR                        0X25
      #define BNO055_QUATERNION_DATA_Z_LSB_ADDR                        0X26
      #define BNO055_QUATERNION_DATA_Z_MSB_ADDR                        0X27

      // Linear acceleration data registers
      #define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                      0X28
      #define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                      0X29
      #define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                      0X2A
      #define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                      0X2B
      #define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                      0X2C
      #define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                      0X2D

      // Gravity data registers
      #define BNO055_GRAVITY_DATA_X_LSB_ADDR                           0X2E
      #define BNO055_GRAVITY_DATA_X_MSB_ADDR                           0X2F
      #define BNO055_GRAVITY_DATA_Y_LSB_ADDR                           0X30
      #define BNO055_GRAVITY_DATA_Y_MSB_ADDR                           0X31
      #define BNO055_GRAVITY_DATA_Z_LSB_ADDR                           0X32
      #define BNO055_GRAVITY_DATA_Z_MSB_ADDR                           0X33

      // Temperature data register
      #define BNO055_TEMP_ADDR                                         0X34

      // Status registers 
      #define BNO055_CALIB_STAT_ADDR                                   0X35
      #define BNO055_SELFTEST_RESULT_ADDR                              0X36
      #define BNO055_INTR_STAT_ADDR                                    0X37

      #define BNO055_SYS_CLK_STAT_ADDR                                 0X38
      #define BNO055_SYS_STAT_ADDR                                     0X39
      #define BNO055_SYS_ERR_ADDR                                      0X3A

      // Unit selection register 
      #define BNO055_UNIT_SEL_ADDR                                     0X3B
      #define BNO055_DATA_SELECT_ADDR                                  0X3C

      // Mode registers 
      #define BNO055_OPR_MODE_ADDR                                     0X3D
      #define BNO055_PWR_MODE_ADDR                                     0X3E

      #define BNO055_SYS_TRIGGER_ADDR                                  0X3F
      #define BNO055_TEMP_SOURCE_ADDR                                  0X40

      // Axis remap registers 
      #define BNO055_AXIS_MAP_CONFIG_ADDR                              0X41
      #define BNO055_AXIS_MAP_SIGN_ADDR                                0X42

      // SIC registers 
      #define BNO055_SIC_MATRIX_0_LSB_ADDR                             0X43
      #define BNO055_SIC_MATRIX_0_MSB_ADDR                             0X44
      #define BNO055_SIC_MATRIX_1_LSB_ADDR                             0X45
      #define BNO055_SIC_MATRIX_1_MSB_ADDR                             0X46
      #define BNO055_SIC_MATRIX_2_LSB_ADDR                             0X47
      #define BNO055_SIC_MATRIX_2_MSB_ADDR                             0X48
      #define BNO055_SIC_MATRIX_3_LSB_ADDR                             0X49
      #define BNO055_SIC_MATRIX_3_MSB_ADDR                             0X4A
      #define BNO055_SIC_MATRIX_4_LSB_ADDR                             0X4B
      #define BNO055_SIC_MATRIX_4_MSB_ADDR                             0X4C
      #define BNO055_SIC_MATRIX_5_LSB_ADDR                             0X4D
      #define BNO055_SIC_MATRIX_5_MSB_ADDR                             0X4E
      #define BNO055_SIC_MATRIX_6_LSB_ADDR                             0X4F
      #define BNO055_SIC_MATRIX_6_MSB_ADDR                             0X50
      #define BNO055_SIC_MATRIX_7_LSB_ADDR                             0X51
      #define BNO055_SIC_MATRIX_7_MSB_ADDR                             0X52
      #define BNO055_SIC_MATRIX_8_LSB_ADDR                             0X53
      #define BNO055_SIC_MATRIX_8_MSB_ADDR                             0X54

      // Accelerometer Offset registers 
      #define ACCEL_OFFSET_X_LSB_ADDR                                  0X55
      #define ACCEL_OFFSET_X_MSB_ADDR                                  0X56
      #define ACCEL_OFFSET_Y_LSB_ADDR                                  0X57
      #define ACCEL_OFFSET_Y_MSB_ADDR                                  0X58
      #define ACCEL_OFFSET_Z_LSB_ADDR                                  0X59
      #define ACCEL_OFFSET_Z_MSB_ADDR                                  0X5A

      // Magnetometer Offset registers 
      #define MAG_OFFSET_X_LSB_ADDR                                    0X5B
      #define MAG_OFFSET_X_MSB_ADDR                                    0X5C
      #define MAG_OFFSET_Y_LSB_ADDR                                    0X5D
      #define MAG_OFFSET_Y_MSB_ADDR                                    0X5E
      #define MAG_OFFSET_Z_LSB_ADDR                                    0X5F
      #define MAG_OFFSET_Z_MSB_ADDR                                    0X60

      // Gyroscope Offset register s
      #define GYRO_OFFSET_X_LSB_ADDR                                   0X61
      #define GYRO_OFFSET_X_MSB_ADDR                                   0X62
      #define GYRO_OFFSET_Y_LSB_ADDR                                   0X63
      #define GYRO_OFFSET_Y_MSB_ADDR                                   0X64
      #define GYRO_OFFSET_Z_LSB_ADDR                                   0X65
      #define GYRO_OFFSET_Z_MSB_ADDR                                   0X66

      // Radius registers 
      #define ACCEL_RADIUS_LSB_ADDR                                    0X67
      #define ACCEL_RADIUS_MSB_ADDR                                    0X68
      #define MAG_RADIUS_LSB_ADDR                                      0X69
      #define MAG_RADIUS_MSB_ADDR                                      0X6A

    } adafruit_bno055_reg_t; 

*/

    typedef enum
    {
      POWER_MODE_NORMAL                                       = 0X00,
      POWER_MODE_LOWPOWER                                     = 0X01,
      POWER_MODE_SUSPEND                                      = 0X02
    } adafruit_bno055_powermode_t;


    typedef enum
    {
      /* Operation mode settings*/
      OPERATION_MODE_CONFIG                                   = 0X00,
      OPERATION_MODE_ACCONLY                                  = 0X01,
      OPERATION_MODE_MAGONLY                                  = 0X02,
      OPERATION_MODE_GYRONLY                                  = 0X03,
      OPERATION_MODE_ACCMAG                                   = 0X04,
      OPERATION_MODE_ACCGYRO                                  = 0X05,
      OPERATION_MODE_MAGGYRO                                  = 0X06,
      OPERATION_MODE_AMG                                      = 0X07,
      OPERATION_MODE_IMUPLUS                                  = 0X08,
      OPERATION_MODE_COMPASS                                  = 0X09,
      OPERATION_MODE_M4G                                      = 0X0A,
      OPERATION_MODE_NDOF_FMC_OFF                             = 0X0B,
      OPERATION_MODE_NDOF                                     = 0X0C
    } adafruit_bno055_opmode_t;

    typedef enum
    {
      REMAP_CONFIG_P0                                         = 0x21,
      REMAP_CONFIG_P1                                         = 0x24, // default
      REMAP_CONFIG_P2                                         = 0x24,
      REMAP_CONFIG_P3                                         = 0x21,
      REMAP_CONFIG_P4                                         = 0x24,
      REMAP_CONFIG_P5                                         = 0x21,
      REMAP_CONFIG_P6                                         = 0x21,
      REMAP_CONFIG_P7                                         = 0x24
    } adafruit_bno055_axis_remap_config_t;

    typedef enum
    {
      REMAP_SIGN_P0                                           = 0x04,
      REMAP_SIGN_P1                                           = 0x00, // default
      REMAP_SIGN_P2                                           = 0x06,
      REMAP_SIGN_P3                                           = 0x02,
      REMAP_SIGN_P4                                           = 0x03,
      REMAP_SIGN_P5                                           = 0x01,
      REMAP_SIGN_P6                                           = 0x07,
      REMAP_SIGN_P7                                           = 0x05
    } adafruit_bno055_axis_remap_sign_t;

    typedef struct
    {
      uint8_t  accel_rev;
      uint8_t  mag_rev;
      uint8_t  gyro_rev;
      uint16_t sw_rev;
      uint8_t  bl_rev;
    } adafruit_bno055_rev_info_t;

    typedef enum
    {
      VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
      VECTOR_MAGNETOMETER  = BNO055_MAG_DATA_X_LSB_ADDR,
      VECTOR_GYROSCOPE     = BNO055_GYRO_DATA_X_LSB_ADDR,
      VECTOR_EULER         = BNO055_EULER_H_LSB_ADDR,
      VECTOR_LINEARACCEL   = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
      VECTOR_GRAVITY       = BNO055_GRAVITY_DATA_X_LSB_ADDR
    } adafruit_vector_type_t;


    BNO055 ( int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A );

    bool  begin               ( adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF );
    void  setMode             ( adafruit_bno055_opmode_t mode );
    void  getRevInfo          ( adafruit_bno055_rev_info_t* );
    void  displayRevInfo      ( void );
    void  setExtCrystalUse    ( bool usextal );
    void  getSystemStatus     ( uint8_t *system_status,
                                uint8_t *self_test_result,
                                uint8_t *system_error);
    void  displaySystemStatus ( void );
    void  getCalibration      ( uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag);

    imu::Vector<3>  getVector ( adafruit_vector_type_t vector_type );
    imu::Quaternion getQuat   ( void );
    int8_t          getTemp   ( void );

    /* Adafruit_Sensor implementation */
    bool  getEvent  ( sensors_event_t* );
    void  getSensor ( sensor_t* );

    /* Functions to deal with raw calibration data */
    bool  getSensorOffsets(uint8_t* calibData);
    bool  getSensorOffsets(adafruit_bno055_offsets_t &offsets_type);
    void  setSensorOffsets(const uint8_t* calibData);
    void  setSensorOffsets(const adafruit_bno055_offsets_t &offsets_type);
    bool  isFullyCalibrated(void);

	struct timeval tval_before_imu;
	time_t nowtime;
        struct tm *nowtm;
    void readlog(void);
    FILE * pIMUFile;
    char imufilename[100];
    char imufile[50];
    imu::Vector<3> euler;
    imu::Vector<3> gyro;
    imu::Vector<3> mag;
    imu::Vector<3> accel;

  private:
    uint8_t _address;
    int32_t _sensorID;
    adafruit_bno055_opmode_t _mode;
  int file;
  int adapter_nr; /* probably dynamically determined */
  char filename[20];
};

#endif

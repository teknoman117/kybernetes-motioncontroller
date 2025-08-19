/**
 * @file    ina219.h
 * @brief   INA219 Current Sensor Driver header.
 *
 * @addtogroup INA219
 * @{
 */

#ifndef INA219_H
#define INA219_H

#include "hal.h"

#if (HAL_USE_I2C == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    INA219 I2C addresses
 * @{
 */
#define INA219_SAD_A1_A0_GND_GND    0x40U  /**< A1=GND, A0=GND          */
#define INA219_SAD_A1_A0_GND_VS     0x41U  /**< A1=GND, A0=VS           */
#define INA219_SAD_A1_A0_GND_SDA    0x42U  /**< A1=GND, A0=SDA          */
#define INA219_SAD_A1_A0_GND_SCL    0x43U  /**< A1=GND, A0=SCL          */
#define INA219_SAD_A1_A0_VS_GND     0x44U  /**< A1=VS, A0=GND           */
#define INA219_SAD_A1_A0_VS_VS      0x45U  /**< A1=VS, A0=VS            */
#define INA219_SAD_A1_A0_VS_SDA     0x46U  /**< A1=VS, A0=SDA           */
#define INA219_SAD_A1_A0_VS_SCL     0x47U  /**< A1=VS, A0=SCL           */
#define INA219_SAD_A1_A0_SDA_GND    0x48U  /**< A1=SDA, A0=GND          */
#define INA219_SAD_A1_A0_SDA_VS     0x49U  /**< A1=SDA, A0=VS           */
#define INA219_SAD_A1_A0_SDA_SDA    0x4AU  /**< A1=SDA, A0=SDA          */
#define INA219_SAD_A1_A0_SDA_SCL    0x4BU  /**< A1=SDA, A0=SCL          */
#define INA219_SAD_A1_A0_SCL_GND    0x4CU  /**< A1=SCL, A0=GND          */
#define INA219_SAD_A1_A0_SCL_VS     0x4DU  /**< A1=SCL, A0=VS           */
#define INA219_SAD_A1_A0_SCL_SDA    0x4EU  /**< A1=SCL, A0=SDA          */
#define INA219_SAD_A1_A0_SCL_SCL    0x4FU  /**< A1=SCL, A0=SCL          */
/** @} */

/**
 * @name    INA219 register addresses
 * @{
 */
#define INA219_AD_CONFIG            0x00U  /**< Configuration register   */
#define INA219_AD_SHUNT_VOLTAGE     0x01U  /**< Shunt voltage register   */
#define INA219_AD_BUS_VOLTAGE       0x02U  /**< Bus voltage register     */
#define INA219_AD_POWER             0x03U  /**< Power register           */
#define INA219_AD_CURRENT           0x04U  /**< Current register         */
#define INA219_AD_CALIBRATION       0x05U  /**< Calibration register     */
/** @} */

/**
 * @name    INA219 configuration register bits
 * @{
 */
#define INA219_CONFIG_RESET         0x8000U /**< Reset bit                */
#define INA219_CONFIG_BVOLTAGERANGE 0x2000U /**< Bus voltage range        */

#define INA219_CONFIG_GAIN_MASK     0x1800U /**< PGA gain mask            */

#define INA219_CONFIG_BADCRES_MASK  0x0780U /**< Bus ADC resolution mask  */
#define INA219_CONFIG_BADCRES_SHIFT 7

#define INA219_CONFIG_SADCRES_MASK  0x0078U /**< Shunt ADC resolution mask*/
#define INA219_CONFIG_SADCRES_SHIFT 3

/** @} */

/**
 * @name    INA219 bus voltage register bits
 * @{
 */
#define INA219_BUS_VOLTAGE_OVF      0x0001U /**< Math overflow flag       */
#define INA219_BUS_VOLTAGE_CNVR     0x0002U /**< Conversion ready flag    */
#define INA219_BUS_VOLTAGE_MASK     0xFFF8U /**< Bus voltage data mask    */
/** @} */

/**
 * @name INA219 helper macros
 * @{
 */
#define INA219_CURRENT_LSB(max_amps) ((double) (((double) max_amps) / 32768.0))
#define INA219_CALIBRATION(max_amps, shunt_ohms) ((uint16_t) ( 0.04096 / ( INA219_CURRENT_LSB(max_amps) * ((double) shunt_ohms))))
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                        */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   INA219 shared I2C switch.
 * @details If set to @p TRUE the device acquires I2C bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires I2C_USE_MUTUAL_EXCLUSION.
 */
#if !defined(INA219_SHARED_I2C) || defined(__DOXYGEN__)
#define INA219_SHARED_I2C                  FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                      */
/*===========================================================================*/

#if INA219_SHARED_I2C && !I2C_USE_MUTUAL_EXCLUSION
#error "INA219_SHARED_I2C requires I2C_USE_MUTUAL_EXCLUSION"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                        */
/*===========================================================================*/

/**
 * @name    INA219 data structures and types
 * @{
 */
/**
 * @brief   INA219 slave address type.
 */
typedef uint8_t ina219_sad_t;

/**
 * @brief   INA219 register address type.
 */
typedef uint8_t ina219_reg_t;

/**
 * @brief   INA219 PGA gain.
 */
typedef enum {
  INA219_PGA_GAIN_1_40MV = 0x0000,     /**< Gain /1, max shunt voltage 40mV  */
  INA219_PGA_GAIN_2_80MV = 0x0800,     /**< Gain /2, max shunt voltage 80mV  */
  INA219_PGA_GAIN_4_160MV = 0x1000,    /**< Gain /4, max shunt voltage 160mV */
  INA219_PGA_GAIN_8_320MV = 0x1800     /**< Gain /8, max shunt voltage 320mV */
} ina219_pga_gain_t;

/**
 * @brief   INA219 ADC conversion time.
 */
typedef enum {
  INA219_ADC_TIME_84US    = 0x0000U, /**< 9-bit resolution, 84 microsecond conversion time */
  INA219_ADC_TIME_148US   = 0x0001U, /**< 10-bit resolution, 148 microsecond conversion time */
  INA219_ADC_TIME_276US   = 0x0002U, /**< 11-bit resolution, 276 microsecond conversion time */
  INA219_ADC_TIME_532US   = 0x0003U, /**< 12-bit resolution, 532 microsecond conversion time */
  INA219_ADC_TIME_1_06MS  = 0x0009U, /**< 12-bit resolution, 2x oversampled, 1.06 millisecond conversion time */
  INA219_ADC_TIME_2_13MS  = 0x000AU, /**< 12-bit resolution, 4x oversampled, 2.13 millisecond conversion time */
  INA219_ADC_TIME_4_26MS  = 0x000BU, /**< 12-bit resolution, 8x oversampled, 4.26 millisecond conversion time */
  INA219_ADC_TIME_8_51MS  = 0x000CU, /**< 12-bit resolution, 16x oversampled, 8.51 millisecond conversion time */
  INA219_ADC_TIME_17_02MS = 0x000DU, /**< 12-bit resolution, 32x oversampled, 17.02 millisecond conversion time */
  INA219_ADC_TIME_34_05MS = 0x000EU, /**< 12-bit resolution, 64x oversampled, 34.05 millisecond conversion time */
  INA219_ADC_TIME_68_10MS = 0x000FU  /**< 12-bit resolution, 128x oversampled, 68.10 millisecond conversion time */
} ina219_adc_time_t;

/**
 * @brief   INA219 operating mode.
 */
typedef enum {
  INA219_MODE_POWER_DOWN = 0,     /**< Power down                       */
  INA219_MODE_SHUNT_TRIG = 1,     /**< Shunt voltage triggered          */
  INA219_MODE_BUS_TRIG = 2,       /**< Bus voltage triggered            */
  INA219_MODE_SHUNT_BUS_TRIG = 3, /**< Shunt and bus voltage triggered  */
  INA219_MODE_ADC_OFF = 4,        /**< ADC off                          */
  INA219_MODE_SHUNT_CONT = 5,     /**< Shunt voltage continuous         */
  INA219_MODE_BUS_CONT = 6,       /**< Bus voltage continuous           */
  INA219_MODE_SHUNT_BUS_CONT = 7  /**< Shunt and bus voltage continuous */
} ina219_mode_t;

/**
 * @brief   INA219 configuration structure.
 */
typedef struct {
  I2CDriver                 *i2cp;          /**< I2C driver associated    */
  const I2CConfig           *i2ccfg;        /**< I2C configuration        */
  ina219_sad_t              slaveaddress;   /**< INA219 slave address     */
  uint16_t                  configuration;
  uint16_t                  calibration;
} INA219Config;

/**
 * @brief   @p INA219Driver specific data.
 */
#define _ina219_data                                                      \
  _base_object_data                                                        \
  /* Driver state.*/                                                       \
  ina219_state_t           state;                                         \
  /* Current configuration data.*/                                         \
  const INA219Config       *config;

/**
 * @brief   INA219 driver state machine possible states.
 */
typedef enum {
  INA219_UNINIT = 0,              /**< Not initialized.                 */
  INA219_STOP = 1,                /**< Stopped.                         */
  INA219_READY = 2                /**< Ready.                           */
} ina219_state_t;

/**
 * @brief   INA219 driver class.
 */
typedef struct INA219Driver INA219Driver;

/**
 * @brief   INA219 driver class.
 */
struct INA219Driver {
  _ina219_data
};
/** @} */

/*===========================================================================*/
/* Driver macros.                                                           */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void ina219ObjectInit(INA219Driver *devp);
  void ina219Start(INA219Driver *devp, const INA219Config *config);
  void ina219Stop(INA219Driver *devp);
  msg_t ina219Trigger(INA219Driver *devp);
  msg_t ina219ReadShuntVoltage(INA219Driver *devp, uint16_t* voltage);
  msg_t ina219ReadBusVoltage(INA219Driver *devp, uint16_t* voltage, bool* ready, bool* overflow);
  msg_t ina219ReadCurrent(INA219Driver *devp, int16_t* current);
  msg_t ina219ReadPower(INA219Driver *devp, int16_t* power);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2C == TRUE */

#endif /* INA219_H */

/** @} */
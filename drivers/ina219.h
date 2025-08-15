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
#define INA219_CONFIG_GAIN_1_40MV   0x0000U /**< PGA gain /1 (40mV)       */
#define INA219_CONFIG_GAIN_2_80MV   0x0800U /**< PGA gain /2 (80mV)       */
#define INA219_CONFIG_GAIN_4_160MV  0x1000U /**< PGA gain /4 (160mV)      */
#define INA219_CONFIG_GAIN_8_320MV  0x1800U /**< PGA gain /8 (320mV)      */
#define INA219_CONFIG_BADCRES_MASK  0x0780U /**< Bus ADC resolution mask  */
#define INA219_CONFIG_BADCRES_9BIT  0x0000U /**< 9-bit resolution         */
#define INA219_CONFIG_BADCRES_10BIT 0x0080U /**< 10-bit resolution        */
#define INA219_CONFIG_BADCRES_11BIT 0x0100U /**< 11-bit resolution        */
#define INA219_CONFIG_BADCRES_12BIT 0x0180U /**< 12-bit resolution        */
#define INA219_CONFIG_SADCRES_MASK  0x0078U /**< Shunt ADC resolution mask*/
#define INA219_CONFIG_SADCRES_9BIT  0x0000U /**< 9-bit resolution         */
#define INA219_CONFIG_SADCRES_10BIT 0x0008U /**< 10-bit resolution        */
#define INA219_CONFIG_SADCRES_11BIT 0x0010U /**< 11-bit resolution        */
#define INA219_CONFIG_SADCRES_12BIT 0x0018U /**< 12-bit resolution        */
#define INA219_CONFIG_MODE_MASK     0x0007U /**< Operating mode mask      */
#define INA219_CONFIG_MODE_PWRDWN   0x0000U /**< Power down               */
#define INA219_CONFIG_MODE_SVOLT    0x0001U /**< Shunt voltage triggered  */
#define INA219_CONFIG_MODE_BVOLT    0x0002U /**< Bus voltage triggered    */
#define INA219_CONFIG_MODE_SANDBVOLT 0x0003U /**< Shunt and bus triggered */
#define INA219_CONFIG_MODE_ADCOFF   0x0004U /**< ADC off                  */
#define INA219_CONFIG_MODE_SVOLT_CONT 0x0005U /**< Shunt voltage continuous*/
#define INA219_CONFIG_MODE_BVOLT_CONT 0x0006U /**< Bus voltage continuous  */
#define INA219_CONFIG_MODE_SANDBVOLT_CONT 0x0007U /**< Shunt & bus continuous*/
/** @} */

/**
 * @name    INA219 bus voltage register bits
 * @{
 */
#define INA219_BUS_VOLTAGE_OVF      0x0001U /**< Math overflow flag       */
#define INA219_BUS_VOLTAGE_CNVR     0x0002U /**< Conversion ready flag    */
#define INA219_BUS_VOLTAGE_MASK     0xFFF8U /**< Bus voltage data mask    */
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
  INA219_PGA_GAIN_1_40MV = 0,     /**< Gain /1, max shunt voltage 40mV  */
  INA219_PGA_GAIN_2_80MV = 1,     /**< Gain /2, max shunt voltage 80mV  */
  INA219_PGA_GAIN_4_160MV = 2,    /**< Gain /4, max shunt voltage 160mV */
  INA219_PGA_GAIN_8_320MV = 3     /**< Gain /8, max shunt voltage 320mV */
} ina219_pga_gain_t;

/**
 * @brief   INA219 ADC resolution.
 */
typedef enum {
  INA219_ADC_RES_9BIT = 0,        /**< 9-bit resolution                 */
  INA219_ADC_RES_10BIT = 1,       /**< 10-bit resolution                */
  INA219_ADC_RES_11BIT = 2,       /**< 11-bit resolution                */
  INA219_ADC_RES_12BIT = 3        /**< 12-bit resolution                */
} ina219_adc_res_t;

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
  ina219_pga_gain_t         pga_gain;       /**< PGA gain setting         */
  ina219_adc_res_t          bus_adc_res;    /**< Bus ADC resolution       */
  ina219_adc_res_t          shunt_adc_res;  /**< Shunt ADC resolution     */
  ina219_mode_t             mode;           /**< Operating mode           */
  uint16_t                  calibration;    /**< Calibration value        */
  float                     shunt_resistor; /**< Shunt resistor value (Ω) */
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
  /** @brief Virtual Methods Table.*/
  const struct INA219VMT  *vmt;
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
  msg_t ina219Reset(INA219Driver *devp);
  msg_t ina219TriggerConversion(INA219Driver *devp);
  msg_t ina219ReadShuntVoltage(INA219Driver *devp, float* voltage);
  msg_t ina219ReadBusVoltage(INA219Driver *devp, float* voltage);
  msg_t ina219ReadCurrent(INA219Driver *devp, float* current);
  msg_t ina219ReadPower(INA219Driver *devp, float* power);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2C == TRUE */

#endif /* INA219_H */

/** @} */
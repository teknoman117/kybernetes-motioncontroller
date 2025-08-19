/**
 * @file    ina219.c
 * @brief   INA219 Current Sensor Driver code.
 *
 * @addtogroup INA219
 * @{
 */

#include "hal.h"
#include "ina219.h"

#if (HAL_USE_I2C == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define INA219_BUS_VOLTAGE_LSB     0.004f   /**< Bus voltage LSB (4mV)    */
#define INA219_SHUNT_VOLTAGE_LSB   0.00001f /**< Shunt voltage LSB (10µV) */

/*===========================================================================*/
/* Driver exported variables.                                               */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                        */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Reads registers from the INA219.
 * @pre     The I2C interface must be initialized and the driver started.
 *
 * @param[in] devp       pointer to @p INA219Driver object
 * @param[in] reg        starting register address
 * @param[out] rxbuf     pointer to the data buffer
 * @param[in] n          number of consecutive register to read
 * @return               the operation status.
 * @retval MSG_OK        if the function succeeded.
 * @retval MSG_RESET     if one or more I2C errors occurred, the errors can
 *                       be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT   if a timeout occurred before operation end.
 *
 * @notapi
 */
static msg_t ina219_read_raw(INA219Driver *devp, ina219_reg_t reg,
                              uint8_t* rxbuf, size_t n) {

  osalDbgCheck((devp != NULL) && (rxbuf != NULL) && (n > 0U) && (n <= 6U));
  osalDbgAssert((devp->state == INA219_READY), "ina219_read_raw(), invalid state");

#if INA219_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp, devp->config->i2ccfg);
#endif /* INA219_SHARED_I2C */

  msg_t msg = i2cMasterTransmitTimeout(devp->config->i2cp,
                                       devp->config->slaveaddress,
                                       &reg, 1, rxbuf, n,
                                       TIME_INFINITE);

#if INA219_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* INA219_SHARED_I2C */

  return msg;
}

/**
 * @brief   Writes data to the INA219.
 * @pre     The I2C interface must be initialized and the driver started.
 *
 * @param[in] devp       pointer to @p INA219Driver object
 * @param[in] txbuf      pointer to the data buffer
 * @param[in] n          number of bytes to send
 * @return               the operation status.
 * @retval MSG_OK        if the function succeeded.
 * @retval MSG_RESET     if one or more I2C errors occurred, the errors can
 *                       be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT   if a timeout occurred before operation end.
 *
 * @notapi
 */
static msg_t ina219_write_raw(INA219Driver *devp, const uint8_t* txbuf,
                               size_t n) {

  osalDbgCheck((devp != NULL) && (txbuf != NULL) && (n > 1U) && (n <= 7U));
  osalDbgAssert((devp->state == INA219_READY), "ina219_write_raw(), invalid state");

#if INA219_SHARED_I2C
  i2cAcquireBus(devp->config->i2cp);
  i2cStart(devp->config->i2cp, devp->config->i2ccfg);
#endif /* INA219_SHARED_I2C */

  msg_t msg = i2cMasterTransmitTimeout(devp->config->i2cp,
                                       devp->config->slaveaddress,
                                       txbuf, n, NULL, 0,
                                       TIME_INFINITE);

#if INA219_SHARED_I2C
  i2cReleaseBus(devp->config->i2cp);
#endif /* INA219_SHARED_I2C */

  return msg;
}

/**
 * @brief   Reads a register from the INA219.
 * @pre     The I2C interface must be initialized and the driver started.
 *
 * @param[in] devp       pointer to @p INA219Driver object
 * @param[in] reg        register address
 * @param[out] value     pointer to store the register value
 * @return               the operation status.
 * @retval MSG_OK        if the function succeeded.
 * @retval MSG_RESET     if one or more I2C errors occurred, the errors can
 *                       be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT   if a timeout occurred before operation end.
 *
 * @notapi
 */
static msg_t ina219_read_register(INA219Driver *devp, ina219_reg_t reg,
                                   uint16_t* value) {
  uint8_t rxbuf[2];
  msg_t msg;

  osalDbgCheck((devp != NULL) && (value != NULL));
  osalDbgAssert((devp->state == INA219_READY), "ina219_read_register(), invalid state");

  msg = ina219_read_raw(devp, reg, rxbuf, 2);
  if (msg == MSG_OK) {
    *value = (rxbuf[0] << 8) | rxbuf[1];
  }

  return msg;
}

/**
 * @brief   Writes a register to the INA219.
 * @pre     The I2C interface must be initialized and the driver started.
 *
 * @param[in] devp       pointer to @p INA219Driver object
 * @param[in] reg        register address
 * @param[in] value      value to write
 * @return               the operation status.
 * @retval MSG_OK        if the function succeeded.
 * @retval MSG_RESET     if one or more I2C errors occurred, the errors can
 *                       be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT   if a timeout occurred before operation end.
 *
 * @notapi
 */
static msg_t ina219_write_register(INA219Driver *devp, ina219_reg_t reg,
                                    uint16_t value) {
  uint8_t txbuf[3];
  osalDbgCheck(devp != NULL);

  osalDbgAssert(devp->state == INA219_READY,
                "ina219_write_register(), invalid state");

  txbuf[0] = reg;
  txbuf[1] = (value >> 8) & 0xFF;
  txbuf[2] = value & 0xFF;

  return ina219_write_raw(devp, txbuf, 3);
}

/*===========================================================================*/
/* Driver exported functions.                                               */
/*===========================================================================*/

/**
 * @brief   Initializes an instance.
 *
 * @param[out] devp     pointer to the @p INA219Driver object
 *
 * @init
 */
void ina219ObjectInit(INA219Driver *devp) {

  osalDbgCheck(devp != NULL);

  devp->state = INA219_UNINIT;
  devp->config = NULL;
}

/**
 * @brief   Configures and activates INA219 Complex Driver peripheral.
 *
 * @param[in] devp      pointer to the @p INA219Driver object
 * @param[in] config    pointer to the @p INA219Config object
 *
 * @api
 */
void ina219Start(INA219Driver *devp, const INA219Config *config) {
  osalDbgCheck((devp != NULL) && (config != NULL));
  osalDbgAssert((devp->state == INA219_STOP) ||
                (devp->state == INA219_READY),
                "ina219Start(), invalid state");
  devp->config = config;
  ina219_write_register(devp, INA219_AD_CONFIG, devp->config->configuration);
  ina219_write_register(devp, INA219_AD_CALIBRATION, devp->config->calibration);
  devp->state = INA219_READY;
}

/**
 * @brief   Deactivates the INA219 Complex Driver peripheral.
 *
 * @param[in] devp       pointer to the @p INA219Driver object
 *
 * @api
 */
void ina219Stop(INA219Driver *devp) {

  osalDbgCheck(devp != NULL);
  osalDbgAssert((devp->state == INA219_STOP) ||
                (devp->state == INA219_READY),
                "ina219Stop(), invalid state");

  if (devp->state == INA219_READY) {
#if INA219_SHARED_I2C
    i2cAcquireBus(devp->config->i2cp);
    i2cStart(devp->config->i2cp, devp->config->i2ccfg);
#endif /* INA219_SHARED_I2C */
    ina219_write_register(devp, INA219_AD_CONFIG, 0);
#if INA219_SHARED_I2C
    i2cReleaseBus(devp->config->i2cp);
#endif /* INA219_SHARED_I2C */
  }
  devp->state = INA219_STOP;
}

/**
 * @brief   Triggers a conversion in triggered mode.
 * @details This function triggers a new conversion when the INA219 is
 *          configured for triggered mode operation. It writes the current
 *          configuration to restart the conversion process.
 *
 * @param[in] devp       pointer to @p INA219Driver object
 * @return               the operation status.
 *
 * @api
 */
msg_t ina219Trigger(INA219Driver *devp) {
  osalDbgCheck(devp != NULL);
  osalDbgAssert(devp->state == INA219_READY,
                "ina219TriggerConversion(), invalid state");

  /* Check if device is in a triggered mode */
  // if ((devp->config->mode != INA219_MODE_SHUNT_TRIG) &&
  //    (devp->config->mode != INA219_MODE_BUS_TRIG) &&
  //    (devp->config->mode != INA219_MODE_SHUNT_BUS_TRIG)) {
  //  return MSG_RESET; /* Not in triggered mode */
  //}

  return ina219_write_register(devp, INA219_AD_CONFIG, devp->config->configuration);
}

/**
 * @brief   Reads the shunt voltage.
 *
 * @param[in] devp       pointer to @p INA219Driver object
 * @param[out] voltage   pointer to store the shunt voltage in volts
 * @return               the operation status.
 *
 * @api
 */
msg_t ina219ReadShuntVoltage(INA219Driver *devp, uint16_t* voltage) {
  osalDbgCheck((devp != NULL) && (voltage != NULL));
  osalDbgAssert(devp->state == INA219_READY,
                "ina219ReadShuntVoltage(), invalid state");

  return ina219_read_register(devp, INA219_AD_SHUNT_VOLTAGE, voltage);
}

/**
 * @brief   Reads the bus voltage.
 *
 * @param[in] devp       pointer to @p INA219Driver object
 * @param[out] voltage   pointer to store the bus voltage in volts
 * @return               the operation status.
 *
 * @api
 */
msg_t ina219ReadBusVoltage(INA219Driver *devp, uint16_t* voltage, bool* ready, bool* overflow) {
  uint16_t raw;
  msg_t msg;

  osalDbgCheck((devp != NULL) && (voltage != NULL));
  osalDbgAssert(devp->state == INA219_READY,
                "ina219ReadBusVoltage(), invalid state");

  msg = ina219_read_register(devp, INA219_AD_BUS_VOLTAGE, (uint16_t*) &raw);
  if (msg == MSG_OK) {
    if (overflow) {
      *overflow = !!(raw & INA219_BUS_VOLTAGE_OVF);
    }
    if (ready) {
      *ready = !!(raw & INA219_BUS_VOLTAGE_CNVR);
    }

    // volate is in mV
    *voltage = (raw & INA219_BUS_VOLTAGE_MASK) >> 1;
  }
  return msg;
}

/**
 * @brief   Reads the current.
 *
 * @param[in] devp       pointer to @p INA219Driver object
 * @param[out] current   pointer to store the current in amperes
 * @return               the operation status.
 *
 * @api
 */
msg_t ina219ReadCurrent(INA219Driver *devp, int16_t* current) {
  osalDbgCheck((devp != NULL) && (current != NULL));
  osalDbgAssert(devp->state == INA219_READY,
                "ina219ReadCurrent(), invalid state");

  return ina219_read_register(devp, INA219_AD_CURRENT, (uint16_t*) current);
}

/**
 * @brief   Reads the power.
 *
 * @param[in] devp       pointer to @p INA219Driver object
 * @param[out] power     pointer to store the power in watts
 * @return               the operation status.
 *
 * @api
 */
msg_t ina219ReadPower(INA219Driver *devp, int16_t* power) {
  osalDbgCheck((devp != NULL) && (power != NULL));
  osalDbgAssert(devp->state == INA219_READY,
                "ina219ReadPower(), invalid state");

  return ina219_read_register(devp, INA219_AD_POWER, (uint16_t*) power);
}

#endif /* HAL_USE_I2C == TRUE */

/** @} */
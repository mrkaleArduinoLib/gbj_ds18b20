/**
 * @name gbj_ds18b20
 *
 * @brief Library for temperature sensors Dallas Semiconductor DS18B20.
 * @note Library does not have conversion to imperial temperature units
 * (degrees of Fahrenheit), which should be provided in a sketch code or in a
 * separate library in order to avoid code duplicities in sketches using
 * multiple libraries with the same conversion functionalities.
 * @note Library provides a device identifier taken from last (CRC) byte of a
 * device's hardware ROM address.
 * @note At temperature alarm processing an alarm temperature is converted from
 * integer to float right before comparison.
 * @note Library is primarily aimed for working with all sensors on the
 * one-wire bus in a loop, so that they need not to be identified by an
 * address in advance. Thus, all getters and setters are valid for currently
 * selected sensor in a loop.
 *
 * @copyright This program is free software; you can redistribute it and/or
 * modify it under the terms of the license GNU GPL v3
 * http://www.gnu.org/licenses/gpl-3.0.html (related to original code) and MIT
 * License (MIT) for added code.
 *
 * @author Libor Gabaj
 */
#ifndef GBJ_DS18B20_H
#define GBJ_DS18B20_H

#if defined(__AVR__)
  #include <Arduino.h>
  #include <inttypes.h>
#elif defined(ESP8266) || defined(ESP32)
  #include <Arduino.h>
#endif
#include <OneWire.h>

/**
 * @class gbj_ds18b20
 * @brief One-wire DS18B20 sensor manager with cached ROM and scratchpad data.
 *
 * @param pinBus GPIO pin used as one-wire data bus.
 * @param alarmHandlerLow Callback invoked when low alarm condition occurs.
 * @param alarmHandlerHigh Callback invoked when high alarm condition occurs.
 */
class gbj_ds18b20 : public OneWire
{
public:
  /// @brief Enumeration of result codes for DS18B20 operations.
  enum ResultCodes : uint8_t
  {
    SUCCESS,
    END_OF_LIST,
    ERROR_NO_DEVICE,
    ERROR_NO_SENSOR,
    ERROR_CRC_ADDRESS,
    ERROR_CRC_SCRATCHPAD,
    ERROR_NO_ALARM,
    ERROR_ALARM_LOW,
    ERROR_ALARM_HIGH,
    ERROR_CONVERSION,
  };

  /// @brief Enumeration of DS18B20 scratchpad register addresses.
  enum Params : uint8_t
  {
    FAMILY_CODE = 0x28,
    ADDRESS_LEN = 8,
    SERNUM_LEN = 6,
    SCRATCHPAD_LEN = 9,
  };

  typedef uint8_t Address[Params::ADDRESS_LEN];
  typedef uint8_t Sernum[Params::SERNUM_LEN];
  typedef uint8_t Scratchpad[Params::SCRATCHPAD_LEN];
  typedef void Handler();

  /**
   * @brief Construct a DS18B20 bus handler and optionally register global
   * alarm callbacks.
   *
   * @param pinBus GPIO pin used as one-wire data bus.
   * @param alarmHandlerLow Callback invoked when low alarm condition occurs.
   * @param alarmHandlerHigh Callback invoked when high alarm condition occurs.
   */
  gbj_ds18b20(uint8_t pinBus,
              Handler *alarmHandlerLow = 0,
              Handler *alarmHandlerHigh = 0)
    : OneWire(pinBus)
  {
    bus_.pinBus = pinBus;
    bus_.alarmHandlerLow = alarmHandlerLow;
    bus_.alarmHandlerHigh = alarmHandlerHigh;
    if (isError(powering()))
    {
      return;
    }
    devices();
  }

  /**
   * @brief Detect and count all devices and supported sensors on the bus.
   *
   * Also updates cached bus properties such as the highest detected
   * temperature resolution.
   *
   * @return Result code.
   */
  ResultCodes devices();

  /**
   * @brief Iterate through active supported sensors on the bus.
   *
   * For each matching sensor, its scratchpad is read and cached for subsequent
   * getters and setters.
   *
   * @return Result code.
   */
  ResultCodes sensors();

  /**
   * @brief Iterate through supported sensors that currently signal an alarm.
   *
   * For each matching sensor, its scratchpad is read and cached for subsequent
   * getters and setters.
   *
   * @return Result code.
   */
  ResultCodes alarms();

  /**
   * @brief Start temperature conversion on all sensors in parallel.
   *
   * @return Result code.
   */
  ResultCodes conversion();

  /**
   * @brief Measure temperature for a specific sensor address.
   *
   * @param address Sensor ROM address.
   * @return Result code.
   */
  ResultCodes measureTemperature(const Address address);

  /** @name Configs */
  ///@{

  /** @brief Cache requested resolution in scratchpad config register format. */
  inline void cacheResolutionBits(uint8_t resolution = 12)
  {
    resolution = constrain(
      resolution,
      bus_.tempBits[0],
      bus_.tempBits[sizeof(bus_.tempBits) / sizeof(bus_.tempBits[0])]);
    memory_.scratchpad.config = 0x1F;
    for (uint8_t i = 0; i < sizeof(bus_.tempBits) / sizeof(bus_.tempBits[0]);
         i++)
    {
      if (bus_.tempBits[i] == resolution)
      {
        memory_.scratchpad.config += 0x20 * i;
        break;
      }
    }
  }

  /** @brief Cache low alarm threshold constrained to sensor limits. */
  inline void cacheAlarmLow(int8_t alarmValue)
  {
    memory_.scratchpad.alarm_lsb =
      constrain(alarmValue, (int)getTemperatureMin(), (int)getTemperatureMax());
  }

  /** @brief Cache high alarm threshold constrained to sensor limits. */
  inline void cacheAlarmHigh(int8_t alarmValue)
  {
    memory_.scratchpad.alarm_msb =
      constrain(alarmValue, (int)getTemperatureMin(), (int)getTemperatureMax());
  }

  /** @brief Cache default alarm thresholds. */
  inline void cacheAlarmsReset()
  {
    cacheAlarmLow(getAlarmLowIni());
    cacheAlarmHigh(getAlarmHighIni());
  }

  /**
   * @brief Copy cached ROM address into user buffer.
   * @param address Destination address buffer.
   */
  inline void cpyAddress(Address address)
  {
    memcpy(address, rom_.buffer, Params::ADDRESS_LEN);
  }

  /**
   * @brief Copy cached serial number from ROM into user buffer.
   * @param sernum Destination serial-number buffer.
   */
  inline void cpySernum(Sernum sernum)
  {
    memcpy(sernum, rom_.address.sernum, Params::SERNUM_LEN);
  }

  /**
   * @brief Copy cached scratchpad into user buffer.
   * @param scratchpad Destination scratchpad buffer.
   */
  inline void cpyScratchpad(Scratchpad scratchpad)
  {
    memcpy(scratchpad, memory_.buffer, Params::SCRATCHPAD_LEN);
  }

  ///@}

  /** @name Setters */
  ///@{

  /** @brief Store an explicit last result code. */
  inline ResultCodes setLastResult(ResultCodes result = ResultCodes::SUCCESS)
  {
    return status_.lastResult = result;
  }

  /** @brief Write cached scratchpad values to the selected sensor. */
  inline ResultCodes setCache() { return writeScratchpad(); }

  ///@}

  /** @name Getters */
  ///@{

  /** @brief Return the most recent result code. */
  inline ResultCodes getLastResult() { return status_.lastResult; }

  /** @brief Read and cache scratchpad of the selected sensor. */
  inline ResultCodes getCache() { return readScratchpad(); }

  /** @brief Check whether the last result represents success. */
  inline bool isSuccess() { return status_.lastResult == ResultCodes::SUCCESS; }

  /**
   * @brief Store and evaluate a result code for success.
   * @param result Result code to store and check.
   * @return True if result is success.
   */
  inline bool isSuccess(ResultCodes result)
  {
    setLastResult(result);
    return isSuccess();
  }

  /** @brief Check whether the last result represents an error. */
  inline bool isError() { return !isSuccess(); }

  /**
   * @brief Store and evaluate a result code for error.
   * @param result Result code to store and check.
   * @return True if result is error.
   */
  inline bool isError(ResultCodes result)
  {
    setLastResult(result);
    return isError();
  }

  /** @brief Check whether low alarm condition is reported. */
  inline bool isAlarmLow()
  {
    return status_.lastResult == ResultCodes::ERROR_ALARM_LOW;
  }

  /**
   * @brief Store and evaluate a result code for low alarm condition.
   * @param result Result code to store and check.
   * @return True if low alarm condition is reported.
   */
  inline bool isAlarmLow(ResultCodes result)
  {
    setLastResult(result);
    return isAlarmLow();
  }

  /** @brief Check whether high alarm condition is reported. */
  inline bool isAlarmHigh()
  {
    return status_.lastResult == ResultCodes::ERROR_ALARM_HIGH;
  }

  /**
   * @brief Store and evaluate a result code for high alarm condition.
   * @param result Result code to store and check.
   * @return True if high alarm condition is reported.
   */
  inline bool isAlarmHigh(ResultCodes result)
  {
    setLastResult(result);
    return isAlarmHigh();
  }

  /** @brief Check whether any alarm condition is reported. */
  inline bool isAlarm() { return isAlarmLow() || isAlarmHigh(); }
  /**
   * @brief Store and evaluate a result code for any alarm condition.
   * @param result Result code to store and check.
   * @return True if low or high alarm condition is reported.
   */
  inline bool isAlarm(ResultCodes result)
  {
    setLastResult(result);
    return isAlarmLow() || isAlarmHigh();
  }

  /** @brief Get cached low alarm threshold. */
  inline int8_t getAlarmLow() { return memory_.scratchpad.alarm_lsb; }

  /** @brief Get cached high alarm threshold. */
  inline int8_t getAlarmHigh() { return memory_.scratchpad.alarm_msb; }

  /** @brief Get default low alarm threshold. */
  static inline int8_t getAlarmLowIni() { return 70; }

  /** @brief Get default high alarm threshold. */
  static inline int8_t getAlarmHighIni() { return 75; }

  /** @brief Get one-wire bus pin number. */
  inline uint8_t getPin() { return bus_.pinBus; }

  /** @brief Get number of detected devices on the bus. */
  inline uint8_t getDevices() { return bus_.devices; }

  /** @brief Get number of detected supported sensors on the bus. */
  inline uint8_t getSensors() { return bus_.sensors; }

  /** @brief Get family code of currently selected sensor. */
  inline uint8_t getFamilyCode() { return rom_.address.family; }

  /** @brief Get identifier derived from CRC byte of selected sensor ROM. */
  inline uint8_t getId() { return rom_.address.crc; }

  /** @brief Get minimum measurable temperature in Celsius. */
  static inline float getTemperatureMin() { return -55.0; }

  /** @brief Get maximum measurable temperature in Celsius. */
  static inline float getTemperatureMax() { return 125.0; }

  /** @brief Get power-on initial sensor temperature value in Celsius. */
  static inline float getTemperatureIni() { return 85.0; }

  /** @brief Check whether all devices are externally powered. */
  inline bool isPowerExternal() { return bus_.powerExternal; }

  /** @brief Check whether at least one device uses parasite power. */
  inline bool isPowerParasite() { return !isPowerExternal(); }

  /** @brief Get pointer to cached ROM address buffer. */
  inline uint8_t *getAddressRef() { return rom_.buffer; }

  /** @brief Get pointer to cached scratchpad buffer. */
  inline uint8_t *getScratchpadRef() { return memory_.buffer; }

  /** @brief Get selected sensor resolution in bits. */
  inline uint8_t getResolutionBits() { return bus_.tempBits[getResolution()]; }

  /** @brief Get selected sensor resolution in Celsius degrees per LSB. */
  inline float getResolutionTemp()
  {
    uint8_t denom = 2 << getResolution();
    return 1.0 / denom;
  }

  /** @brief Get selected sensor resolution index from cached config register.
   */
  inline uint8_t getResolution()
  {
    uint8_t resolution = memory_.scratchpad.config >> ConfigRegBit::R0;
    return resolution & 0b11;
  }

  /**
   * @brief Get temperature decoded from cached scratchpad.
   * @return Temperature in Celsius.
   */
  float getTemperature()
  {
    int16_t temp = memory_.scratchpad.temp_msb << 8;
    temp |= memory_.scratchpad.temp_lsb & bus_.tempMask[getResolution()];
    return (float)temp / 16.0;
  }

  /** @brief Get maximum conversion time for current resolution in milliseconds.
   */
  inline uint16_t getConvMillis() { return bus_.tempMillis[getResolution()]; }

  ///@}

private:
  /// @brief Enumeration of DS18B20 scratchpad config register bits.
  enum ConfigRegBit : uint8_t
  {
    R0 = 5,
    R1 = 6,
  };

  /// @brief Enumeration of DS18B20 ROM commands.
  enum CommandsRom : uint8_t
  {
    SEARCH_ROM = 0xF0,
    READ_ROM = 0x33,
    MATCH_ROM = 0x55,
    SKIP_ROM = 0xCC,
    ALARM_SEARCH = 0xEC,
  };

  /// @brief Enumeration of DS18B20 function commands.
  enum CommandsFnc : uint8_t
  {
    CONVERT_T = 0x44,
    WRITE_SCRATCHPAD = 0x4E,
    READ_SCRATCHPAD = 0xBE,
    COPY_SCRATCHPAD = 0x48,
    RECALL = 0xB8,
    READ_POWER_SUPPLY = 0xB4,
  };

  /// @brief Address register structure.
  union ROM
  {
    uint8_t buffer[Params::ADDRESS_LEN];
    struct Address
    {
      uint8_t family;
      uint8_t sernum[6];
      uint8_t crc;
    } address;
  } rom_;

  /// @brief Scratchpad register structure.
  union Memory
  {
    uint8_t buffer[Params::SCRATCHPAD_LEN];
    struct Scratchpad
    {
      uint8_t temp_lsb;
      uint8_t temp_msb;
      uint8_t alarm_msb;
      uint8_t alarm_lsb;
      uint8_t config;
      uint8_t res_ff;
      uint8_t res_0c;
      uint8_t res_10;
      uint8_t crc;
    } scratchpad;
  } memory_;

  /// @brief One-wire bus parameters.
  struct Bus
  {
    // Resolutions in bits - Values { 0x1F, 0x3F, 0x5F, 0x7F }
    uint8_t tempBits[4] = { 9, 10, 11, 12 };

    // LSB bits masks
    uint8_t tempMask[4] = { 0xF8, 0xFC, 0xFE, 0xFF };

    // Maximal conversion times in milliseconds
    uint16_t tempMillis[4] = { 94, 188, 375, 750 };

    // GPIO pin used as one-wire data bus
    uint8_t pinBus;

    // The highest resolution of all devices
    uint8_t resolution;

    // Flag about all devices powered externally
    bool powerExternal;

    // The number of all devices on the bus
    uint8_t devices;

    // The number of temperature sensors on the bus
    uint8_t sensors;

    // Global alarm handlers
    Handler *alarmHandlerLow;
    Handler *alarmHandlerHigh;
  } bus_;

  /// @brief Status of the last operation.
  struct Status
  {
    ResultCodes lastResult;
  } status_;

  /** @brief Detect bus power mode and cache result. */
  ResultCodes powering();

  /**
   * @brief Copy provided ROM address to internal cache.
   * @param address Source ROM address.
   * @return Result code.
   */
  ResultCodes cpyRom(const Address address);

  /** @brief Clear cached ROM address buffer. */
  inline void resetRom() { memset(rom_.buffer, 0, Params::ADDRESS_LEN); }

  /** @brief Wait for conversion completion according to current bus mode. */
  ResultCodes conversionWait();

  /** @brief Read selected sensor scratchpad to internal cache. */
  ResultCodes readScratchpad();

  /** @brief Write cached scratchpad values to selected sensor. */
  ResultCodes writeScratchpad();

  /** @brief Clear cached scratchpad buffer. */
  inline void resetScratchpad()
  {
    memset(memory_.buffer, 0, Params::SCRATCHPAD_LEN);
  }
};

#endif

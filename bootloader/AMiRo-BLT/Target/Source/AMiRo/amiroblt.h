/**
 * @file    interfaces.h
 * @brief   Interfaces to access and interpret information of the bootloader by an operating system.
 *
 * @addtogroup  AMiRo-BLT
 * @details     Data structures and constants to interfere with the bootloader form an operating system.
 * @{
 */

#ifndef _AMIROBLT_H_
#define _AMIROBLT_H_

#include <stdint.h>

/*============================================================================*/
/**
 * @defgroup  AMiRo-BLT-IF AMiRo Bootloader Interface
 * @details   This interface can be used to interface the AMiRo bootloader from an operating system.
 *
 * @{
 */
/*============================================================================*/

/*============================================================================*/
/**
 * @name    Callback Table
 * @details The callback table is a static struct, located in the MCUs flash memory (ROM) with offset @ref BL_CALLBACK_TABLE_OFFSET .
 *          It contains version information and a bunch of callback function pointers.
 */
///@{

/**
 * @brief   The offset from the flash memory base address to where the @ref blCallbackTable_t struct is stored.
 * @note    This offset must correspond with the @p _callback_table section in file @p memory.x of the bootloader.
 */
#define BL_CALLBACK_TABLE_OFFSET                  0x000001C0u

/**
 * @brief   The memory address where the @ref blCallbackTable_t struct is located in flash memory.
 * @note    This adress must correspond with the @p _callback_table section in file @p memory.x of the bootloader.
 * @deprecated  The @ref BL_CALLBACK_TABLE_OFFSET macro must be used for new software.
 */
#define BL_CALLBACK_TABLE_ADDRESS                 ((blCallbackTable_t*)(0x08000000u + BL_CALLBACK_TABLE_OFFSET))

/**
 * @brief   The major version number of the bootloader.
 */
#define BL_VERSION_MAJOR                          1

/**
 * @brief   The minor version number of the bootloader.
 */
#define BL_VERSION_MINOR                          1

/**
 * @brief   The major version number of the implemented SSSP (Startup & Shutdown Synchronization Protocol).
 */
#define SSSP_VERSION_MAJOR                        1

/**
 * @brief   The minor version number of the implemented SSSP (Startup & Shutdown Synchronization Protocol).
 */
#define SSSP_VERSION_MINOR                        1

/**
 * @brief   A struct to define a version in a generic way.
 */
struct blVersion_t {
  const uint8_t identifier;   /**< The identifier can be used if the version can be ambiguous.  */
  const uint8_t major;        /**< Major version number.                                        */
  const uint8_t minor;        /**< Minor version number.                                        */
  const uint8_t patch;        /**< Patch level number.                                          */
} __attribute__((packed));

/**
 * @brief   Synonym for simplified access.
 */
typedef struct blVersion_t blVersion_t;

/**
 * @brief   An enum that defines possible values for the 'identifier' member of the @p blVersion_t struct.
 * @note    The values must be in (unsigned) 8 bit integer range.
 */
enum blVersionIdentifier_t {
  BL_VERSION_ID_UNKNOWN                   = 0x00u,  /**< Unknown identifier                                       */
  BL_VERSION_ID_NA                        = 0xFFu,  /**< Identifier not applicable                                */
  BL_VERSION_ID_AMiRoBLT_PreAlpha         = 0x01u,  /**< AMiRo-BLT pre-alpha identifier.                          */
  BL_VERSION_ID_AMiRoBLT_Alpha            = 0x02u,  /**< AMiRo-BLT alpha version identifier.                      */
  BL_VERSION_ID_AMiRoBLT_Beta             = 0x03u,  /**< AMiRo-BLT beta version identifier.                       */
  BL_VERSION_ID_AMiRoBLT_ReleaseCandidate = 0x04u,  /**< AMiRo-BLT release candidate (RC) identifier.             */
  BL_VERSION_ID_AMiRoBLT_Release          = 0x05u,  /**< AMiRo-BLT release version identifier.                    */
  BL_VERSION_ID_SSSP                      = 0x01u,  /**< Startup & Shutdown Synchronization Protocol identifier.  */
  BL_VERSION_ID_GCC                       = 0x01u,  /**< Gnu Compiler Collection identifier.                      */
};

/**
 * @brief   A magic number to detect whether the callback table exists.
 * @details The magic number can be interpreted as four character string "A-BL".
 *          The according hexadecimal values are 0x41, 0x2D, 0x42, and 0x4C.
 *          The according 32 bit integer is 0x412D424C.
 */
#define BL_MAGIC_NUMBER                           ((uint32_t)(('A'<<24) | ('-'<<16) | ('B'<<8) | ('L'<<0)))

/**
 * @brief   This struct contains information about the bootloader version and pointers to several callback function.
 *          It is located at the constant address @ref BL_CALLBACK_TABLE_OFFSET in the flash memory.
 * @note    Sicne the actual implementations of the callback functions are optional, the pointers must be checked not to be @p NULL before use!
 */
struct blCallbackTable_t {
  const uint32_t magicNumber;                     /**< A magic number that can be used for sanity checks. @sa BL_MAGIC_NUMBER.            */
  const blVersion_t vBootloader;                  /**< Version information about the bootloader.                                          */
  const blVersion_t vSSSP;                        /**< Version information about the Startup & Shutdown Synchronization Protocol (SSSP).  */
  const blVersion_t vCompiler;                    /**< Version information about the used compiler.                                       */
  /**
   * @brief   Callback function pointer to enter hibernate mode.
   * @details In hibernate mode, the system enters maximum power-saving mode, but will wake up periodically in order to read some sensors.
   */
  void (*const cbShutdownHibernate)(void);
  /**
   * @brief   Callback function pointer to enter deepsleep mode.
   * @details In deepsleep mode, the system enters maximum power-saving mode, but basic sensors (e.g. charging plug) can wake the system.
   */
  void (*const cbShutdownDeepsleep)(void);
  /**
   * @brief   Callback function pointer to enter transportation mode.
   * @details In transportation mode, the system enters maximum power-saving mode and all wakeup signals are deactivated.
   *          Thus, the only way to reactivate the system will be a hard reset.
   */
  void (*const cbShutdownTransportation)(void);   /**< Callback function pointer to enter transportation mode. */
  /**
   * @brief   Callback function pointer to restart the system.
   */
  void (*const cbShutdownRestart)(void);
  /**
   * @brief   Callback function pointer to handle a shutdown request from another module.
   * @details Depending on the result of the disambiguation procedure, the module will enter the requested low-power mode.
   */
  void (*const cbHandleShutdownRequest)(void);
  void (*const cb5)(void);                        /**< Reserved but currently unused callback function pointer. */
  void (*const cb6)(void);                        /**< Reserved but currently unused callback function pointer. */
  void (*const cb7)(void);                        /**< Reserved but currently unused callback function pointer. */
  void (*const cb8)(void);                        /**< Reserved but currently unused callback function pointer. */
  void (*const cb9)(void);                        /**< Reserved but currently unused callback function pointer. */
  void (*const cb10)(void);                       /**< Reserved but currently unused callback function pointer. */
  void (*const cb11)(void);                       /**< Reserved but currently unused callback function pointer. */
} __attribute__((__packed__));

/**
 * @brief Synonym for simplified access.
 */
typedef struct blCallbackTable_t blCallbackTable_t;

///@}
/*============================================================================*/

/*============================================================================*/
/**
 * @name    Backup Register
 * @details For the STM32F4 MCU (PowerManagement), the backup register contains additional information about the last shutdown and wakeup of the system.
 */
///@{

/**
 * @brief   Number of the RTC backup register in which some persistent information is stored.
 * @note    In the RTC section of the STM32F4 reference manual, the RTC backup registers are named @p RTC_BKPxR, where x is the register number.
 */
#define BL_RTC_BACKUP_REG                         0

/**
 * @brief   This union is used for interpretation of the content of the backup register.
 */
union blBackupRegister_t {
  /**
   * @brief The raw data of the register.
   */
  uint32_t raw;

  /**
   * @brief A struct to interpret the content of the register.
   */
  struct {
    /**
     * @brief The primary reason for the last system shutdown.
     * @note  The content of this member is enum-like.
     * @sa    BL_SHUTDOWN_PRI_RSN_UNKNOWN, BL_SHUTDOWN_PRI_RSN_HIBERNATE, BL_SHUTDOWN_PRI_RSN_DEEPSLEEP, BL_SHUTDOWN_PRI_RSN_TRANSPORT, BL_SHUTDOWN_PRI_RSN_RESTART
     */
    uint8_t shutdown_pri_reason;

    /**
     * @brief The secondary reason for the last system shutdown.
     * @note  The content of this member is enum-like.
     * @sa    BL_SHUTDOWN_SEC_RSN_UNKNOWN
     */
    uint8_t shutdown_sec_reason;

    /**
     * @brief The primary reason for the last system wakeup.
     * @note  The content of this member is bitmask-like.
     * @sa    BL_WAKEUP_PRI_RSN_UNKNOWN, BL_WAKEUP_PRI_RSN_LPWRRST, BL_WAKEUP_PRI_RSN_WWDGRST, BL_WAKEUP_PRI_RSN_IWDGRST, BL_WAKEUP_PRI_RSN_SFTRST, BL_WAKEUP_PRI_RSN_PORRST, BL_WAKEUP_PRI_RSN_PINRST, BL_WAKEUP_PRI_RSN_BORRST, BL_WAKEUP_PRI_RSN_WKUP
     */
    uint8_t wakeup_pri_reason;

    /**
     * @brief The secondary reason for the last system wakeup.
     * @note  The content of this member is bitmask-like.
     * @sa    BL_WAKEUP_SEC_RSN_UNKNOWN, BL_WAKEUP_SEC_RSN_UART, BL_WAKEUP_SEC_RSN_PWRPLUG, BL_WAKEUP_SEC_RSN_VSYSHIGH, BL_WAKEUP_SEC_RSN_VSYSLOW, BL_WAKEUP_SEC_RSN_TOUCH, BL_WAKEUP_SEC_RSN_ACCEL, BL_WAKEUP_SEC_RSN_RSVD
     */
    uint8_t wakeup_sec_reason;
  } __attribute__((__packed__));
};

/**
 * @brief   Synonym for simplified access.
 */
typedef union blBackupRegister_t blBackupRegister_t;

/**
 * @brief   An enum defining identifiers for possible reasons for system shutdown.
 * @note    The values must be in (unsigned) 8 bit integer range.
 */
enum blShutdownReason_t {
  /**
   * @brief The reason was not set or the register was reset.
   */
  BL_SHUTDOWN_PRI_RSN_UNKNOWN   = 0x00u,

  /**
   * @brief The system was shut down to enter hibernate mode.
   */
  BL_SHUTDOWN_PRI_RSN_HIBERNATE = 0x01u,

  /**
   * @brief The system was shut down to enter deepsleep mode.
   */
  BL_SHUTDOWN_PRI_RSN_DEEPSLEEP = 0x02u,

  /**
   * @brief The system was shut down to enter transportation mode.
   */
  BL_SHUTDOWN_PRI_RSN_TRANSPORT = 0x03u,

  /**
   * @brief The system was shut down to restart.
   */
  BL_SHUTDOWN_PRI_RSN_RESTART   = 0x04u,

  /**
   * @brief   The default shutdown mode.
   * @details This mode is used, if the shutdown sequence was triggered by another module than the PowerManagement.
   */
  BL_SHUTDOWN_PRI_RSN_DEFAULT   = BL_SHUTDOWN_PRI_RSN_DEEPSLEEP,

  /**
   * @brief The reason was not set or the regsiter was reset.
   */
  BL_SHUTDOWN_SEC_RSN_UNKNOWN   = 0x00u,
};

/**
 * @brief   An enum defining identifiers for possible reasons for system wakeup.
 * @note    The values must be in (unsigned) 8 bit integer range.
 */
enum blWakeupReason_t {
  /**
   * @brief The reason could not be determined.
   */
  BL_WAKEUP_PRI_RSN_UNKNOWN    = 0x00u,

  /**
   * @brief The system was reset because of the low-power management configuration.
   */
  BL_WAKEUP_PRI_RSN_LPWRRST    = 0x01u,

  /**
   * @brief The system was woken by the window watchdog.
   */
  BL_WAKEUP_PRI_RSN_WWDGRST    = 0x02u,

  /**
   * @brief The system was woken by the independant watchdog.
   */
  BL_WAKEUP_PRI_RSN_IWDGRST    = 0x04u,

  /**
   * @brief The system was reset by software.
   */
  BL_WAKEUP_PRI_RSN_SFTRST     = 0x08u,

  /**
   * @brief The system was woken by a power-on/power-down reset.
   */
  BL_WAKEUP_PRI_RSN_PORRST     = 0x10u,

  /**
   * @brief The system was reset via the NRST pin.
   */
  BL_WAKEUP_PRI_RSN_PINRST     = 0x20u,

  /**
   * @brief The system was woken by a power-on/power-down, or a brownout reset.
   */
  BL_WAKEUP_PRI_RSN_BORRST     = 0x40u,

  /**
   * @brief The system was woken via the WKUP pin.
   */
  BL_WAKEUP_PRI_RSN_WKUP       = 0x80u,

  /**
   * @brief The reason could not be determined.
   */
  BL_WAKEUP_SEC_RSN_UNKNOWN    = 0x00u,

  /**
   * @brief The WKUP pin was triggered by the UART signal.
   */
  BL_WAKEUP_SEC_RSN_UART       = 0x01u,

  /**
   * @brief The WKUP pin was triggered by plugging in a power plug.
   */
  BL_WAKEUP_SEC_RSN_PWRPLUG    = 0x02u,

  /**
   * @brief The internal ADC detected a system voltage higher than the configured threshold.
   * @note  Usually the threshold is set to 9V.
   */
  BL_WAKEUP_SEC_RSN_VSYSHIGH   = 0x04u,

  /**
   * @brief The internal ADC detected a system voltage lower than the configured threshold.
   */
  BL_WAKEUP_SEC_RSN_VSYSLOW    = 0x08u,

  /**
   * @brief The WKUP pin was triggered by an interrupt from the touch sensors.
   */
  BL_WAKEUP_SEC_RSN_TOUCH      = 0x10u,

  /**
   * @brief The WKUP pin was triggered by an interrupt from the accelerometer.
   */
  BL_WAKEUP_SEC_RSN_ACCEL      = 0x20u,

  /**
   * @brief Reserved value that must not be used right now, but might become valid in a future version.
   */
  BL_WAKEUP_SEC_RSN_RSVD       = 0xC0u,
};

///@}

/** @} */

#endif // _AMIROBLT_H_

/** @} */

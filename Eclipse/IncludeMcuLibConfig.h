
/* header file is included with -include compiler option */

#define McuLib_CONFIG_CPU_IS_KINETIS    (0)
#define McuLib_CONFIG_CPU_IS_NORDIC_NRF (1)
#define McuLib_CONFIG_SDK_VERSION_USED  McuLib_CONFIG_SDK_NORDIC_NRF5
#define McuLib_CONFIG_SDK_USE_FREERTOS  (0)

/* LED pins */
#define LEDpin1_CONFIG_PIN_NUMBER      25u
#define LEDpin2_CONFIG_PIN_NUMBER      26u

#if 0
/* OLED */
/* ------------------- I2C ---------------------------*/
#define SCL1_CONFIG_GPIO_NAME     GPIOA
#define SCL1_CONFIG_PORT_NAME     PORTA
#define SCL1_CONFIG_PIN_NUMBER    12u

#define SDA1_CONFIG_GPIO_NAME     GPIOA
#define SDA1_CONFIG_PORT_NAME     PORTA
#define SDA1_CONFIG_PIN_NUMBER    13u

#define McuGenericI2C_CONFIG_USE_ON_ERROR_EVENT (0)
#define McuGenericI2C_CONFIG_USE_MUTEX          (0)

#define McuGenericSWI2C_CONFIG_DO_YIELD (0) /* because of Yield in GenericSWI2C */
#define McuGenericSWI2C_CONFIG_DELAY_NS (0)
/* I2C Pin Muxing */
#define SDA1_CONFIG_DO_PIN_MUXING (1)
#define SCL1_CONFIG_DO_PIN_MUXING (1)
/* LCD settings */
#if 1
  #define McuSSD1306_CONFIG_SSD1306_DRIVER_TYPE  (1106)
#else
  #define McuSSD1306_CONFIG_SSD1306_DRIVER_TYPE  (1306)
#endif

#endif

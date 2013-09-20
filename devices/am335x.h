/**
 * @file am335x.h
 * @author Ethan Hayon
 *
 * This file contains important config for TI am335x ARM Cortex-A8 SoC
 *
 * PLEASE DO NOT MODIFY
 *
 * Licensed under the MIT License (MIT)
 * See MIT-LICENSE file for more information
 */

#ifndef _AM335X_H_
#define _AM335X_H_

#define MMAP_OFFSET (0x44C00000)
#define MMAP_SIZE   (0x481AEFFF-MMAP_OFFSET)

/* Clock Module Memory Registers */
#define CM_WKUP (0x44E00400)
#define CM_WKUP_ADC_TSC_CLKCTRL (CM_WKUP+0xBC)
#define CM_WKUP_MODULEMODE_ENABLE (0x02)
#define CM_WKUP_IDLEST_DISABLED (0x03<<16)

#define CM_PER (0x44E00000)
#define CM_PER_EPWMSS1_CLKCTRL (CM_PER+0xCC)
#define CM_PER_EPWMSS0_CLKCTRL (CM_PER+0xD4)
#define CM_PER_EPWMSS2_CLKCTRL (CM_PER+0xD8)


/* GPIO Memory Registers */
#define GPIO_REGISTER_SIZE (4)

#define GPIO0 (0x44E07000)
#define GPIO1 (0x4804C000)
#define GPIO2 (0x481AC000)
#define GPIO3 (0x481AE000)

#define GPIO_CLEARDATAOUT (0x190)
#define GPIO_SETDATAOUT   (0x194)
#define GPIO_OE           (0x134)
#define GPIO_DATAOUT      (0x13C)
#define GPIO_DATAIN       (0x138)

/* Analog Digital Converter Memory Registers */
#define ADC_TSC (0x44E0D000)

#define ADC_CTRL (ADC_TSC+0x40)
#define ADC_STEPCONFIG_WRITE_PROTECT_OFF (0x01<<2)
#define ADC_STEPENABLE (ADC_TSC+0x54)

#define ADCSTEPCONFIG1 (ADC_TSC+0x64)
#define ADCSTEPDELAY1  (ADC_TSC+0x68)
#define ADCSTEPCONFIG2 (ADC_TSC+0x6C)
#define ADCSTEPDELAY2  (ADC_TSC+0x70)
#define ADCSTEPCONFIG3 (ADC_TSC+0x74)
#define ADCSTEPDELAY3  (ADC_TSC+0x78)
#define ADCSTEPCONFIG4 (ADC_TSC+0x7C)
#define ADCSTEPDELAY4  (ADC_TSC+0x80)
#define ADCSTEPCONFIG5 (ADC_TSC+0x84)
#define ADCSTEPDELAY5  (ADC_TSC+0x88)
#define ADCSTEPCONFIG6 (ADC_TSC+0x8C)
#define ADCSTEPDELAY6  (ADC_TSC+0x90)
#define ADCSTEPCONFIG7 (ADC_TSC+0x94)
#define ADCSTEPDELAY7  (ADC_TSC+0x98)
#define ADCSTEPCONFIG8 (ADC_TSC+0x9C)
#define ADCSTEPDELAY8  (ADC_TSC+0xA0)

#define ADC_AVG0  (0x000)
#define ADC_AVG2  (0x001)
#define ADC_AVG4  (0x010)
#define ADC_AVG8  (0x011)
#define ADC_AVG16 (0x100)

#define ADC_FIFO0DATA (ADC_TSC+0x100)
#define ADC_FIFO_MASK (0xFFF)

typedef struct s_PWM
{
    char muxmode; /*!< mux mode, 0-7, see am335x technical manual */
    char *name;   /*!< name of pwm pin, i.e.: "EHRPWM2B" */
    char *path;   /*!< path to the pwm, i.e.: "ehrpwm.2:1" */
} PWM;

typedef struct s_PIN
{
    char *name;   /*!< readable name of pin, i.e.: "GPIO1_21", see beaglebone user guide */
    unsigned int gpio_bank; /*!< which of the four gpio banks is this pin in, i.e.: GPIO1, r 0x4804C000 */
    uint8_t gpio; /*!< pin number on the am335x processor */
    uint8_t bank_id; /*!< pin number within each bank, should be 0-31 */
    char *mux;    /*!< file name for setting mux */
    uint8_t eeprom; /*!< position in eeprom */
    unsigned char pwm_present; /*!< whether or not this pin can be used for PWM */
    PWM pwm;      /*!< pwm struct if pwm_present is true */

} PIN;

#define TRUE 1
#define FALSE 0

#define USR0  ((PIN){ const_cast<char *>("GPIO1_21"),   GPIO1, 0,  1,  const_cast<char *>(""),               0,  FALSE, 0   })
#define USR1  ((PIN){ const_cast<char *>("GPIO1_22"),   GPIO1, 0,  2,  const_cast<char *>(""),               0,  FALSE, 0   })
#define USR2  ((PIN){ const_cast<char *>("GPIO1_23"),   GPIO1, 0,  3,  const_cast<char *>(""),               0,  FALSE, 0   })
#define USR3  ((PIN){ const_cast<char *>("GPIO1_24"),   GPIO1, 0,  4,  const_cast<char *>(""),               0,  FALSE, 0   })
#define P8_3  ((PIN){ const_cast<char *>("GPIO1_6"),    GPIO1, 38, 6,  const_cast<char *>("gpmc_ad6"),       26, FALSE, 0   })
#define P8_4  ((PIN){ const_cast<char *>("GPIO1_7"),    GPIO1, 39, 7,  const_cast<char *>("gpmc_ad7"),       27, FALSE, 0   })
#define P8_5  ((PIN){ const_cast<char *>("GPIO1_2"),    GPIO1, 34, 2,  const_cast<char *>("gpmc_ad2"),       22, FALSE, 0   })
#define P8_6  ((PIN){ const_cast<char *>("GPIO1_3"),    GPIO1, 35, 3,  const_cast<char *>("gpmc_ad3"),       23, FALSE, 0   })
#define P8_7  ((PIN){ const_cast<char *>("TIMER4"),     GPIO2, 66, 2,  const_cast<char *>("gpmc_advn_ale"),  41, FALSE, 0   })
#define P8_8  ((PIN){ const_cast<char *>("TIMER7"),     GPIO2, 67, 3,  const_cast<char *>("gpmc_oen_ren"),   44, FALSE, 0   })
#define P8_9  ((PIN){ const_cast<char *>("TIMER5"),     GPIO2, 69, 5,  const_cast<char *>("gpmc_ben0_cle"),  42, FALSE, 0   })
#define P8_10 ((PIN){ const_cast<char *>("TIMER6"),     GPIO2, 68, 4,  const_cast<char *>("gpmc_wen"),       43, FALSE, 0   })
#define P8_11 ((PIN){ const_cast<char *>("GPIO1_13"),   GPIO1, 45, 13, const_cast<char *>("gpmc_ad13"),      29, FALSE, 0   })
#define P8_12 ((PIN){ const_cast<char *>("GPIO1_12"),   GPIO1, 44, 12, const_cast<char *>("gpmc_ad12"),      28, FALSE, 0   })
#define P8_13 ((PIN){ const_cast<char *>("EHRPWM2B"),   GPIO0, 23, 23, const_cast<char *>("gpmc_ad9"),       15, TRUE,  (PWM){4, const_cast<char *>("EHRPWM2B"), const_cast<char *>("ehrpwm.2:1")}})
#define P8_14 ((PIN){ const_cast<char *>("GPIO0_26"),   GPIO0, 26, 26, const_cast<char *>("gpmc_ad10"),      16, FALSE, 0   })
#define P8_15 ((PIN){ const_cast<char *>("GPIO1_15"),   GPIO1, 47, 15, const_cast<char *>("gpmc_ad15"),      31, FALSE, 0   })
#define P8_16 ((PIN){ const_cast<char *>("GPIO1_14"),   GPIO1, 46, 14, const_cast<char *>("gpmc_ad14"),      30, FALSE, 0   })
#define P8_17 ((PIN){ const_cast<char *>("GPIO0_27"),   GPIO0, 27, 27, const_cast<char *>("gpmc_ad11"),      17, FALSE, 0   })
#define P8_18 ((PIN){ const_cast<char *>("GPIO2_1"),    GPIO2, 65, 1,  const_cast<char *>("gpmc_clk"),       40, FALSE, 0   })
#define P8_19 ((PIN){ const_cast<char *>("EHRPWM2A"),   GPIO0, 22, 22, const_cast<char *>("gpmc_ad8"),       14, TRUE,  (PWM){4, const_cast<char *>("EHRPWM2A"), const_cast<char *>("ehrpwm.2:0")}})
#define P8_20 ((PIN){ const_cast<char *>("GPIO1_31"),   GPIO1, 63, 31, const_cast<char *>("gpmc_csn2"),      39, FALSE, 0   })
#define P8_21 ((PIN){ const_cast<char *>("GPIO1_30"),   GPIO1, 62, 30, const_cast<char *>("gpmc_csn1"),      38, FALSE, 0   })
#define P8_22 ((PIN){ const_cast<char *>("GPIO1_5"),    GPIO1, 37, 5,  const_cast<char *>("gpmc_ad5"),       25, FALSE, 0   })
#define P8_23 ((PIN){ const_cast<char *>("GPIO1_4"),    GPIO1, 36, 4,  const_cast<char *>("gpmc_ad4"),       24, FALSE, 0   })
#define P8_24 ((PIN){ const_cast<char *>("GPIO1_1"),    GPIO1, 33, 1,  const_cast<char *>("gpmc_ad1"),       21, FALSE, 0   })
#define P8_25 ((PIN){ const_cast<char *>("GPIO1_0"),    GPIO1, 32, 0,  const_cast<char *>("gpmc_ad0"),       20, FALSE, 0   })
#define P8_26 ((PIN){ const_cast<char *>("GPIO1_29"),   GPIO1, 61, 29, const_cast<char *>("gpmc_csn0"),      37, FALSE, 0   })
#define P8_27 ((PIN){ const_cast<char *>("GPIO2_22"),   GPIO2, 86, 22, const_cast<char *>("lcd_vsync"),      57, FALSE, 0   })
#define P8_28 ((PIN){ const_cast<char *>("GPIO2_24"),   GPIO2, 88, 24, const_cast<char *>("lcd_pclk"),       59, FALSE, 0   })
#define P8_29 ((PIN){ const_cast<char *>("GPIO2_23"),   GPIO2, 87, 23, const_cast<char *>("lcd_hsync"),      58, FALSE, 0   })
#define P8_30 ((PIN){ const_cast<char *>("GPIO2_25"),   GPIO2, 89, 25, const_cast<char *>("lcd_ac_bias_en"), 60, FALSE, 0   })
#define P8_31 ((PIN){ const_cast<char *>("UART5_CTSN"), GPIO0, 10, 10, const_cast<char *>("lcd_data14"),      7, FALSE, 0   })
#define P8_32 ((PIN){ const_cast<char *>("UART5_RTSN"), GPIO0, 11, 11, const_cast<char *>("lcd_data15"),      8, FALSE, 0   })
#define P8_33 ((PIN){ const_cast<char *>("UART4_RTSN"), GPIO0, 9,  9,  const_cast<char *>("lcd_data13"),      6, FALSE, 0   })
#define P8_34 ((PIN){ const_cast<char *>("UART3_RTSN"), GPIO2, 81, 17, const_cast<char *>("lcd_data11"),     56, TRUE,  (PWM){2, const_cast<char *>("EHRPWM1B"), const_cast<char *>("ehrpwm.1:1")}})
#define P8_35 ((PIN){ const_cast<char *>("UART4_CTSN"), GPIO0, 8,  8,  const_cast<char *>("lcd_data12"),      5, FALSE, 0   })
#define P8_36 ((PIN){ const_cast<char *>("UART3_CTSN"), GPIO2, 80, 16, const_cast<char *>("lcd_data10"),     55, TRUE,  (PWM){2, const_cast<char *>("EHRPWM1A"), const_cast<char *>("ehrpwm.1:0")}})
#define P8_37 ((PIN){ const_cast<char *>("UART5_TXD"),  GPIO2, 78, 14, const_cast<char *>("lcd_data8"),      53, FALSE, 0   })
#define P8_38 ((PIN){ const_cast<char *>("UART5_RXD"),  GPIO2, 79, 15, const_cast<char *>("lcd_data9"),      54, FALSE, 0   })
#define P8_39 ((PIN){ const_cast<char *>("GPIO2_12"),   GPIO2, 76, 12, const_cast<char *>("lcd_data6"),      51, FALSE, 0   })
#define P8_40 ((PIN){ const_cast<char *>("GPIO2_13"),   GPIO2, 77, 13, const_cast<char *>("lcd_data7"),      52, FALSE, 0   })
#define P8_41 ((PIN){ const_cast<char *>("GPIO2_10"),   GPIO2, 74, 10, const_cast<char *>("lcd_data4"),      49, FALSE, 0   })
#define P8_42 ((PIN){ const_cast<char *>("GPIO2_11"),   GPIO2, 75, 11, const_cast<char *>("lcd_data5"),      50, FALSE, 0   })
#define P8_43 ((PIN){ const_cast<char *>("GPIO2_8"),    GPIO2, 72, 8,  const_cast<char *>("lcd_data2"),      47, FALSE, 0   })
#define P8_44 ((PIN){ const_cast<char *>("GPIO2_9"),    GPIO2, 73, 9,  const_cast<char *>("lcd_data3"),      48, FALSE, 0   })
#define P8_45 ((PIN){ const_cast<char *>("GPIO2_6"),    GPIO2, 70, 6,  const_cast<char *>("lcd_data0"),      45, TRUE,  (PWM){3, const_cast<char *>("EHRPWM2A"), const_cast<char *>("ehrpwm.2:0")}})
#define P8_46 ((PIN){ const_cast<char *>("GPIO2_7"),    GPIO2, 71, 7,  const_cast<char *>("lcd_data1"),      46, TRUE,  (PWM){3, const_cast<char *>("EHRPWM2B"), const_cast<char *>("ehrpwm.2:1")}})

#define P9_11 ((PIN){   const_cast<char *>("UART4_RXD"),   GPIO0,     30,  30,   const_cast<char *>("gpmc_wait0"),       18,   FALSE, 0})
#define P9_12 ((PIN){   const_cast<char *>("GPIO1_28"),    GPIO1,     60,  28,   const_cast<char *>("gpmc_ben1"),        36,   FALSE, 0})
#define P9_13 ((PIN){   const_cast<char *>("UART4_TXD"),   GPIO0,     31,  31,   const_cast<char *>("gpmc_wpn"),         19,   FALSE, 0})
#define P9_14 ((PIN){   const_cast<char *>("EHRPWM1A"),    GPIO1,     50,  18,   const_cast<char *>("gpmc_a2"),          34,   TRUE, (PWM){6, const_cast<char *>("EHRPWM1A"), const_cast<char *>("ehrpwm.1:0")}})
#define P9_15 ((PIN){   const_cast<char *>("GPIO1_16"),    GPIO1,     48,  16,   const_cast<char *>("mii1_rxd3"),        32,   FALSE, 0})
#define P9_16 ((PIN){   const_cast<char *>("EHRPWM1B"),    GPIO1,     51,  19,   const_cast<char *>("gpmc_a3"),          35,   TRUE, (PWM){6, const_cast<char *>("EHRPWM1B"), const_cast<char *>("ehrpwm.1:1")}})
#define P9_17 ((PIN){   const_cast<char *>("I2C1_SCL"),    GPIO0,      5,   5,   const_cast<char *>("spi0_cs0"),          3,   FALSE, 0})
#define P9_18 ((PIN){   const_cast<char *>("I2C1_SDA"),    GPIO0,      4,   4,   const_cast<char *>("spi0_d1"),           2,   FALSE, 0})
#define P9_19 ((PIN){   const_cast<char *>("I2C2_SCL"),    GPIO0,     13,  13,   const_cast<char *>("uart1_rtsn"),        9,   FALSE, 0})
#define P9_20 ((PIN){   const_cast<char *>("I2C2_SDA"),    GPIO0,     12,  12,   const_cast<char *>("uart1_ctsn"),       10,   FALSE, 0})
#define P9_21 ((PIN){   const_cast<char *>("UART2_TXD"),   GPIO0,      3,   3,   const_cast<char *>("spi0_d0"),           1,   TRUE, (PWM){3, const_cast<char *>("EHRPWM0B"), const_cast<char *>("ehrpwm.0:1")}})
#define P9_22 ((PIN){   const_cast<char *>("UART2_RXD"),   GPIO0,      2,   2,   const_cast<char *>("spi0_sclk"),         0,   TRUE, (PWM){3, const_cast<char *>("EHRPWM0A"), const_cast<char *>("ehrpwm.0:0")}})
#define P9_23 ((PIN){   const_cast<char *>("GPIO1_17"),    GPIO1,     49,  17,   const_cast<char *>("gpmc_a1"),          33,   FALSE, 0})
#define P9_24 ((PIN){   const_cast<char *>("UART1_TXD"),   GPIO0,     15,  15,   const_cast<char *>("uart1_txd"),        12,   FALSE, 0})
#define P9_25 ((PIN){   const_cast<char *>("GPIO3_21"),    GPIO3,    117,  21,   const_cast<char *>("mcasp0_ahclkx"),    66,   FALSE, 0})
#define P9_26 ((PIN){   const_cast<char *>("UART1_RXD"),   GPIO0,     14,  14,   const_cast<char *>("uart1_rxd"),        11,   FALSE, 0})
#define P9_27 ((PIN){   const_cast<char *>("GPIO3_19"),    GPIO3,    115,  19,   const_cast<char *>("mcasp0_fsr"),       64,   FALSE, 0})
#define P9_28 ((PIN){   const_cast<char *>("SPI1_CS0"),    GPIO3,    113,  17,   const_cast<char *>("mcasp0_ahclkr"),    63,   TRUE, (PWM){4, const_cast<char *>("ECAPPWM2"), const_cast<char *>("ecap.2")}})
#define P9_29 ((PIN){   const_cast<char *>("SPI1_D0"),     GPIO3,    111,  15,   const_cast<char *>("mcasp0_fsx"),       61,   TRUE, (PWM){1, const_cast<char *>("EHRPWM0B"), const_cast<char *>("ehrpwm.0:1")}})
#define P9_30 ((PIN){   const_cast<char *>("SPI1_D1"),     GPIO3,    112,  16,   const_cast<char *>("mcasp0_axr0"),      62,   FALSE, 0})
#define P9_31 ((PIN){   const_cast<char *>("SPI1_SCLK"),   GPIO3,    110,  14,   const_cast<char *>("mcasp0_aclkx"),     65,   TRUE, (PWM){1, const_cast<char *>("EHRPWM0A"), const_cast<char *>("ehrpwm.0:0")}})
#define P9_33 ((PIN){   const_cast<char *>("AIN4"),        0,          4,   4,   const_cast<char *>(""),                 71,   FALSE, 0})
#define P9_35 ((PIN){   const_cast<char *>("AIN6"),        0,          6,   6,   const_cast<char *>(""),                 73,   FALSE, 0})
#define P9_36 ((PIN){   const_cast<char *>("AIN5"),        0,          5,   5,   const_cast<char *>(""),                 72,   FALSE, 0})
#define P9_37 ((PIN){   const_cast<char *>("AIN2"),        0,          2,   2,   const_cast<char *>(""),                 69,   FALSE, 0})
#define P9_38 ((PIN){   const_cast<char *>("AIN3"),        0,          3,   3,   const_cast<char *>(""),                 70,   FALSE, 0})
#define P9_39 ((PIN){   const_cast<char *>("AIN0"),        0,          0,   0,   const_cast<char *>(""),                 67,   FALSE, 0})
#define P9_40 ((PIN){   const_cast<char *>("AIN1"),        0,          1,   1,   const_cast<char *>(""),                 68,   FALSE, 0})
#define P9_41 ((PIN){   const_cast<char *>("CLKOUT2"),     GPIO0,     20,  20,   const_cast<char *>("xdma_event_intr1"), 13,   FALSE, 0})
#define P9_42 ((PIN){   const_cast<char *>("GPIO0_7"),     GPIO0,      7,   7,   const_cast<char *>("ecap0_in_pwm0_out"), 4,   TRUE, (PWM){0, const_cast<char *>("ECAPPWM0"), const_cast<char *>("ecap.0")}})


#define INPUT    ((unsigned char)(1))
#define OUTPUT   ((unsigned char)(0))
#define PULLUP   ((unsigned char)(1))
#define PULLDOWN ((unsigned char)(0))
#define DISABLED ((unsigned char)(2))

#define CONFIG_MUX_PATH "/sys/kernel/debug/omap_mux"



#endif

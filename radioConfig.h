#ifndef __RADIOCONFIG_H__
#define __RADIOCONFIG_H__

#define LORA                                        1         // [0: OFF, 1: ON]

#define SX1278SPI               hspi1

#define NSS_IOPORT              LORA_CS_GPIO_Port
#define NSS_PIN                 LORA_CS_Pin

#define RST_IOPORT              LORA_RST_GPIO_Port
#define RST_PIN                 LORA_RST_Pin

#define DIO0_IOPORT             LORA_DIO0_GPIO_Port
#define DIO0_PIN                LORA_DIO0_Pin





#endif

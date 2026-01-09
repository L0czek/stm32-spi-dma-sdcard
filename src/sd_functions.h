#ifndef __SD_FUNCTIONS_H
#define __SD_FUNCTIONS_H

#include <stdint.h>
#include "stm32c0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Type definitions (compatible with FatFS diskio) */
typedef uint8_t  BYTE;
typedef uint16_t WORD;
typedef uint32_t DWORD;
typedef unsigned int UINT;

/* Disk Status Bits */
#define STA_NOINIT   0x01  /* Drive not initialized */
#define STA_NODISK   0x02  /* No medium in the drive */
#define STA_PROTECT  0x04  /* Write protected */

typedef BYTE DSTATUS;

/* Disk function return values */
typedef enum {
    RES_OK = 0,    /* Successful */
    RES_ERROR,     /* R/W Error */
    RES_WRPRT,     /* Write Protected */
    RES_NOTRDY,    /* Not Ready */
    RES_PARERR     /* Invalid Parameter */
} DRESULT;

/* IOCTL commands */
#define CTRL_SYNC         0
#define GET_SECTOR_COUNT  1
#define GET_SECTOR_SIZE   2
#define GET_BLOCK_SIZE    3
#define CTRL_TRIM         4
#define CTRL_POWER        5
#define MMC_GET_CSD       10
#define MMC_GET_CID       11
#define MMC_GET_OCR       12

/* Definitions for MMC/SDC command */
#define CMD0     (0x40+0)       /* GO_IDLE_STATE */
#define CMD1     (0x40+1)       /* SEND_OP_COND */
#define CMD8     (0x40+8)       /* SEND_IF_COND */
#define CMD9     (0x40+9)       /* SEND_CSD */
#define CMD10    (0x40+10)      /* SEND_CID */
#define CMD12    (0x40+12)      /* STOP_TRANSMISSION */
#define CMD16    (0x40+16)      /* SET_BLOCKLEN */
#define CMD17    (0x40+17)      /* READ_SINGLE_BLOCK */
#define CMD18    (0x40+18)      /* READ_MULTIPLE_BLOCK */
#define CMD23    (0x40+23)      /* SET_BLOCK_COUNT */
#define CMD24    (0x40+24)      /* WRITE_BLOCK */
#define CMD25    (0x40+25)      /* WRITE_MULTIPLE_BLOCK */
#define CMD41    (0x40+41)      /* SEND_OP_COND (ACMD) */
#define CMD55    (0x40+55)      /* APP_CMD */
#define CMD58    (0x40+58)      /* READ_OCR */

/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC    0x01    /* MMC ver 3 */
#define CT_SD1    0x02    /* SD ver 1 */
#define CT_SD2    0x04    /* SD ver 2 */
#define CT_SDC    0x06    /* SD */
#define CT_BLOCK  0x08    /* Block addressing */

#define SPI_TIMEOUT 100

/* SD Card Context - holds all configuration and state */
typedef struct {
    SPI_HandleTypeDef *spi_handle;           /* SPI peripheral handle */
    GPIO_TypeDef *cs_port;                   /* Chip select GPIO port */
    uint16_t cs_pin;                         /* Chip select GPIO pin */
    
    /* Internal state */
    volatile DSTATUS status;                 /* Disk status */
    uint8_t card_type;                       /* Card type flags */
    uint8_t power_flag;                      /* Power status */
    volatile uint8_t dma_complete;           /* DMA transfer complete flag */
    uint16_t timer1;                         /* Timeout timer 1 */
    uint16_t timer2;                         /* Timeout timer 2 */
} SD_Context;

/* Initialize SD card context with hardware configuration */
void SD_init(SD_Context *ctx, SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* Timer tick - call this every 1ms (e.g., from SysTick handler) */
void SD_timer_tick(SD_Context *ctx);

/* SPI DMA callbacks - call these from HAL callbacks */
void SD_spi_tx_complete(SD_Context *ctx);
void SD_spi_rx_complete(SD_Context *ctx);
void SD_spi_txrx_complete(SD_Context *ctx);

/* Disk functions */
DSTATUS SD_disk_initialize(SD_Context *ctx);
DSTATUS SD_disk_status(SD_Context *ctx);
DRESULT SD_disk_read(SD_Context *ctx, BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_write(SD_Context *ctx, const BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_ioctl(SD_Context *ctx, BYTE cmd, void* buff);

#ifdef __cplusplus
}
#endif

#endif /* __SD_FUNCTIONS_H */

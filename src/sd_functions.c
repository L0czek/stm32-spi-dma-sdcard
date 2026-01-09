#include "main.h"
#include "sd_functions.h"

#define TRUE  1
#define FALSE 0
#define bool BYTE

/***************************************
 * Public API - Initialization & Callbacks
 **************************************/

/* Initialize SD card context with hardware configuration */
void SD_init(SD_Context *ctx, SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
  ctx->spi_handle = spi_handle;
  ctx->cs_port = cs_port;
  ctx->cs_pin = cs_pin;
  ctx->status = STA_NOINIT;
  ctx->card_type = 0;
  ctx->power_flag = 0;
  ctx->dma_complete = 1;
  ctx->timer1 = 0;
  ctx->timer2 = 0;
}

/* 1ms timer tick - call from SysTick handler */
void SD_timer_tick(SD_Context *ctx)
{
  if (ctx->timer1 > 0) ctx->timer1--;
  if (ctx->timer2 > 0) ctx->timer2--;
}

/* SPI TX complete callback */
void SD_spi_tx_complete(SD_Context *ctx)
{
  ctx->dma_complete = 1;
}

/* SPI RX complete callback */
void SD_spi_rx_complete(SD_Context *ctx)
{
  ctx->dma_complete = 1;
}

/* SPI TX/RX complete callback */
void SD_spi_txrx_complete(SD_Context *ctx)
{
  ctx->dma_complete = 1;
}

/***************************************
 * SPI functions
 **************************************/

/* Slave select */
static void SELECT(SD_Context *ctx)
{
  HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_RESET);
  HAL_Delay(1);
}

/* Slave deselect */
static void DESELECT(SD_Context *ctx)
{
  HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_SET);
  HAL_Delay(1);
}

/* SPI transmit a byte */
static void SPI_TxByte(SD_Context *ctx, uint8_t data)
{
  if (ctx->dma_complete == 1) {
    ctx->dma_complete = 0;
    HAL_SPI_Transmit_DMA(ctx->spi_handle, &data, 1);
  }
}

/* SPI transmit buffer */
static void SPI_TxBuffer(SD_Context *ctx, uint8_t *buffer, uint16_t len)
{
  if (ctx->dma_complete == 1) {
    ctx->dma_complete = 0;
    HAL_SPI_Transmit_DMA(ctx->spi_handle, buffer, len);
  }
}

/* SPI receive a byte */
static uint8_t SPI_RxByte(SD_Context *ctx)
{
  uint8_t dummy = 0xFF, data = 0xFF;

  if (ctx->dma_complete == 1) {
    ctx->dma_complete = 0;
    HAL_SPI_TransmitReceive_DMA(ctx->spi_handle, &dummy, &data, 1);
  }
  
  return data;
}

/* SPI receive a byte via pointer */
static void SPI_RxBytePtr(SD_Context *ctx, uint8_t *buff)
{
  *buff = SPI_RxByte(ctx);
}

/***************************************
 * SD functions
 **************************************/

/* Wait SD ready */
static uint8_t SD_ReadyWait(SD_Context *ctx)
{
  uint8_t res;

  /* timeout 500ms */
  ctx->timer2 = 500;

  /* if SD goes ready, receives 0xFF */
  do {
    res = SPI_RxByte(ctx);
  } while ((res != 0xFF) && ctx->timer2);

  return res;
}

/* Power on */
static void SD_PowerOn(SD_Context *ctx)
{
  uint8_t args[6];
  uint32_t cnt = 0x1FFF;

  /* transmit bytes to wake up */
  DESELECT(ctx);
  for (int i = 0; i < 10; i++) {
    SPI_TxByte(ctx, 0xFF);
  }

  /* slave select */
  SELECT(ctx);

  /* make idle state */
  args[0] = CMD0;   /* CMD0:GO_IDLE_STATE */
  args[1] = 0;
  args[2] = 0;
  args[3] = 0;
  args[4] = 0;
  args[5] = 0x95;   /* CRC */

  SPI_TxBuffer(ctx, args, sizeof(args));

  /* wait response */
  while ((SPI_RxByte(ctx) != 0x01) && cnt) {
    cnt--;
  }

  DESELECT(ctx);
  SPI_TxByte(ctx, 0xFF);

  ctx->power_flag = 1;
}

/* Power off */
static void SD_PowerOff(SD_Context *ctx)
{
  ctx->power_flag = 0;
}

/* Check power flag */
static uint8_t SD_CheckPower(SD_Context *ctx)
{
  return ctx->power_flag;
}

/* Receive data block */
static bool SD_RxDataBlock(SD_Context *ctx, BYTE *buff, UINT len)
{
  uint8_t token;

  /* timeout 200ms */
  ctx->timer1 = 200;

  /* loop until receive a response or timeout */
  do {
    token = SPI_RxByte(ctx);
  } while ((token == 0xFF) && ctx->timer1);

  /* invalid response */
  if (token != 0xFE) return FALSE;

  /* receive data */
  do {
    SPI_RxBytePtr(ctx, buff++);
  } while (len--);

  /* discard CRC */
  SPI_RxByte(ctx);
  SPI_RxByte(ctx);

  return TRUE;
}

/* Transmit data block */
static bool SD_TxDataBlock(SD_Context *ctx, const uint8_t *buff, BYTE token)
{
  uint8_t resp = 0;
  uint8_t i = 0;

  /* wait SD ready */
  if (SD_ReadyWait(ctx) != 0xFF) return FALSE;

  /* transmit token */
  SPI_TxByte(ctx, token);

  /* if it's not STOP token, transmit data */
  if (token != 0xFD) {
    SPI_TxBuffer(ctx, (uint8_t *)buff, 512);

    /* discard CRC */
    SPI_RxByte(ctx);
    SPI_RxByte(ctx);

    /* receive response */
    while (i <= 64) {
      resp = SPI_RxByte(ctx);
      /* transmit 0x05 accepted */
      if ((resp & 0x1F) == 0x05) break;
      i++;
    }

    /* recv buffer clear */
    while (SPI_RxByte(ctx) == 0);
  }

  /* transmit 0x05 accepted */
  if ((resp & 0x1F) == 0x05) return TRUE;

  return FALSE;
}

/* Transmit command */
static BYTE SD_SendCmd(SD_Context *ctx, BYTE cmd, uint32_t arg)
{
  uint8_t crc, res;

  /* wait SD ready */
  if (SD_ReadyWait(ctx) != 0xFF) return 0xFF;

  /* transmit command */
  SPI_TxByte(ctx, cmd);                       /* Command */
  SPI_TxByte(ctx, (uint8_t)(arg >> 24));      /* Argument[31..24] */
  SPI_TxByte(ctx, (uint8_t)(arg >> 16));      /* Argument[23..16] */
  SPI_TxByte(ctx, (uint8_t)(arg >> 8));       /* Argument[15..8] */
  SPI_TxByte(ctx, (uint8_t)arg);              /* Argument[7..0] */

  /* prepare CRC */
  if (cmd == CMD0) crc = 0x95;        /* CRC for CMD0(0) */
  else if (cmd == CMD8) crc = 0x87;   /* CRC for CMD8(0x1AA) */
  else crc = 1;

  /* transmit CRC */
  SPI_TxByte(ctx, crc);

  /* Skip a stuff byte when STOP_TRANSMISSION */
  if (cmd == CMD12) SPI_RxByte(ctx);

  /* receive response */
  uint8_t n = 10;
  do {
    res = SPI_RxByte(ctx);
  } while ((res & 0x80) && --n);

  return res;
}

/***************************************
 * Public Disk Functions
 **************************************/

/* Initialize SD card */
DSTATUS SD_disk_initialize(SD_Context *ctx)
{
  uint8_t n, type, ocr[4];

  /* no disk */
  if (ctx->status & STA_NODISK) return ctx->status;

  /* power on */
  SD_PowerOn(ctx);

  /* slave select */
  SELECT(ctx);

  /* check disk type */
  type = 0;

  /* send GO_IDLE_STATE command */
  if (SD_SendCmd(ctx, CMD0, 0) == 1) {
    /* timeout 1 sec */
    ctx->timer1 = 1000;

    /* SDC V2+ accept CMD8 command, http://elm-chan.org/docs/mmc/mmc_e.html */
    if (SD_SendCmd(ctx, CMD8, 0x1AA) == 1) {
      /* operation condition register */
      for (n = 0; n < 4; n++) {
        ocr[n] = SPI_RxByte(ctx);
      }

      /* voltage range 2.7-3.6V */
      if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
        /* ACMD41 with HCS bit */
        do {
          if (SD_SendCmd(ctx, CMD55, 0) <= 1 && SD_SendCmd(ctx, CMD41, 1UL << 30) == 0) break;
        } while (ctx->timer1);

        /* READ_OCR */
        if (ctx->timer1 && SD_SendCmd(ctx, CMD58, 0) == 0) {
          /* Check CCS bit */
          for (n = 0; n < 4; n++) {
            ocr[n] = SPI_RxByte(ctx);
          }
          /* SDv2 (HC or SC) */
          type = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
        }
      }
    } else {
      /* SDC V1 or MMC */
      type = (SD_SendCmd(ctx, CMD55, 0) <= 1 && SD_SendCmd(ctx, CMD41, 0) <= 1) ? CT_SD1 : CT_MMC;

      do {
        if (type == CT_SD1) {
          if (SD_SendCmd(ctx, CMD55, 0) <= 1 && SD_SendCmd(ctx, CMD41, 0) == 0) break; /* ACMD41 */
        } else {
          if (SD_SendCmd(ctx, CMD1, 0) == 0) break; /* CMD1 */
        }
      } while (ctx->timer1);

      /* SET_BLOCKLEN */
      if (!ctx->timer1 || SD_SendCmd(ctx, CMD16, 512) != 0) type = 0;
    }
  }

  ctx->card_type = type;

  /* Idle */
  DESELECT(ctx);
  SPI_RxByte(ctx);

  /* Clear STA_NOINIT */
  if (type) {
    ctx->status &= ~STA_NOINIT;
  } else {
    /* Initialization failed */
    SD_PowerOff(ctx);
  }

  return ctx->status;
}

/* Return disk status */
DSTATUS SD_disk_status(SD_Context *ctx)
{
  return ctx->status;
}

/* Read sector */
DRESULT SD_disk_read(SD_Context *ctx, BYTE* buff, DWORD sector, UINT count)
{
  if (!count) return RES_PARERR;

  /* no disk */
  if (ctx->status & STA_NOINIT) return RES_NOTRDY;

  /* convert to byte address */
  if (!(ctx->card_type & CT_SD2)) sector *= 512;

  SELECT(ctx);

  if (count == 1) {
    /* READ_SINGLE_BLOCK */
    if ((SD_SendCmd(ctx, CMD17, sector) == 0) && SD_RxDataBlock(ctx, buff, 512)) count = 0;
  } else {
    /* READ_MULTIPLE_BLOCK */
    if (SD_SendCmd(ctx, CMD18, sector) == 0) {
      do {
        if (!SD_RxDataBlock(ctx, buff, 512)) break;
        buff += 512;
      } while (--count);

      /* STOP_TRANSMISSION */
      SD_SendCmd(ctx, CMD12, 0);
    }
  }

  /* Idle */
  DESELECT(ctx);
  SPI_RxByte(ctx);

  return count ? RES_ERROR : RES_OK;
}

/* Write sector */
DRESULT SD_disk_write(SD_Context *ctx, const BYTE* buff, DWORD sector, UINT count)
{
  if (!count) return RES_PARERR;

  /* no disk */
  if (ctx->status & STA_NOINIT) return RES_NOTRDY;

  /* write protection */
  if (ctx->status & STA_PROTECT) return RES_WRPRT;

  /* convert to byte address */
  if (!(ctx->card_type & CT_SD2)) sector *= 512;

  SELECT(ctx);

  if (count == 1) {
    /* WRITE_BLOCK */
    if ((SD_SendCmd(ctx, CMD24, sector) == 0) && SD_TxDataBlock(ctx, buff, 0xFE))
      count = 0;
  } else {
    /* WRITE_MULTIPLE_BLOCK */
    if (ctx->card_type & CT_SD1) {
      SD_SendCmd(ctx, CMD55, 0);
      SD_SendCmd(ctx, CMD23, count); /* ACMD23 */
    }

    if (SD_SendCmd(ctx, CMD25, sector) == 0) {
      do {
        if (!SD_TxDataBlock(ctx, buff, 0xFC)) break;
        buff += 512;
      } while (--count);

      /* STOP_TRAN token */
      if (!SD_TxDataBlock(ctx, 0, 0xFD)) {
        count = 1;
      }
    }
  }

  /* Idle */
  DESELECT(ctx);
  SPI_RxByte(ctx);

  return count ? RES_ERROR : RES_OK;
}

/* ioctl */
DRESULT SD_disk_ioctl(SD_Context *ctx, BYTE ctrl, void *buff)
{
  DRESULT res;
  uint8_t n, csd[16], *ptr = buff;
  WORD csize;

  res = RES_ERROR;

  if (ctrl == CTRL_POWER) {
    switch (*ptr) {
    case 0:
      SD_PowerOff(ctx);       /* Power Off */
      res = RES_OK;
      break;
    case 1:
      SD_PowerOn(ctx);        /* Power On */
      res = RES_OK;
      break;
    case 2:
      *(ptr + 1) = SD_CheckPower(ctx);
      res = RES_OK;           /* Power Check */
      break;
    default:
      res = RES_PARERR;
    }
  } else {
    /* no disk */
    if (ctx->status & STA_NOINIT) return RES_NOTRDY;

    SELECT(ctx);

    switch (ctrl) {
    case GET_SECTOR_COUNT:
      /* SEND_CSD */
      if ((SD_SendCmd(ctx, CMD9, 0) == 0) && SD_RxDataBlock(ctx, csd, 16)) {
        if ((csd[0] >> 6) == 1) {
          /* SDC V2 */
          csize = csd[9] + ((WORD)csd[8] << 8) + 1;
          *(DWORD *)buff = (DWORD)csize << 10;
        } else {
          /* MMC or SDC V1 */
          n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
          csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
          *(DWORD *)buff = (DWORD)csize << (n - 9);
        }
        res = RES_OK;
      }
      break;
    case GET_SECTOR_SIZE:
      *(WORD *)buff = 512;
      res = RES_OK;
      break;
    case CTRL_SYNC:
      if (SD_ReadyWait(ctx) == 0xFF) res = RES_OK;
      break;
    case MMC_GET_CSD:
      /* SEND_CSD */
      if (SD_SendCmd(ctx, CMD9, 0) == 0 && SD_RxDataBlock(ctx, ptr, 16)) res = RES_OK;
      break;
    case MMC_GET_CID:
      /* SEND_CID */
      if (SD_SendCmd(ctx, CMD10, 0) == 0 && SD_RxDataBlock(ctx, ptr, 16)) res = RES_OK;
      break;
    case MMC_GET_OCR:
      /* READ_OCR */
      if (SD_SendCmd(ctx, CMD58, 0) == 0) {
        for (n = 0; n < 4; n++) {
          *ptr++ = SPI_RxByte(ctx);
        }
        res = RES_OK;
      }
      break;
    default:
      res = RES_PARERR;
    }

    DESELECT(ctx);
    SPI_RxByte(ctx);
  }

  return res;
}

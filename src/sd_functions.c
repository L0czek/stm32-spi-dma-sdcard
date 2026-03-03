#include "sd_functions.h"

#ifndef SD_SEMIHOST_LOG
#define SD_SEMIHOST_LOG 1
#endif

static const uint8_t sd_dma_dummy_tx[512] = {
  [0 ... 511] = 0xFF
};

#if SD_SEMIHOST_LOG
static void SD_LogText(const char *text)
{
  __asm__ volatile (
      "mov r0, #0x04\n"
      "mov r1, %0\n"
      "bkpt 0xab\n"
      :
      : "r"(text)
      : "r0", "r1", "memory");
}

static void SD_LogHex8(uint8_t value)
{
  static const char digits[] = "0123456789ABCDEF";
  char msg[] = "0x00\n";

  msg[2] = digits[(value >> 4) & 0x0F];
  msg[3] = digits[value & 0x0F];
  SD_LogText(msg);
}

static void SD_LogError(const char *text)
{
  SD_LogText(text);
}

static void SD_LogErrorHex(const char *prefix, uint8_t value)
{
  SD_LogText(prefix);
  SD_LogHex8(value);
}
#else
static void SD_LogError(const char *text)
{
  (void)text;
}

static void SD_LogErrorHex(const char *prefix, uint8_t value)
{
  (void)prefix;
  (void)value;
}
#endif

static void SELECT(SD_Context *ctx)
{
  HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_RESET);
}

static void DESELECT(SD_Context *ctx)
{
  HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef SPI_WaitDMA(SD_Context *ctx, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (ctx->dma_complete == 0U) {
    if (timeout_ms != HAL_MAX_DELAY) {
      if ((HAL_GetTick() - start) >= timeout_ms) {
        (void)HAL_SPI_Abort(ctx->spi_handle);
        ctx->dma_complete = 1U;
        SD_LogError("sd: dma wait timeout\n");
        return HAL_TIMEOUT;
      }
    }
  }

  return HAL_OK;
}

static HAL_StatusTypeDef SPI_TxByte(SD_Context *ctx, uint8_t data)
{
  return HAL_SPI_Transmit(ctx->spi_handle, &data, 1, SPI_TIMEOUT);
}

static HAL_StatusTypeDef SPI_RxByte(SD_Context *ctx, uint8_t *out)
{
  uint8_t dummy = 0xFF;

  return HAL_SPI_TransmitReceive(ctx->spi_handle, &dummy, out, 1, SPI_TIMEOUT);
}

static HAL_StatusTypeDef SPI_TxBuffer_DMA(SD_Context *ctx, uint8_t *buffer,
                                          uint16_t len, uint32_t timeout_ms)
{
  HAL_StatusTypeDef status;

  ctx->dma_complete = 0U;
  status = HAL_SPI_Transmit_DMA(ctx->spi_handle, buffer, len);
  if (status != HAL_OK) {
    ctx->dma_complete = 1U;
    SD_LogError("sd: tx dma start failed\n");
    return status;
  }

  return SPI_WaitDMA(ctx, timeout_ms);
}

static HAL_StatusTypeDef SPI_RxBuffer_DMA(SD_Context *ctx, uint8_t *buffer,
                                          uint16_t len, uint32_t timeout_ms)
{
  HAL_StatusTypeDef status;

  if (len > sizeof(sd_dma_dummy_tx)) {
    SD_LogError("sd: rx dma oversize\n");
    return HAL_ERROR;
  }

  ctx->dma_complete = 0U;
  status = HAL_SPI_TransmitReceive_DMA(ctx->spi_handle, sd_dma_dummy_tx,
                                       buffer, len);
  if (status != HAL_OK) {
    ctx->dma_complete = 1U;
    SD_LogError("sd: rx dma start failed\n");
    return status;
  }

  return SPI_WaitDMA(ctx, timeout_ms);
}

static HAL_StatusTypeDef SPI_RxBuffer(SD_Context *ctx, uint8_t *buffer, UINT len)
{
  HAL_StatusTypeDef status;

  if (len == 512U) {
    return SPI_RxBuffer_DMA(ctx, buffer, (uint16_t)len, SD_DMA_TIMEOUT_MS);
  }

  for (UINT i = 0; i < len; ++i) {
    status = SPI_RxByte(ctx, &buffer[i]);
    if (status != HAL_OK) {
      return status;
    }
  }

  return HAL_OK;
}

static HAL_StatusTypeDef SD_SetSpiPrescaler(SD_Context *ctx, uint32_t prescaler)
{
  if (ctx->spi_handle->Init.BaudRatePrescaler == prescaler) {
    return HAL_OK;
  }

  ctx->spi_handle->Init.BaudRatePrescaler = prescaler;
  return HAL_SPI_Init(ctx->spi_handle);
}

static HAL_StatusTypeDef SD_ReadyWait(SD_Context *ctx)
{
  uint8_t res = 0x00;
  HAL_StatusTypeDef status;

  ctx->timer2 = 500;
  do {
    status = SPI_RxByte(ctx, &res);
    if (status != HAL_OK) {
      return status;
    }
  } while ((res != 0xFFU) && (ctx->timer2 > 0U));

  return (res == 0xFFU) ? HAL_OK : HAL_TIMEOUT;
}

static HAL_StatusTypeDef SD_PowerOn(SD_Context *ctx)
{
  uint8_t resp = 0xFF;
  HAL_StatusTypeDef status;

  DESELECT(ctx);
  for (uint8_t i = 0; i < 10U; ++i) {
    status = SPI_TxByte(ctx, 0xFF);
    if (status != HAL_OK) {
      return status;
    }
  }

  SELECT(ctx);
  status = SPI_TxByte(ctx, CMD0);
  if (status != HAL_OK) {
    return status;
  }
  for (uint8_t i = 0; i < 4U; ++i) {
    status = SPI_TxByte(ctx, 0x00);
    if (status != HAL_OK) {
      return status;
    }
  }
  status = SPI_TxByte(ctx, 0x95);
  if (status != HAL_OK) {
    return status;
  }

  for (uint32_t tries = 0; tries < 0x1FFFU; ++tries) {
    status = SPI_RxByte(ctx, &resp);
    if (status != HAL_OK) {
      return status;
    }
    if (resp == 0x01U) {
      DESELECT(ctx);
      status = SPI_TxByte(ctx, 0xFF);
      if (status != HAL_OK) {
        return status;
      }
      ctx->power_flag = 1U;
      return HAL_OK;
    }
  }

  DESELECT(ctx);
  (void)SPI_TxByte(ctx, 0xFF);
  SD_LogError("sd: cmd0 idle timeout\n");
  return HAL_TIMEOUT;
}

static void SD_PowerOff(SD_Context *ctx)
{
  ctx->power_flag = 0U;
}

static uint8_t SD_CheckPower(SD_Context *ctx)
{
  return ctx->power_flag;
}

static HAL_StatusTypeDef SD_RxDataBlock(SD_Context *ctx, BYTE *buff, UINT len)
{
  uint8_t token = 0xFF;
  HAL_StatusTypeDef status;

  ctx->timer1 = 200;
  do {
    status = SPI_RxByte(ctx, &token);
    if (status != HAL_OK) {
      return status;
    }
  } while ((token == 0xFFU) && (ctx->timer1 > 0U));

  if (token != 0xFEU) {
    SD_LogErrorHex("sd: bad read token ", token);
    return HAL_ERROR;
  }

  status = SPI_RxBuffer(ctx, buff, len);
  if (status != HAL_OK) {
    return status;
  }

  status = SPI_RxByte(ctx, &token);
  if (status != HAL_OK) {
    return status;
  }

  return SPI_RxByte(ctx, &token);
}

static HAL_StatusTypeDef SD_TxDataBlock(SD_Context *ctx, const BYTE *buff,
                                        BYTE token)
{
  uint8_t resp = 0xFF;
  HAL_StatusTypeDef status;

  status = SD_ReadyWait(ctx);
  if (status != HAL_OK) {
    return status;
  }

  status = SPI_TxByte(ctx, token);
  if (status != HAL_OK) {
    return status;
  }

  if (token == 0xFDU) {
    return SD_ReadyWait(ctx);
  }

  status = SPI_TxBuffer_DMA(ctx, (uint8_t *)buff, 512U, SD_DMA_TIMEOUT_MS);
  if (status != HAL_OK) {
    return status;
  }

  status = SPI_TxByte(ctx, 0xFF);
  if (status != HAL_OK) {
    return status;
  }
  status = SPI_TxByte(ctx, 0xFF);
  if (status != HAL_OK) {
    return status;
  }

  for (uint8_t tries = 0; tries < 64U; ++tries) {
    status = SPI_RxByte(ctx, &resp);
    if (status != HAL_OK) {
      return status;
    }
    if ((resp & 0x1FU) == 0x05U) {
      return SD_ReadyWait(ctx);
    }
  }

  SD_LogErrorHex("sd: write reject ", resp);
  return HAL_ERROR;
}

static HAL_StatusTypeDef SD_SendCmd(SD_Context *ctx, BYTE cmd, uint32_t arg,
                                    BYTE *response)
{
  uint8_t crc = 0x01;
  HAL_StatusTypeDef status;

  status = SD_ReadyWait(ctx);
  if (status != HAL_OK) {
    return status;
  }

  if (cmd == CMD0) {
    crc = 0x95;
  } else if (cmd == CMD8) {
    crc = 0x87;
  }

  status = SPI_TxByte(ctx, cmd);
  if (status != HAL_OK) {
    return status;
  }
  status = SPI_TxByte(ctx, (uint8_t)(arg >> 24));
  if (status != HAL_OK) {
    return status;
  }
  status = SPI_TxByte(ctx, (uint8_t)(arg >> 16));
  if (status != HAL_OK) {
    return status;
  }
  status = SPI_TxByte(ctx, (uint8_t)(arg >> 8));
  if (status != HAL_OK) {
    return status;
  }
  status = SPI_TxByte(ctx, (uint8_t)arg);
  if (status != HAL_OK) {
    return status;
  }
  status = SPI_TxByte(ctx, crc);
  if (status != HAL_OK) {
    return status;
  }

  if (cmd == CMD12) {
    status = SPI_RxByte(ctx, response);
    if (status != HAL_OK) {
      return status;
    }
  }

  for (uint8_t tries = 0; tries < 10U; ++tries) {
    status = SPI_RxByte(ctx, response);
    if (status != HAL_OK) {
      return status;
    }
    if ((*response & 0x80U) == 0U) {
      return HAL_OK;
    }
  }

  SD_LogErrorHex("sd: cmd timeout ", cmd);
  return HAL_TIMEOUT;
}

static DRESULT SD_ResultFromHal(HAL_StatusTypeDef status)
{
  return (status == HAL_OK) ? RES_OK : RES_ERROR;
}

static uint8_t SD_InitCardV2(SD_Context *ctx, BYTE *response)
{
  uint8_t ocr[4];
  HAL_StatusTypeDef status;

  status = SD_SendCmd(ctx, CMD8, 0x1AAU, response);
  if ((status != HAL_OK) || (*response != 1U)) {
    return 0U;
  }

  for (uint8_t i = 0; i < 4U; ++i) {
    status = SPI_RxByte(ctx, &ocr[i]);
    if (status != HAL_OK) {
      return 0U;
    }
  }
  if ((ocr[2] != 0x01U) || (ocr[3] != 0xAAU)) {
    return 0U;
  }

  do {
    status = SD_SendCmd(ctx, CMD55, 0U, response);
    if ((status != HAL_OK) || (*response > 1U)) {
      return 0U;
    }
    status = SD_SendCmd(ctx, CMD41, 1UL << 30, response);
    if (status != HAL_OK) {
      return 0U;
    }
  } while ((*response != 0U) && (ctx->timer1 > 0U));

  if ((*response != 0U) || (ctx->timer1 == 0U)) {
    return 0U;
  }

  status = SD_SendCmd(ctx, CMD58, 0U, response);
  if ((status != HAL_OK) || (*response != 0U)) {
    return 0U;
  }
  for (uint8_t i = 0; i < 4U; ++i) {
    status = SPI_RxByte(ctx, &ocr[i]);
    if (status != HAL_OK) {
      return 0U;
    }
  }

  return ((ocr[0] & 0x40U) != 0U) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
}

static uint8_t SD_InitCardLegacy(SD_Context *ctx, BYTE *response)
{
  uint8_t type = CT_MMC;
  HAL_StatusTypeDef status;

  status = SD_SendCmd(ctx, CMD55, 0U, response);
  if ((status == HAL_OK) && (*response <= 1U)) {
    status = SD_SendCmd(ctx, CMD41, 0U, response);
    if ((status == HAL_OK) && (*response <= 1U)) {
      type = CT_SD1;
    }
  }

  do {
    status = SD_SendCmd(ctx, (type == CT_SD1) ? CMD55 : CMD1, 0U, response);
    if (status != HAL_OK) {
      return 0U;
    }
    if (type == CT_SD1) {
      if (*response > 1U) {
        return 0U;
      }
      status = SD_SendCmd(ctx, CMD41, 0U, response);
      if (status != HAL_OK) {
        return 0U;
      }
    }
  } while ((*response != 0U) && (ctx->timer1 > 0U));

  if ((*response != 0U) || (ctx->timer1 == 0U)) {
    return 0U;
  }

  status = SD_SendCmd(ctx, CMD16, 512U, response);
  if ((status != HAL_OK) || (*response != 0U)) {
    return 0U;
  }

  return type;
}

static DRESULT SD_ReadRegister(SD_Context *ctx, BYTE cmd, BYTE *buffer, UINT len)
{
  BYTE response = 0xFF;
  HAL_StatusTypeDef status;

  status = SD_SendCmd(ctx, cmd, 0U, &response);
  if ((status != HAL_OK) || (response != 0U)) {
    return RES_ERROR;
  }

  if (cmd == CMD58) {
    for (UINT i = 0; i < len; ++i) {
      status = SPI_RxByte(ctx, &buffer[i]);
      if (status != HAL_OK) {
        return RES_ERROR;
      }
    }
    return RES_OK;
  }

  return SD_ResultFromHal(SD_RxDataBlock(ctx, buffer, len));
}

static void SD_StoreSectorCount(const uint8_t *csd, void *buff)
{
  WORD csize;
  uint8_t n;

  if ((csd[0] >> 6) == 1U) {
    csize = (WORD)csd[9] + ((WORD)csd[8] << 8) + 1U;
    *(DWORD *)buff = (DWORD)csize << 10;
    return;
  }

  n = (uint8_t)((csd[5] & 15U) + ((csd[10] & 128U) >> 7)
                + ((csd[9] & 3U) << 1) + 2U);
  csize = (WORD)(csd[8] >> 6) + ((WORD)csd[7] << 2)
          + ((WORD)(csd[6] & 3U) << 10) + 1U;
  *(DWORD *)buff = (DWORD)csize << (n - 9U);
}

void SD_init(SD_Context *ctx, SPI_HandleTypeDef *spi_handle,
             GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
  ctx->spi_handle = spi_handle;
  ctx->cs_port = cs_port;
  ctx->cs_pin = cs_pin;
  ctx->status = STA_NOINIT;
  ctx->card_type = 0U;
  ctx->power_flag = 0U;
  ctx->dma_complete = 1U;
  ctx->timer1 = 0U;
  ctx->timer2 = 0U;
}

void SD_timer_tick(SD_Context *ctx)
{
  if (ctx->timer1 > 0U) {
    ctx->timer1--;
  }
  if (ctx->timer2 > 0U) {
    ctx->timer2--;
  }
}

void SD_spi_tx_complete(SD_Context *ctx)
{
  ctx->dma_complete = 1U;
}

void SD_spi_rx_complete(SD_Context *ctx)
{
  ctx->dma_complete = 1U;
}

void SD_spi_txrx_complete(SD_Context *ctx)
{
  ctx->dma_complete = 1U;
}

DSTATUS SD_disk_initialize(SD_Context *ctx)
{
  BYTE response = 0xFF;
  uint8_t type = 0U;
  uint8_t idle_byte = 0xFF;
  HAL_StatusTypeDef hal_status;

  if ((ctx->status & STA_NODISK) != 0U) {
    return ctx->status;
  }

  hal_status = SD_SetSpiPrescaler(ctx, SD_SPI_INIT_PRESCALER);
  if (hal_status != HAL_OK) {
    return ctx->status;
  }

  hal_status = SD_PowerOn(ctx);
  if (hal_status != HAL_OK) {
    SD_PowerOff(ctx);
    return ctx->status;
  }

  SELECT(ctx);
  hal_status = SD_SendCmd(ctx, CMD0, 0U, &response);
  if ((hal_status != HAL_OK) || (response != 1U)) {
    goto init_done;
  }

  ctx->timer1 = 1000;
  type = SD_InitCardV2(ctx, &response);
  if (type == 0U) {
    type = SD_InitCardLegacy(ctx, &response);
  }

init_done:
  ctx->card_type = type;
  DESELECT(ctx);
  (void)SPI_RxByte(ctx, &idle_byte);

  if (type != 0U) {
    ctx->status &= (DSTATUS)~STA_NOINIT;
    (void)SD_SetSpiPrescaler(ctx, SD_SPI_TRANSFER_PRESCALER);
  } else {
    SD_PowerOff(ctx);
    SD_LogError("sd: init failed\n");
  }

  return ctx->status;
}

DSTATUS SD_disk_status(SD_Context *ctx)
{
  return ctx->status;
}

DRESULT SD_disk_read(SD_Context *ctx, BYTE *buff, DWORD sector, UINT count)
{
  BYTE response = 0xFF;
  uint8_t idle_byte = 0xFF;
  uint8_t success = 0U;
  HAL_StatusTypeDef hal_status = HAL_OK;

  if (count == 0U) {
    return RES_PARERR;
  }
  if ((ctx->status & STA_NOINIT) != 0U) {
    return RES_NOTRDY;
  }
  if ((ctx->card_type & CT_BLOCK) == 0U) {
    sector *= 512U;
  }

  SELECT(ctx);
  if (count == 1U) {
    hal_status = SD_SendCmd(ctx, CMD17, sector, &response);
    if ((hal_status == HAL_OK) && (response == 0U)) {
      hal_status = SD_RxDataBlock(ctx, buff, 512U);
      success = (hal_status == HAL_OK) ? 1U : 0U;
    }
  } else {
    hal_status = SD_SendCmd(ctx, CMD18, sector, &response);
    if ((hal_status == HAL_OK) && (response == 0U)) {
      UINT remaining = count;
      while (count > 0U) {
        hal_status = SD_RxDataBlock(ctx, buff, 512U);
        if (hal_status != HAL_OK) {
          break;
        }
        buff += 512;
        count--;
      }
      (void)SD_SendCmd(ctx, CMD12, 0U, &response);
      success = (hal_status == HAL_OK) && (count == 0U) && (remaining > 0U);
    }
  }

  DESELECT(ctx);
  (void)SPI_RxByte(ctx, &idle_byte);

  return (success != 0U) ? RES_OK : RES_ERROR;
}

DRESULT SD_disk_write(SD_Context *ctx, const BYTE *buff, DWORD sector, UINT count)
{
  BYTE response = 0xFF;
  uint8_t idle_byte = 0xFF;
  uint8_t success = 0U;
  HAL_StatusTypeDef hal_status = HAL_OK;

  if (count == 0U) {
    return RES_PARERR;
  }
  if ((ctx->status & STA_NOINIT) != 0U) {
    return RES_NOTRDY;
  }
  if ((ctx->status & STA_PROTECT) != 0U) {
    return RES_WRPRT;
  }
  if ((ctx->card_type & CT_BLOCK) == 0U) {
    sector *= 512U;
  }

  SELECT(ctx);
  if (count == 1U) {
    hal_status = SD_SendCmd(ctx, CMD24, sector, &response);
    if ((hal_status == HAL_OK) && (response == 0U)) {
      hal_status = SD_TxDataBlock(ctx, buff, 0xFEU);
      success = (hal_status == HAL_OK) ? 1U : 0U;
    }
  } else {
    if ((ctx->card_type & CT_SDC) != 0U) {
      hal_status = SD_SendCmd(ctx, CMD55, 0U, &response);
      if ((hal_status == HAL_OK) && (response <= 1U)) {
        hal_status = SD_SendCmd(ctx, CMD23, count, &response);
      }
      if (hal_status != HAL_OK) {
        goto write_done;
      }
    }

    hal_status = SD_SendCmd(ctx, CMD25, sector, &response);
    if ((hal_status == HAL_OK) && (response == 0U)) {
      UINT remaining = count;
      while (count > 0U) {
        hal_status = SD_TxDataBlock(ctx, buff, 0xFCU);
        if (hal_status != HAL_OK) {
          break;
        }
        buff += 512;
        count--;
      }
      if (hal_status == HAL_OK) {
        hal_status = SD_TxDataBlock(ctx, NULL, 0xFDU);
      }
      success = (hal_status == HAL_OK) && (count == 0U) && (remaining > 0U);
    }
  }

write_done:
  DESELECT(ctx);
  (void)SPI_RxByte(ctx, &idle_byte);

  return (success != 0U) ? RES_OK : RES_ERROR;
}

DRESULT SD_disk_ioctl(SD_Context *ctx, BYTE ctrl, void *buff)
{
  BYTE *ptr = (BYTE *)buff;
  uint8_t csd[16];
  uint8_t idle_byte = 0xFF;
  DRESULT result = RES_ERROR;

  if (ctrl == CTRL_POWER) {
    if (ptr == NULL) {
      return RES_PARERR;
    }
    switch (*ptr) {
    case 0:
      SD_PowerOff(ctx);
      return RES_OK;
    case 1:
      return SD_ResultFromHal(SD_PowerOn(ctx));
    case 2:
      *(ptr + 1) = SD_CheckPower(ctx);
      return RES_OK;
    default:
      return RES_PARERR;
    }
  }

  if (((ctx->status & STA_NOINIT) != 0U)) {
    return RES_NOTRDY;
  }
  if ((buff == NULL) && (ctrl != CTRL_SYNC)) {
    return RES_PARERR;
  }

  SELECT(ctx);
  switch (ctrl) {
  case GET_SECTOR_COUNT:
    result = SD_ReadRegister(ctx, CMD9, csd, sizeof(csd));
    if (result == RES_OK) {
      SD_StoreSectorCount(csd, buff);
    }
    break;
  case GET_SECTOR_SIZE:
    *(WORD *)buff = 512U;
    result = RES_OK;
    break;
  case CTRL_SYNC:
    result = SD_ResultFromHal(SD_ReadyWait(ctx));
    break;
  case MMC_GET_CSD:
    result = SD_ReadRegister(ctx, CMD9, ptr, 16U);
    break;
  case MMC_GET_CID:
    result = SD_ReadRegister(ctx, CMD10, ptr, 16U);
    break;
  case MMC_GET_OCR:
    result = SD_ReadRegister(ctx, CMD58, ptr, 4U);
    break;
  default:
    result = RES_PARERR;
    break;
  }

  DESELECT(ctx);
  (void)SPI_RxByte(ctx, &idle_byte);
  return result;
}

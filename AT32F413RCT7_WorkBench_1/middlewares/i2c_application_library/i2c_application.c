/**
  **************************************************************************
  * @file     i2c_application.c
  * @brief    the driver library of the i2c peripheral
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#include "i2c_application.h"

/** @addtogroup AT32F413_middlewares_i2c_application_library
  * @{
  */


/**
  * @brief get the dma transfer complete flag through the channel
  */
#define DMA_GET_TC_FLAG(DMA_CHANNEL) \
(((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL1))? DMA1_FDT1_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL2))? DMA1_FDT2_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL3))? DMA1_FDT3_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL4))? DMA1_FDT4_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL5))? DMA1_FDT5_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL6))? DMA1_FDT6_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL7))? DMA1_FDT7_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL1))? DMA2_FDT1_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL2))? DMA2_FDT2_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL3))? DMA2_FDT3_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL4))? DMA2_FDT4_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL5))? DMA2_FDT5_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL6))? DMA2_FDT6_FLAG : \
                                                         DMA2_FDT7_FLAG)

/**
  * @brief get the dma half transfer flag through the channel
  */
#define DMA_GET_HT_FLAG(DMA_CHANNEL) \
(((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL1))? DMA1_HDT1_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL2))? DMA1_HDT2_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL3))? DMA1_HDT3_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL4))? DMA1_HDT4_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL5))? DMA1_HDT5_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL6))? DMA1_HDT6_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL7))? DMA1_HDT7_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL1))? DMA2_HDT1_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL2))? DMA2_HDT2_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL3))? DMA2_HDT3_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL4))? DMA2_HDT4_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL5))? DMA2_HDT5_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL6))? DMA2_HDT6_FLAG : \
                                                         DMA2_HDT7_FLAG)

/**
  * @brief get the dma transfer error flag through the channel
  */
#define DMA_GET_TERR_FLAG(DMA_CHANNEL) \
(((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL1))? DMA1_DTERR1_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL2))? DMA1_DTERR2_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL3))? DMA1_DTERR3_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL4))? DMA1_DTERR4_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL5))? DMA1_DTERR5_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL6))? DMA1_DTERR6_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA1_CHANNEL7))? DMA1_DTERR7_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL1))? DMA2_DTERR1_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL2))? DMA2_DTERR2_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL3))? DMA2_DTERR3_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL4))? DMA2_DTERR4_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL5))? DMA2_DTERR5_FLAG : \
 ((uint32_t)(DMA_CHANNEL) == ((uint32_t)DMA2_CHANNEL6))? DMA2_DTERR6_FLAG : \
                                                         DMA2_DTERR7_FLAG)

/**
  * @brief i2c transmission status
  */
#define I2C_START                        0
#define I2C_END                          1

/**
  * @brief  initializes peripherals used by the i2c.
  * @param  none
  * @retval none
  */
__WEAK void i2c_lowlevel_init(i2c_handle_type* hi2c)
{

}

/**
  * @brief  i2c peripheral initialization.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_config(i2c_handle_type* hi2c)
{
  /* reset i2c peripheral */
  i2c_reset(hi2c->i2cx);

  /* i2c peripheral initialization */
  i2c_lowlevel_init(hi2c);

  /* i2c peripheral enable */
  i2c_enable(hi2c->i2cx, TRUE);
}

/**
  * @brief  wait for the transfer to end.
  * @param  hi2c: the handle points to the operation information.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_wait_end(i2c_handle_type* hi2c, uint32_t timeout)
{
  while(hi2c->status != I2C_END)
  {
    /* check timeout */
    if((timeout--) == 0)
    {
      return I2C_ERR_TIMEOUT;
    }
  }

  if(hi2c->error_code != I2C_OK)
  {
    return hi2c->error_code;
  }
  else
  {
    return I2C_OK;
  }
}

/**
  * @brief  wait for the flag to be set or reset, only BUSYF flag
  *         is waiting to be reset, and other flags are waiting to be set
  * @param  hi2c: the handle points to the operation information.
  * @param  flag: specifies the flag to check.
  *         this parameter can be one of the following values:
  *         - I2C_STARTF_FLAG: start condition generation complete flag.
  *         - I2C_ADDR7F_FLAG: 0~7 bit address match flag.
  *         - I2C_TDC_FLAG: transmit data complete flag.
  *         - I2C_ADDRHF_FLAG: master 9~8 bit address header match flag.
  *         - I2C_STOPF_FLAG: stop condition generation complete flag.
  *         - I2C_RDBF_FLAG: receive data buffer full flag.
  *         - I2C_TDBE_FLAG: transmit data buffer empty flag.
  *         - I2C_BUSERR_FLAG: bus error flag.
  *         - I2C_ARLOST_FLAG: arbitration lost flag.
  *         - I2C_ACKFAIL_FLAG: acknowledge failure flag.
  *         - I2C_OUF_FLAG: overflow or underflow flag.
  *         - I2C_PECERR_FLAG: pec receive error flag.
  *         - I2C_TMOUT_FLAG: smbus timeout flag.
  *         - I2C_ALERTF_FLAG: smbus alert flag.
  *         - I2C_TRMODE_FLAG: transmission mode.
  *         - I2C_BUSYF_FLAG: bus busy flag transmission mode.
  *         - I2C_DIRF_FLAG: transmission direction flag.
  *         - I2C_GCADDRF_FLAG: general call address received flag.
  *         - I2C_DEVADDRF_FLAG: smbus device address received flag.
  *         - I2C_HOSTADDRF_FLAG: smbus host address received flag.
  *         - I2C_ADDR2_FLAG: own address 2 received flag.
  * @param  event_check: check other error flags while waiting for the flag.
  *         parameter as following values:
  *         - I2C_EVENT_CHECK_NONE
  *         - I2C_EVENT_CHECK_ACKFAIL
  *         - I2C_EVENT_CHECK_STOP
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_wait_flag(i2c_handle_type* hi2c, uint32_t flag, uint32_t event_check, uint32_t timeout)
{
  if(flag == I2C_BUSYF_FLAG)
  {
    while(i2c_flag_get(hi2c->i2cx, flag) != RESET)
    {
      /* check timeout */
      if((timeout--) == 0)
      {
        hi2c->error_code = I2C_ERR_TIMEOUT;

        return I2C_ERR_TIMEOUT;
      }
    }
  }
  else
  {
    while(i2c_flag_get(hi2c->i2cx, flag) == RESET)
    {
      /* check the ack fail flag */
      if(event_check & I2C_EVENT_CHECK_ACKFAIL)
      {
        if(i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
        {
          /* generate stop condtion */
          i2c_stop_generate(hi2c->i2cx);

          /* clear ack fail flag */
          i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);

          hi2c->error_code = I2C_ERR_ACKFAIL;

          return I2C_ERR_ACKFAIL;
        }
      }

      /* check the stop flag */
      if(event_check & I2C_EVENT_CHECK_STOP)
      {
        if(i2c_flag_get(hi2c->i2cx, I2C_STOPF_FLAG) != RESET)
        {
          /* clear stop flag */
          i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

          hi2c->error_code = I2C_ERR_STOP;

          return I2C_ERR_STOP;
        }
      }

      /* check timeout */
      if((timeout--) == 0)
      {
        hi2c->error_code = I2C_ERR_TIMEOUT;

        return I2C_ERR_TIMEOUT;
      }
    }
  }

  return I2C_OK;
}

/**
  * @brief  dma transfer cofiguration.
  * @param  hi2c: the handle points to the operation information.
  * @param  dma_channelx: dma channel to be cofigured.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @retval none.
  */
void i2c_dma_config(i2c_handle_type* hi2c, dma_channel_type* dma_channelx, uint8_t* pdata, uint16_t size)
{
  /* disable the dma channel */
  dma_channel_enable(dma_channelx, FALSE);

  /* disable the transfer complete interrupt */
  dma_interrupt_enable(dma_channelx, DMA_FDT_INT , FALSE);

  /* configure buffer address and buffer size */
  hi2c->dma_init_struct.memory_base_addr     = (uint32_t)pdata;
  hi2c->dma_init_struct.direction            = (dma_channelx == hi2c->dma_tx_channel) ? DMA_DIR_MEMORY_TO_PERIPHERAL : DMA_DIR_PERIPHERAL_TO_MEMORY;
  hi2c->dma_init_struct.peripheral_base_addr = (uint32_t)&hi2c->i2cx->dt;
  hi2c->dma_init_struct.buffer_size          = (uint32_t)size;
  dma_init(dma_channelx, &hi2c->dma_init_struct);

  /* enable the transfer complete interrupt */
  dma_interrupt_enable(dma_channelx, DMA_FDT_INT , TRUE);

  /* enable the dma channel */
  dma_channel_enable(dma_channelx, TRUE);
}

/**
  * @brief  send address in master transmits mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_write_addr(i2c_handle_type *hi2c, uint16_t address, uint32_t timeout)
{
  /* generate start condtion */
  i2c_start_generate(hi2c->i2cx);

  /* wait for the start flag to be set */
  if(i2c_wait_flag(hi2c, I2C_STARTF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_START;

    return I2C_ERR_START;
  }

  if(hi2c->i2cx->oaddr1_bit.addr1mode == I2C_ADDRESS_MODE_7BIT)
  {
    /* send slave address */
    i2c_7bit_address_send(hi2c->i2cx, address, I2C_DIRECTION_TRANSMIT);
  }
  else
  {
    /* send slave 10-bit address header */
    i2c_data_send(hi2c->i2cx, (uint8_t)((address & 0x0300) >> 7) | 0xF0);

    /* wait for the addrh flag to be set */
    if(i2c_wait_flag(hi2c, I2C_ADDRHF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      hi2c->error_code = I2C_ERR_ADDR10;

      return I2C_ERR_ADDR10;
    }

    /* send slave address */
    i2c_data_send(hi2c->i2cx, (uint8_t)(address & 0x00FF));
  }

  /* wait for the addr7 flag to be set */
  if(i2c_wait_flag(hi2c, I2C_ADDR7F_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_ADDR;

    return I2C_ERR_ADDR;
  }

  return I2C_OK;
}

/**
  * @brief  send address in master receive mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_read_addr(i2c_handle_type *hi2c, uint16_t address, uint32_t timeout)
{
  /* enable ack */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* generate start condtion */
  i2c_start_generate(hi2c->i2cx);

  /* wait for the start flag to be set */
  if(i2c_wait_flag(hi2c, I2C_STARTF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_START;

    return I2C_ERR_START;
  }

  if(hi2c->i2cx->oaddr1_bit.addr1mode == I2C_ADDRESS_MODE_7BIT)
  {
    /* send slave address */
    i2c_7bit_address_send(hi2c->i2cx, address, I2C_DIRECTION_RECEIVE);
  }
  else
  {
    /* send slave 10-bit address header */
    i2c_data_send(hi2c->i2cx, (uint8_t)((address & 0x0300) >> 7) | 0xF0);

    /* wait for the addrh flag to be set */
    if(i2c_wait_flag(hi2c, I2C_ADDRHF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      hi2c->error_code = I2C_ERR_ADDR10;

      return I2C_ERR_ADDR10;
    }

    /* send slave address */
    i2c_data_send(hi2c->i2cx, (uint8_t)(address & 0x00FF));

    /* wait for the addr7 flag to be set */
    if(i2c_wait_flag(hi2c, I2C_ADDR7F_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      hi2c->error_code = I2C_ERR_ADDR;

      return I2C_ERR_ADDR;
    }

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

    /* generate restart condtion */
    i2c_start_generate(hi2c->i2cx);

    /* wait for the start flag to be set */
    if(i2c_wait_flag(hi2c, I2C_STARTF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
    {
      hi2c->error_code = I2C_ERR_START;

      return I2C_ERR_START;
    }

    /* send slave 10-bit address header */
    i2c_data_send(hi2c->i2cx, (uint8_t)((address & 0x0300) >> 7) | 0xF1);
  }

  /* wait for the addr7 flag to be set */
  if(i2c_wait_flag(hi2c, I2C_ADDR7F_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    hi2c->error_code = I2C_ERR_ADDR;

    return I2C_ERR_ADDR;
  }

  return I2C_OK;
}

/**
  * @brief  the master transmits data through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_transmit(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* send slave address */
  if(i2c_master_write_addr(hi2c, address, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_2;
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

  while(size > 0)
  {
    /* wait for the tdbe flag to be set */
    if(i2c_wait_flag(hi2c, I2C_TDBE_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      /* generate stop condtion */
      i2c_stop_generate(hi2c->i2cx);

      return I2C_ERR_STEP_3;
    }

    /* write data */
    i2c_data_send(hi2c->i2cx, (*pdata++));
    size--;
  }

  /* wait for the tdc flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDC_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_4;
  }

  /* generate stop condtion */
  i2c_stop_generate(hi2c->i2cx);

  return I2C_OK;
}

/**
  * @brief  the slave receive data through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_receive(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* enable ack */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* wait for the addr7 flag to be set */
  if(i2c_wait_flag(hi2c, I2C_ADDR7F_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable ack */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    return I2C_ERR_STEP_2;
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

  while(size > 0)
  {
    /* wait for the rdbf flag to be set */
    if(i2c_wait_flag(hi2c, I2C_RDBF_FLAG, I2C_EVENT_CHECK_STOP, timeout) != I2C_OK)
    {
      /* disable ack */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      return I2C_ERR_STEP_3;
    }

    /* read data */
    (*pdata++) = i2c_data_receive(hi2c->i2cx);
    size--;
  }

  /* wait for the stop flag to be set */
  if(i2c_wait_flag(hi2c, I2C_STOPF_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    /* disable ack */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    return I2C_ERR_STEP_4;
  }

  /* clear stop flag */
  i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

  /* disable ack */
  i2c_ack_enable(hi2c->i2cx, FALSE);

  return I2C_OK;
}

/**
  * @brief  the master receive data through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_receive(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* enable ack */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* send slave address */
  if(i2c_master_read_addr(hi2c, address, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_2;
  }

  if(size == 1)
  {
    /* disable ack */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);
  }
  else if(size == 2)
  {
    /* ack acts on the next byte */
    i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_NEXT);

    /* disable ack */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);
  }
  else
  {
    /* enable ack */
    i2c_ack_enable(hi2c->i2cx, TRUE);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);
  }

  while(size > 0)
  {
    if(size <= 3)
    {
      /* 1 byte */
      if(size == 1)
      {
        /* wait for the rdbf flag to be set */
        if(i2c_wait_flag(hi2c, I2C_RDBF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
        {
          /* generate stop condtion */
          i2c_stop_generate(hi2c->i2cx);

          return I2C_ERR_STEP_3;
        }

        /* read data */
        (*pdata++) = i2c_data_receive(hi2c->i2cx);
        size--;
      }
      /* 2 bytes */
      else if(size == 2)
      {
        /* wait for the tdc flag to be set */
        if(i2c_wait_flag(hi2c, I2C_TDC_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
        {
          /* generate stop condtion */
          i2c_stop_generate(hi2c->i2cx);

          return I2C_ERR_STEP_4;
        }

        /* generate stop condtion */
        i2c_stop_generate(hi2c->i2cx);

        /* read data */
        (*pdata++) = i2c_data_receive(hi2c->i2cx);
        size--;

        /* read data */
        (*pdata++) = i2c_data_receive(hi2c->i2cx);
        size--;
      }
      /* 3 last bytes */
      else
      {
        /* wait for the tdc flag to be set */
        if(i2c_wait_flag(hi2c, I2C_TDC_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
        {
          /* generate stop condtion */
          i2c_stop_generate(hi2c->i2cx);

          return I2C_ERR_STEP_5;
        }

        /* disable ack */
        i2c_ack_enable(hi2c->i2cx, FALSE);

        /* read data */
        (*pdata++) = i2c_data_receive(hi2c->i2cx);
        size--;

        /* wait for the tdc flag to be set */
        if(i2c_wait_flag(hi2c, I2C_TDC_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
        {
          /* generate stop condtion */
          i2c_stop_generate(hi2c->i2cx);

          return I2C_ERR_STEP_6;
        }

        /* generate stop condtion */
        i2c_stop_generate(hi2c->i2cx);

        /* read data */
        (*pdata++) = i2c_data_receive(hi2c->i2cx);
        size--;

        /* read data */
        (*pdata++) = i2c_data_receive(hi2c->i2cx);
        size--;
      }
    }
    else
    {
      /* wait for the rdbf flag to be set */
      if(i2c_wait_flag(hi2c, I2C_RDBF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
      {
        /* generate stop condtion */
        i2c_stop_generate(hi2c->i2cx);

        return I2C_ERR_STEP_7;
      }

      /* read data */
      (*pdata++) = i2c_data_receive(hi2c->i2cx);
      size--;
    }
  }

  return I2C_OK;
}

/**
  * @brief  the slave transmits data through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_transmit(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* enable ack */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* wait for the addr7 flag to be set */
  if(i2c_wait_flag(hi2c, I2C_ADDR7F_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    /* disable ack */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    return I2C_ERR_STEP_2;
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

  if(hi2c->i2cx->oaddr1_bit.addr1mode == I2C_ADDRESS_MODE_10BIT)
  {
    /* wait for the addr7 flag to be set */
    if(i2c_wait_flag(hi2c, I2C_ADDR7F_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
    {
      /* disable ack */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      return I2C_ERR_STEP_3;
    }

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);
  }

  while(size > 0)
  {
    /* wait for the tdbe flag to be set */
    if(i2c_wait_flag(hi2c, I2C_TDBE_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      /* disable ack */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      return I2C_ERR_STEP_4;
    }

    /* write data */
    i2c_data_send(hi2c->i2cx, *pdata++);
    size--;
  }

  /* wait for the ackfail flag to be set */
  if(i2c_wait_flag(hi2c, I2C_ACKFAIL_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_5;
  }

  /* clear ackfail flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);

  /* disable ack */
  i2c_ack_enable(hi2c->i2cx, FALSE);

  return I2C_OK;
}

/**
  * @brief  the master transmits data through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_transmit_int(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_MA_TX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->timeout = timeout;
  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* send slave address */
  if(i2c_master_write_addr(hi2c, address, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_2;
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

  /* enable interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT | I2C_DATA_INT | I2C_ERR_INT, TRUE);

  return I2C_OK;
}

/**
  * @brief  the slave receive data through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_receive_int(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_SLA_RX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->timeout = timeout;
  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* enable ack */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* enable interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT | I2C_DATA_INT | I2C_ERR_INT, TRUE);

  return I2C_OK;
}

/**
  * @brief  the master receive data through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_receive_int(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_MA_RX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->timeout = timeout;
  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* enable ack */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* send slave address */
  if(i2c_master_read_addr(hi2c, address, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_2;
  }

  if(hi2c->pcount == 1)
  {
    /* disable ack */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);
  }
  else if(hi2c->pcount == 2)
  {
    /* ack acts on the next byte */
    i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_NEXT);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

    /* disable ack */
    i2c_ack_enable(hi2c->i2cx, FALSE);
  }
  else
  {
    /* enable ack */
    i2c_ack_enable(hi2c->i2cx, TRUE);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);
  }

  /* enable interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT | I2C_DATA_INT | I2C_ERR_INT, TRUE);

  return I2C_OK;
}

/**
  * @brief  the slave transmits data through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_transmit_int(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_SLA_TX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->timeout = timeout;
  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* enable ack */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* enable interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT | I2C_DATA_INT | I2C_ERR_INT, TRUE);

  return I2C_OK;
}

/**
  * @brief  the master transmits data through dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_transmit_dma(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_MA_TX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->timeout = timeout;
  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* disable dma request */
  i2c_dma_enable(hi2c->i2cx, FALSE);

  /* configure the dma channel */
  i2c_dma_config(hi2c, hi2c->dma_tx_channel, pdata, size);

  /* send slave address */
  if(i2c_master_write_addr(hi2c, address, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_2;
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

  /* enable dma request */
  i2c_dma_enable(hi2c->i2cx, TRUE);

  return I2C_OK;
}

/**
  * @brief  the slave receive data through dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_receive_dma(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_SLA_RX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->timeout = timeout;
  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* enable ack */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* disable dma request */
  i2c_dma_enable(hi2c->i2cx, FALSE);

  /* configure the dma channel */
  i2c_dma_config(hi2c, hi2c->dma_rx_channel, pdata, size);

  /* enable dma request */
  i2c_dma_enable(hi2c->i2cx, TRUE);

  /* enable address interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT, TRUE);

  return I2C_OK;
}

/**
  * @brief  the master receive data through dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  address: slave address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_master_receive_dma(i2c_handle_type* hi2c, uint16_t address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_MA_RX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->timeout = timeout;
  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* enable ack */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* disable dma request */
  i2c_dma_enable(hi2c->i2cx, FALSE);

  /* configure the dma channel */
  i2c_dma_config(hi2c, hi2c->dma_rx_channel, pdata, size);

  /* send slave address */
  if(i2c_master_read_addr(hi2c, address, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_2;
  }

  if(size == 1)
  {
    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

    /* disable ack */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    /* enable dma request */
    i2c_dma_enable(hi2c->i2cx, TRUE);
  }
  else
  {
    /* enable dma end transfer */
    i2c_dma_end_transfer_set(hi2c->i2cx, TRUE);

    /* enable dma request */
    i2c_dma_enable(hi2c->i2cx, TRUE);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);
  }

  return I2C_OK;
}

/**
  * @brief  the slave transmits data through dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_slave_transmit_dma(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_SLA_TX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->timeout = timeout;
  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* enable ack */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* disable dma request */
  i2c_dma_enable(hi2c->i2cx, FALSE);

  /* configure the dma channel */
  i2c_dma_config(hi2c, hi2c->dma_tx_channel, pdata, size);

  /* enable dma request */
  i2c_dma_enable(hi2c->i2cx, TRUE);

  /* enable address interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT, TRUE);

  return I2C_OK;
}

/**
  * @brief  send memory address.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_address_send(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t mem_address, int32_t timeout)
{
  i2c_status_type err_code;

  if(mem_address_width == I2C_MEM_ADDR_WIDIH_8)
  {
    /* send memory address */
    i2c_data_send(hi2c->i2cx, mem_address & 0xFF);
  }
  else
  {
    /* send memory address */
    i2c_data_send(hi2c->i2cx, (mem_address >> 8) & 0xFF);

    /* wait for the tdbe flag to be set */
    err_code = i2c_wait_flag(hi2c, I2C_TDBE_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout);

    if(err_code != I2C_OK)
    {
      /* generate stop condtion */
      i2c_stop_generate(hi2c->i2cx);

      return err_code;
    }

    /* send memory address */
    i2c_data_send(hi2c->i2cx, mem_address & 0xFF);
  }

  return I2C_OK;
}

/**
  * @brief  write data to the memory device through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_write(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* send slave address */
  if(i2c_master_write_addr(hi2c, address, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_2;
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

  /* wait for the tdbe flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDBE_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_3;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_4;
  }

  while(size > 0)
  {
    /* wait for the tdbe flag to be set */
    if(i2c_wait_flag(hi2c, I2C_TDBE_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
      /* generate stop condtion */
      i2c_stop_generate(hi2c->i2cx);

      return I2C_ERR_STEP_5;
    }

    /* write data */
    i2c_data_send(hi2c->i2cx, (*pdata++));
    size--;
  }

  /* wait for the tdc flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDC_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_6;
  }

  /* generate stop condtion */
  i2c_stop_generate(hi2c->i2cx);

  return I2C_OK;
}

/**
  * @brief  read data from memory device through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_read(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->pbuff = pdata;
  hi2c->pcount = size;

  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* enable ack */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* send slave address */
  if(i2c_master_write_addr(hi2c, address, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_2;
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

  /* wait for the tdbe flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDBE_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_3;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_4;
  }

  /* wait for the tdbe flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDBE_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_5;
  }

  /* send slave address */
  if(i2c_master_read_addr(hi2c, address, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_6;
  }

  if(size == 1)
  {
    /* disable ack */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);
  }
  else if(size == 2)
  {
    /* ack acts on the next byte */
    i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_NEXT);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

    /* disable ack */
    i2c_ack_enable(hi2c->i2cx, FALSE);
  }
  else
  {
    /* enable ack */
    i2c_ack_enable(hi2c->i2cx, TRUE);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);
  }

  while(size > 0)
  {
    if(size <= 3)
    {
      /* 1 byte */
      if(size == 1)
      {
        /* wait for the rdbf flag to be set */
        if(i2c_wait_flag(hi2c, I2C_RDBF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
        {
          /* generate stop condtion */
          i2c_stop_generate(hi2c->i2cx);

          return I2C_ERR_STEP_7;
        }

        /* read data */
        (*pdata++) = i2c_data_receive(hi2c->i2cx);
        size--;
      }
      /* 2 bytes */
      else if(size == 2)
      {
        /* wait for the tdc flag to be set */
        if(i2c_wait_flag(hi2c, I2C_TDC_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
        {
          /* generate stop condtion */
          i2c_stop_generate(hi2c->i2cx);

          return I2C_ERR_STEP_8;
        }

        /* generate stop condtion */
        i2c_stop_generate(hi2c->i2cx);

        /* read data */
        (*pdata++) = i2c_data_receive(hi2c->i2cx);
        size--;

        /* read data */
        (*pdata++) = i2c_data_receive(hi2c->i2cx);
        size--;
      }
      /* 3 last bytes */
      else
      {
        /* wait for the tdc flag to be set */
        if(i2c_wait_flag(hi2c, I2C_TDC_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
        {
          /* generate stop condtion */
          i2c_stop_generate(hi2c->i2cx);

          return I2C_ERR_STEP_9;
        }

        /* disable ack */
        i2c_ack_enable(hi2c->i2cx, FALSE);

        /* read data */
        (*pdata++) = i2c_data_receive(hi2c->i2cx);
        size--;

        /* wait for the tdc flag to be set */
        if(i2c_wait_flag(hi2c, I2C_TDC_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
        {
          /* generate stop condtion */
          i2c_stop_generate(hi2c->i2cx);

          return I2C_ERR_STEP_10;
        }

        /* generate stop condtion */
        i2c_stop_generate(hi2c->i2cx);

        /* read data */
        (*pdata++) = i2c_data_receive(hi2c->i2cx);
        size--;

        /* read data */
        (*pdata++) = i2c_data_receive(hi2c->i2cx);
        size--;
      }
    }
    else
    {
      /* wait for the rdbf flag to be set */
      if(i2c_wait_flag(hi2c, I2C_RDBF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
      {
        /* generate stop condtion */
        i2c_stop_generate(hi2c->i2cx);

        return I2C_ERR_STEP_11;
      }

      /* read data */
      (*pdata++) = i2c_data_receive(hi2c->i2cx);
      size--;
    }
  }

  return I2C_OK;
}

/**
  * @brief  write data to the memory device through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_write_int(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_MA_TX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->timeout = timeout;
  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* send slave address */
  if(i2c_master_write_addr(hi2c, address, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_2;
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

  /* wait for the tdbe flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDBE_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_3;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_4;
  }

  /* wait for the tdbe flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDBE_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_5;
  }

  /* enable interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT | I2C_DATA_INT | I2C_ERR_INT, TRUE);

  return I2C_OK;
}

/**
  * @brief  read data from memory device through interrupt mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_read_int(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_MA_RX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->timeout = timeout;
  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* send slave address */
  if(i2c_master_write_addr(hi2c, address, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_2;
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

  /* wait for the tdbe flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDBE_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_3;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_4;
  }

  /* wait for the tdbe flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDBE_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_5;
  }

  /* send slave address */
  if(i2c_master_read_addr(hi2c, address, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_6;
  }

  if(hi2c->pcount == 1)
  {
    /* disable ack */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);
  }
  else if(hi2c->pcount == 2)
  {
    /* ack acts on the next byte */
    i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_NEXT);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

    /* disable ack */
    i2c_ack_enable(hi2c->i2cx, FALSE);
  }
  else
  {
    /* enable ack */
    i2c_ack_enable(hi2c->i2cx, TRUE);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);
  }

  /* enable interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT | I2C_DATA_INT | I2C_ERR_INT, TRUE);

  return I2C_OK;
}

/**
  * @brief  write data to the memory device through dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_write_dma(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_MA_TX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->timeout = timeout;
  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* disable dma request */
  i2c_dma_enable(hi2c->i2cx, FALSE);

  /* configure the dma channel */
  i2c_dma_config(hi2c, hi2c->dma_tx_channel, pdata, size);

  /* send slave address */
  if(i2c_master_write_addr(hi2c, address, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_2;
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

  /* wait for the tdbe flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDBE_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_3;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_4;
  }

  /* wait for the tdbe flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDBE_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_5;
  }

  /* enable dma request */
  i2c_dma_enable(hi2c->i2cx, TRUE);

  return I2C_OK;
}

/**
  * @brief  read data from memory device through polling mode.
  * @param  hi2c: the handle points to the operation information.
  * @param  mem_address_width: memory address width.
  *         this parameter can be one of the following values:
  *         - I2C_MEM_ADDR_WIDIH_8:  memory address is 8 bit
  *         - I2C_MEM_ADDR_WIDIH_16:  memory address is 16 bit
  * @param  address: memory device address.
  * @param  mem_address: memory address.
  * @param  pdata: data buffer.
  * @param  size: data size.
  * @param  timeout: maximum waiting time.
  * @retval i2c status.
  */
i2c_status_type i2c_memory_read_dma(i2c_handle_type* hi2c, i2c_mem_address_width_type mem_address_width, uint16_t address, uint16_t mem_address, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_DMA_MA_RX;
  hi2c->status = I2C_START;

  hi2c->pbuff  = pdata;
  hi2c->pcount = size;

  hi2c->timeout = timeout;
  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* enable ack */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* disable dma request */
  i2c_dma_enable(hi2c->i2cx, FALSE);

  /* configure the dma channel */
  i2c_dma_config(hi2c, hi2c->dma_rx_channel, pdata, size);

  /* send slave address */
  if(i2c_master_write_addr(hi2c, address, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_2;
  }

  /* clear addr flag */
  i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

  /* wait for the tdbe flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDBE_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_3;
  }

  /* send memory address */
  if(i2c_memory_address_send(hi2c, mem_address_width, mem_address, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_4;
  }

  /* wait for the tdbe flag to be set */
  if(i2c_wait_flag(hi2c, I2C_TDBE_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_5;
  }

  /* send slave address */
  if(i2c_master_read_addr(hi2c, address, timeout) != I2C_OK)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    return I2C_ERR_STEP_6;
  }

  if(size == 1)
  {
    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);

    /* disable ack */
    i2c_ack_enable(hi2c->i2cx, FALSE);

    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);

    /* enable dma request */
    i2c_dma_enable(hi2c->i2cx, TRUE);
  }
  else
  {
    /* enable dma end transfer */
    i2c_dma_end_transfer_set(hi2c->i2cx, TRUE);

    /* enable dma request */
    i2c_dma_enable(hi2c->i2cx, TRUE);

    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);
  }

  return I2C_OK;
}

/**
  * @brief  master transfer mode interrupt procession function
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_master_tx_isr_int(i2c_handle_type* hi2c)
{
  /* step 1: transfer data */
  if(i2c_flag_get(hi2c->i2cx, I2C_TDBE_FLAG) != RESET)
  {
    if(hi2c->pcount == 0)
    {
      /* transfer complete */
      hi2c->status = I2C_END;

      /* disable interrupt */
      i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT | I2C_DATA_INT | I2C_ERR_INT, FALSE);

      /* generate stop condtion */
      i2c_stop_generate(hi2c->i2cx);
    }
    else
    {
      /* write data */
      i2c_data_send(hi2c->i2cx, *hi2c->pbuff++);
      hi2c->pcount--;
    }
  }
}

/**
  * @brief  master receive mode interrupt procession function
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_master_rx_isr_int(i2c_handle_type* hi2c)
{
  if(i2c_flag_get(hi2c->i2cx, I2C_TDC_FLAG) != RESET)
  {
    if(hi2c->pcount == 3)
    {
      /* disable ack */
      i2c_ack_enable(hi2c->i2cx, FALSE);

      /* read data */
      (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);
      hi2c->pcount--;
    }
    else if(hi2c->pcount == 2)
    {
      /* generate stop condtion */
      i2c_stop_generate(hi2c->i2cx);

      /* read data */
      (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);
      hi2c->pcount--;

      /* read data */
      (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);
      hi2c->pcount--;

      /* transfer complete */
      hi2c->status = I2C_END;

      /* disable interrupt */
      i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT | I2C_DATA_INT | I2C_ERR_INT, FALSE);
    }
    else
    {
      /* read data */
      (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);
      hi2c->pcount--;
    }
  }

  else if(i2c_flag_get(hi2c->i2cx, I2C_RDBF_FLAG) != RESET)
  {
    if(hi2c->pcount > 3)
    {
      /* read data */
      (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);
      hi2c->pcount--;
    }
    else if((hi2c->pcount == 3) || (hi2c->pcount == 2))
    {
      /* disable rdbf interrupt */
      i2c_interrupt_enable(hi2c->i2cx, I2C_DATA_INT, FALSE);
    }
    else
    {
      /* read data */
      (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);
      hi2c->pcount--;

      /* transfer complete */
      hi2c->status = I2C_END;

      /* disable interrupt */
      i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT | I2C_DATA_INT | I2C_ERR_INT, FALSE);
    }
  }
}

/**
  * @brief  slave transfer mode interrupt procession function
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_slave_tx_isr_int(i2c_handle_type* hi2c)
{
  /* step 1: receive slave address */
  if(i2c_flag_get(hi2c->i2cx, I2C_ADDR7F_FLAG) != RESET)
  {
    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);
  }

  /* step 2: transfer data */
  else if(i2c_flag_get(hi2c->i2cx, I2C_TDBE_FLAG) != RESET)
  {
    if(hi2c->pcount)
    {
      /* send data */
      i2c_data_send(hi2c->i2cx, (*hi2c->pbuff++));
      hi2c->pcount--;

      if(hi2c->pcount == 0)
      {
        /* disable interrupt */
        i2c_interrupt_enable(hi2c->i2cx, I2C_DATA_INT | I2C_EVT_INT | I2C_ERR_INT, FALSE);

        /* wait for the ackfail flag to be set */
        hi2c->status = i2c_wait_flag(hi2c, I2C_ACKFAIL_FLAG, I2C_EVENT_CHECK_NONE, hi2c->timeout);

        /* clear ackfail flag */
        i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);

        /* transfer complete */
        hi2c->status = I2C_END;
      }
    }
  }
}

/**
  * @brief  slave receive mode interrupt procession function
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_slave_rx_isr_int(i2c_handle_type* hi2c)
{
  /* step 1: receive slave address */
  if(i2c_flag_get(hi2c->i2cx, I2C_ADDR7F_FLAG) != RESET)
  {
    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);
  }

  /* step 2: receive data */
  else if(i2c_flag_get(hi2c->i2cx, I2C_RDBF_FLAG) != RESET)
  {
    if(hi2c->pcount)
    {
      /* read data */
      (*hi2c->pbuff++) = i2c_data_receive(hi2c->i2cx);

      hi2c->pcount--;
    }
  }

  /* step 3: stop conditon is received, transfer ends */
  else if(i2c_flag_get(hi2c->i2cx, I2C_STOPF_FLAG) != RESET)
  {
    /* clear stop flag */
    i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

    /* transfer complete */
    hi2c->status = I2C_END;

    /* disable interrupt */
    i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT | I2C_DATA_INT | I2C_ERR_INT, FALSE);
  }
}

/**
  * @brief  master interrupt processing function in dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_master_tx_isr_dma(i2c_handle_type* hi2c)
{
  /* tdc interrupt */
  if(i2c_flag_get(hi2c->i2cx, I2C_TDC_FLAG) != RESET)
  {
    /* generate stop condtion */
    i2c_stop_generate(hi2c->i2cx);
    
    /* disable evt interrupt */
    i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT, FALSE);
    
    /* transfer complete */
    hi2c->status = I2C_END;
  }
}

/**
  * @brief  slave interrupt processing function in dma mode.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_slave_tx_rx_isr_dma(i2c_handle_type* hi2c)
{
  /* receive slave address */
  if(i2c_flag_get(hi2c->i2cx, I2C_ADDR7F_FLAG) != RESET)
  {
    /* clear addr flag */
    i2c_flag_clear(hi2c->i2cx, I2C_ADDR7F_FLAG);
  }

  /* wait for the stop flag to be set, trasnfer end */
  if(i2c_flag_get(hi2c->i2cx, I2C_STOPF_FLAG) != RESET)
  {
    /* clear stop flag */
    i2c_flag_clear(hi2c->i2cx, I2C_STOPF_FLAG);

    /* disable evt interrupt */
    i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT, FALSE);    
    
    /* disable ack */
    i2c_ack_enable(hi2c->i2cx, TRUE);
    
    /* transfer complete */
    hi2c->status = I2C_END;
  }
}

/**
  * @brief  interrupt procession function.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_evt_irq_handler(i2c_handle_type* hi2c)
{
  switch(hi2c->mode)
  {
    case I2C_INT_MA_TX:
      i2c_master_tx_isr_int(hi2c);
      break;
    case I2C_INT_MA_RX:
      i2c_master_rx_isr_int(hi2c);
      break;
    case I2C_INT_SLA_TX:
      i2c_slave_tx_isr_int(hi2c);
      break;
    case I2C_INT_SLA_RX:
      i2c_slave_rx_isr_int(hi2c);
      break;
    case I2C_DMA_MA_TX:
      i2c_master_tx_isr_dma(hi2c);
      break;
    case I2C_DMA_SLA_TX:
    case I2C_DMA_SLA_RX:
      i2c_slave_tx_rx_isr_dma(hi2c);
      break;
    default:
      break;
  }
}

/**
  * @brief  dma transmission complete interrupt function.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_dma_tx_irq_handler(i2c_handle_type* hi2c)
{
  /* transfer complete */
  if(dma_flag_get(DMA_GET_TC_FLAG(hi2c->dma_tx_channel)) != RESET)
  {
    /* disable the transfer complete interrupt */
    dma_interrupt_enable(hi2c->dma_tx_channel, DMA_FDT_INT, FALSE);

    /* clear the transfer complete flag */
    dma_flag_clear(DMA_GET_TC_FLAG(hi2c->dma_tx_channel));

    /* disable dma request */
    i2c_dma_enable(hi2c->i2cx, FALSE);

    hi2c->pcount = 0;
    
    switch(hi2c->mode)
    {
      case I2C_DMA_MA_TX:
        /* enable tdc interrupt, generate stop condition in tdc interrupt */
        i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT, TRUE);
        break;
      case I2C_DMA_SLA_TX:
        /* enable ackfail interrupt, generate stop condition in ackfail interrupt */
        i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT, TRUE);
        break;
      default:
        break;
    }
  }
}

/**
  * @brief  dma reveive complete interrupt function.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_dma_rx_irq_handler(i2c_handle_type* hi2c)
{
  /* transfer complete */
  if(dma_flag_get(DMA_GET_TC_FLAG(hi2c->dma_rx_channel)) != RESET)
  {
    /* disable the transfer complete interrupt */
    dma_interrupt_enable(hi2c->dma_rx_channel, DMA_FDT_INT, FALSE);

    /* clear the transfer complete flag */
    dma_flag_clear(DMA_GET_TC_FLAG(hi2c->dma_rx_channel));

    /* disable dma request */
    i2c_dma_enable(hi2c->i2cx, FALSE);

    hi2c->pcount = 0;
    
    switch(hi2c->mode)
    {
      case I2C_DMA_MA_RX:
        /* clear ackfail flag  */
        i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);

        if(hi2c->pcount != 1)
        {
          /* generate stop condtion */
          i2c_stop_generate(hi2c->i2cx);
        }
        
        /* transfer complete */
        hi2c->status = I2C_END;
        break;
      case I2C_DMA_SLA_RX:
        /* enable stop interrupt, wait for the stop flag to be set  */
        i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT, TRUE);
        break;
      default:
        break;
    }
  }
}

/**
  * @brief  i2c error interrupt function.
  * @param  hi2c: the handle points to the operation information.
  * @retval none.
  */
void i2c_err_irq_handler(i2c_handle_type* hi2c)
{
  /* buserr */
  if(i2c_flag_get(hi2c->i2cx, I2C_BUSERR_FLAG) != RESET)
  {
    i2c_flag_clear(hi2c->i2cx, I2C_BUSERR_FLAG);

    hi2c->error_code = I2C_ERR_INTERRUPT;
  }

  /* arlost */
  if(i2c_flag_get(hi2c->i2cx, I2C_ARLOST_FLAG) != RESET)
  {
    i2c_flag_clear(hi2c->i2cx, I2C_ARLOST_FLAG);

    hi2c->error_code = I2C_ERR_INTERRUPT;
  }

  /* ackfail */
  if(i2c_flag_get(hi2c->i2cx, I2C_ACKFAIL_FLAG) != RESET)
  {
    i2c_flag_clear(hi2c->i2cx, I2C_ACKFAIL_FLAG);

    switch(hi2c->mode)
    {
      case I2C_DMA_SLA_TX:
        /* disable ack */
        i2c_ack_enable(hi2c->i2cx, FALSE);

        /* disable evt interrupt */
        i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT, FALSE);    
      
        /* transfer complete */
        hi2c->status = I2C_END;
        break;
      default:
        hi2c->error_code = I2C_ERR_INTERRUPT;
        break;
    }
  }

  /* ouf */
  if(i2c_flag_get(hi2c->i2cx, I2C_OUF_FLAG) != RESET)
  {
    i2c_flag_clear(hi2c->i2cx, I2C_OUF_FLAG);

    hi2c->error_code = I2C_ERR_INTERRUPT;
  }

  /* pecerr */
  if(i2c_flag_get(hi2c->i2cx, I2C_PECERR_FLAG) != RESET)
  {
    i2c_flag_clear(hi2c->i2cx, I2C_PECERR_FLAG);

    hi2c->error_code = I2C_ERR_INTERRUPT;
  }

  /* tmout */
  if(i2c_flag_get(hi2c->i2cx, I2C_TMOUT_FLAG) != RESET)
  {
    i2c_flag_clear(hi2c->i2cx, I2C_TMOUT_FLAG);

    hi2c->error_code = I2C_ERR_INTERRUPT;
  }

  /* alertf */
  if(i2c_flag_get(hi2c->i2cx, I2C_ALERTF_FLAG) != RESET)
  {
    i2c_flag_clear(hi2c->i2cx, I2C_ALERTF_FLAG);

    hi2c->error_code = I2C_ERR_INTERRUPT;
  }

  /* disable all interrupts */
  i2c_interrupt_enable(hi2c->i2cx, I2C_ERR_INT, FALSE);
}

/**
  * @}
  */

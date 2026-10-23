#include "can_bus.h"

//过滤器配置,ID在21位之后，所以ID左移21位。
//取高16位作为FilterIdHigh，所以ID << 6
//取低16位作为FilterIdLow
//过滤器掩码要高16位符合ID的掩码
//低16位作为FilterMaskIdLow，比较IDE和RTR位，只接收标准数据帧，所以低16位为0x0006

/*
 * CAN底层只负责通用收发，不保存任何电机状态。
 * 上层设备驱动（大疆电机、RS05）只需给出需要接收的ID。
 */
HAL_StatusTypeDef CAN_Bus_ConfigStdIdList(CAN_HandleTypeDef *hcan,
                                          uint32_t filter_bank,
                                          uint32_t slave_start_bank,
                                          const uint16_t std_ids[4])
{
    CAN_FilterTypeDef filter = {0};

    if ((hcan == NULL) || (std_ids == NULL))
    {
        return HAL_ERROR;
    }

    /* 16位列表模式下，一个过滤器可以精确匹配4个11位标准ID。 */
    filter.FilterBank = filter_bank;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterIdHigh = (uint32_t)std_ids[0] << 5;
    filter.FilterIdLow = (uint32_t)std_ids[1] << 5;
    filter.FilterMaskIdHigh = (uint32_t)std_ids[2] << 5;
    filter.FilterMaskIdLow = (uint32_t)std_ids[3] << 5;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = slave_start_bank;

    return HAL_CAN_ConfigFilter(hcan, &filter);
}

HAL_StatusTypeDef CAN_Bus_ConfigStdIdMask(CAN_HandleTypeDef *hcan,
                                          uint32_t filter_bank,
                                          uint32_t slave_start_bank,
                                          uint16_t std_id,
                                          uint16_t std_id_mask)
{
    CAN_FilterTypeDef filter = {0};

    if ((hcan == NULL) || (std_id > 0x7FFU) ||
        (std_id_mask > 0x7FFU))
    {
        return HAL_ERROR;
    }

    /* 0x200配合0x7F0，仅忽略ID的低4位，即接收0x200~0x20F。 */
    filter.FilterBank = filter_bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = (uint32_t)std_id << 5;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = (uint32_t)std_id_mask << 5;
    /* 比较IDE和RTR位，只接收标准数据帧。 */
    filter.FilterMaskIdLow = 0x0006U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = slave_start_bank;

    return HAL_CAN_ConfigFilter(hcan, &filter);
}

HAL_StatusTypeDef CAN_Bus_ConfigExtIdMask(CAN_HandleTypeDef *hcan,
                                          uint32_t filter_bank,
                                          uint32_t slave_start_bank,
                                          uint32_t ext_id,
                                          uint32_t ext_id_mask)
{
    CAN_FilterTypeDef filter = {0};

    if ((hcan == NULL) || (ext_id > 0x1FFFFFFFU) ||
        (ext_id_mask > 0x1FFFFFFFU))
    {
        return HAL_ERROR;
    }

    /* 32位过滤器里扩展ID占bit31~3，后面是IDE(bit2)、RTR(bit1)，所以ID左移3位。 */
    filter.FilterBank = filter_bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = (ext_id << 3) >> 16;
    filter.FilterIdLow = ((ext_id << 3) & 0xFFFFU) | CAN_ID_EXT;
    filter.FilterMaskIdHigh = (ext_id_mask << 3) >> 16;
    /* 比较IDE和RTR位，只接收扩展数据帧。 */
    filter.FilterMaskIdLow = ((ext_id_mask << 3) & 0xFFFFU) |
                             CAN_ID_EXT | CAN_RTR_REMOTE;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = slave_start_bank;

    return HAL_CAN_ConfigFilter(hcan, &filter);
}

HAL_StatusTypeDef CAN_Bus_StartRxFifo0(CAN_HandleTypeDef *hcan)
{
    HAL_StatusTypeDef status;

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    /* 同一路CAN上的多个驱动（如CAN2上的3508和RS05）都会调用一次，
       已经启动过就不再调用HAL_CAN_Start，否则它会返回HAL_ERROR。 */
    if (HAL_CAN_GetState(hcan) != HAL_CAN_STATE_LISTENING)
    {
        /* 必须先启动CAN，再开启接收通知。 */
        status = HAL_CAN_Start(hcan);
        if (status != HAL_OK)
        {
            return status;
        }
    }

    return HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

HAL_StatusTypeDef CAN_Bus_SendStdData(CAN_HandleTypeDef *hcan,
                                      uint16_t std_id,
                                      const uint8_t *data,
                                      uint8_t length)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    /* 标准ID最大为0x7FF，经典CAN一帧最多携带8字节。 */
    if ((hcan == NULL) || (data == NULL) ||
        (std_id > 0x7FFU) || (length > 8U))
    {
        return HAL_ERROR;
    }

    /* 没有空邮箱时立即返回HAL_BUSY，由上层决定何时重试。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return HAL_BUSY;
    }

    header.StdId = std_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = length;
    header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &header, (uint8_t *)data, &mailbox);
}

HAL_StatusTypeDef CAN_Bus_SendExtData(CAN_HandleTypeDef *hcan,
                                      uint32_t ext_id,
                                      const uint8_t *data,
                                      uint8_t length)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    /* 扩展ID最大为0x1FFFFFFF，经典CAN一帧最多携带8字节。 */
    if ((hcan == NULL) || (data == NULL) ||
        (ext_id > 0x1FFFFFFFU) || (length > 8U))
    {
        return HAL_ERROR;
    }

    /* 没有空邮箱时立即返回HAL_BUSY，由上层决定何时重试。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return HAL_BUSY;
    }

    header.ExtId = ext_id;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = length;
    header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &header, (uint8_t *)data, &mailbox);
}

HAL_StatusTypeDef CAN_Bus_ReadFifo0(CAN_HandleTypeDef *hcan,
                                    CAN_RxHeaderTypeDef *header,
                                    uint8_t data[8])
{
    if ((hcan == NULL) || (header == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    return HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, header, data);
}

/**
 * CANopen_OD —— CANopen SDO 式命令解析（协议层）
 *
 * 帧格式（标准 11 位 ID，Rx=0x601，Tx=0x581，8 字节）：
 *   data[0]=命令(0x60写/0x40读)
 *   data[1..2]=对象索引(小端)  data[3]=子索引
 *   data[4..7]=32 位数据(小端)
 *
 * 对象表：
 *   0x6040 控制字   0x06=停止 / 0x1F=启动(触发 commit)
 *   0x6060 工作模式 0=转矩 / 1=速度 / 2=位置(预留)
 *   0x60FF 目标速度 int32 milli-RPS
 *   0x6073 目标电流 int32 mA (iq_ref)
 *   0x607A 目标位置 int32 milli-rad（模型无位置环，仅存参）
 *   0x3001 读驱动器状态   0x3002 读版本号(握手)
 */
#include "CANopen_OD.h"
#include "CAN_bsp.h"
#include "MotorCtrl.h"
#include "motor_state_machine.h"

#define FW_VERSION 0x0100U    /* v1.0 */

static void send_reply(const uint8_t d[8])
{
    CAN_bsp_Send(CAN_ID_SDO_TX, d, 8);
}

static void reply_pos(const uint8_t d[8])   /* 写成功 */
{
    uint8_t r[8] = {0x60, d[1], d[2], 0x00, 0,0,0,0};
    send_reply(r);
}

static void reply_neg(const uint8_t d[8])   /* 写失败（SDO 消极应答） */
{
    uint8_t r[8] = {0x80, d[1], d[2], 0x00, 0xFF,0xFF,0xFF,0xFF};
    send_reply(r);
}

static void reply_version(void)
{
    uint8_t r[8] = {0x60, 0x02, 0x30, 0x00, 0,0,0,0};
    uint32_t v = FW_VERSION;
    r[4] = (uint8_t)(v & 0xFFU);
    r[5] = (uint8_t)((v >> 8) & 0xFFU);
    r[6] = (uint8_t)((v >> 16) & 0xFFU);
    r[7] = (uint8_t)((v >> 24) & 0xFFU);
    send_reply(r);
}

static void reply_status(void)
{
    uint8_t r[8] = {0x60, 0x01, 0x30, 0x00, 0,0,0,0};
    MotorCtrl_GetStatus(&r[4]);
    send_reply(r);
}

/* 命令 → 状态机事件（参数取自 MotorCtrl 影子值，3b 由状态机提交快照） */
static void post_cmd_event(motor_event_id_t id)
{
    MotorEvent_t evt = { 0 };
    evt.id          = id;
    evt.timestamp_ms = HAL_GetTick();
    evt.mode        = MotorCtrl.mode_temp;
    evt.speed_rps   = MotorCtrl.speed_rps_temp;
    evt.iq_ref_A    = MotorCtrl.iq_ref_A_temp;
    evt.pos_ref     = MotorCtrl.pos_ref_temp;
    MotorStateMachine_PostEvent(&evt);
}

static void HandleSDO(const uint8_t d[8])
{
    uint16_t index = (uint16_t)((uint16_t)d[2] << 8) | d[1];
    int32_t  raw   = (int32_t)((uint32_t)d[4] | ((uint32_t)d[5] << 8) |
                               ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24));

    switch (index) {
    case 0x6040:  /* 控制字：启停 → 投递状态机事件（Phase 3，不再直接调 MotorCtrl） */
        if      (d[4] == 0x1FU) { post_cmd_event(EVENT_toRUN);      reply_pos(d); }
        else if (d[4] == 0x06U) { post_cmd_event(EVENT_RUNtoSTOP);   reply_pos(d); }
        else                     reply_neg(d);
        break;
    case 0x6060:  /* 工作模式 */
        MotorCtrl_SetMode((uint8_t)d[4]); reply_pos(d); break;
    case 0x60FF:  /* 目标速度 milli-RPS */
        MotorCtrl_SetSpeedRPS((float)raw * 0.001f); reply_pos(d); break;
    case 0x6073:  /* 目标电流 mA */
        MotorCtrl_SetTorqueA((float)raw * 0.001f); reply_pos(d); break;
    case 0x607A:  /* 目标位置 milli-rad（预留） */
        MotorCtrl_SetPosition((float)raw * 0.001f); reply_pos(d); break;
    case 0x3001:  /* 读状态 */
        reply_status(); break;
    case 0x3002:  /* 握手/版本 */
        reply_version(); break;
    default:
        break;
    }
}

void CANopen_OD_Process(void)
{
    CAN_Frame_t f;
    while (CAN_bsp_PopFrame(&f)) {
        if ((f.id == CAN_ID_SDO_RX) && (f.dlc == 8U)) HandleSDO(f.data);
    }
}

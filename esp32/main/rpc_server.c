/* JSON-RPC 2.0 服务端（移植自 mqttagent/air780e/rpc.lua，见 docs/rpc.md）。
 * 方法：agent.ping/getConfig/setConfig/reboot、bms.getState/listParams/getParam/
 *      setParam/readRegisters/writeRegisters/action。 */

#include "rpc_server.h"

#include <stdio.h>
#include <string.h>

#include "bms_interface.h"
#include "esp_log.h"
#include "json_util.h"

static const char *TAG = "rpc";

static rpc_deps_t s_deps;
static bool s_ready;

/* ---------------- BMS 参数目录 ---------------- */

enum {
    PT_U16 = 0,
    PT_I16,
    PT_U32,
    PT_I32,
    PT_U8L,
    PT_U8H,
    PT_BIT,
};

typedef struct {
    const char *name;
    uint16_t reg;
    uint8_t type;
    uint8_t len;      /* 寄存器数 */
    bool writable;
    uint8_t bit;      /* 仅 BIT */
    const char *unit;
    const char *desc;
} rpc_param_t;

static const rpc_param_t PARAMS[] = {
    /* 配置区（RW） */
    {"VolCellUV", 0x1004, PT_U32, 2, true, 0, "mV", "单体欠压保护"},
    {"VolCellOV", 0x100C, PT_U32, 2, true, 0, "mV", "单体过充保护"},
    {"VolBalanTrig", 0x1014, PT_U32, 2, true, 0, "mV", "触发均衡压差"},
    {"VolCellRCV", 0x1020, PT_U32, 2, true, 0, "mV", "推荐充电电压"},
    {"VolCellRFV", 0x1024, PT_U32, 2, true, 0, "mV", "浮充电压"},
    {"VolSysPwrOff", 0x1028, PT_U32, 2, true, 0, "mV", "自动关机电压"},
    {"CurBatCOC", 0x102C, PT_U32, 2, true, 0, "mA", "持续充电电流"},
    {"CurBatDcOC", 0x1038, PT_U32, 2, true, 0, "mA", "持续放电电流"},
    {"CurBalanMax", 0x1048, PT_U32, 2, true, 0, "mA", "最大均衡电流"},
    {"TMPBatCOT", 0x104C, PT_I32, 2, true, 0, "0.1℃", "充电过温保护"},
    {"TMPBatDcOT", 0x1054, PT_I32, 2, true, 0, "0.1℃", "放电过温保护"},
    {"TMPBatCUT", 0x105C, PT_I32, 2, true, 0, "0.1℃", "充电低温保护"},
    {"TMPMosOT", 0x1064, PT_I32, 2, true, 0, "0.1℃", "MOS 过温保护"},
    {"CellCount", 0x106C, PT_U32, 2, true, 0, "串", "单体数量"},
    {"BatChargeEN", 0x1070, PT_U32, 2, true, 0, "-", "充电开关 (1/0)"},
    {"BatDisChargeEN", 0x1074, PT_U32, 2, true, 0, "-", "放电开关 (1/0)"},
    {"BalanEN", 0x1078, PT_U32, 2, true, 0, "-", "均衡开关 (1/0)"},
    {"CapBatCell", 0x107C, PT_U32, 2, true, 0, "mAH", "电池设计容量"},
    {"DevAddr", 0x1108, PT_U32, 2, true, 0, "H", "设备地址"},
    {"ControlFlags", 0x1114, PT_U16, 1, true, 0, "-", "控制位域整值"},
    {"HeatEN", 0x1114, PT_BIT, 1, true, 0, "-", "加热开关"},
    {"SmartSleep", 0x1114, PT_BIT, 1, true, 6, "-", "智能休眠"},
    /* 实时区（只读） */
    {"CellSta", 0x1240, PT_U32, 2, false, 0, "-", "电池状态位图"},
    {"CellVolAve", 0x1244, PT_U16, 1, false, 0, "mV", "单体平均电压"},
    {"CellVdifMax", 0x1246, PT_U16, 1, false, 0, "mV", "最大压差"},
    {"TempMos", 0x128A, PT_I16, 1, false, 0, "0.1℃", "功率板温度"},
    {"BatVol", 0x1290, PT_U32, 2, false, 0, "mV", "电池总电压"},
    {"BatCurrent", 0x1298, PT_I32, 2, false, 0, "mA", "电池电流"},
    {"TempBat1", 0x129C, PT_I16, 1, false, 0, "0.1℃", "电池温度 1"},
    {"TempBat2", 0x129E, PT_I16, 1, false, 0, "0.1℃", "电池温度 2"},
    {"AlarmFlags", 0x12A0, PT_U32, 2, false, 0, "-", "报警标志位域"},
    {"BalanCurrent", 0x12A4, PT_I16, 1, false, 0, "mA", "均衡电流"},
    {"SOC", 0x12A6, PT_U8L, 1, false, 0, "%", "剩余电量"},
    {"SOCCapRemain", 0x12A8, PT_I32, 2, false, 0, "mAH", "剩余容量"},
    {"SOCFullChargeCap", 0x12AC, PT_U32, 2, false, 0, "mAH", "电池实际容量"},
    {"SOCCycleCount", 0x12B0, PT_U32, 2, false, 0, "次", "循环次数"},
};

#define PARAM_COUNT ((int)(sizeof(PARAMS) / sizeof(PARAMS[0])))

static const char *type_name(uint8_t type)
{
    switch (type) {
    case PT_U16: return "UINT16";
    case PT_I16: return "INT16";
    case PT_U32: return "UINT32";
    case PT_I32: return "INT32";
    case PT_U8L: return "U8L";
    case PT_U8H: return "U8H";
    case PT_BIT: return "BIT";
    default: return "UNKNOWN";
    }
}

static const rpc_param_t *find_param(const char *name, size_t len)
{
    for (int i = 0; i < PARAM_COUNT; i++) {
        if (strlen(PARAMS[i].name) == len && memcmp(PARAMS[i].name, name, len) == 0) {
            return &PARAMS[i];
        }
    }
    return NULL;
}

/* 把寄存器原始值按类型解码为整数 */
static int64_t decode_value(const rpc_param_t *p, const uint16_t *regs)
{
    uint32_t u32 = ((uint32_t)regs[0] << 16) | (p->len > 1 ? regs[1] : 0);
    if (p->type == PT_U16) {
        return (int64_t)regs[0];
    }
    if (p->type == PT_I16) {
        return (int64_t)(int16_t)regs[0];
    }
    if (p->type == PT_U32) {
        return (int64_t)u32;
    }
    if (p->type == PT_I32) {
        return (int64_t)(int32_t)u32;
    }
    if (p->type == PT_U8L) {
        return (int64_t)(regs[0] & 0xFF);
    }
    if (p->type == PT_U8H) {
        return (int64_t)(regs[0] >> 8);
    }
    return (int64_t)((regs[0] >> p->bit) & 1u);
}

/* 把整数值编码成寄存器（BIT 类型需要读改写） */
static int encode_value(const rpc_param_t *p, long value, uint16_t *regs)
{
    if (p->type == PT_BIT) {
        if (value != 0 && value != 1) {
            return -1;
        }
        regs[0] = (uint16_t)((regs[0] & ~(1u << p->bit)) | ((value ? 1u : 0u) << p->bit));
        return 0;
    }
    if (p->type == PT_U16) {
        if (value < 0 || value > 0xFFFF) {
            return -1;
        }
        regs[0] = (uint16_t)value;
        return 0;
    }
    if (p->type == PT_I16) {
        if (value < -32768 || value > 32767) {
            return -1;
        }
        regs[0] = (uint16_t)(int16_t)value;
        return 0;
    }
    if (p->len == 2) {
        uint32_t u = (uint32_t)value;
        regs[0] = (uint16_t)(u >> 16);
        regs[1] = (uint16_t)(u & 0xFFFF);
        return 0;
    }
    return -1;
}

/* ---------------- 响应构造 ---------------- */

static void put_error(sbuf_t *sb, const json_node_t *id, int code, const char *msg,
                      const char *data_err)
{
    sb_puts(sb, "{\"jsonrpc\":\"2.0\",\"id\":");
    if (id != NULL && id->type == JSON_STR) {
        sb_json_quoted(sb, id->str);
    } else if (id != NULL && id->type == JSON_NUM) {
        sb_printf(sb, "%ld", (long)id->num);
    } else {
        sb_puts(sb, "null");
    }
    sb_printf(sb, ",\"error\":{\"code\":%d,\"message\":", code);
    sb_json_quoted(sb, msg);
    if (data_err != NULL) {
        sb_puts(sb, ",\"data\":{\"err\":");
        sb_json_quoted(sb, data_err);
        sb_puts(sb, "}");
    }
    sb_puts(sb, "}}");
}

static void put_result_open(sbuf_t *sb, const json_node_t *id)
{
    sb_puts(sb, "{\"jsonrpc\":\"2.0\",\"id\":");
    if (id != NULL && id->type == JSON_STR) {
        sb_json_quoted(sb, id->str);
    } else if (id != NULL && id->type == JSON_NUM) {
        sb_printf(sb, "%ld", (long)id->num);
    } else {
        sb_puts(sb, "null");
    }
    sb_puts(sb, ",\"result\":");
}

/* ---------------- bms.* ---------------- */

static bool read_param_regs(const rpc_param_t *p, uint16_t *regs, char *err, size_t err_cap)
{
    if (s_deps.read_regs == NULL) {
        snprintf(err, err_cap, "当前采集通道不支持原始寄存器访问");
        return false;
    }
    return s_deps.read_regs(p->reg, p->len, regs, err, err_cap) == 0;
}

static int m_bms_get_param(const json_node_t *params, const json_node_t *id,
                           sbuf_t *sb)
{
    const json_node_t *names = json_obj_get(params, "names");
    if (names == NULL || names->type != JSON_ARR || json_arr_len(names) == 0) {
        put_error(sb, id, RPC_ERR_PARAMS, "params.names 必须是非空数组", NULL);
        return 0;
    }
    char err[64] = {0};
    put_result_open(sb, id);
    sb_puts(sb, "{\"values\":{");
    bool first = true;
    for (const json_node_t *n = names->child; n != NULL; n = n->next) {
        const char *name = json_str(n, NULL);
        if (name == NULL) {
            put_error(sb, id, RPC_ERR_PARAMS, "names 元素必须是字符串", NULL);
            return 0;
        }
        const rpc_param_t *p = find_param(name, strlen(name));
        if (p == NULL) {
            put_error(sb, id, RPC_ERR_UNKNOWN_PARAM, "未知参数名", name);
            return 0;
        }
        uint16_t regs[2] = {0, 0};
        if (!read_param_regs(p, regs, err, sizeof(err))) {
            put_error(sb, id, RPC_ERR_MODBUS, "读取 BMS 参数失败", err);
            return 0;
        }
        if (!first) {
            sb_puts(sb, ",");
        }
        first = false;
        sb_json_quoted(sb, name);
        sb_printf(sb, ":%ld", (long)decode_value(p, regs));
    }
    sb_puts(sb, "}}}");
    return 0;
}

static int m_bms_list_params(const json_node_t *params, const json_node_t *id, sbuf_t *sb)
{
    (void)params;
    put_result_open(sb, id);
    sb_puts(sb, "{\"params\":[");
    for (int i = 0; i < PARAM_COUNT; i++) {
        const rpc_param_t *p = &PARAMS[i];
        if (i > 0) {
            sb_puts(sb, ",");
        }
        sb_puts(sb, "{\"name\":");
        sb_json_quoted(sb, p->name);
        sb_printf(sb, ",\"register\":%u,\"type\":", (unsigned)p->reg);
        sb_json_quoted(sb, type_name(p->type));
        sb_printf(sb, ",\"length\":%u,\"access\":\"%s\",\"unit\":",
                  (unsigned)p->len, p->writable ? "RW" : "R");
        sb_json_quoted(sb, p->unit);
        sb_puts(sb, ",\"description\":");
        sb_json_quoted(sb, p->desc);
        if (p->type == PT_BIT) {
            sb_printf(sb, ",\"bit\":%u", (unsigned)p->bit);
        }
        sb_puts(sb, "}");
    }
    sb_puts(sb, "]}");
    return 0;
}

static int m_bms_set_param(const json_node_t *params, const json_node_t *id, sbuf_t *sb)
{
    const json_node_t *values = json_obj_get(params, "values");
    if (values == NULL || values->type != JSON_OBJ || values->child == NULL) {
        put_error(sb, id, RPC_ERR_PARAMS, "params.values 必须是非空 {name=value} 对象", NULL);
        return 0;
    }
    char err[64] = {0};
    sbuf_t applied;
    char applied_buf[256];
    sb_init(&applied, applied_buf, sizeof(applied_buf));

    for (const json_node_t *v = values->child; v != NULL; v = v->next) {
        const rpc_param_t *p = find_param(v->key, v->key_len);
        if (p == NULL) {
            put_error(sb, id, RPC_ERR_UNKNOWN_PARAM, "未知参数名", v->key);
            return 0;
        }
        if (!p->writable) {
            put_error(sb, id, RPC_ERR_READ_ONLY, "参数只读", p->name);
            return 0;
        }
        long value = 0;
        if (!json_int(v, &value)) {
            put_error(sb, id, RPC_ERR_PARAMS, "参数值必须是整数", p->name);
            return 0;
        }
        uint16_t regs[2] = {0, 0};
        if (p->type == PT_BIT) {
            /* 位域需要先读回原值再改写 */
            if (!read_param_regs(p, regs, err, sizeof(err))) {
                put_error(sb, id, RPC_ERR_MODBUS, "读取位域失败", err);
                return 0;
            }
        }
        if (encode_value(p, value, regs) != 0) {
            put_error(sb, id, RPC_ERR_PARAMS, "参数值超出范围", p->name);
            return 0;
        }
        if (s_deps.write_regs == NULL ||
            s_deps.write_regs(p->reg, regs, p->len, err, sizeof(err)) != 0) {
            put_error(sb, id, RPC_ERR_MODBUS, "写入 BMS 参数失败", err);
            return 0;
        }
        if (applied.len > 0) {
            sb_puts(&applied, ",");
        }
        sb_json_quoted(&applied, p->name);
        sb_printf(&applied, ":%ld", value);
    }

    put_result_open(sb, id);
    sb_puts(sb, "{\"applied\":{");
    sb_puts(sb, applied_buf);
    sb_puts(sb, "},\"restartRequired\":false}");
    return 0;
}

static int m_bms_get_state(const json_node_t *params, const json_node_t *id, sbuf_t *sb)
{
    (void)params;
    bms_snapshot_t snap;
    if (s_deps.get_snapshot == NULL || !s_deps.get_snapshot(&snap)) {
        put_error(sb, id, RPC_ERR_UNAVAILABLE, "设备当前没有 BMS 数据", NULL);
        return 0;
    }
    rpc_device_info_t info;
    memset(&info, 0, sizeof(info));
    if (s_deps.get_info != NULL) {
        s_deps.get_info(&info);
    }

    put_result_open(sb, id);
    sb_printf(sb, "{\"time\":%u,\"temp1\":%d,\"temp2\":%d,\"tempMos\":%d,"
                  "\"balanCurrent\":%u,\"batVol\":%u,\"batCurrent\":%ld,"
                  "\"socCycleCap\":%u,\"socCapRemain\":%ld,\"soc\":%u,\"cellCount\":%u,"
                  "\"fastCharging\":%u,\"protection\":[%u,%u,%u,%u,%u],\"cellVols\":[",
             (unsigned)info.epoch, snap.temp1_tenths, snap.temp2_tenths,
             snap.board_temp_tenths, (unsigned)snap.balan_current_ma,
             (unsigned)snap.total_voltage_mv, (long)snap.charge_current_ma,
             (unsigned)snap.cycle_capacity_mah, (long)snap.capacity_remain_mah,
             (unsigned)snap.soc, (unsigned)snap.cell_count,
             (unsigned)snap.fast_charging, snap.protection[0], snap.protection[1],
             snap.protection[2], snap.protection[3], snap.protection[4]);
    for (uint8_t i = 0; i < snap.cell_count && i < BMS_MAX_CELLS; i++) {
        sb_printf(sb, "%s%u", i ? "," : "", (unsigned)snap.cell_mv[i]);
    }
    sb_puts(sb, "]}");
    return 0;
}

static int m_bms_read_registers(const json_node_t *params, const json_node_t *id, sbuf_t *sb)
{
    long start = 0, count = 0;
    if (!json_int(json_obj_get(params, "start"), &start) ||
        !json_int(json_obj_get(params, "count"), &count) ||
        start < 0 || start > 0xFFFF || count < 1 || count > 64) {
        put_error(sb, id, RPC_ERR_PARAMS, "params 需要 start(0..65535) 与 count(1..64)", NULL);
        return 0;
    }
    if (s_deps.read_regs == NULL) {
        put_error(sb, id, RPC_ERR_UNAVAILABLE, "当前采集通道不支持原始寄存器访问", NULL);
        return 0;
    }
    uint16_t regs[64];
    char err[64] = {0};
    if (s_deps.read_regs((uint16_t)start, (uint16_t)count, regs, err, sizeof(err)) != 0) {
        put_error(sb, id, RPC_ERR_MODBUS, "Modbus 读取失败", err);
        return 0;
    }
    put_result_open(sb, id);
    sb_printf(sb, "{\"start\":%ld,\"count\":%ld,\"registers\":[", start, count);
    for (long i = 0; i < count; i++) {
        sb_printf(sb, "%s%u", i ? "," : "", (unsigned)regs[i]);
    }
    sb_puts(sb, "]}");
    return 0;
}

static int m_bms_write_registers(const json_node_t *params, const json_node_t *id, sbuf_t *sb)
{
    long start = 0;
    const json_node_t *values = json_obj_get(params, "values");
    size_t count = json_arr_len(values);
    if (!json_int(json_obj_get(params, "start"), &start) || start < 0 || start > 0xFFFF ||
        count == 0 || count > 32) {
        put_error(sb, id, RPC_ERR_PARAMS, "params 需要 start(0..65535) 与 values(1..32 个整数)", NULL);
        return 0;
    }
    if (s_deps.write_regs == NULL) {
        put_error(sb, id, RPC_ERR_UNAVAILABLE, "当前采集通道不支持原始寄存器写入", NULL);
        return 0;
    }
    uint16_t regs[32];
    for (size_t i = 0; i < count; i++) {
        long v = 0;
        if (!json_int(json_arr_get(values, i), &v) || v < 0 || v > 0xFFFF) {
            put_error(sb, id, RPC_ERR_PARAMS, "values 元素必须是 0..65535 整数", NULL);
            return 0;
        }
        regs[i] = (uint16_t)v;
    }
    char err[64] = {0};
    if (s_deps.write_regs((uint16_t)start, regs, count, err, sizeof(err)) != 0) {
        put_error(sb, id, RPC_ERR_MODBUS, "Modbus 写入失败", err);
        return 0;
    }
    put_result_open(sb, id);
    sb_printf(sb, "{\"start\":%ld,\"count\":%u}", start, (unsigned)count);
    return 0;
}

static int m_bms_action(const json_node_t *params, const json_node_t *id, sbuf_t *sb)
{
    const char *action = json_str(json_obj_get(params, "action"), NULL);
    if (action == NULL) {
        put_error(sb, id, RPC_ERR_PARAMS, "params.action 必须是字符串", NULL);
        return 0;
    }
    /* 框架已就位：具体动作（关机/应急/校准/对时等）按需在此登记，
     * 目前统一返回 -32004，避免误写保护板。 */
    put_error(sb, id, RPC_ERR_UNAVAILABLE, "该动作尚未实现", action);
    return 0;
}

/* ---------------- agent.* ---------------- */

static int m_agent_ping(const json_node_t *params, const json_node_t *id, sbuf_t *sb)
{
    (void)params;
    rpc_device_info_t info;
    memset(&info, 0, sizeof(info));
    if (s_deps.get_info != NULL) {
        s_deps.get_info(&info);
    }
    put_result_open(sb, id);
    sb_puts(sb, "{\"deviceId\":");
    sb_json_quoted(sb, info.device_id);
    sb_printf(sb, ",\"time\":%u,\"uptime\":%u,\"version\":", (unsigned)info.epoch,
              (unsigned)info.uptime_s);
    sb_json_quoted(sb, info.version);
    sb_puts(sb, ",\"mode\":");
    sb_json_quoted(sb, info.mode);
    sb_printf(sb, ",\"mqttConnected\":%s}", info.mqtt_connected ? "true" : "false");
    return 0;
}

static int m_agent_get_config(const json_node_t *params, const json_node_t *id, sbuf_t *sb)
{
    (void)params;
    char cfg[768];
    cfg[0] = '\0';
    if (s_deps.get_config_json == NULL || s_deps.get_config_json(cfg, sizeof(cfg)) != 0) {
        put_error(sb, id, RPC_ERR_UNAVAILABLE, "配置不可用", NULL);
        return 0;
    }
    put_result_open(sb, id);
    sb_puts(sb, "{\"config\":");
    sb_puts(sb, cfg);
    sb_puts(sb, "}");
    return 0;
}

static int m_agent_set_config(const json_node_t *params, const json_node_t *id, sbuf_t *sb)
{
    if (params == NULL || params->type != JSON_OBJ) {
        put_error(sb, id, RPC_ERR_PARAMS, "params 必须是对象", NULL);
        return 0;
    }
    if (s_deps.set_config_json == NULL) {
        put_error(sb, id, RPC_ERR_UNAVAILABLE, "运行期配置修改不可用", NULL);
        return 0;
    }
    char params_json[768];
    sbuf_t tmp;
    sb_init(&tmp, params_json, sizeof(params_json));
    json_serialize(params, &tmp);
    if (tmp.overflow) {
        put_error(sb, id, RPC_ERR_PARAMS, "配置内容过长", NULL);
        return 0;
    }

    char err[64] = {0};
    int rc = s_deps.set_config_json(params_json, strlen(params_json), err, sizeof(err));
    if (rc < 0) {
        put_error(sb, id, RPC_ERR_PARAMS, "配置修改失败", err);
        return 0;
    }
    /* 变更后回读一次，返回最新配置（与 mqttagent 行为一致） */
    char cfg[768];
    cfg[0] = '\0';
    if (s_deps.get_config_json != NULL) {
        s_deps.get_config_json(cfg, sizeof(cfg));
    }
    put_result_open(sb, id);
    sb_puts(sb, "{\"config\":");
    sb_puts(sb, cfg[0] ? cfg : "{}");
    sb_puts(sb, rc == 1 ? ",\"restartRequired\":true}" : ",\"restartRequired\":false}");
    return 0;
}

static int m_agent_reboot(const json_node_t *params, const json_node_t *id, sbuf_t *sb)
{
    bool confirm = false;
    if (!json_bool(json_obj_get(params, "confirm"), &confirm) || !confirm) {
        put_error(sb, id, RPC_ERR_PARAMS, "params.confirm 必须为 true", NULL);
        return 0;
    }
    put_result_open(sb, id);
    sb_puts(sb, "{\"rebooting\":true}");
    if (s_deps.reboot != NULL) {
        s_deps.reboot();
    }
    return 0;
}

/* ---------------- 分发 ---------------- */

typedef int (*rpc_method_fn)(const json_node_t *params, const json_node_t *id, sbuf_t *sb);

typedef struct {
    const char *method;
    rpc_method_fn fn;
} rpc_method_t;

static const rpc_method_t METHODS[] = {
    {"bms.getParam", m_bms_get_param},
    {"bms.listParams", m_bms_list_params},
    {"bms.setParam", m_bms_set_param},
    {"bms.getState", m_bms_get_state},
    {"bms.readRegisters", m_bms_read_registers},
    {"bms.writeRegisters", m_bms_write_registers},
    {"bms.action", m_bms_action},
    {"agent.ping", m_agent_ping},
    {"agent.getConfig", m_agent_get_config},
    {"agent.setConfig", m_agent_set_config},
    {"agent.reboot", m_agent_reboot},
};

#define METHOD_COUNT ((int)(sizeof(METHODS) / sizeof(METHODS[0])))

void rpc_server_init(const rpc_deps_t *deps)
{
    if (deps != NULL) {
        s_deps = *deps;
    }
    s_ready = true;
}

int rpc_server_handle(const char *payload, size_t len, char *out, size_t out_cap)
{
    static char buf[2048];
    static json_pool_t pool;

    if (!s_ready || payload == NULL || out == NULL || out_cap == 0) {
        return -1;
    }
    if (len >= sizeof(buf)) {
        len = sizeof(buf) - 1;
    }
    memcpy(buf, payload, len);
    buf[len] = '\0';

    sbuf_t sb;
    sb_init(&sb, out, out_cap);

    const char *err = NULL;
    json_node_t *req = json_parse(buf, len, &pool, &err);
    if (req == NULL) {
        ESP_LOGW(TAG, "JSON-RPC 解析失败: %s", err ? err : "?");
        put_error(&sb, NULL, RPC_ERR_PARSE, "parse error", err);
        return sb.overflow ? -1 : (int)sb.len;
    }
    if (req->type != JSON_OBJ) {
        put_error(&sb, NULL, RPC_ERR_REQUEST, "invalid request", NULL);
        return sb.overflow ? -1 : (int)sb.len;
    }

    const json_node_t *id = json_obj_get(req, "id");
    const char *method = json_str(json_obj_get(req, "method"), NULL);
    const json_node_t *version = json_obj_get(req, "jsonrpc");
    if (method == NULL || version == NULL || version->type != JSON_STR ||
        strcmp(version->str, "2.0") != 0) {
        put_error(&sb, id, RPC_ERR_REQUEST, "invalid request", NULL);
        return sb.overflow ? -1 : (int)sb.len;
    }
    if (id == NULL) {
        /* 通知：不回复 */
        ESP_LOGI(TAG, "RPC 通知 %s（无需回复）", method);
        return 0;
    }

    for (int i = 0; i < METHOD_COUNT; i++) {
        if (strcmp(METHODS[i].method, method) == 0) {
            const json_node_t *params = json_obj_get(req, "params");
            METHODS[i].fn(params, id, &sb);
            ESP_LOGI(TAG, "RPC %s -> %u 字节", method, (unsigned)sb.len);
            return sb.overflow ? -1 : (int)sb.len;
        }
    }
    put_error(&sb, id, RPC_ERR_METHOD, "method not found", method);
    return sb.overflow ? -1 : (int)sb.len;
}

int rpc_server_device_state_json(char *out, size_t cap)
{
    rpc_device_info_t info;
    memset(&info, 0, sizeof(info));
    if (s_deps.get_info != NULL) {
        s_deps.get_info(&info);
    }
    char cfg[768];
    cfg[0] = '\0';
    if (s_deps.get_config_json != NULL) {
        s_deps.get_config_json(cfg, sizeof(cfg));
    }

    sbuf_t sb;
    sb_init(&sb, out, cap);
    sb_puts(&sb, "{\"device\":{\"id\":");
    sb_json_quoted(&sb, info.device_id);
    sb_printf(&sb, ",\"version\":");
    sb_json_quoted(&sb, info.version);
    sb_printf(&sb, ",\"mode\":");
    sb_json_quoted(&sb, info.mode);
    sb_printf(&sb, ",\"uptimeS\":%u,\"epoch\":%u,\"mqttConnected\":%s}",
              (unsigned)info.uptime_s, (unsigned)info.epoch,
              info.mqtt_connected ? "true" : "false");
    sb_puts(&sb, ",\"config\":");
    sb_puts(&sb, cfg[0] ? cfg : "{}");
    sb_puts(&sb, "}");
    return sb.overflow ? -1 : (int)sb.len;
}

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* JSON-RPC 2.0 服务端框架（移植自 mqttagent/air780e/rpc.lua + docs/rpc.md）。
 * 同一套分发同时服务两条通道：
 *   - MQTT：<prefix>/<device>/call → 回复 <prefix>/<device>/reply（QoS1）
 *   - Web： POST /api/rpc
 * 错误码与 mqttagent 保持一致（见下面 RPC_ERR_*）。 */

#define RPC_RESPONSE_MAX 6144  /* bms.listParams 全目录约 5KB */
#define RPC_DEVICE_ID_LEN 24
#define RPC_VERSION_LEN   16

#define RPC_ERR_PARSE         -32700
#define RPC_ERR_REQUEST       -32600
#define RPC_ERR_METHOD        -32601
#define RPC_ERR_PARAMS        -32602
#define RPC_ERR_MODBUS        -32000
#define RPC_ERR_UNKNOWN_PARAM -32001
#define RPC_ERR_READ_ONLY     -32002
#define RPC_ERR_RESTART       -32003
#define RPC_ERR_UNAVAILABLE   -32004

typedef struct {
    char device_id[RPC_DEVICE_ID_LEN];
    char version[RPC_VERSION_LEN];
    char mode[8];          /* uart / ble */
    uint32_t uptime_s;
    uint32_t epoch;        /* 0 = 未对时 */
    bool mqtt_connected;
} rpc_device_info_t;

/* 上层注入的能力（main.c 装配；BLE 模式没有原始 485 通道时相应回调可为 NULL） */
typedef struct {
    void (*get_info)(rpc_device_info_t *out);
    int (*get_config_json)(char *out, size_t cap);                    /* 配置对象 JSON */
    /* 应用配置：0 = 已即时生效；1 = 已保存但需重启；-1 = 失败（err 说明原因） */
    int (*set_config_json)(const char *json, size_t len,
                           char *err, size_t err_cap);
    void (*reboot)(void);
    bool (*get_snapshot)(void *snapshot);                             /* bms_snapshot_t * */
    int (*read_regs)(uint16_t start, uint16_t count, uint16_t *out,
                     char *err, size_t err_cap);
    int (*write_regs)(uint16_t start, const uint16_t *vals, size_t count,
                      char *err, size_t err_cap);
} rpc_deps_t;

void rpc_server_init(const rpc_deps_t *deps);

/* 处理一条 JSON-RPC 请求（UTF-8 JSON 文本）。
 * 返回写入 out 的响应长度；0 表示这是通知（无需回复/无响应）；<0 表示内部错误。 */
int rpc_server_handle(const char *payload, size_t len, char *out, size_t out_cap);

/* 生成 Web 用的结构化状态/配置（供 /api/state 使用）。 */
int rpc_server_device_state_json(char *out, size_t cap);

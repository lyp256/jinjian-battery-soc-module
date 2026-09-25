/* rpc_server（JSON-RPC 2.0 框架）宿主机冒烟测试：
 *   gcc -I main -I tools/stub tools/rpc_test.c main/rpc_server.c main/json_util.c -o /tmp/rpc_test
 * 覆盖：agent.ping / bms.getState / bms.listParams / bms.getParam / bms.setParam /
 *       未知方法(-32601) / 解析错误(-32700) / 非法请求(-32600) / 通知(无回复)。 */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "../main/bms_interface.h"
#include "../main/rpc_server.h"

static int s_writes;

static void get_info(rpc_device_info_t *out)
{
    memset(out, 0, sizeof(*out));
    snprintf(out->device_id, sizeof(out->device_id), "TESTDEVICE");
    snprintf(out->version, sizeof(out->version), "1.0.0");
    snprintf(out->mode, sizeof(out->mode), "uart");
    out->uptime_s = 1234;
    out->epoch = 1800000000;
    out->mqtt_connected = true;
}

static int get_config_json(char *out, size_t cap)
{
    snprintf(out, cap, "{\"transport\":\"uart\",\"sampleIntervalMs\":1000}");
    return 0;
}

static int set_config_json(const char *json, size_t len, char *err, size_t err_cap)
{
    (void)json;
    (void)len;
    (void)err;
    (void)err_cap;
    return 0;
}

static bool get_snapshot(void *snapshot)
{
    bms_snapshot_t *s = (bms_snapshot_t *)snapshot;
    memset(s, 0, sizeof(*s));
    s->have_data = true;
    s->fresh = true;
    s->soc = 88;
    s->total_voltage_mv = 52310;
    s->charge_current_ma = -1230;
    s->cell_count = 3;
    s->cell_mv[0] = 3300;
    s->cell_mv[1] = 3301;
    s->cell_mv[2] = 3302;
    s->temp1_tenths = 250;
    s->temp2_tenths = 248;
    s->board_temp_tenths = 300;
    s->cycle_capacity_mah = 100000;
    s->capacity_remain_mah = 90000;
    return true;
}

static int read_regs(uint16_t start, uint16_t count, uint16_t *out, char *err, size_t err_cap)
{
    (void)err;
    (void)err_cap;
    if (start == 0x1290) { /* BatVol = UINT32 52310 mV */
        out[0] = 0;
        out[1] = 52310;
        return 0;
    }
    if (start == 0x1114) { /* ControlFlags 位域原值 */
        out[0] = 0x0040;
        return 0;
    }
    for (uint16_t i = 0; i < count; i++) {
        out[i] = (uint16_t)(start + i);
    }
    return 0;
}

static int write_regs(uint16_t start, const uint16_t *vals, size_t count,
                      char *err, size_t err_cap)
{
    (void)start;
    (void)vals;
    (void)err;
    (void)err_cap;
    s_writes += (int)count;
    return 0;
}

static int failures;

#define RPC_CALL(json) rpc_server_handle((json), strlen(json), out, sizeof(out))

static void check(const char *name, const char *resp, const char *expect_substr, int want_len_nonzero)
{
    bool ok = true;
    if (want_len_nonzero && (resp == NULL || resp[0] == '\0')) {
        ok = false;
    }
    if (expect_substr != NULL && (resp == NULL || strstr(resp, expect_substr) == NULL)) {
        ok = false;
    }
    printf("%-52s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) {
        printf("    resp: %s\n", resp ? resp : "(null)");
        failures++;
    }
}

int main(void)
{
    static char out[RPC_RESPONSE_MAX];
    rpc_deps_t deps = {
        .get_info = get_info,
        .get_config_json = get_config_json,
        .set_config_json = set_config_json,
        .reboot = NULL,
        .get_snapshot = get_snapshot,
        .read_regs = read_regs,
        .write_regs = write_regs,
    };
    rpc_server_init(&deps);

    int n = RPC_CALL("{\"jsonrpc\":\"2.0\",\"id\":\"aGVsbG8h\",\"method\":\"agent.ping\"}");
    out[n > 0 ? n : 0] = '\0';
    check("agent.ping", out, "\"deviceId\":\"TESTDEVICE\"", 1);

    n = RPC_CALL("{\"jsonrpc\":\"2.0\",\"id\":\"x\",\"method\":\"bms.getState\"}");
    out[n > 0 ? n : 0] = '\0';
    check("bms.getState", out, "\"batVol\":52310", 1);

    n = RPC_CALL("{\"jsonrpc\":\"2.0\",\"id\":\"x\",\"method\":\"bms.listParams\"}");
    out[n > 0 ? n : 0] = '\0';
    check("bms.listParams", out, "\"name\":\"VolCellUV\"", 1);

    n = RPC_CALL("{\"jsonrpc\":\"2.0\",\"id\":\"x\",\"method\":\"bms.getParam\","
                 "\"params\":{\"names\":[\"BatVol\"]}}");
    out[n > 0 ? n : 0] = '\0';
    check("bms.getParam(BatVol)", out, "\"BatVol\":52310", 1);

    n = RPC_CALL("{\"jsonrpc\":\"2.0\",\"id\":\"x\",\"method\":\"bms.setParam\","
                 "\"params\":{\"values\":{\"BatChargeEN\":1}}}");
    out[n > 0 ? n : 0] = '\0';
    check("bms.setParam(BatChargeEN)", out, "\"applied\":{\"BatChargeEN\":1}", 1);

    n = RPC_CALL("{\"jsonrpc\":\"2.0\",\"id\":\"x\",\"method\":\"no.such\"}");
    out[n > 0 ? n : 0] = '\0';
    check("unknown method -> -32601", out, "\"code\":-32601", 1);

    n = RPC_CALL("{\"jsonrpc\":\"2.0\",\"id\":\"x\",\"method\":\"bms.getParam\","
                 "\"params\":{\"names\":[\"NoSuchParam\"]}}");
    out[n > 0 ? n : 0] = '\0';
    check("unknown param -> -32001", out, "\"code\":-32001", 1);

    n = RPC_CALL("{not json");
    out[n > 0 ? n : 0] = '\0';
    check("broken json -> -32700", out, "\"code\":-32700", 1);

    n = RPC_CALL("{\"id\":\"x\",\"method\":\"agent.ping\"}");
    out[n > 0 ? n : 0] = '\0';
    check("missing jsonrpc -> -32600", out, "\"code\":-32600", 1);

    n = RPC_CALL("{\"jsonrpc\":\"2.0\",\"method\":\"agent.ping\"}");
    check("notification (no id) -> no reply", n == 0 ? "" : out, NULL, 0);
    if (n != 0) {
        failures++;
        printf("    expected n=0, got %d\n", n);
    }

    printf("\n%s (%d failures, writes=%d)\n", failures == 0 ? "ALL RPC TESTS PASSED" : "RPC TESTS FAILED",
           failures, s_writes);
    return failures == 0 ? 0 : 1;
}

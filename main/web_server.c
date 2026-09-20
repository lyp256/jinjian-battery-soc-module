#include "web_server.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bms_interface.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_log_level.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "jk_bms_ble.h"
#include "log_stream.h"
#include "ota_update.h"

static const char *TAG = "web";
static httpd_handle_t s_server;

static void json_escape(const char *in, char *out, size_t cap)
{
    size_t o = 0;
    for (size_t i = 0; in[i] && o + 2 < cap; i++) {
        char c = in[i];
        switch (c) {
        case '"':
            out[o++] = '\\';
            out[o++] = '"';
            break;
        case '\\':
            out[o++] = '\\';
            out[o++] = '\\';
            break;
        case '\n':
            out[o++] = '\\';
            out[o++] = 'n';
            break;
        case '\r':
            out[o++] = '\\';
            out[o++] = 'r';
            break;
        case '\t':
            out[o++] = '\\';
            out[o++] = 't';
            break;
        default:
            out[o++] = c;
            break;
        }
    }
    out[o] = '\0';
}

static bool json_get_string(const char *json, const char *key, char *out, size_t cap)
{
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (p == NULL) {
        return false;
    }
    p += strlen(pat);
    while (*p && *p != ':') {
        p++;
    }
    if (*p++ != ':') {
        return false;
    }
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }
    if (*p != '"') {
        return false;
    }
    p++;
    size_t o = 0;
    while (*p && *p != '"' && o + 1 < cap) {
        if (*p == '\\' && p[1]) {
            p++;
        }
        out[o++] = *p++;
    }
    out[o] = '\0';
    return true;
}

static bool json_get_number(const char *json, const char *key, double *out)
{
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (p == NULL) {
        return false;
    }
    p += strlen(pat);
    while (*p && *p != ':') {
        p++;
    }
    if (*p++ != ':') {
        return false;
    }
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }
    char *end = NULL;
    double v = strtod(p, &end);
    if (end == p) {
        return false;
    }
    *out = v;
    return true;
}

static void copy_str(char *dst, size_t size, const char *src)
{
    if (size == 0) {
        return;
    }
    strncpy(dst, src, size - 1);
    dst[size - 1] = '\0';
}

static void reboot_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
    vTaskDelete(NULL);
}

static const char PAGE_HTML[] =
    "<!DOCTYPE html><html lang='zh-CN'><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Jinjian BMS 模块</title>"
    "<style>body{font-family:sans-serif;margin:0;background:#f3f5f7;color:#222}"
    "header{background:#0b5ed7;color:#fff;padding:14px 16px;font-size:18px}"
    "main{max-width:760px;margin:16px auto;padding:0 12px}"
    ".card{background:#fff;border-radius:10px;padding:14px;margin:12px 0;box-shadow:0 1px 4px #0002}"
    "table{width:100%;border-collapse:collapse}td,th{padding:6px;border-bottom:1px solid #eee;text-align:left}"
    "input,select{width:100%;padding:8px;margin:4px 0;box-sizing:border-box}"
    "button{background:#0b5ed7;color:#fff;border:0;padding:10px 18px;border-radius:8px;font-size:15px;margin:4px}"
    "#logs{white-space:pre-wrap;background:#111;color:#8f8;font:12px monospace;padding:10px;max-height:260px;overflow:auto}"
    ".ok{color:#080}.bad{color:#c00}</style></head><body>"
    "<header>金箭 SOC 模块 · 配置与调试</header><main>"
    "<div class='card'><h3>电池状态</h3><table id='status'></table></div>"
    "<div class='card'><h3>配置</h3><form id='cfg'>"
    "<label>采集通道</label><select name='transport'><option value='uart'>UART (RS485)</option><option value='ble'>蓝牙 BLE</option></select>"
    "<label>热点 SSID</label><input name='apSsid' maxlength='32'>"
    "<label>热点密码</label><input name='apPassword' type='password' maxlength='63'>"
    "<label>极空蓝牙名称过滤</label><input name='bleName' maxlength='32'>"
    "<label>极空蓝牙 MAC</label><input name='bleAddr' placeholder='aa:bb:cc:dd:ee:ff' maxlength='17'>"
    "<label>蓝牙设备</label><select id='bleDevices'><option value=''>—</option></select>"
    "<button onclick='startBleScan()'>扫描</button><button onclick='useBleDevice()'>使用所选</button><span id='bleScanState'></span>"
    "<label>BLE 协议</label><select name='bleProtocol'><option value='2'>JK02 24S</option><option value='3'>JK02 32S</option></select>"
    "<label>BLE 轮询间隔(ms)</label><input name='blePollMs' type='number' min='1000' max='60000'>"
    "<label>BLE 扫描超时(ms)</label><input name='bleScanTimeoutMs' type='number' min='1000' max='120000'>"
    "<label>BLE 重连间隔(ms)</label><input name='bleReconnectMs' type='number' min='1000' max='120000'>"
    "<label>UART 轮询间隔(ms)</label><input name='uartPollMs' type='number' min='100' max='60000'>"
    "<br><button type='submit'>保存并重启</button></form></div>"
    "<div class='card'><h3>在线升级</h3>"
    "<input type='file' id='otaFile' accept='.bin'><br>"
    "<button onclick='doOta()'>上传并升级</button>"
    "<progress id='otaProgress' value='0' max='100' style='width:100%'></progress>"
    "<span id='otaState'></span></div>"
    "<div class='card'><h3>调试日志</h3>"
    "<label>日志级别</label><select id='logLevel'><option value=0>NONE</option><option value=1>ERROR</option>"
    "<option value=2>WARN</option><option value=3>INFO</option><option value=4>DEBUG</option><option value=5>VERBOSE</option></select>"
    "<button onclick='setLevel()'>应用</button><span id='logState'></span><div id='logs'></div></div>"
    "</main><script>"
    "async function j(u,o){const r=await fetch(u,o);return r.json()}"
    "async function loadStatus(){try{const d=await j('/api/status');let h='';"
    "h+='<tr><td>采集通道</td><td>'+d.driver+'</td></tr>';"
    "h+='<tr><td>数据有效</td><td>'+(d.haveData?'<span class=ok>是':'<span class=bad>否')+'</span></td></tr>';"
    "h+='<tr><td>总电压</td><td>'+d.voltage+' V</td></tr>';"
    "h+='<tr><td>SOC</td><td>'+d.soc+' %</td></tr>';"
    "h+='<tr><td>电流</td><td>'+d.current+' A</td></tr>';"
    "h+='<tr><td>串数</td><td>'+d.cellCount+'</td></tr>';"
    "h+='<tr><td>容量</td><td>'+d.capacity+' Ah</td></tr>';"
    "h+='<tr><td>温度</td><td>'+d.temp1+' / '+d.temp2+' ℃</td></tr>';"
    "h+='<tr><td>板温</td><td>'+d.boardTemp+' ℃</td></tr>';"
    "h+='<tr><td>循环</td><td>'+d.cycleCount+'</td></tr>';"
    "h+='<tr><td>PN</td><td>'+d.pn+'</td></tr>';"
    "h+='<tr><td>快充</td><td>'+(d.fastCharging?'是':'否')+'</td></tr>';"
    "h+='<tr><td>日志级别</td><td>'+d.logLevel+'</td></tr>';"
    "h+='<tr><td>运行时间</td><td>'+d.uptime+' s</td></tr>';"
    "h+='<tr><td>采集成功/失败</td><td>'+d.pollCount+' / '+d.pollFailures+'</td></tr>';"
    "document.getElementById('status').innerHTML=h}catch(e){}}"
    "async function loadCfg(){try{const d=await j('/api/config');"
    "const f=document.getElementById('cfg');f.transport.value=d.transport;"
    "f.apSsid.value=d.apSsid;f.apPassword.value=d.apPassword;"
    "f.bleName.value=d.bleName;f.bleAddr.value=d.bleAddr;"
    "f.bleProtocol.value=String(d.bleProtocol);f.blePollMs.value=d.blePollMs;"
    "f.bleScanTimeoutMs.value=d.bleScanTimeoutMs;f.bleReconnectMs.value=d.bleReconnectMs;"
    "f.uartPollMs.value=d.uartPollMs}catch(e){}}"
    "async function loadLevel(){try{const d=await j('/api/status');document.getElementById('logLevel').value=String(d.logLevel)}catch(e){}}"
    "async function startBleScan(){document.getElementById('bleScanState').textContent='扫描中…';await j('/api/ble/scan/start',{method:'POST'});pollBleResults()}"
    "async function pollBleResults(){const d=await j('/api/ble/scan/results');const sel=document.getElementById('bleDevices');"
    "sel.innerHTML=\"<option value=''>—</option>\";"
    "for(const x of d.devices){const o=document.createElement('option');o.value=x.mac;o.textContent=x.name+' ('+x.mac+')';sel.appendChild(o)}"
    "if(d.active){setTimeout(pollBleResults,1000)}else{document.getElementById('bleScanState').textContent='完成 ('+d.devices.length+')'}}"
    "function useBleDevice(){const sel=document.getElementById('bleDevices');if(!sel.value){alert('请先扫描并选择设备');return}"
    "const t=sel.selectedOptions[0].textContent;const i=t.lastIndexOf(' (');"
    "document.getElementById('bleName').value=i>0?t.slice(0,i):'';document.getElementById('bleAddr').value=sel.value}"
    "async function setLevel(){const lv=Number(document.getElementById('logLevel').value);"
    "const d=await j('/api/logs/level',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({level:lv})});"
    "alert(d.ok?'日志级别已应用':'设置失败')}"
    "function doOta(){const f=document.getElementById('otaFile').files[0];"
    "if(!f){alert('请选择固件');return}const st=document.getElementById('otaState'),pr=document.getElementById('otaProgress');"
    "st.textContent='上传中…';pr.value=0;const x=new XMLHttpRequest();x.open('POST','/api/ota',true);"
    "x.setRequestHeader('Content-Type','application/octet-stream');"
    "x.upload.onprogress=e=>{if(e.lengthComputable){pr.value=e.loaded/e.total*100;st.textContent=Math.round(e.loaded/e.total*100)+'%'}};"
    "x.onload=()=>{try{const r=JSON.parse(x.responseText);if(r.ok){st.textContent='升级成功，重启中';alert('升级成功，模块重启中')}else{st.textContent='失败';alert('升级失败: '+(r.error||'unknown'))}}catch(e){st.textContent='失败';alert('响应异常')}};"
    "x.onerror=()=>{st.textContent='失败';alert('上传失败')};x.send(f)}"
    "async function startLogs(){const el=document.getElementById('logs'),st=document.getElementById('logState');"
    "st.textContent='连接中…';try{const r=await fetch('http://'+location.hostname+':8080/api/logs/stream');"
    "if(!r.ok){st.textContent='连接失败('+r.status+')';return}st.textContent='实时';"
    "const rd=r.body.getReader(),dec=new TextDecoder();"
    "while(true){const {done,value}=await rd.read();if(done)break;"
    "let t=dec.decode(value,{stream:true});t=t.split('\\n').filter(x=>x.trim()).join('\\n')+'\\n';"
    "if(t){el.textContent+=t;if(el.textContent.length>8000)el.textContent=el.textContent.slice(-8000);el.scrollTop=el.scrollHeight}}"
    "st.textContent='已断开'}catch(e){st.textContent='连接错误'}setTimeout(startLogs,3000)}"
    "document.getElementById('cfg').onsubmit=async(e)=>{e.preventDefault();const f=e.target;"
    "const body={transport:f.transport.value,apSsid:f.apSsid.value,apPassword:f.apPassword.value,"
    "bleName:f.bleName.value,bleAddr:f.bleAddr.value,bleProtocol:Number(f.bleProtocol.value),"
    "blePollMs:Number(f.blePollMs.value),bleScanTimeoutMs:Number(f.bleScanTimeoutMs.value),"
    "bleReconnectMs:Number(f.bleReconnectMs.value),uartPollMs:Number(f.uartPollMs.value)};"
    "const d=await j('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});"
    "alert(d.ok?'已保存，正在重启':'保存失败')};"
    "loadStatus();loadCfg();loadLevel();startLogs();setInterval(loadStatus,2000);</script></body></html>";

static void schedule_reboot(void)
{
    xTaskCreate(&reboot_task, "web_reboot", 2048, NULL, 5, NULL);
}

static esp_err_t handle_root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, PAGE_HTML);
    return ESP_OK;
}

static esp_err_t handle_status(httpd_req_t *req)
{
    bms_snapshot_t snap;
    bool have = bms_manager_get_snapshot(&snap);

    char driver[64], pn[64];
    json_escape(bms_manager_name(), driver, sizeof(driver));
    json_escape(snap.pn, pn, sizeof(pn));

    char buf[4096];
    int o = snprintf(buf, sizeof(buf),
                     "{\"driver\":\"%s\",\"haveData\":%s,\"fresh\":%s,"
                     "\"voltage\":%.2f,\"soc\":%u,\"current\":%.2f,"
                     "\"cellCount\":%u,\"capacity\":%u,\"temp1\":%d,\"temp2\":%d,"
                     "\"boardTemp\":%d,\"cycleCount\":%u,\"pn\":\"%s\","
                     "\"fastCharging\":%s,\"logLevel\":%d,\"uptime\":%u,"
                     "\"pollCount\":%u,\"pollFailures\":%u,"
                     "\"logStreams\":%u,\"cells\":[",
                     driver,
                     (have && snap.have_data) ? "true" : "false",
                     (have && snap.fresh) ? "true" : "false",
                     (double)snap.total_voltage_raw / 100.0,
                     snap.soc,
                     (double)snap.charge_current_raw / 100.0,
                     snap.cell_count,
                     snap.capacity_ah,
                     snap.temp1,
                     snap.temp2,
                     snap.board_temp,
                     snap.cycle_count,
                     pn,
                     snap.fast_charging != 0 ? "true" : "false",
                     (int)esp_log_level_get("*"),
                     (unsigned)(esp_timer_get_time() / 1000000),
                     (unsigned)snap.poll_count,
                     (unsigned)snap.poll_failures,
                     (unsigned)log_stream_active_count());
    for (int i = 0; i < BMS_MAX_CELLS; i++) {
        int n = snprintf(buf + o, sizeof(buf) - o, "%s%u", i ? "," : "", snap.cell_mv[i]);
        if (n < 0 || (size_t)n >= sizeof(buf) - o) {
            break;
        }
        o += n;
    }
    snprintf(buf + o, sizeof(buf) - o, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t handle_config_get(httpd_req_t *req)
{
    app_config_t cfg;
    app_config_get(&cfg);
    char t[16], s[40], p[96], bn[64], ba[64];
    json_escape(cfg.bms_transport, t, sizeof(t));
    json_escape(cfg.ap_ssid, s, sizeof(s));
    json_escape(cfg.ap_password, p, sizeof(p));
    json_escape(cfg.ble_target_name, bn, sizeof(bn));
    json_escape(cfg.ble_target_addr, ba, sizeof(ba));
    char buf[1024];
    snprintf(buf, sizeof(buf),
             "{\"transport\":\"%s\",\"apSsid\":\"%s\",\"apPassword\":\"%s\","
             "\"bleName\":\"%s\",\"bleAddr\":\"%s\",\"bleProtocol\":%d,"
             "\"blePollMs\":%u,\"bleScanTimeoutMs\":%u,\"bleReconnectMs\":%u,"
             "\"uartPollMs\":%u,\"logLevel\":%d}",
             t, s, p, bn, ba, cfg.ble_protocol,
             (unsigned)cfg.ble_poll_ms, (unsigned)cfg.ble_scan_timeout_ms,
             (unsigned)cfg.ble_reconnect_ms, (unsigned)cfg.jk_uart_poll_ms,
             cfg.log_level);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t handle_config_post(httpd_req_t *req)
{
    char buf[2048];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no body");
        return ESP_FAIL;
    }
    buf[len] = '\0';
    app_config_t cfg;
    app_config_get(&cfg);
    char tmp[128];
    double num;
    if (json_get_string(buf, "transport", tmp, sizeof(tmp))) {
        copy_str(cfg.bms_transport, sizeof(cfg.bms_transport), tmp);
    }
    if (json_get_string(buf, "apSsid", tmp, sizeof(tmp))) {
        copy_str(cfg.ap_ssid, sizeof(cfg.ap_ssid), tmp);
    }
    if (json_get_string(buf, "apPassword", tmp, sizeof(tmp))) {
        copy_str(cfg.ap_password, sizeof(cfg.ap_password), tmp);
    }
    if (json_get_string(buf, "bleName", tmp, sizeof(tmp))) {
        copy_str(cfg.ble_target_name, sizeof(cfg.ble_target_name), tmp);
    }
    if (json_get_string(buf, "bleAddr", tmp, sizeof(tmp))) {
        copy_str(cfg.ble_target_addr, sizeof(cfg.ble_target_addr), tmp);
    }
    if (json_get_number(buf, "bleProtocol", &num)) {
        cfg.ble_protocol = (int)num;
    }
    if (json_get_number(buf, "blePollMs", &num)) {
        cfg.ble_poll_ms = (uint32_t)num;
    }
    if (json_get_number(buf, "bleScanTimeoutMs", &num)) {
        cfg.ble_scan_timeout_ms = (uint32_t)num;
    }
    if (json_get_number(buf, "bleReconnectMs", &num)) {
        cfg.ble_reconnect_ms = (uint32_t)num;
    }
    if (json_get_number(buf, "uartPollMs", &num)) {
        cfg.jk_uart_poll_ms = (uint32_t)num;
    }
    if (json_get_number(buf, "logLevel", &num)) {
        cfg.log_level = (int)num;
    }

    if (strcmp(cfg.bms_transport, "uart") != 0 && strcmp(cfg.bms_transport, "ble") != 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "transport must be uart|ble");
        return ESP_FAIL;
    }
    if (strlen(cfg.ap_password) < 8) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "password too short");
        return ESP_FAIL;
    }
    if (cfg.log_level < 0 || cfg.log_level > 5) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "logLevel must be 0..5");
        return ESP_FAIL;
    }
    app_config_save(&cfg);
    app_config_apply_log_level();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"reboot\":true}");
    schedule_reboot();
    return ESP_OK;
}

static esp_err_t handle_reboot(httpd_req_t *req)
{
    httpd_resp_sendstr(req, "{\"ok\":true}");
    schedule_reboot();
    return ESP_OK;
}

static esp_err_t handle_log_level_post(httpd_req_t *req)
{
    char buf[256];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no body");
        return ESP_FAIL;
    }
    buf[len] = '\0';
    double num;
    if (!json_get_number(buf, "level", &num)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad level");
        return ESP_FAIL;
    }
    int level = (int)num;
    if (level < 0 || level > 5) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "level must be 0..5");
        return ESP_FAIL;
    }
    app_config_t cfg;
    app_config_get(&cfg);
    cfg.log_level = level;
    app_config_save(&cfg);
    app_config_apply_log_level();
    httpd_resp_set_type(req, "application/json");
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"ok\":true,\"level\":%d}", level);
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

static esp_err_t handle_ble_scan_start(httpd_req_t *req)
{
    jk_bms_ble_web_scan_start(5000);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t handle_ble_scan_results(httpd_req_t *req)
{
    char buf[4096];
    int o = snprintf(buf, sizeof(buf), "{\"active\":%s,\"devices\":[",
                     jk_bms_ble_web_scan_active() ? "true" : "false");
    size_t n = jk_bms_ble_web_scan_count();
    for (size_t i = 0; i < n && o < (int)sizeof(buf) - 128; i++) {
        char name[64], mac[32], en[128];
        if (jk_bms_ble_web_scan_get(i, name, sizeof(name), mac, sizeof(mac)) != 0) {
            continue;
        }
        json_escape(name, en, sizeof(en));
        int len = snprintf(buf + o, sizeof(buf) - o,
                           "%s{\"name\":\"%s\",\"mac\":\"%s\"}",
                           i ? "," : "", en, mac);
        if (len < 0 || o + len >= (int)sizeof(buf)) {
            break;
        }
        o += len;
    }
    snprintf(buf + o, sizeof(buf) - o, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t send_ota_error(httpd_req_t *req, const char *msg)
{
    httpd_resp_set_type(req, "application/json");
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"ok\":false,\"error\":\"%s\"}", msg);
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t handle_ota_post(httpd_req_t *req)
{
    if (ota_update_active()) {
        return send_ota_error(req, "ota busy");
    }
    if (req->content_len <= 0) {
        return send_ota_error(req, "empty body");
    }
    esp_err_t err = ota_update_start();
    if (err != ESP_OK) {
        return send_ota_error(req, "ota start failed");
    }

    char buf[4096];
    int remaining = req->content_len;
    while (remaining > 0) {
        size_t want = sizeof(buf) < (size_t)remaining ? sizeof(buf) : (size_t)remaining;
        int len = httpd_req_recv(req, buf, want);
        if (len <= 0) {
            ota_update_abort();
            return send_ota_error(req, "recv failed");
        }
        if (ota_update_write((const uint8_t *)buf, (size_t)len) != ESP_OK) {
            ota_update_abort();
            return send_ota_error(req, "ota write failed");
        }
        remaining -= len;
    }

    err = ota_update_finish();
    if (err != ESP_OK) {
        return send_ota_error(req, "ota finish failed");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"reboot\":true}");
    schedule_reboot();
    return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;
    (void)base;
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "station connected");
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "station disconnected");
    } else if (id == WIFI_EVENT_STA_START) {
        (void)data;
    }
}

static void start_softap(void)
{
    app_config_t cfg;
    app_config_get(&cfg);
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    wifi_config_t wc = {
        .ap = {
            .ssid = {0},
            .password = {0},
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    memcpy(wc.ap.ssid, cfg.ap_ssid, strlen(cfg.ap_ssid));
    wc.ap.ssid_len = strlen(cfg.ap_ssid);
    memcpy(wc.ap.password, cfg.ap_password, strlen(cfg.ap_password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "softAP started: %s", cfg.ap_ssid);
}

void web_server_start(void)
{
    start_softap();
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    cfg.recv_wait_timeout = 60;
    cfg.send_wait_timeout = 60;
    if (httpd_start(&s_server, &cfg) == ESP_OK) {
        httpd_uri_t uris[] = {
            {.uri = "/", .method = HTTP_GET, .handler = handle_root},
            {.uri = "/api/status", .method = HTTP_GET, .handler = handle_status},
            {.uri = "/api/config", .method = HTTP_GET, .handler = handle_config_get},
            {.uri = "/api/config", .method = HTTP_POST, .handler = handle_config_post},
            {.uri = "/api/reboot", .method = HTTP_POST, .handler = handle_reboot},
            {.uri = "/api/logs/level", .method = HTTP_POST, .handler = handle_log_level_post},
            {.uri = "/api/ota", .method = HTTP_POST, .handler = handle_ota_post},
            {.uri = "/api/ble/scan/start", .method = HTTP_POST, .handler = handle_ble_scan_start},
            {.uri = "/api/ble/scan/results", .method = HTTP_GET, .handler = handle_ble_scan_results},
        };
        for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
            httpd_register_uri_handler(s_server, &uris[i]);
        }
        ESP_LOGI(TAG, "http://192.168.4.1");
    }
}

void web_server_stop(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
}

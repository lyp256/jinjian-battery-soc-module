-- ntp_sync.lua
-- NTP 对时任务 (Air780E / LuatOS)
--
-- 行为:
--   1. 等待 main.lua 联网任务发布 "net_ready" 后开始;
--   2. 立即发起一次 SNTP 同步, 成功时发布 "ntp_synced" 事件;
--   3. 成功后每 3600 秒 (1 小时) 对时一次;
--   4. 失败后 10 秒重试, 成功后才回到小时周期。
--
-- 使用 LuatOS 内置 socket.sntp: 同步成功后系统自动更新 RTC,
-- 并发布 "NTP_UPDATE" 事件; 出错时发布 "NTP_ERROR"。
-- bms.lua 上报的 Time 字段为 Unix 秒, 应基于对时后的 os.time()。

-- 自定义 NTP 服务器 (可选), 默认使用 ntp.aliyun.com
-- local NTP_SERVER = "ntp.ntsc.ac.cn"
local NTP_SERVER = "ntp.aliyun.com"

local SYNC_TIMEOUT_MS = 5000           -- 单次同步等待上限
local HOURLY_INTERVAL_MS = 3600 * 1000 -- 成功后 1 小时对时一次
local RETRY_INTERVAL_MS = 10 * 1000    -- 失败后 10 秒重试

sys.taskInit(function()
    -- 等待联网：限时等待 + 周期提示，便于现场判断卡在哪一步
    -- （sys.waitUntil 挂起协程，等待期间不占 CPU，不影响其它任务）
    while not sys.waitUntil("net_ready", 60000) do
        log.warn("ntp_sync", "waiting for net_ready ...")
    end

    sys.subscribe("NTP_ERROR", function(err_info)
        log.error("ntp_sync", "ntp error", err_info or "unknown")
    end)

    while true do
        socket.sntp(NTP_SERVER)
        local ok = sys.waitUntil("NTP_UPDATE", SYNC_TIMEOUT_MS)
        if ok then
            log.info("ntp_sync", "time synced, unix", os.time())
            sys.publish("ntp_synced", os.time())
            sys.wait(HOURLY_INTERVAL_MS)
        else
            log.warn("ntp_sync", "sync failed, retry in", RETRY_INTERVAL_MS, "ms")
            sys.wait(RETRY_INTERVAL_MS)
        end
    end
end)

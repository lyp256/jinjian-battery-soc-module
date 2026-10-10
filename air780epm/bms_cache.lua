--[[
@module  bms_cache
@summary 共享 BMS 采样缓存（采集适配层 bms_data 写入，金箭从机 / 4G 上报只读）
@version 1.0
@date    2026.10.12
@usage
把「采集（jk_display / jk_modbus）」与「消费（jinjian_slave / bms_uplink）」解耦：

    jk_display / jk_modbus ──"JK_ROUND"──► bms_data.normalize ──push()──► bms_cache
                                                                              │  只读
                                          jinjian_slave (latest) ◄────────────┤
                                          bms_uplink   (since)  ◄────────────┘

固定 BMS_CACHE_SAMPLES（默认 32）条环形缓冲，按序号取模存整份规范样本：
  · 单条写入 O(1)、查询最多遍历 32 条，符合单线程不阻塞约定；
  · 按「原始采集时间戳」原样存储（同秒多条允许，不回退、不插值、不补零）；
  · 序号 seq 由本模块单调递增分配（push 时写回 sample.seq），是上报窗口的唯一游标，
    与时间戳无关，因此 NTP 跳变/同秒多条都不会让窗口重叠或丢样。

对外接口：
  bms_cache.push(sample)        入缓存（返回分配的 seq）；sample 必须停下不再修改
  bms_cache.latest()            最新一条样本（无样本返回 nil）
  bms_cache.count()             当前有效样本数（<= CAPACITY）
  bms_cache.history(from, to)   闭区间 [from, to] 的样本，按采集顺序返回
  bms_cache.since(seq)          返回 seq 之后的样本、最新 seq、被覆盖丢失的样本数
  bms_cache.stats()             统计信息（容量/条数/累计写入/覆盖丢失）
  bms_cache.reset()             清空（离线自测用）
  bms_cache.CAPACITY            容量（= config.BMS_CACHE_SAMPLES 或 32）
]]

local cfg = require "config"

local M = {}

local CAPACITY = tonumber(cfg.BMS_CACHE_SAMPLES) or 32
CAPACITY = math.floor(CAPACITY)
if CAPACITY < 1 then
    CAPACITY = 1
end
M.CAPACITY = CAPACITY

-- slots[i] = 样本表；slot_seq[i] = 该槽位的序号（nil = 空槽）
-- 槽位下标 = ((seq - 1) % CAPACITY) + 1：写入总落在下一次要覆盖的那个槽位
local slots = {}
local slot_seq = {}
local pushed = 0 -- 累计写入条数 = 最新样本序号
local count = 0  -- 当前有效条数

local stats = {
    capacity = CAPACITY,
    count = 0,
    pushed = 0,
    overwritten = 0, -- 被新样本覆盖而丢弃的旧样本数
    oldest_seq = 0,
    newest_seq = 0,
}

-- 当前最旧有效样本的序号（空缓存返回 0）
local function oldest_seq()
    if count == 0 then
        return 0
    end
    return pushed - count + 1
end

function M.push(sample)
    if type(sample) ~= "table" then
        return nil
    end
    pushed = pushed + 1
    sample.seq = pushed
    local idx = ((pushed - 1) % CAPACITY) + 1
    if slot_seq[idx] ~= nil then
        stats.overwritten = stats.overwritten + 1
    end
    slots[idx] = sample
    slot_seq[idx] = pushed
    if count < CAPACITY then
        count = count + 1
    end

    stats.count = count
    stats.pushed = pushed
    stats.oldest_seq = oldest_seq()
    stats.newest_seq = pushed
    return pushed
end

function M.latest()
    if count == 0 then
        return nil
    end
    return slots[((pushed - 1) % CAPACITY) + 1]
end

function M.count()
    return count
end

function M.history(from_ts, to_ts)
    local out = {}
    if count == 0 then
        return out
    end
    for seq = oldest_seq(), pushed do
        local idx = ((seq - 1) % CAPACITY) + 1
        local s = slots[idx]
        if s ~= nil and slot_seq[idx] == seq then
            local ts = s.ts or 0
            if (from_ts == nil or ts >= from_ts) and (to_ts == nil or ts <= to_ts) then
                out[#out + 1] = s
            end
        end
    end
    return out
end

-- 返回：seq 之后的样本（按采集顺序）、最新序号、因缓存覆盖而丢失的样本数
-- 游标 last 之后一条都没有时返回空表与 last（保持游标单调，不会重复上报）
function M.since(after_seq)
    after_seq = tonumber(after_seq) or 0
    local out = {}
    if count == 0 then
        return out, after_seq, 0
    end
    local first = oldest_seq()
    if pushed < after_seq then
        -- 缓存被重置/换实例而过期：不猜测丢了多少，游标保持单调
        return out, after_seq, 0
    end
    local lost = 0
    if after_seq < first - 1 then
        lost = (first - 1) - after_seq
    end
    for seq = math.max(after_seq + 1, first), pushed do
        local idx = ((seq - 1) % CAPACITY) + 1
        local s = slots[idx]
        if s ~= nil and slot_seq[idx] == seq then
            out[#out + 1] = s
        end
    end
    return out, pushed, lost
end

function M.stats()
    return {
        capacity = CAPACITY,
        count = count,
        pushed = pushed,
        overwritten = stats.overwritten,
        oldest_seq = oldest_seq(),
        newest_seq = pushed,
    }
end

-- 离线自测用：清空缓存与统计
function M.reset()
    slots = {}
    slot_seq = {}
    pushed = 0
    count = 0
    stats.count = 0
    stats.pushed = 0
    stats.overwritten = 0
    stats.oldest_seq = 0
    stats.newest_seq = 0
end

return M

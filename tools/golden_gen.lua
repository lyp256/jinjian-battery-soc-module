-- 用 mqttagent 的参考实现 (air780e/bms_codec.lua) 生成编码黄金向量。
-- 用法: lua tools/golden_gen.lua <mqttagent 根目录>
--   lua tools/golden_gen.lua F:/project/mqttagent
-- 输出 4 组十六进制串，供 tools/bms_hist_test.c 做字节级比对。
local root = arg[1] or "F:/project/mqttagent"
package.path = root .. "/air780e/?.lua;" .. package.path

local codec = require "bms_codec"

local function to_hex(s)
    return (s:gsub(".", function(c) return string.format("%02x", c:byte()) end))
end

local function build(samples, cells)
    local h = {
        count = #samples, cellCount = cells,
        temp1 = {}, temp2 = {}, tempMos = {}, balanCurrent = {},
        batVol = {}, batCurrent = {}, socCycleCap = {}, socCapRemain = {},
        time = {}, cellVols = {},
    }
    for i = 1, #samples do
        local s = samples[i]
        h.temp1[i] = s.temp1
        h.temp2[i] = s.temp2
        h.tempMos[i] = s.tempMos
        h.balanCurrent[i] = s.balanCurrent
        h.batVol[i] = s.batVol
        h.batCurrent[i] = s.batCurrent
        h.socCycleCap[i] = s.socCycleCap
        h.socCapRemain[i] = s.socCapRemain
        h.time[i] = s.time
        for j = 1, cells do
            h.cellVols[(i - 1) * cells + j] = s.cellVols[j]
        end
    end
    return h
end

-- golden1: 两样本三电芯（docs/bms.md 11.1）
local golden1 = {
    { temp1 = -125, temp2 = 231, tempMos = 450, balanCurrent = 12, batVol = 52340,
      batCurrent = -1800, socCycleCap = 123456, socCapRemain = 98765,
      time = 1700000000, cellVols = { 3300, 3301, 3302 } },
    { temp1 = -124, temp2 = 232, tempMos = 451, balanCurrent = 13, batVol = 52341,
      batCurrent = -1799, socCycleCap = 123457, socCapRemain = 98766,
      time = 1700000001, cellVols = { 3301, 3302, 3303 } },
}

-- golden2: 三样本两电芯全静态（docs/bms.md 11.2）
local st = { temp1 = 250, temp2 = 248, tempMos = 300, balanCurrent = 10, batVol = 52000,
             batCurrent = 0, socCycleCap = 100000, socCapRemain = 90000,
             time = 1700000000, cellVols = { 3300, 3301 } }
local golden2 = { st, st, st }

-- 30x20 运行态（与 Go makeActiveHistory / Lua active_samples 一致）
local function active(samples, cells, base_time)
    local arr = {}
    for i = 0, samples - 1 do
        local s = {
            temp1 = 250 + i % 3,
            temp2 = 248 + i % 2,
            tempMos = 300 + i % 4,
            balanCurrent = 10 + i % 2,
            batVol = 52000 + i,
            batCurrent = -20000 + (i * 3700) % 90001,
            socCycleCap = 100000 + i,
            socCapRemain = 90000 - i,
            time = base_time and (base_time + i) or 1800000000,
            cellVols = {},
        }
        for j = 0, cells - 1 do
            s.cellVols[j + 1] = 3300 + j * 2 + (i + j) % 11
        end
        arr[i + 1] = s
    end
    return arr
end

-- 输出 C 头文件（供 tools/bms_hist_test.c include），避免手抄长十六进制串
print("/* 由 tools/golden_gen.lua 生成，勿手改：mqttagent 参考实现的编码黄金向量 */")
print("#pragma once")
print("")
print('#define GOLDEN1 "' .. to_hex(codec.encode(build(golden1, 3))) .. '"')
print('#define GOLDEN2 "' .. to_hex(codec.encode(build(golden2, 2))) .. '"')
print('#define ACTIVE30 "' .. to_hex(codec.encode(build(active(30, 20, 1800000000), 20))) .. '"')
print('#define FAKE30 "' .. to_hex(codec.encode(build(active(30, 20, nil), 20))) .. '"')

-- bms_codec.lua
-- BMS 上报消息 MessageV1 压缩编码器 (Air780E / LuatOS, Lua 5.3+ 64 位整数)
-- 与上报服务端 (mqttagent/pkg/bms) 的线格式字节级兼容。
--
-- 线上格式（见 mqttagent/pkg/bms/bms.md）：
--   u8 0xAA | uvarint version=1 | uvarint(sampleCount-1) | uvarint header
--           | [modeTable] | 6 × uvarint 消息级字段 | 各列
--   header = (cellCount << 2) | flags
--            flags bit0 = 有 modeTable，bit1 = 电芯相邻压差布局，bit10 = 0 样本报文
--   消息级字段顺序 = SOCFullChargeCap, BaseTimestamp, ModuleUPTime, BMSUPTime,
--                    JinjianModbusRequestCount, JikongModbusRequestCount
--   列顺序 = Temp1, Temp2, TempMos, BalanCurrent, BatVol, BatCurrent,
--            SOCCycleCap(循环总容量), SOCCapRemain, Time(距 BaseTimestamp 秒数), 电芯列…
--
-- 用法:
--   local bms_codec = require "bms_codec"
--   local hist = { count=N, cellCount=C, temp1={…}, … , cellVols={…} }
--   local meta = { socFullChargeCap=…, baseTimestamp=…, moduleUpTime=…, bmsUpTime=…,
--                  jinjianModbusRequestCount=…, jikongModbusRequestCount=… }
--   local payload = bms_codec.encode(hist, meta)          -- N >= 1
--   local payload = bms_codec.encode_empty(meta)          -- 0 样本（读不到 BMS）
--   payload 为 string, 可直接作为 MQTT 二进制 payload 上报。
--
-- 可选第三参 on_slice: 编码是纯 CPU 计算（无 IO），每编码 SLICE_COLUMNS 列调用一次
--   该回调，供 LuatOS 侧的调用者主动让出（如 sys.wait(1)），避免长 CPU 段挤占串口收发。
--   不传（PC 端测试/离线工具）则完全同步执行，输出字节不变。
--
-- hist 为扁平结构, 避免嵌套 table 造成的 GC 压力:
--   hist.count      = 样本数 N
--   hist.cellCount  = 电芯数 C（同一批内固定）
--   hist.temp1[1..N]        0.1 摄氏度, int16
--   hist.temp2[1..N]
--   hist.tempMos[1..N]
--   hist.balanCurrent[1..N] mA, uint16
--   hist.batVol[1..N]       mV,  uint32
--   hist.batCurrent[1..N]   mA,  int32
--   hist.socCycleCap[1..N]  循环总容量 mAh, uint32
--   hist.socCapRemain[1..N] 剩余容量 mAh, uint32
--   hist.time[1..N]         距今秒数 0..255, uint8（真实时间 = BaseTimestamp - Time）
--   hist.cellVols[1..N*C]   mV,  uint16, 按样本连续排列

local M = {}

local floor = math.floor

-- 帧头与 flag（与 mqttagent/pkg/bms 一致）
local HEADER = 0xAA
local VERSION = 1
local FLAG_HAS_MODE_TABLE = 1    -- bit0
local FLAG_CELL_SPATIAL = 2      -- bit1
local FLAG_NO_SAMPLES = 1 << 10  -- bit10 = 0x400

local MAX_CELL_COUNT = 255

--------------------------------------------------------------------------------
-- 输出缓冲: 字节先放 table, 最后只 concat 一次
--------------------------------------------------------------------------------

local function new_writer()
    return { n = 0, t = {} }
end

local function w_byte(w, b)
    w.n = w.n + 1
    w.t[w.n] = string.char(b)
end

local function w_bytes(w, s)
    w.n = w.n + 1
    w.t[w.n] = s
end

local function w_uvarint(w, v)
    -- 防御：无符号列（time/balanCurrent/batVol/socCycleCap/socCapRemain/单体电压）
    -- 理论上不含负值（bms_uplink 写入缓存前已钳位）。这里再兜一层，避免
    -- string.char(负值) 抛错把上报任务整个打死（modbus 数据源的均衡电流/剩余容量
    -- 是"有符号"读取，放电均衡或容量为负时可能出现负值）。
    if v < 0 then
        v = 0
    end
    while v >= 0x80 do
        w_byte(w, (v & 0x7f) | 0x80)
        v = v >> 7
    end
    w_byte(w, v)
end

local function w_result(w)
    return table.concat(w.t, "", 1, w.n)
end

local function uvarint_len(v)
    local n = 1
    while v >= 0x80 do
        v = v >> 7
        n = n + 1
    end
    return n
end

--------------------------------------------------------------------------------
-- ZigZag / Delta 游程编码
--------------------------------------------------------------------------------

local function zigzag(n)
    if n >= 0 then
        return n << 1
    end
    return ((-n) << 1) - 1
end

local function write_delta_code(w, delta)
    if delta == 0 then
        w_uvarint(w, 1)
        return
    end
    w_uvarint(w, zigzag(delta) + 2)
end

local function encode_delta_sequence(w, deltas, count)
    local i = 1
    local remaining = count
    while remaining > 0 do
        local delta = deltas[i]
        local run = 1
        while run < remaining and deltas[i + run] == delta do
            run = run + 1
        end

        if delta == 0 then
            if run == 1 then
                w_uvarint(w, 1)
            else
                w_uvarint(w, 0)
                w_uvarint(w, run)
            end
        else
            local ed = zigzag(delta)
            local singleSize = uvarint_len(ed + 2)
            local runSize = 1 + uvarint_len(ed) + uvarint_len(run)
            if runSize < run * singleSize then
                w_uvarint(w, 2)
                w_uvarint(w, ed)
                w_uvarint(w, run)
            else
                local code = ed + 2
                for _ = 1, run do
                    w_uvarint(w, code)
                end
            end
        end

        i = i + run
        remaining = remaining - run
    end
end

--------------------------------------------------------------------------------
-- 列上下文
--------------------------------------------------------------------------------

local function make_column(values, count, signed)
    local first = values[1] or 0
    local const = true
    for i = 2, count do
        if (values[i] or 0) ~= first then
            const = false
            break
        end
    end

    local delta1, delta2 = nil, nil
    local linear = true
    if count > 1 then
        delta1 = {}
        delta2 = {}
        local prev = (values[2] or 0) - (values[1] or 0)
        delta1[1] = prev
        delta2[1] = prev
        for i = 2, count - 1 do
            local d = (values[i + 1] or 0) - (values[i] or 0)
            delta1[i] = d
            delta2[i] = d - prev
            if d ~= prev then
                linear = false
            end
            prev = d
        end
    end

    return {
        values = values,
        count = count,
        signed = signed,
        delta1 = delta1,
        delta2 = delta2,
        const = const,
        linear = linear,
    }
end

local function encode_column(w, col, mode)
    local values = col.values
    local count = col.count
    if col.signed then
        w_uvarint(w, zigzag(values[1] or 0))
    else
        w_uvarint(w, values[1] or 0)
    end

    if mode == 2 then
        -- constant: 只写首值
        return
    end

    if mode == 3 then
        -- linear: 只写首值和首 delta
        if count > 1 then
            write_delta_code(w, col.delta1[1])
        end
        return
    end

    if count > 1 then
        if mode == 1 then
            encode_delta_sequence(w, col.delta2, count - 1)
        else
            encode_delta_sequence(w, col.delta1, count - 1)
        end
    end
end

local function column_payload(col, mode)
    local w = new_writer()
    encode_column(w, col, mode)
    return w_result(w)
end

-- 与 Go chooseColumnMode 一致: 恒短才换模式（顺序也一样：delta → 常量 → 线性 → 二阶）
local function choose_column_mode(col)
    local bestMode = 0
    local best = column_payload(col, 0)

    if col.const then
        local cand = column_payload(col, 2)
        if #cand < #best then
            bestMode, best = 2, cand
        end
    end
    if col.linear then
        local cand = column_payload(col, 3)
        if #cand < #best then
            bestMode, best = 3, cand
        end
    end
    local cand = column_payload(col, 1)
    if #cand < #best then
        bestMode, best = 1, cand
    end
    return bestMode, best
end

--------------------------------------------------------------------------------
-- 电芯两布局 (offset / spatial) 与整体编码
--------------------------------------------------------------------------------

local function build_columns(hist, cellSpatial)
    local n = hist.count
    local cols = {
        make_column(hist.temp1, n, true),
        make_column(hist.temp2, n, true),
        make_column(hist.tempMos, n, true),
        make_column(hist.balanCurrent, n, false),
        make_column(hist.batVol, n, false),
        make_column(hist.batCurrent, n, true),
        make_column(hist.socCycleCap, n, false),
        make_column(hist.socCapRemain, n, false),
        make_column(hist.time, n, false),
    }

    local cellCount = hist.cellCount
    if cellCount and cellCount > 0 then
        local cv = hist.cellVols
        local base = {}
        for s = 1, n do
            base[s] = cv[(s - 1) * cellCount + 1] or 0
        end
        cols[#cols + 1] = make_column(base, n, false)

        for j = 2, cellCount do
            local d = {}
            if cellSpatial then
                for s = 1, n do
                    local idx = (s - 1) * cellCount
                    d[s] = (cv[idx + j] or 0) - (cv[idx + j - 1] or 0)
                end
            else
                for s = 1, n do
                    d[s] = (cv[(s - 1) * cellCount + j] or 0) - (cv[(s - 1) * cellCount + 1] or 0)
                end
            end
            cols[#cols + 1] = make_column(d, n, true)
        end
    end
    return cols
end

-- 每处理这么多列回调一次 on_slice，供上层插入 sys.wait 让出调度（见文件头说明）
local SLICE_COLUMNS = 16

-- 消息级字段：整条消息只写一次（与 Go encodeMessageScalars 顺序一致）
local function encode_scalars(meta)
    meta = meta or {}
    local w = new_writer()
    w_uvarint(w, math.max(0, floor(meta.socFullChargeCap or 0)))
    w_uvarint(w, math.max(0, floor(meta.baseTimestamp or 0)))
    w_uvarint(w, math.max(0, floor(meta.moduleUpTime or 0)))
    w_uvarint(w, math.max(0, floor(meta.bmsUpTime or 0)))
    w_uvarint(w, math.max(0, floor(meta.jinjianModbusRequestCount or 0)))
    w_uvarint(w, math.max(0, floor(meta.jikongModbusRequestCount or 0)))
    return w_result(w)
end

local function encode_columns(cols, count, cellCount, cellSpatial, scalars, on_slice)
    local encoded, defaults, modes = {}, {}, {}
    local saved = 0
    for i = 1, #cols do
        local col = cols[i]
        defaults[i] = column_payload(col, 0)
        local mode, best = choose_column_mode(col)
        if mode ~= 0 and #best < #defaults[i] then
            modes[i] = mode
            encoded[i] = best
            saved = saved + #defaults[i] - #best
        else
            encoded[i] = defaults[i]
        end
        if on_slice and (i % SLICE_COLUMNS) == 0 then
            on_slice()
        end
    end

    local modeSize = floor((#cols * 2 + 7) / 8)
    local headerWithout = (cellCount << 2) | (cellSpatial and FLAG_CELL_SPATIAL or 0)
    local headerWith = headerWithout | FLAG_HAS_MODE_TABLE
    local useModes = saved > modeSize + uvarint_len(headerWith) - uvarint_len(headerWithout)

    local w = new_writer()
    w_uvarint(w, count - 1)
    if useModes then
        w_uvarint(w, headerWith)
        local mt = {}
        for i = 1, #cols do
            local mode = modes[i] or 0
            local bit = (i - 1) * 2
            local idx = floor(bit / 8) + 1
            mt[idx] = (mt[idx] or 0) | (mode << (bit % 8))
        end
        for i = 1, #mt do
            w_byte(w, mt[i])
        end
        w_bytes(w, scalars)
        for i = 1, #cols do
            w_bytes(w, encoded[i])
        end
    else
        w_uvarint(w, headerWithout)
        w_bytes(w, scalars)
        for i = 1, #cols do
            w_bytes(w, defaults[i])
        end
    end
    return w_result(w)
end

-- hist.count >= 1 时编码成 MessageV1；count < 1 返回 nil（0 样本请用 encode_empty）
-- hist.cellCount 可为 nil/0, 表示无电芯列
-- on_slice: 可选，每编码 SLICE_COLUMNS 列回调一次（供上层让出），不传则纯同步
function M.encode(hist, meta, on_slice)
    if type(hist) ~= "table" then
        return nil
    end
    local n = hist.count or 0
    local c = hist.cellCount or 0
    if n < 1 then
        return nil
    end
    if c < 0 or c > MAX_CELL_COUNT then
        return nil
    end

    local scalars = encode_scalars(meta)
    local best = encode_columns(build_columns(hist, false), n, c, false, scalars, on_slice)
    if c > 0 then
        local cand = encode_columns(build_columns(hist, true), n, c, true, scalars, on_slice)
        if #cand < #best then
            best = cand
        end
    end
    return string.char(HEADER, VERSION) .. best
end

-- 0 样本报文（设备读不到 BMS）：只写帧头 + 消息级字段，没有任何列，共 13 字节。
-- SOCFullChargeCap 与 BaseTimestamp 无意义（固定写 0），ModuleUPTime、BMSUPTime 与两个
-- modbus 请求计数有效。
function M.encode_empty(meta)
    local w = new_writer()
    w_byte(w, HEADER)
    w_uvarint(w, VERSION)
    w_uvarint(w, 0)               -- sampleCount - 1（占位，真实样本数由 flag 决定）
    w_uvarint(w, FLAG_NO_SAMPLES) -- header：cellCount = 0
    w_uvarint(w, 0)               -- SOCFullChargeCap（无意义）
    w_uvarint(w, 0)               -- BaseTimestamp（无意义）
    local t = meta or {}
    w_uvarint(w, math.max(0, floor(t.moduleUpTime or 0)))
    w_uvarint(w, math.max(0, floor(t.bmsUpTime or 0)))
    w_uvarint(w, math.max(0, floor(t.jinjianModbusRequestCount or 0)))
    w_uvarint(w, math.max(0, floor(t.jikongModbusRequestCount or 0)))
    return w_result(w)
end

return M

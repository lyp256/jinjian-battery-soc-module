--[[
@module  sha256
@summary 纯 Lua SHA-256（MQTT 鉴权用，不依赖固件 crypto 库，PC 端可离线自测）
@version 1.0
@date    2026.10.12
@usage
只实现上报鉴权需要的两个入口：
    sha256.digest(str) -> 32 字节二进制摘要
    sha256.hex(str)    -> 64 字符小写十六进制

依据 FIPS 180-4（§4.2.2 轮常量 / §5.1.1 填充 / §5.3.3 初始值 / §6.2 压缩函数）。
要求 Lua 5.3+（整数与位运算 & | ~ << >>），LuatOS 与 PC Lua 5.4 都满足；
不依赖 string.pack / 固件 crypto，离线可测，单条约 0.1ms（PC）。
]]

local M = {}

local MASK = 0xFFFFFFFF

-- 64 个轮常量（FIPS 180-4 §4.2.2）
local K = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
}

-- 初始哈希值（FIPS 180-4 §5.3.3）
local H0 = {
    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
    0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
}

local function rotr(x, n)
    return ((x >> n) | (x << (32 - n))) & MASK
end

local function to_hex(s)
    return (s:gsub(".", function(c) return string.format("%02x", c:byte()) end))
end

-- 填充：0x80 + 0x00* + 64bit 大端比特长度（§5.1.1）
local function pad(msg)
    local len = #msg
    -- 位宽处理：Air780EPM 固件的 Lua 是 32 位整数（LUA_32BITS），十六进制字面量
    -- 0x100000000 会被回绕成 0，写成 bits // 0x100000000 就成了「除以 0」。
    -- 因此把 64 位比特长度拆成两个 32 位半字，只用放得进 32 位整数的常量（2^29）：
    --   高 32 位 = len // 2^29       低 32 位 = (len % 2^29) * 8
    -- 低位乘积即使回绕成负数，位模式仍是正确的低 32 位，>> 是逻辑移位，取字节结果一致。
    local hi = len // 0x20000000
    local lo = (len % 0x20000000) * 8
    local k = (56 - (len + 1) % 64) % 64
    return msg .. "\128" .. string.rep("\0", k) .. string.char(
        (hi >> 24) & 0xFF, (hi >> 16) & 0xFF, (hi >> 8) & 0xFF, hi & 0xFF,
        (lo >> 24) & 0xFF, (lo >> 16) & 0xFF, (lo >> 8) & 0xFF, lo & 0xFF)
end

-- 32 字节二进制摘要
function M.digest(msg)
    msg = pad(msg)
    local h = { H0[1], H0[2], H0[3], H0[4], H0[5], H0[6], H0[7], H0[8] }
    local w = {} -- 消息展开表（每块复用，避免每块新建 table）
    local blocks = #msg // 64
    for block = 0, blocks - 1 do
        local base = block * 64
        for i = 1, 16 do
            local o = base + (i - 1) * 4
            w[i] = (string.byte(msg, o + 1) << 24) | (string.byte(msg, o + 2) << 16)
                | (string.byte(msg, o + 3) << 8) | string.byte(msg, o + 4)
        end
        for i = 17, 64 do
            local x, y = w[i - 15], w[i - 2]
            local s0 = rotr(x, 7) ~ rotr(x, 18) ~ (x >> 3)
            local s1 = rotr(y, 17) ~ rotr(y, 19) ~ (y >> 10)
            w[i] = (w[i - 16] + s0 + w[i - 7] + s1) & MASK
        end

        local a, b, c, d = h[1], h[2], h[3], h[4]
        local e, f, g, hh = h[5], h[6], h[7], h[8]
        for i = 1, 64 do
            local s1 = rotr(e, 6) ~ rotr(e, 11) ~ rotr(e, 25)
            local ch = (e & f) ~ (((~e) & MASK) & g)
            local t1 = (hh + s1 + ch + K[i] + w[i]) & MASK
            local s0 = rotr(a, 2) ~ rotr(a, 13) ~ rotr(a, 22)
            local maj = (a & b) ~ (a & c) ~ (b & c)
            local t2 = (s0 + maj) & MASK
            hh = g
            g = f
            f = e
            e = (d + t1) & MASK
            d = c
            c = b
            b = a
            a = (t1 + t2) & MASK
        end

        h[1] = (h[1] + a) & MASK
        h[2] = (h[2] + b) & MASK
        h[3] = (h[3] + c) & MASK
        h[4] = (h[4] + d) & MASK
        h[5] = (h[5] + e) & MASK
        h[6] = (h[6] + f) & MASK
        h[7] = (h[7] + g) & MASK
        h[8] = (h[8] + hh) & MASK
    end

    local out = {}
    for i = 1, 8 do
        local v = h[i]
        out[i] = string.char((v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF)
    end
    return table.concat(out)
end

-- 64 字符小写十六进制摘要
function M.hex(msg)
    return to_hex(M.digest(msg))
end

M.to_hex = to_hex

return M

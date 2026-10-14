--[[
@module  hmac
@summary HMAC-SHA256（MQTT 密码 = HMAC(key="soc", msg=IMEI)，小写 hex）
@version 1.0
@date    2026.10.12
@usage
依据 RFC 2104：
    hmac.sha256(key, msg)     -> 32 字节二进制摘要
    hmac.sha256_hex(key, msg) -> 64 字符小写十六进制

只在 MQTT 连接时算一次（连接由 autoreconn 自动重连，重连走同一个 mqttc，不会重复计算），
纯 Lua + sha256.lua，不依赖固件 crypto 库。

自测已知答案（tools/selftest.lua 有断言）：
    sha256("abc") = ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad
    hmac_sha256("Jefe", "what do ya want for nothing?")
        = 5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843   (RFC 4231 TC2)
    hmac_sha256("soc", "861234567890123")
        = 1b281b472b217c6e99ff6aba726dc5fae8331dbadfaead19ba7fa8c0c29e409a  (设备 KAT)
]]

local sha256 = require "sha256"

local M = {}

local BLOCK_SIZE = 64

-- 把 key 异或成 ipad/opad 块（key 先按 RFC 2104 归一为 <= 64 字节）
local function xor_pad(key, pad_byte)
    local out = {}
    for i = 1, BLOCK_SIZE do
        out[i] = string.char((string.byte(key, i) or 0) ~ pad_byte)
    end
    return table.concat(out)
end

function M.sha256(key, msg)
    if #key > BLOCK_SIZE then
        key = sha256.digest(key)
    end
    local inner = sha256.digest(xor_pad(key, 0x36) .. msg)
    return sha256.digest(xor_pad(key, 0x5C) .. inner)
end

function M.sha256_hex(key, msg)
    return sha256.to_hex(M.sha256(key, msg))
end

return M

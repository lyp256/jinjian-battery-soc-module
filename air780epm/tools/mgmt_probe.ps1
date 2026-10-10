<#
.SYNOPSIS
  PC 侧管理链路探针：按自定义裸帧 RPC 读写模组 RS485-1/UART1 的管理服务（air780epm/mgmt*）。

.DESCRIPTION
  模组那端跑的是「统一管理服务 + UART1 裸帧传输」（air780epm/mgmt.lua + mgmt_uart.lua）：
    UART1 / RS485-1，115200 8N1，方向脚 EN = GPIO25（驱动自动控制），**没有从站地址**（点对点）。

  帧格式（双向同一套）：
    SOF(2)=A5 5A | VER(1)=01 | TYPE(1: 1 REQ / 2 RESP / 3 EVT / 4 BLOB) | ID(1) |
    LEN(2, 小端) | PAYLOAD | CRC16(2, 0xA001 低字节在前，覆盖 VER..PAYLOAD)
    REQ  = {"method","params"}；RESP = {"ok":true,"result":{…}} / {"ok":false,"error":{…}}
    EVT  = {"evt":"hello|fota","data":{…}}；BLOB = SEQ(2 LE) | LEN(2 LE) | DATA（≤238）

  动作（覆盖 README §7 的现场验收项）：
    ping      sys.ping               链路连通性自检（返回设备时间/上电秒数）
    status    status.get             状态大对象（网络/MQTT/极空/缓存/上报/FOTA/各链路计数）
    config    config.get             配置白名单当前值 + 需重启/覆盖标志
    set       config.set             批量写配置（-Set "KEY=VAL,KEY2=VAL2"，全量校验、原子写入）
    reboot    cmd.reboot             回帧后 200ms 重启
    factory   cmd.factory_reset      清 fskv 覆盖 + 置待生效位（回帧后由主机重启）
    url       fota.url               -Text <url> 触发 URL 升级（设备联网下载）
    localfota fota.begin + BLOB      把本地升级包按 238 字节分块经 BLOB 帧写入（不联网）
    abort     fota.abort             中止正在进行的本地升级
    raw       -Text <json>           直接发一段原始 JSON 请求
    echo      配合 MGMT_ECHO_TEST    回环自测（PC→模组、模组→PC 两个方向）
    listen    只收不发                 监听设备主动事件（evt.hello / evt.fota）

  适配器方向极性：USB-485 有的是自动方向、有的把 RTS 当 DE（少数当 /RE）。默认每帧先在
  RTS=0、再在 RTS=1 下各试一次，两种极性都覆盖；已知极性时用 -Rts 0 / -Rts 1 固定。

  退出码：0 = 本次动作成功，1 = 失败（-Retries 次都没收到合法响应）。

.EXAMPLE
  # 连通性自检 + 读状态（最常用）
  powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action ping
  powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action status

  # 读配置；改配置（实时项 / [R] 项）
  powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action config
  powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 `
      -Action set -Set "JK_MODBUS_POLL_MS=1500"
  powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 `
      -Action set -Set "MQTT_HOST=mqtt.399.run,MQTT_PORT=1883"

  # 重启 / 恢复出厂
  powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action reboot
  powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action factory

  # URL FOTA
  powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 `
      -Action url -Text "http://ota.example/fw.binpkg"

  # 本地分段升级（BLOB 帧；成功后再 -Action reboot）
  powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 `
      -Action localfota -File .\jjsoc_2052.1.0_LuatOS-SoC_Air780EPM.bin
  powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action abort

  # 监听事件（evt.hello / evt.fota）
  powershell -ExecutionPolicy Bypass -File air780epm\tools\mgmt_probe.ps1 -Port COM11 -Action listen
#>
param(
    [string]$Port = 'COM11',
    [int]$Baud = 115200,
    [int]$Timeout = 500,                     # 单帧等待应答的超时（毫秒）
    [int]$Retries = 2,                       # 每个极性下的重试次数
    [int]$Rts = -1,                          # -1 = RTS=0/1 都试；0 / 1 = 只用该极性
    [ValidateSet('ping', 'status', 'config', 'set', 'reboot', 'factory',
                 'url', 'localfota', 'abort', 'raw', 'echo', 'listen')]
    [string]$Action = 'ping',
    [string]$Set = '',                       # set 动作：KEY=VAL,KEY2=VAL2
    [string]$Text = '',                      # url 的 URL / raw 的 JSON / echo 的文本
    [string]$Hex = '',                       # raw：直接给的 payload hex（与 -Text 二选一）
    [Alias('Path', 'FilePath')]
    [string]$File = '',                      # localfota 的升级包路径（.bin）
    [int]$Chunk = 238,                       # localfota 每块有效字节数（1-238）
    [switch]$Abort,                          # localfota：中止正在进行的本地升级
    [int]$Seconds = 5,                       # listen 动作的监听时长（秒）
    [switch]$Ascii                           # 额外按 ASCII 打印收到的字节
)

$ErrorActionPreference = 'Stop'

$SOF1 = 0xA5; $SOF2 = 0x5A; $VER = 0x01
$T_REQ = 0x01; $T_RESP = 0x02; $T_EVT = 0x03; $T_BLOB = 0x04

function Get-Crc16([byte[]]$data) {
    $crc = 0xFFFF
    foreach ($b in $data) {
        $crc = $crc -bxor $b
        for ($i = 0; $i -lt 8; $i++) {
            if (($crc -band 1) -ne 0) { $crc = ($crc -shr 1) -bxor 0xA001 }
            else { $crc = $crc -shr 1 }
        }
    }
    return ($crc -band 0xFFFF)
}

# 组一帧：TYPE + ID + PAYLOAD（CRC 覆盖 VER..PAYLOAD，低字节在前）
function New-Frame([int]$type, [int]$id, [byte[]]$payload) {
    if ($null -eq $payload) { $payload = @() }
    $len = $payload.Length
    $body = @([byte]$VER, [byte]$type, [byte]$id,
              [byte]($len -band 0xFF), [byte](($len -shr 8) -band 0xFF)) + $payload
    $crc = Get-Crc16 $body
    return @([byte]$SOF1, [byte]$SOF2) + $body + @([byte]($crc -band 0xFF), [byte](($crc -shr 8) -band 0xFF))
}

# 标准 CRC32（IEEE 802.3，多项式 0xEDB88320，初值/结果取反）——与设备 fota_mgr.crc32 一致
function Get-Crc32([byte[]]$data) {
    [int64]$mask = ([int64]0xFFFF -shl 16) -bor [int64]0xFFFF
    [int64]$poly = ([int64]0xEDB8 -shl 16) -bor [int64]0x8320
    [int64]$crc = $mask
    foreach ($b in $data) {
        $crc = $crc -bxor [int64]$b
        for ($i = 0; $i -lt 8; $i++) {
            if (($crc -band 1) -ne 0) { $crc = (($crc -shr 1) -bxor $poly) -band $mask }
            else { $crc = $crc -shr 1 }
        }
    }
    return ($crc -bxor $mask) -band $mask
}

function To-JsonBytes([object]$obj) {
    $json = $obj | ConvertTo-Json -Compress -Depth 12
    return [System.Text.Encoding]::UTF8.GetBytes($json)
}

function New-Request([string]$method, $params) {
    $req = @{ method = $method }
    if ($null -ne $params) { $req['params'] = $params }
    return To-JsonBytes $req
}

function Show-Hex([string]$title, [byte[]]$data) {
    if (-not $data -or $data.Length -eq 0) {
        Write-Host ("  {0}: 没收到数据（0 字节）" -f $title) -ForegroundColor Yellow
        return
    }
    $hex = ($data | ForEach-Object { $_.ToString('x2') }) -join ' '
    Write-Host ("  {0}: {1} 字节" -f $title, $data.Length) -ForegroundColor DarkGray
    Write-Host ("    hex : {0}" -f $hex)
    if ($Ascii) {
        $txt = -join ($data | ForEach-Object { if ($_ -ge 32 -and $_ -lt 127) { [char]$_ } else { '.' } })
        Write-Host ("    text: {0}" -f $txt)
    }
}

# 从串口读一帧（按 SOF/LEN 定界 + CRC 校验）；返回 @{type;id;payload} 或 $null
function Read-OneFrame([System.IO.Ports.SerialPort]$sp, [double]$seconds) {
    $buf = New-Object System.Collections.Generic.List[byte]
    $deadline = (Get-Date).AddSeconds($seconds)
    while ((Get-Date) -lt $deadline) {
        try {
            $n = $sp.BytesToRead
            if ($n -gt 0) {
                $tmp = New-Object byte[] $n
                [void]$sp.Read($tmp, 0, $n)
                $buf.AddRange($tmp)
            } else {
                Start-Sleep -Milliseconds 10
            }
        } catch { break }
        # 在缓冲里找 SOF，并判断是否已够一整帧
        $arr = $buf.ToArray()
        for ($i = 0; $i -le $arr.Length - 9; $i++) {
            if ($arr[$i] -eq $SOF1 -and $arr[$i + 1] -eq $SOF2) {
                $len = [int]$arr[$i + 5] -bor ([int]$arr[$i + 6] -shl 8)
                $total = $len + 9
                if ($arr.Length - $i -ge $total) {
                    $frame = $arr[$i..($i + $total - 1)]
                    # body = VER..PAYLOAD（帧头 2 字节 SOF + VER(1)+TYPE(1)+ID(1)+LEN(2) = 索引 2..6+len）
                    $body = $frame[2..(6 + $len)]
                    $crc = Get-Crc16 $body
                    if ($frame[$total - 2] -eq ($crc -band 0xFF) -and
                        $frame[$total - 1] -eq (($crc -shr 8) -band 0xFF)) {
                        return @{ type = [int]$frame[3]; id = [int]$frame[4];
                                  payload = $(if ($len -gt 0) { [byte[]]$frame[7..(6 + $len)] } else { @() }) }
                    }
                }
                break        # 找到 SOF 但还没齐/CRC 待定：继续读
            }
        }
    }
    return $null
}

# 发一帧、等一帧「type/id 匹配」的响应；期间打印/丢弃事件帧。返回响应帧或 $null
function Send-Frame([System.IO.Ports.SerialPort]$sp, [byte[]]$frame, [int]$wantId,
                    [bool]$writeRts, [bool]$readRts, [switch]$Quiet) {
    $sp.RtsEnable = $writeRts
    Start-Sleep -Milliseconds 30
    try { $sp.DiscardInBuffer(); $sp.DiscardOutBuffer() } catch { }
    $sp.Write($frame, 0, $frame.Length)
    Start-Sleep -Milliseconds 5
    $sp.RtsEnable = $readRts               # 发完立刻切回接收方向
    $deadline = (Get-Date).AddMilliseconds($Timeout)
    while ((Get-Date) -lt $deadline) {
        $got = Read-OneFrame $sp ([Math]::Max(0.05, ($deadline - (Get-Date)).TotalSeconds))
        if ($got) {
            if ($got.type -eq $T_EVT) {
                if (-not $Quiet) { Show-Event $got }
                continue
            }
            if ($got.type -eq $T_RESP -and ($wantId -lt 0 -or $got.id -eq $wantId)) {
                return $got
            }
        } else {
            break
        }
    }
    return $null
}

function Invoke-Rpc([System.IO.Ports.SerialPort]$sp, [string]$method, $params, [int]$id = 1) {
    $payload = New-Request $method $params
    $frame = New-Frame $T_REQ $id $payload
    $combos = if ($Rts -eq -1) {
        @(, @($false, $false)) + @(, @($true, $false)) + @(, @($false, $true)) + @(, @($true, $true))
    } else {
        @(, @([bool]$Rts, $false)) + @(, @([bool]$Rts, $true))
    }
    foreach ($combo in $combos) {
        for ($i = 1; $i -le $Retries; $i++) {
            $r = Send-Frame $sp $frame $id $combo[0] $combo[1]
            if ($r) { return $r }
        }
    }
    Write-Host ("  [失败] {0}：各种 RTS 组合都没收到合法响应" -f $method) -ForegroundColor Red
    return $null
}

function Show-Event([hashtable]$f) {
    $txt = [System.Text.Encoding]::UTF8.GetString($f.payload)
    Write-Host ("  [事件 evt#{0}] {1}" -f $f.id, $txt) -ForegroundColor Cyan
}

function Show-Respond([hashtable]$r) {
    $txt = [System.Text.Encoding]::UTF8.GetString($r.payload)
    try { $obj = $txt | ConvertFrom-Json } catch { $obj = $null }
    if ($null -eq $obj) {
        Write-Host ("  [响应 id={0}] 非 JSON：{1}" -f $r.id, $txt) -ForegroundColor Yellow
        return $obj
    }
    if ($obj.ok) {
        Write-Host ("  [响应 id={0}] ok" -f $r.id) -ForegroundColor Green
    } else {
        Write-Host ("  [响应 id={0}] 错误 {1}：{2}" -f $r.id, $obj.error.code, $obj.error.msg) -ForegroundColor Red
    }
    return $obj
}

function Show-StatusObj($o) {
    if (-not $o.ok) { Show-ErrorObj $o; return }
    $s = $o.result
    Write-Host "  == 状态（status.get） ==" -ForegroundColor Cyan
    Write-Host ("    net_ready={0}  mqtt={1}  jk_valid={2}  power_on={3}" -f `
        $s.net_ready, $s.mqtt, $s.jk_valid, $s.power_on)
    Write-Host ("    need_restart={0}  has_override={1}  factory_pending={2}  fota_busy={3}" -f `
        $s.needs_restart, $s.has_override, $s.factory_pending, $s.fota_busy)
    Write-Host ("    reset_reason={0}  uptime_s={1}  供电={2:N2} V" -f `
        $s.reset_reason, $s.uptime_s, ($s.supply_mv / 1000.0))
    Write-Host ("    设备: project={0} model={1} version={2} proto={3} pn={4}" -f `
        $s.device.project, $s.device.model, $s.device.version, $s.device.proto, $s.device.pn)
    Write-Host ("    链路: jk(rounds={0} frames={1} rx={2} sync={3})  jinjian(query={4} reply={5} ignored={6} rx={7})" -f `
        $s.links.jk.rounds, $s.links.jk.frames, $s.links.jk.rx, $s.links.jk.sync, `
        $s.links.jinjian.queries, $s.links.jinjian.responses, $s.links.jinjian.ignored, $s.links.jinjian.rx)
    $mu = $s.links.mgmt_uart
    Write-Host ("    管理UART: rx={0} frames={1} reply={2} crc={3} sync={4} drop={5} tx={6} evt={7}" -f `
        $mu.rx, $mu.frames, $mu.reply, $mu.crc, $mu.sync, $mu.drop, $mu.tx, $mu.evt)
    $mm = $s.links.mgmt_mqtt
    Write-Host ("    管理MQTT: rx={0} reply={1} ignored={2} bad={3}" -f `
        $mm.rx, $mm.reply, $mm.ignored, $mm.bad)
    Write-Host ("    缓存: {0}/{1}  上报: 包={2} 样本={3} 空={4} 丢={5} 发布={6} 丢弃={7}" -f `
        $s.cache.count, $s.cache.capacity, $s.uplink.batches, $s.uplink.samples, `
        $s.uplink.empty, $s.uplink.lost, $s.uplink.published, $s.uplink.dropped)
    $fo = $s.fota
    Write-Host ("    FOTA: mode={0} state={1} stage={2} result={3} progress={4}% rx={5}/{6} seq={7}" -f `
        $fo.mode, $fo.state, $fo.stage, $fo.result, $fo.progress, $fo.rx, $fo.total, $fo.last_seq)
    $c = $fo.caps
    # core_type: nil = 固件没这个库；userdata = LuatOS rotable（正常形态）；table = 别的实现
    $coreType = if ($c.core_type) { "($($c.core_type))" } else { "(nil)" }
    Write-Host ("    FOTA 能力: 核心库fota={0}{1} libfota2={2} 本地通道={3} URL通道={4} fskv={5} json={6}" -f `
        $c.core, $coreType, $c.lib, $c.local_enabled, $c.url_enabled, $c.fskv, $c.json) `
        -ForegroundColor $(if ($c.core) { 'Green' } else { 'Yellow' })
}

function Show-ErrorObj($o) {
    Write-Host ("  错误 {0}：{1}" -f $o.error.code, $o.error.msg) -ForegroundColor Red
}

# 只解码不打印：轮询 fota.status 用（Show-Respond 每次都会打一行，轮询里太吵）。
# 注意 payload 是 byte[]，必须先按 UTF-8 解成字符串再 ConvertFrom-Json。
function Get-RespondObj([hashtable]$r) {
    if (-not $r) { return $null }
    $txt = [System.Text.Encoding]::UTF8.GetString($r.payload)
    try { return ($txt | ConvertFrom-Json) } catch { return $null }
}

# 每次请求都必须换新 ID：设备对「同 ID + 同方法」在 MGMT_DEDUP_WINDOW_MS（默认 10s）内**重放**
# 缓存响应，不重新执行 —— 用固定 ID 轮询 fota.status 会一直读到第一次那份快照。
# 毫秒派生（1..255）保证 10s 窗口内不重复（255 个值 × 100ms = 25.5s > 10s）。
function New-RpcId {
    return [int]([DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() % 255) + 1
}

function Show-ConfigObj($o) {
    if (-not $o.ok) { Show-ErrorObj $o; return }
    $r = $o.result
    Write-Host "  == 配置（config.get） ==" -ForegroundColor Cyan
    Write-Host ("    needs_restart={0}  has_override={1}" -f $r.needs_restart, $r.has_override)
    Write-Host ("    restart_keys={0}" -f ($r.restart_keys -join ',')) -ForegroundColor DarkGray
    foreach ($p in $r.values.PSObject.Properties) {
        Write-Host ("    {0} = {1}" -f $p.Name, $p.Value)
    }
}

#=============================================================================
# 主流程
#=============================================================================
$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$sp.ReadTimeout = $Timeout
$sp.WriteTimeout = $Timeout
try {
    $sp.Open()
} catch {
    Write-Host ("打开 {0} 失败：{1}" -f $Port, $_.Exception.Message) -ForegroundColor Red
    Write-Host "→ 多半是串口助手/其它工具还占着这个口；关掉再跑。另外确认已装 USB-RS485 驱动。" -ForegroundColor Yellow
    exit 1
}

Write-Host ("== {0} @ {1} 8N1，动作 {2} ==" -f $Port, $Baud, $Action) -ForegroundColor Cyan
Write-Host "  接线：USB-485 的 A↔模组 A、B↔模组 B，两侧共地；模组管理链路 = RS485-1（UART1，EN=GPIO25）" -ForegroundColor DarkGray

$ok = $false
# 请求号 ID：设备对「同 ID + 同方法」在 MGMT_DEDUP_WINDOW_MS（默认 10s）内会重放缓存响应
# （防止丢应答后重试造成二次重启/二次 FOTA），所以主站每次请求必须换新 ID —— 用毫秒派生。
$id = [int]([DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() % 240) + 1
try {
    switch ($Action) {
        'ping' {
            $r = Invoke-Rpc $sp 'sys.ping' $null $id
            if ($r) { $o = Show-Respond $r; if ($o -and $o.ok) { Write-Host ("    time={0} uptime_s={1}" -f $o.result.time, $o.result.uptime_s) -ForegroundColor Green; $ok = $true } }
        }
        'status' {
            $r = Invoke-Rpc $sp 'status.get' $null $id
            if ($r) { $o = Show-Respond $r; if ($o) { Show-StatusObj $o; $ok = $o.ok } }
        }
        'config' {
            $r = Invoke-Rpc $sp 'config.get' $null $id
            if ($r) { $o = Show-Respond $r; if ($o) { Show-ConfigObj $o; $ok = $o.ok } }
        }
        'set' {
            if ([string]::IsNullOrWhiteSpace($Set)) {
                Write-Host "  set 需要 -Set `"KEY=VAL,KEY2=VAL2`"" -ForegroundColor Red
            } else {
                $params = [ordered]@{}
                foreach ($kv in ($Set -split ',')) {
                    $parts = $kv.Split('=', 2)
                    if ($parts.Count -ne 2) { throw "非法 -Set 项：$kv" }
                    $k = $parts[0].Trim(); $v = $parts[1].Trim()
                    if ($v -match '^(true|false)$') { $params[$k] = [bool]::Parse($v) }
                    elseif ($v -match '^-?\d+$') { $params[$k] = [int]$v }
                    else { $params[$k] = $v }
                }
                $r = Invoke-Rpc $sp 'config.set' $params $id
                if ($r) {
                    $o = Show-Respond $r
                    if ($o -and $o.ok) {
                        Write-Host ("    applied={0}  restart_required={1}" -f `
                            (($o.result.applied.PSObject.Properties | ForEach-Object { "$($_.Name)=$($_.Value)" }) -join ' '), `
                            $o.result.restart_required) -ForegroundColor Green
                        if ($o.result.restart_required) {
                            Write-Host "    [R] 项需重启生效：-Action reboot" -ForegroundColor Yellow
                        }
                        $ok = $true
                    }
                }
            }
        }
        'reboot' {
            $r = Invoke-Rpc $sp 'cmd.reboot' $null $id
            if ($r) { $o = Show-Respond $r; if ($o -and $o.ok) { Write-Host "    已受理，回帧之后约 200ms 重启" -ForegroundColor Green; $ok = $true } }
        }
        'factory' {
            $r = Invoke-Rpc $sp 'cmd.factory_reset' $null $id
            if ($r) { $o = Show-Respond $r; if ($o -and $o.ok) { Write-Host "    已清持久化覆盖；重启后回默认值（用 -Action reboot）" -ForegroundColor Green; $ok = $true } }
        }
        'url' {
            if ([string]::IsNullOrWhiteSpace($Text)) {
                Write-Host "  url 需要 -Text <升级包 URL>" -ForegroundColor Red
            } else {
                $r = Invoke-Rpc $sp 'fota.url' @{ url = $Text } $id
                if ($r) { $o = Show-Respond $r; if ($o -and $o.ok) { Write-Host "    已启动 URL 升级；用 -Action status 看 fota.state/progress" -ForegroundColor Green; $ok = $true } }
            }
        }
        'abort' {
            $r = Invoke-Rpc $sp 'fota.abort' $null $id
            if ($r) { $o = Show-Respond $r; if ($o -and $o.ok) { Write-Host ("    aborted={0}" -f $o.result.aborted) -ForegroundColor Green; $ok = $true } }
        }
        'raw' {
            $payload = $null
            if (-not [string]::IsNullOrWhiteSpace($Hex)) {
                $clean = ($Hex -replace '[^0-9a-fA-F]', '')
                $payload = @()
                for ($i = 0; $i -lt $clean.Length; $i += 2) { $payload += [byte][Convert]::ToInt32($clean.Substring($i, 2), 16) }
            } elseif (-not [string]::IsNullOrWhiteSpace($Text)) {
                $payload = [System.Text.Encoding]::UTF8.GetBytes($Text)
            } else {
                Write-Host "  raw 需要 -Text <json> 或 -Hex <payload hex>" -ForegroundColor Red
            }
            if ($payload) {
                $frame = New-Frame $T_REQ $id $payload
                Write-Host ("  发送: {0}" -f (($frame | ForEach-Object { $_.ToString('x2') }) -join ' ')) -ForegroundColor DarkGray
                $combos = @(@($false, $false), @($true, $false), @($false, $true), @($true, $true))
                foreach ($c in $combos) {
                    $r = Send-Frame $sp $frame $id $c[0] $c[1]
                    if ($r) { [void](Show-Respond $r); $ok = $true; break }
                }
                if (-not $ok) { Write-Host "  没收到响应" -ForegroundColor Red }
            }
        }
        'echo' {
            $payload = if ([string]::IsNullOrEmpty($Text)) { 'MGMT-ECHO-PROBE' } else { $Text }
            $bytes = [System.Text.Encoding]::ASCII.GetBytes($payload + "`r`n")
            Write-Host ("  发送 '{0}'（{1} 字节），两种方向极性各发一次，每次监听 1.5s" -f $payload, $bytes.Length) -ForegroundColor DarkGray
            $all = @()
            foreach ($writeRts in @($false, $true)) {
                $sp.RtsEnable = $writeRts
                Start-Sleep -Milliseconds 40
                try { $sp.DiscardInBuffer() } catch { }
                try { $sp.Write($bytes, 0, $bytes.Length) } catch { continue }
                $sp.RtsEnable = -not $writeRts
                $deadline = (Get-Date).AddSeconds(1.5)
                while ((Get-Date) -lt $deadline) {
                    try {
                        $n = $sp.BytesToRead
                        if ($n -gt 0) {
                            $tmp = New-Object byte[] $n
                            [void]$sp.Read($tmp, 0, $n)
                            $all += $tmp
                        } else { Start-Sleep -Milliseconds 20 }
                    } catch { break }
                }
            }
            $got = [byte[]]$all
            Show-Hex '收到' $got
            $text = if ($got.Length -gt 0) {
                -join ($got | ForEach-Object { if ($_ -ge 32 -and $_ -lt 127) { [char]$_ } else { '.' } })
            } else { '' }
            if ($text.Contains($payload)) {
                Write-Host "  [echo] 回显一致：PC→模组、模组→PC 两个方向都通（电气层没问题）" -ForegroundColor Green
                $ok = $true
            } elseif ($text.Contains('mg echo')) {
                Write-Host "  [echo] 只收到心跳行：模组→PC 通，PC→模组 方向不通（或回显没到）" -ForegroundColor Yellow
            } else {
                Write-Host "  [echo] 既没回显也没心跳：模组→PC 方向也不通（查 A/B、共地、适配器方向）" -ForegroundColor Red
            }
        }
        'listen' {
            Write-Host ("  监听 {0}s（设备打开串口后 ~1s 会发 evt.hello；FOTA 变化会发 evt.fota）..." -f $Seconds) -ForegroundColor DarkGray
            $deadline = (Get-Date).AddSeconds([Math]::Max(1, $Seconds))
            while ((Get-Date) -lt $deadline) {
                $got = Read-OneFrame $sp 0.5
                if ($got) { Show-Event $got; $ok = $true }
            }
        }
        'localfota' {
            if ($Abort) {
                $r = Invoke-Rpc $sp 'fota.abort' $null $id
                if ($r) { $o = Show-Respond $r; if ($o -and $o.ok) { $ok = $true } }
            } elseif ([string]::IsNullOrWhiteSpace($File)) {
                Write-Host "  localfota 需要 -File <升级包 .bin>" -ForegroundColor Red
            } elseif (-not (Test-Path -LiteralPath $File -PathType Leaf)) {
                Write-Host ("  升级包不存在：{0}" -f $File) -ForegroundColor Red
            } elseif ($Chunk -lt 1 -or $Chunk -gt 238) {
                Write-Host ("  -Chunk 必须在 1-238 之间（现在 {0}）" -f $Chunk) -ForegroundColor Red
            } else {
                $bytes = [System.IO.File]::ReadAllBytes((Resolve-Path -LiteralPath $File))
                if ($bytes.Length -eq 0) { throw "升级包是空文件" }
                $total = $bytes.Length
                $crc = Get-Crc32 $bytes
                Write-Host ("  [localfota] {0}：{1} 字节，标准 CRC32 = 0x{2:x8}（每块 {3} 字节）" -f `
                    (Split-Path -Leaf $File), $total, $crc, $Chunk) -ForegroundColor Cyan

                # 先确认没有进行中的会话
                $st = Invoke-Rpc $sp 'fota.status' $null (New-RpcId)
                if ($st) {
                    $so = Show-Respond $st
                    if ($so -and -not $so.result.caps.core) {
                        $ct = if ($so.result.caps.core_type) { $so.result.caps.core_type } else { 'nil' }
                        Write-Host ("    ⚠ 能力位显示「核心库 fota = 无」(core_type={0})：本地升级会回 ENOLIB。" -f $ct) -ForegroundColor Yellow
                        Write-Host "       core_type='nil' 才是底层固件真没这个库（要刷带 LUAT_USE_FOTA 的固件）；" -ForegroundColor Yellow
                        Write-Host "       'userdata'/'table' 却 core=false 属探测异常（本仓已修，见 fota_mgr.lua get_core）" -ForegroundColor Yellow
                    }
                    if ($so -and $so.result.state -eq 1) {
                        Write-Host "  设备已有进行中的 FOTA 会话；先 -Action abort 或等它结束" -ForegroundColor Red
                        break
                    }
                }

                $id2 = (($id + 9) % 240) + 1
                $r = Invoke-Rpc $sp 'fota.begin' @{ total = $total; crc32 = $crc } $id2
                $o = if ($r) { Show-Respond $r } else { $null }
                if (-not ($o -and $o.ok)) {
                    Write-Host "  fota.begin 被拒（EFOTA/ENOLIB 见 README §7.5）" -ForegroundColor Red
                } else {
                    # fota.init() 是**设备后台任务**：先轮询 fota.status 等到 stage=ready(2) 再发第一块。
                    # 早发的块会落进「会话还在 preparing」→ 设备回 EBADSEQ（挡住乱序/未就绪写入）。
                    $ready = $false
                    $rdLine = (Get-Date).AddSeconds(6)
                    while ((Get-Date) -lt $rdLine) {
                        $sr = Invoke-Rpc $sp 'fota.status' $null (New-RpcId)
                        $so2 = Get-RespondObj $sr
                        if ($so2 -and $so2.ok) {
                            if ($so2.result.stage -eq 6) {
                                Write-Host ("  fota.init 失败：stage=6 result={0}（-1 核心库缺失 / -10 init）" -f $so2.result.result) -ForegroundColor Red
                                break
                            }
                            if ($so2.result.stage -ge 2) { $ready = $true; break }
                        }
                        Start-Sleep -Milliseconds 100
                    }
                    if (-not $ready) {
                        Write-Host "  fota.begin 后等 ready(stage=2) 超时：本次本地升级未开始" -ForegroundColor Red
                        break
                    }
                    $offset = 0; $seq = 1; $t0 = Get-Date
                    while ($offset -lt $total) {
                        $id2 = ($id2 % 200) + 2
                        $len = [Math]::Min($Chunk, $total - $offset)
                        $blob = @([byte](($seq) -band 0xFF), [byte](($seq -shr 8) -band 0xFF),
                                  [byte]($len -band 0xFF), [byte](($len -shr 8) -band 0xFF))
                        for ($i = 0; $i -lt $len; $i++) { $blob += $bytes[$offset + $i] }
                        $frame = New-Frame $T_BLOB $id2 $blob
                        $combos = if ($Rts -eq -1) {
                            @(, @($false, $false)) + @(, @($true, $false)) + @(, @($false, $true)) + @(, @($true, $true))
                        } else { @(, @([bool]$Rts, $false)) + @(, @([bool]$Rts, $true)) }
                        $rd = $null
                        foreach ($c in $combos) {
                            for ($k = 1; $k -le $Retries; $k++) {
                                $rd = Send-Frame $sp $frame $id2 $c[0] $c[1] -Quiet
                                if ($rd) { break }
                            }
                            if ($rd) { break }
                        }
                        if (-not $rd) { Write-Host ("  数据块 #{0} 没有应答" -f $seq) -ForegroundColor Red; break }
                        $ro = Show-Respond $rd
                        if (-not ($ro -and $ro.ok)) { break }
                        $offset += $len
                        if (($seq % 16) -eq 0 -or $offset -ge $total) {
                            Write-Host ("    已上传 {0}/{1} 字节（{2}%），块 #{3}" -f `
                                $offset, $total, [Math]::Floor($offset * 100.0 / $total), $seq) -ForegroundColor DarkGray
                        }
                        $seq++
                    }
                    if ($offset -ge $total) {
                        Write-Host ("  [localfota] {0} 个块已发完（{1:N1}s），等设备收尾…" -f `
                            ($seq - 1), ((Get-Date) - $t0).TotalSeconds) -ForegroundColor DarkGray
                        # 轮询 fota.status 到 success(5)/failed(6)
                        $deadline = (Get-Date).AddSeconds(20)
                        while ((Get-Date) -lt $deadline) {
                            $sr = Invoke-Rpc $sp 'fota.status' $null (New-RpcId)
                            if ($sr) {
                                $so = Get-RespondObj $sr
                                if ($so -and $so.ok) {
                                    $f = $so.result
                                    if ($f.stage -eq 5) {
                                        Write-Host "  [localfota] 成功：已写入 Flash、状态 = 成功待重启（不自动重启）" -ForegroundColor Green
                                        Write-Host "  → -Action reboot 进新版本，然后 -Action status 确认 version 已变" -ForegroundColor Yellow
                                        $ok = $true; break
                                    } elseif ($f.stage -eq 6) {
                                        Write-Host ("  升级失败：stage=6 result={0}（-1 核心库缺失 / -10 init / -12 空闲超时 / -21 run / -30 isDone / -31 CRC32 / -32 finish / -40 中止）" -f $f.result) -ForegroundColor Red
                                        break
                                    }
                                }
                            }
                            Start-Sleep -Milliseconds 200
                        }
                    }
                }
            }
        }
    }
} finally {
    $sp.Close()
}

Write-Host ""
if ($ok) {
    Write-Host "== 结论：本次动作成功 ==" -ForegroundColor Green
    exit 0
}
Write-Host "== 结论：没拿到合法响应。按顺序查 ==" -ForegroundColor Red
Write-Host "   ① 模组日志有没有 'mg uart 1 opened 115200 8N1 (mgmt rpc, raw frame)'（没有 = 没上电/没跑到 main）"
Write-Host "   ② 模组日志有没有 'pm io vol set : 0 3300'（GPIO 高电平不是 3.3V 时 RS485-1 的 EN 认不出来，见 README §2.1）"
Write-Host "   ③ A/B 是否接反、是否共地；USB-485 方向极性（试 -Rts 0 / -Rts 1）"
Write-Host "   ④ 波特率是否与设备一致（默认 115200 8N1）；设备是否误开了 MGMT_ECHO_TEST（诊断模式不跑 RPC）"
exit 1

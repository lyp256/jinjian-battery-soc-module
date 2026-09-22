param(
    [Parameter(Mandatory = $true)][string]$Path
)

$ErrorActionPreference = 'Stop'

# ---------- Parse documents ----------
$docs = New-Object System.Collections.ArrayList
$current = $null
foreach ($line in Get-Content -LiteralPath $Path) {
    if ([string]::IsNullOrWhiteSpace($line)) { continue }
    $parts = $line -split '\|\|', 2
    try { $head = $parts[0] | ConvertFrom-Json } catch { continue }
    if ($null -eq $head -or -not $head.type) { continue }
    $data = $null
    if ($parts.Count -gt 1) {
        $dataStr = $parts[1].TrimEnd('|')
        if (-not [string]::IsNullOrWhiteSpace($dataStr)) {
            try { $data = $dataStr | ConvertFrom-Json } catch { $data = $null }
        }
    }
    if ($head.type -eq 'DOCHEAD') {
        $current = [pscustomobject]@{
            Uuid = $data.uuid; DocType = $data.docType; Title = ''
            Records = New-Object System.Collections.ArrayList
        }
        [void]$docs.Add($current)
        continue
    }
    if ($null -ne $current) {
        [void]$current.Records.Add([pscustomobject]@{ Type = $head.type; Id = $head.id; Data = $data })
        if ($head.type -eq 'META' -and $data.title) { $current.Title = $data.title }
    }
}

# ---------- Device -> symbol / netflag mapping ----------
$deviceByUuid = @{}
foreach ($d in $docs | Where-Object { $_.DocType -eq 'DEVICE' }) {
    $meta = $d.Records | Where-Object Type -eq 'META' | Select-Object -First 1
    if ($meta -and $meta.Data.attributes) {
        $deviceByUuid[$d.Uuid] = [pscustomobject]@{
            Name        = $meta.Data.attributes.Name
            SymbolUuid  = $meta.Data.attributes.Symbol
            GlobalNet   = $meta.Data.attributes.'Global Net Name'
        }
    }
}

# ---------- Symbol -> pins ----------
$symbolByUuid = @{}
foreach ($d in $docs | Where-Object { $_.DocType -eq 'SYMBOL' }) {
    $pins = New-Object System.Collections.ArrayList
    foreach ($r in $d.Records | Where-Object Type -eq 'PIN') {
        $name = ''; $num = ''
        foreach ($a in $d.Records | Where-Object { $_.Type -eq 'ATTR' -and $_.Data.parentId -eq $r.Id }) {
            if ($a.Data.key -eq 'Pin Name') { $name = $a.Data.value }
            if ($a.Data.key -eq 'Pin Number') { $num = $a.Data.value }
        }
        [void]$pins.Add([pscustomobject]@{ Id = $r.Id; X = $r.Data.x; Y = $r.Data.y; Name = $name; Number = $num })
    }
    $symbolByUuid[$d.Uuid] = $pins
}

# ---------- Sheet ----------
$sheet = $docs | Where-Object { $_.Records | Where-Object Type -eq 'COMPONENT' } | Select-Object -First 1

$attrsByParent = @{}
foreach ($r in $sheet.Records | Where-Object Type -eq 'ATTR') {
    if ($r.Data -and $r.Data.parentId) {
        if (-not $attrsByParent.ContainsKey($r.Data.parentId)) { $attrsByParent[$r.Data.parentId] = New-Object System.Collections.ArrayList }
        [void]$attrsByParent[$r.Data.parentId].Add($r.Data)
    }
}

# Wires: wire id -> segment
$wires = @{}
foreach ($r in $sheet.Records | Where-Object Type -eq 'LINE') {
    if ($r.Data.lineGroup) {
        $wires[$r.Data.lineGroup] = [pscustomobject]@{
            Id = $r.Data.lineGroup; X1 = [double]$r.Data.startX; Y1 = [double]$r.Data.startY
            X2 = [double]$r.Data.endX; Y2 = [double]$r.Data.endY
        }
    }
}

# Wire net names from ATTR "Global Net Name" or "NET" attached to wires
$wireNetName = @{}
foreach ($r in $sheet.Records | Where-Object Type -eq 'ATTR') {
    if ($r.Data -and $r.Data.parentId -and $wires.ContainsKey($r.Data.parentId) -and
        ($r.Data.key -eq 'Global Net Name' -or $r.Data.key -eq 'NET')) {
        $wireNetName[$r.Data.parentId] = $r.Data.value
    }
}

# ---------- Placed components & absolute pins ----------
$pinNodes = New-Object System.Collections.ArrayList   # electrical nodes
function Get-AttrValue($compId, $key) {
    if ($attrsByParent.ContainsKey($compId)) {
        $a = $attrsByParent[$compId] | Where-Object { $_.key -eq $key } | Select-Object -First 1
        if ($a) { return $a.value }
    }
    return $null
}

$components = New-Object System.Collections.ArrayList
foreach ($r in $sheet.Records | Where-Object Type -eq 'COMPONENT') {
    $d = $r.Data
    $dev = $null
    if ($d.attrs -and $d.attrs.DeviceName) {
        try {
            $dn = $d.attrs.DeviceName | ConvertFrom-Json
            if ($deviceByUuid.ContainsKey($dn.uuid)) { $dev = $deviceByUuid[$dn.uuid] }
        } catch {}
    }
    if ($null -eq $dev -and $attrsByParent.ContainsKey($r.Id)) {
        # Net flags and some components reference their device/symbol via ATTR records
        $devUuid = Get-AttrValue $r.Id 'Device'
        if ($devUuid -and $deviceByUuid.ContainsKey($devUuid)) {
            $dev = $deviceByUuid[$devUuid]
        } else {
            $symUuid = Get-AttrValue $r.Id 'Symbol'
            $globalNet = Get-AttrValue $r.Id 'Global Net Name'
            if ($symUuid) {
                $dev = [pscustomobject]@{
                    Name = (Get-AttrValue $r.Id 'Name'); SymbolUuid = $symUuid; GlobalNet = $globalNet
                }
            }
        }
    }
    if ($null -eq $dev) { continue }
    $des = Get-AttrValue $r.Id 'Designator'
    if (-not $des) { $des = '?' }
    $comp = [pscustomobject]@{
        Id = $r.Id; Des = $des; Dev = $dev; X = [double]$d.x; Y = [double]$d.y
        Rot = [double]$d.rotation; Mirror = [bool]$d.isMirror
        IsNetFlag = [bool]$dev.GlobalNet
    }
    [void]$components.Add($comp)

    if ($symbolByUuid.ContainsKey($dev.SymbolUuid)) {
        foreach ($p in $symbolByUuid[$dev.SymbolUuid]) {
            $x = [double]$p.X; $y = [double]$p.Y
            # EasyEDA schematic: y-up, rotation CCW positive
            $rad = $comp.Rot * [Math]::PI / 180.0
            $ax = $x * [Math]::Cos($rad) - $y * [Math]::Sin($rad)
            $ay = $x * [Math]::Sin($rad) + $y * [Math]::Cos($rad)
            $ax += $comp.X; $ay += $comp.Y
            [void]$pinNodes.Add([pscustomobject]@{
                Comp = $comp; PinName = $p.Name; PinNum = $p.Number; X = $ax; Y = $ay
            })
        }
    }
}

# ---------- Union-find ----------
$uf = @{}
function Find-Root($k) {
    if (-not $uf.ContainsKey($k)) { $uf[$k] = $k; return $k }
    if ($uf[$k] -eq $k) { return $k }
    $uf[$k] = Find-Root $uf[$k]
    return $uf[$k]
}
function Union-Key($a, $b) {
    $ra = Find-Root $a; $rb = Find-Root $b
    if ($ra -ne $rb) { $uf[$ra] = $rb }
}

function Test-PointOnSeg($px, $py, $s) {
    $eps = 0.5
    $cross = ($s.X2 - $s.X1) * ($py - $s.Y1) - ($s.Y2 - $s.Y1) * ($px - $s.X1)
    if ([Math]::Abs($cross) -gt $eps) { return $false }
    $dot = ($px - $s.X1) * ($s.X2 - $s.X1) + ($py - $s.Y1) * ($s.Y2 - $s.Y1)
    $len2 = ($s.X2 - $s.X1) * ($s.X2 - $s.X1) + ($s.Y2 - $s.Y1) * ($s.Y2 - $s.Y1)
    if ($dot -lt -$eps -or $dot -gt $len2 + $eps) { return $false }
    return $true
}

# Pin -> wire
$pinIndex = 0
foreach ($pin in $pinNodes) {
    $pk = "pin$pinIndex"; $pinIndex++
    $pin | Add-Member -NotePropertyName Key -NotePropertyValue $pk
    foreach ($w in $wires.Values) {
        if (Test-PointOnSeg $pin.X $pin.Y $w) { Union-Key $pk $w.Id }
    }
}
# Pin -> pin (net flags sit directly on pins without a wire)
for ($i = 0; $i -lt $pinNodes.Count; $i++) {
    for ($j = $i + 1; $j -lt $pinNodes.Count; $j++) {
        $a = $pinNodes[$i]; $b = $pinNodes[$j]
        if ([Math]::Abs($a.X - $b.X) -le 0.5 -and [Math]::Abs($a.Y - $b.Y) -le 0.5) {
            Union-Key $a.Key $b.Key
        }
    }
}
# Wire endpoint -> wire (T junctions / chains)
foreach ($w1 in $wires.Values) {
    foreach ($w2 in $wires.Values) {
        if ($w1.Id -eq $w2.Id) { continue }
        $hit = (Test-PointOnSeg $w1.X1 $w1.Y1 $w2) -or (Test-PointOnSeg $w1.X2 $w1.Y2 $w2) -or
               (Test-PointOnSeg $w2.X1 $w2.Y1 $w1) -or (Test-PointOnSeg $w2.X2 $w2.Y2 $w1)
        if ($hit) { Union-Key $w1.Id $w2.Id }
    }
}

# ---------- Net naming ----------
$rootName = @{}
foreach ($w in $wires.Values) {
    if ($wireNetName.ContainsKey($w.Id)) {
        $rootName[(Find-Root $w.Id)] = $wireNetName[$w.Id]
    }
}
# Net flags (1-pin components with Global Net Name)
foreach ($pin in $pinNodes) {
    if ($pin.Comp.IsNetFlag) {
        $rootName[(Find-Root $pin.Key)] = $pin.Comp.Dev.GlobalNet
    }
}

# ---------- Output ----------
Write-Output ("COMPONENTS: {0}  PINS: {1}  WIRES: {2}" -f $components.Count, $pinNodes.Count, $wires.Count)
Write-Output ""

$ordered = $pinNodes | Sort-Object { $_.Comp.Des }, { [int]($_.PinNum -replace '\D.*$', '0') }
foreach ($pin in $ordered) {
    if ($pin.Comp.IsNetFlag) { continue }
    $root = Find-Root $pin.Key
    $net = '<NC>'
    if ($rootName.ContainsKey($root)) { $net = $rootName[$root] }
    elseif ($root -notin $wires.Values.Id) { $net = '<NC>' }
    else { $net = '<UNNAMED:' + $root.Substring(0, 8) + '>' }
    Write-Output ("{0,-5} pin{1,-3}({2,-6}) @({3,4},{4,4}) -> {5}" -f $pin.Comp.Des, $pin.PinNum, $pin.PinName, $pin.X, $pin.Y, $net)
}

param(
    [Parameter(Mandatory = $true)][string]$Path
)

$ErrorActionPreference = 'Stop'

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
        $current = [pscustomobject]@{ Uuid = $data.uuid; DocType = $data.docType; Records = New-Object System.Collections.ArrayList }
        [void]$docs.Add($current); continue
    }
    if ($null -ne $current) { [void]$current.Records.Add([pscustomobject]@{ Type = $head.type; Id = $head.id; Data = $data }) }
}

$sheet = $docs | Where-Object { $_.Records | Where-Object Type -eq 'COMPONENT' } | Select-Object -First 1

Write-Output "WIRES:"
foreach ($r in $sheet.Records | Where-Object Type -eq 'LINE') {
    if ($r.Data.lineGroup) {
        $name = ''
        foreach ($a in $sheet.Records | Where-Object { $_.Type -eq 'ATTR' -and $_.Data.parentId -eq $r.Data.lineGroup -and $_.Data.key -eq 'Global Net Name' }) { $name = $a.Data.value }
        Write-Output ("  {0}  ({1},{2})-({3},{4})  name='{5}'" -f $r.Data.lineGroup, $r.Data.startX, $r.Data.startY, $r.Data.endX, $r.Data.endY, $name)
    }
}

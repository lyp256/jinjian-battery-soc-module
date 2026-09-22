param(
    [Parameter(Mandatory = $true)][string]$Path
)

$ErrorActionPreference = 'Stop'

# Parse the EasyEDA Pro .esch2 JSON-lines file into document-scoped records.
$docs = New-Object System.Collections.ArrayList
$current = $null
$lineNo = 0

foreach ($line in Get-Content -LiteralPath $Path) {
    $lineNo++
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
            Index   = $docs.Count
            Uuid    = $data.uuid
            DocType = $data.docType
            Title   = ''
            Records = New-Object System.Collections.ArrayList
            FirstLine = $lineNo
        }
        [void]$docs.Add($current)
        continue
    }

    if ($null -ne $current) {
        [void]$current.Records.Add([pscustomobject]@{
            Line = $lineNo; Type = $head.type; Id = $head.id; Data = $data
        })
        if ($head.type -eq 'META' -and $data.title) {
            $current.Title = $data.title
        }
    }
}

Write-Output ("DOCUMENTS: {0}" -f $docs.Count)
foreach ($d in $docs) {
    $counts = $d.Records | Group-Object Type | ForEach-Object { "{0}={1}" -f $_.Name, $_.Count }
    Write-Output ("#{0} line={1} type={2} title='{3}' uuid={4} [{5}]" -f `
        $d.Index, $d.FirstLine, $d.DocType, $d.Title, $d.Uuid, ($counts -join ' '))
}

# Dump components of the sheet document (the one that owns COMPONENT records).
$sheet = $docs | Where-Object { $_.Records | Where-Object Type -eq 'COMPONENT' } | Select-Object -First 1
if ($null -eq $sheet) { Write-Output 'NO SHEET DOCUMENT FOUND'; exit }

Write-Output ""
Write-Output ("SHEET: #{0} title='{1}' uuid={2}" -f $sheet.Index, $sheet.Title, $sheet.Uuid)

# Attributes indexed by parentId.
$attrsByParent = @{}
foreach ($r in $sheet.Records) {
    if ($r.Type -eq 'ATTR' -and $r.Data -and $r.Data.parentId) {
        if (-not $attrsByParent.ContainsKey($r.Data.parentId)) {
            $attrsByParent[$r.Data.parentId] = New-Object System.Collections.ArrayList
        }
        [void]$attrsByParent[$r.Data.parentId].Add($r.Data)
    }
}

Write-Output ""
Write-Output "COMPONENTS (sheet):"
foreach ($r in $sheet.Records | Where-Object Type -eq 'COMPONENT') {
    $d = $r.Data
    $name = ''
    if ($d.attrs -and $d.attrs.DeviceName) {
        try { $name = ($d.attrs.DeviceName | ConvertFrom-Json).name } catch { $name = $d.attrs.DeviceName }
    }
    $des = ''
    $val = ''
    if ($attrsByParent.ContainsKey($r.Id)) {
        $des = ($attrsByParent[$r.Id] | Where-Object { $_.key -eq 'Designator' } | Select-Object -First 1).value
        $val = ($attrsByParent[$r.Id] | Where-Object { $_.key -eq 'Value' -or $_.key -eq 'Name' } | Select-Object -First 1).value
    }
    Write-Output ("  {0,-6} {1,-28} x={2,5} y={3,5} rot={4} val='{5}' id={6}" -f `
        $des, $name, $d.x, $d.y, $d.rotation, $val, $r.Id)
}

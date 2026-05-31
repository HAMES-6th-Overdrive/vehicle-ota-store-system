param(
    [string]$HexPath = ".\TriCore Debug (TASKING)\sensor-ecu.hex",
    [string]$OutDir = ".\ota_segments",

    # App Slot A 기준 logical address 범위
    [string]$AppBaseHex = "0x80020000",
    [string]$AppEndHex  = "0x80300000",

    # 이 값 이하의 gap은 하나의 segment로 병합
    [UInt64]$MergeGapBytes = 0x1000,

    # Flash page / transfer alignment
    [UInt64]$AlignBytes = 32
)

Add-Type @"
using System;

public static class Crc32Util
{
    static readonly uint[] Table = CreateTable();

    static uint[] CreateTable()
    {
        uint[] table = new uint[256];
        for (uint i = 0; i < 256; i++)
        {
            uint crc = i;
            for (int j = 0; j < 8; j++)
            {
                if ((crc & 1) != 0)
                    crc = (crc >> 1) ^ 0xEDB88320u;
                else
                    crc >>= 1;
            }
            table[i] = crc;
        }
        return table;
    }

    public static uint Compute(byte[] data)
    {
        uint crc = 0xFFFFFFFFu;
        foreach (byte b in data)
        {
            crc = (crc >> 8) ^ Table[(crc ^ b) & 0xFF];
        }
        return crc ^ 0xFFFFFFFFu;
    }
}
"@

function U64Hex([string]$hex) {
    $s = $hex.Trim()
    if ($s.StartsWith("0x") -or $s.StartsWith("0X")) {
        $s = $s.Substring(2)
    }
    return [Convert]::ToUInt64($s, 16)
}

function Align-Down([UInt64]$value, [UInt64]$align) {
    return [UInt64]($value - ($value % $align))
}

function Align-Up([UInt64]$value, [UInt64]$align) {
    $rem = $value % $align
    if ($rem -eq 0) {
        return $value
    }
    return [UInt64]($value + ($align - $rem))
}

function Normalize-Address([UInt64]$addr) {
    # AURIX PFLASH cached/non-cached alias 정리
    # 0xA0020000 -> 0x80020000
    $A_ALIAS_START = U64Hex "0xA0000000"
    $A_ALIAS_END   = U64Hex "0xAFFFFFFF"
    $ALIAS_OFFSET  = U64Hex "0x20000000"

    if ($addr -ge $A_ALIAS_START -and $addr -le $A_ALIAS_END) {
        return [UInt64]($addr - $ALIAS_OFFSET)
    }

    return $addr
}

function Read-IntelHex($path) {
    $mem = New-Object 'System.Collections.Generic.Dictionary[UInt64,Byte]'
    [UInt64]$upper = 0

    foreach ($lineRaw in Get-Content $path) {
        $line = $lineRaw.Trim()
        if ($line.Length -lt 11 -or -not $line.StartsWith(":")) {
            continue
        }

        $len  = [Convert]::ToInt32($line.Substring(1, 2), 16)
        $addr = [Convert]::ToUInt32($line.Substring(3, 4), 16)
        $type = [Convert]::ToInt32($line.Substring(7, 2), 16)

        if ($type -eq 0x00) {
            [UInt64]$base = $upper + $addr

            for ($i = 0; $i -lt $len; $i++) {
                $b = [Convert]::ToByte($line.Substring(9 + ($i * 2), 2), 16)
                $absolute = Normalize-Address ([UInt64]($base + $i))
                $mem[$absolute] = $b
            }
        }
        elseif ($type -eq 0x04) {
            $hi = [Convert]::ToUInt32($line.Substring(9, 4), 16)
            $upper = [UInt64]($hi -shl 16)
        }
        elseif ($type -eq 0x02) {
            $seg = [Convert]::ToUInt32($line.Substring(9, 4), 16)
            $upper = [UInt64]($seg -shl 4)
        }
        elseif ($type -eq 0x01) {
            break
        }
    }

    return $mem
}

function Build-AutoSegments(
    $mem,
    [UInt64]$appBase,
    [UInt64]$appEnd,
    [UInt64]$mergeGap,
    [UInt64]$align
) {
    $keys = @(
        $mem.Keys |
        Where-Object { $_ -ge $appBase -and $_ -lt $appEnd } |
        Sort-Object
    )

    if ($keys.Count -eq 0) {
        throw "No HEX data found in app range 0x$($appBase.ToString('X8')) ~ 0x$($appEnd.ToString('X8'))"
    }

    # 1) 실제 연속 주소 run 만들기
    $runs = @()
    [UInt64]$runStart = $keys[0]
    [UInt64]$prev = $keys[0]

    for ($i = 1; $i -lt $keys.Count; $i++) {
        [UInt64]$cur = $keys[$i]

        if ($cur -eq ($prev + 1)) {
            $prev = $cur
            continue
        }

        $runs += [PSCustomObject]@{
            Start = $runStart
            EndExclusive = [UInt64]($prev + 1)
        }

        $runStart = $cur
        $prev = $cur
    }

    $runs += [PSCustomObject]@{
        Start = $runStart
        EndExclusive = [UInt64]($prev + 1)
    }

    # 2) 작은 gap 병합
    $merged = @()
    [UInt64]$curStart = $runs[0].Start
    [UInt64]$curEnd = $runs[0].EndExclusive

    for ($i = 1; $i -lt $runs.Count; $i++) {
        [UInt64]$nextStart = $runs[$i].Start
        [UInt64]$nextEnd = $runs[$i].EndExclusive
        [UInt64]$gap = $nextStart - $curEnd

        if ($gap -le $mergeGap) {
            $curEnd = $nextEnd
        }
        else {
            $merged += [PSCustomObject]@{
                Start = Align-Down $curStart $align
                EndExclusive = Align-Up $curEnd $align
            }

            $curStart = $nextStart
            $curEnd = $nextEnd
        }
    }

    $merged += [PSCustomObject]@{
        Start = Align-Down $curStart $align
        EndExclusive = Align-Up $curEnd $align
    }

    return $merged
}

function Make-Segment(
    $mem,
    [int]$index,
    [UInt64]$start,
    [UInt64]$endExclusive,
    [UInt64]$appBase,
    [string]$outDir
) {
    [UInt64]$size64 = $endExclusive - $start
    if ($size64 -gt [UInt64][Int32]::MaxValue) {
        throw "Segment too large"
    }

    $size = [int]$size64
    $data = New-Object byte[] $size

    # 빈 영역은 erased flash 상태인 0xFF로 채움
    for ($i = 0; $i -lt $size; $i++) {
        $data[$i] = 0xFF
    }

    $used = 0
    for ($i = 0; $i -lt $size; $i++) {
        [UInt64]$addr = $start + [UInt64]$i
        if ($mem.ContainsKey($addr)) {
            $data[$i] = $mem[$addr]
            $used++
        }
    }

    $name = "segment$index"
    $path = Join-Path $outDir "$name.bin"
    [IO.File]::WriteAllBytes($path, $data)

    $crc = [Crc32Util]::Compute($data)
    $offsetFromAppBase = [UInt64]($start - $appBase)

    return [PSCustomObject]@{
        index = $index
        name = $name
        address = ("0x{0:X8}" -f $start)
        offsetFromAppBase = ("0x{0:X8}" -f $offsetFromAppBase)
        endExclusive = ("0x{0:X8}" -f $endExclusive)
        size = $size
        usedBytesFromHex = $used
        fillBytes = ($size - $used)
        crc32 = ("0x{0:X8}" -f $crc)
        file = $path
    }
}

if (-not (Test-Path $HexPath)) {
    throw "HEX not found: $HexPath"
}

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

$appBase = U64Hex $AppBaseHex
$appEnd  = U64Hex $AppEndHex

Write-Host "[1] Read HEX: $HexPath"
$mem = Read-IntelHex $HexPath
Write-Host "[2] HEX bytes loaded:" $mem.Count

Write-Host ("[3] App range: 0x{0:X8} ~ 0x{1:X8}" -f $appBase, $appEnd)
Write-Host ("[4] Merge gap <= {0} bytes, align={1} bytes" -f $MergeGapBytes, $AlignBytes)

$autoSegments = Build-AutoSegments $mem $appBase $appEnd $MergeGapBytes $AlignBytes

$result = @()
$idx = 1
foreach ($seg in $autoSegments) {
    $r = Make-Segment $mem $idx ([UInt64]$seg.Start) ([UInt64]$seg.EndExclusive) $appBase $OutDir
    $result += $r

    Write-Host ("[SEG] {0}: {1} ~ {2}, offset={3}, size={4}, used={5}, fill={6}, crc={7}" -f `
        $r.name, $r.address, $r.endExclusive, $r.offsetFromAppBase, `
        $r.size, $r.usedBytesFromHex, $r.fillBytes, $r.crc32)

    $idx++
}

$totalSize = ($result | Measure-Object -Property size -Sum).Sum
$totalUsed = ($result | Measure-Object -Property usedBytesFromHex -Sum).Sum
$totalFill = ($result | Measure-Object -Property fillBytes -Sum).Sum

# ------------------------------------------------------------
# Virtual image CRC
# - CAN으로는 segment data만 보냄
# - 하지만 Bootloader는 AppBase부터 VirtualEnd까지 연속 CRC를 계산한다고 가정
# - HEX에 없는 gap은 erased Flash 상태인 0xFF로 채움
# ------------------------------------------------------------

[UInt64]$virtualEnd = 0
foreach ($seg in $autoSegments) {
    if ([UInt64]$seg.EndExclusive -gt $virtualEnd) {
        $virtualEnd = [UInt64]$seg.EndExclusive
    }
}

$virtualSize64 = [UInt64]($virtualEnd - $appBase)
if ($virtualSize64 -gt [UInt64][Int32]::MaxValue) {
    throw "Virtual image too large"
}

$virtualSize = [int]$virtualSize64
$virtualData = New-Object byte[] $virtualSize

# Sparse OTA 실제 Flash gap 기준:
# segment와 segment 사이의 큰 gap은 erased/read 상태인 0x00으로 계산한다.
for ($i = 0; $i -lt $virtualSize; $i++) {
    $virtualData[$i] = 0x00
}

# 중요:
# virtual CRC는 HEX byte만 overlay하면 안 된다.
# 실제 CAN으로 전송되는 것은 segmentN.bin이고,
# segmentN.bin 내부의 alignment fill byte(현재 0xFF)도 실제 Flash에 써진다.
# 따라서 result에 기록된 segment 파일을 그대로 virtual image에 overlay한다.
$virtualUsed = 0
foreach ($segInfo in $result) {
    $segBytes = [IO.File]::ReadAllBytes($segInfo.file)

    $segOffsetStr = $segInfo.offsetFromAppBase
    $segOffset = U64Hex $segOffsetStr

    if (($segOffset + [UInt64]$segBytes.Length) -gt [UInt64]$virtualSize) {
        throw "Segment exceeds virtual image range: $($segInfo.name)"
    }

    [Array]::Copy($segBytes, 0, $virtualData, [int]$segOffset, $segBytes.Length)

    $virtualUsed += $segBytes.Length
}

$virtualCrc = [Crc32Util]::Compute($virtualData)

$manifest = [PSCustomObject]@{
    format = "OVD_SPARSE_OTA_MANIFEST_V1"
    sourceHex = $HexPath
    note = "Generated from Intel HEX. 0xAxxxxxxx addresses are normalized to 0x8xxxxxxx. Only App range is included."

    appBase = ("0x{0:X8}" -f $appBase)
    appEnd = ("0x{0:X8}" -f $appEnd)

    virtualBase = ("0x{0:X8}" -f $appBase)
    virtualEnd = ("0x{0:X8}" -f $virtualEnd)
    virtualSize = $virtualSize
    virtualImageCrc32 = ("0x{0:X8}" -f $virtualCrc)
    virtualGapFill = "0x00"
    virtualUsedBytesFromHex = $virtualUsed
    virtualFillBytes = ($virtualSize - $virtualUsed)

    mergeGapBytes = $MergeGapBytes
    alignBytes = $AlignBytes

    segmentCount = $result.Count
    totalPayloadSize = $totalSize
    totalUsedBytesFromHex = $totalUsed
    totalFillBytes = $totalFill

    segments = $result
}

$manifestPath = Join-Path $OutDir "manifest.json"
$manifest | ConvertTo-Json -Depth 8 | Set-Content -Encoding UTF8 $manifestPath

Write-Host "[5] Manifest:" $manifestPath
Write-Host "[6] Total payload size:" $totalSize "bytes"
Write-Host "[7] Total used bytes   :" $totalUsed "bytes"
Write-Host "[8] Total fill bytes   :" $totalFill "bytes"

Write-Host ("[9] Virtual image: 0x{0:X8} ~ 0x{1:X8}, size={2} bytes" -f `
    $appBase, $virtualEnd, $virtualSize)

Write-Host ("[10] Virtual CRC32 : 0x{0:X8}" -f $virtualCrc)
Write-Host ("[11] Virtual gap fill: 0x00, gap bytes={0}" -f ($virtualSize - $virtualUsed))
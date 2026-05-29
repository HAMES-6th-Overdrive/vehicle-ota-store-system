param(
    [string]$Segment1Path = ".\ota_segments\segment1.bin",
    [string]$Segment2Path = ".\ota_segments\segment2.bin",
    [UInt16]$Channel = 0x51,
    [string]$BitrateFD = "f_clock_mhz=80,nom_brp=2,nom_tseg1=63,nom_tseg2=16,nom_sjw=16,data_brp=2,data_tseg1=15,data_tseg2=4,data_sjw=4",
    [UInt32]$ReqId = 0x600,
    [UInt32]$RespId = 0x601,
    [int]$BlockSize = 32,
    [int]$ResponseTimeoutMs = 60000
)

Add-Type @"
using System;
using System.Runtime.InteropServices;
using System.Text;

public static class PeakCan
{
    public const UInt32 PCAN_ERROR_OK = 0x00000;
    public const UInt32 PCAN_ERROR_QRCVEMPTY = 0x00020;

    public const byte PCAN_MESSAGE_FD = 0x04;

    [StructLayout(LayoutKind.Sequential)]
    public struct TPCANMsgFD
    {
        public UInt32 ID;
        public byte MSGTYPE;
        public byte DLC;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 64)]
        public byte[] DATA;
    }

    [DllImport("PCANBasic.dll", EntryPoint="CAN_InitializeFD")]
    public static extern UInt32 InitializeFD(UInt16 Channel, StringBuilder BitrateFD);

    [DllImport("PCANBasic.dll", EntryPoint="CAN_Uninitialize")]
    public static extern UInt32 Uninitialize(UInt16 Channel);

    [DllImport("PCANBasic.dll", EntryPoint="CAN_WriteFD")]
    public static extern UInt32 WriteFD(UInt16 Channel, ref TPCANMsgFD MessageBuffer);

    [DllImport("PCANBasic.dll", EntryPoint="CAN_ReadFD")]
    public static extern UInt32 ReadFD(UInt16 Channel, out TPCANMsgFD MessageBuffer, out UInt64 TimestampBuffer);
}
"@

function HexStr($bytes, [int]$count = -1) {
    if ($count -lt 0) {
        $count = $bytes.Length
    }

    if ($count -le 0) {
        return ""
    }

    return (($bytes[0..($count - 1)] | ForEach-Object { "{0:X2}" -f $_ }) -join " ")
}

function U32LE([UInt32]$value) {
    return [byte[]]@(
        [byte]($value -band 0xFF),
        [byte](($value -shr 8) -band 0xFF),
        [byte](($value -shr 16) -band 0xFF),
        [byte](($value -shr 24) -band 0xFF)
    )
}

function New-FdMsg([UInt32]$id, [byte[]]$payload) {
    $msg = New-Object PeakCan+TPCANMsgFD
    $msg.ID = $id
    $msg.MSGTYPE = [PeakCan]::PCAN_MESSAGE_FD
    $msg.DLC = 15
    $msg.DATA = New-Object byte[] 64

    for ($i = 0; $i -lt 64; $i++) {
        $msg.DATA[$i] = 0
    }

    for ($i = 0; $i -lt $payload.Length -and $i -lt 64; $i++) {
        $msg.DATA[$i] = $payload[$i]
    }

    return $msg
}

function Drain-Rx() {
    while ($true) {
        $rx = New-Object PeakCan+TPCANMsgFD
        [UInt64]$ts = 0
        $st = [PeakCan]::ReadFD($Channel, [ref]$rx, [ref]$ts)

        if ($st -ne [PeakCan]::PCAN_ERROR_OK) {
            break
        }
    }
}

function Send-Frame([byte[]]$payload) {
    $msg = New-FdMsg $ReqId $payload
    $st = [PeakCan]::WriteFD($Channel, [ref]$msg)

    if ($st -ne [PeakCan]::PCAN_ERROR_OK) {
        throw ("CAN_WriteFD failed. status=0x{0:X8}, payload={1}" -f $st, (HexStr $payload))
    }
}

function Wait-Response([byte]$positiveSid, [int]$expectedSeq) {
    $sw = [Diagnostics.Stopwatch]::StartNew()

    while ($sw.ElapsedMilliseconds -lt $ResponseTimeoutMs) {
        $rx = New-Object PeakCan+TPCANMsgFD
        [UInt64]$ts = 0
        $st = [PeakCan]::ReadFD($Channel, [ref]$rx, [ref]$ts)

        if ($st -eq [PeakCan]::PCAN_ERROR_QRCVEMPTY) {
            Start-Sleep -Milliseconds 1
            continue
        }

        if ($st -ne [PeakCan]::PCAN_ERROR_OK) {
            throw ("CAN_ReadFD failed. status=0x{0:X8}" -f $st)
        }

        if ($rx.ID -ne $RespId) {
            continue
        }

        if ($rx.DATA[0] -eq 0x7F) {
            throw ("Negative Response: 7F {0:X2} {1:X2}" -f $rx.DATA[1], $rx.DATA[2])
        }

        if ($rx.DATA[0] -ne $positiveSid) {
            continue
        }

        if ($expectedSeq -ge 0) {
            if ($rx.DATA[1] -ne [byte]$expectedSeq) {
                continue
            }
        }

        return $rx
    }

    throw ("Timeout waiting response SID=0x{0:X2}" -f $positiveSid)
}

function Send-And-Expect([byte[]]$payload, [byte]$positiveSid, [int]$expectedSeq = -1) {
    Drain-Rx
    Send-Frame $payload
    $rx = Wait-Response $positiveSid $expectedSeq
    Write-Host ("[OK] TX={0}  RX={1}" -f (HexStr $payload), (HexStr $rx.DATA 8))
}

function Send-RequestDownload([UInt32]$offsetFromAppBase, [UInt32]$segmentSize) {
    $offsetLe = U32LE $offsetFromAppBase
    $sizeLe = U32LE $segmentSize

    $payload = New-Object byte[] 11
    $payload[0] = 0x34
    $payload[1] = 0x00
    $payload[2] = 0x44

    $payload[3] = $offsetLe[0]
    $payload[4] = $offsetLe[1]
    $payload[5] = $offsetLe[2]
    $payload[6] = $offsetLe[3]

    $payload[7] = $sizeLe[0]
    $payload[8] = $sizeLe[1]
    $payload[9] = $sizeLe[2]
    $payload[10] = $sizeLe[3]

    Send-And-Expect $payload 0x74
}

function Send-Segment([string]$name, [string]$path, [UInt32]$offsetFromAppBase) {
    if (-not (Test-Path $path)) {
        throw ("File not found: {0}" -f $path)
    }

    $segment = [IO.File]::ReadAllBytes($path)

    if (($segment.Length % $BlockSize) -ne 0) {
        throw ("Segment size is not aligned. size={0}" -f $segment.Length)
    }

    Write-Host ""
    Write-Host ("=== {0} START ===" -f $name)
    Write-Host ("Path   = {0}" -f $path)
    Write-Host ("Offset = 0x{0:X8}" -f $offsetFromAppBase)
    Write-Host ("Size   = {0} bytes" -f $segment.Length)

    Send-RequestDownload $offsetFromAppBase ([UInt32]$segment.Length)

    $blockCount = [int]($segment.Length / $BlockSize)
    [byte]$seq = 0x01

    for ($block = 0; $block -lt $blockCount; $block++) {
        $offset = $block * $BlockSize

        $payload = New-Object byte[] (2 + $BlockSize)
        $payload[0] = 0x36
        $payload[1] = $seq

        [Array]::Copy($segment, $offset, $payload, 2, $BlockSize)

        Send-And-Expect $payload 0x76 ([int]$seq)

        if (($block % 50) -eq 0 -or $block -eq ($blockCount - 1)) {
            Write-Host ("{0} Progress: {1}/{2} blocks, seq=0x{3:X2}" -f $name, ($block + 1), $blockCount, $seq)
        }

        $seq = [byte](($seq + 1) -band 0xFF)
    }

    [byte[]]$req37 = @(0x37)
    Send-And-Expect $req37 0x77

    Write-Host ("=== {0} DONE ===" -f $name)
}

Write-Host "=== Sparse Segment OTA Test ==="
Write-Host ("Channel = 0x{0:X2}" -f $Channel)

$initSt = [PeakCan]::InitializeFD($Channel, [Text.StringBuilder]::new($BitrateFD))

if ($initSt -ne [PeakCan]::PCAN_ERROR_OK) {
    throw ("CAN_InitializeFD failed. status=0x{0:X8}" -f $initSt)
}

try {
    Drain-Rx

    [byte[]]$req10 = @(0x10, 0x02)
    Send-And-Expect $req10 0x50

    Send-Segment "SEGMENT1" $Segment1Path 0x00000000
    Send-Segment "SEGMENT2" $Segment2Path 0x002DE020

    # Final RoutineControl CRC
    # virtualImageCrc32 = 0x79BBA762
    # little endian = 62 A7 BB 79
    [byte[]]$req31 = @(0x31, 0x01, 0x02, 0x02, 0x92, 0x4F, 0x1D, 0x83)

    Send-And-Expect $req31 0x71

    Write-Host ""
    Write-Host "=== SEGMENT1 + SEGMENT2 + FINAL CRC DONE ==="
    Write-Host "Expected: ECU stores pending flag and resets."
    Start-Sleep -Milliseconds 1000
}
finally {
    [void][PeakCan]::Uninitialize($Channel)
    Write-Host "PCAN uninitialized."
}
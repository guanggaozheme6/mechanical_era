# padview.ps1 - PS5 -> PS2 bridge terminal viewer (fast: no Clear-Host)

$portName = 'COM10'
$port = New-Object System.IO.Ports.SerialPort $portName, 921600, None, 8, one
$port.ReadTimeout = 100
try { $port.Open() } catch {
    [Console]::WriteLine("Cannot open $portName : $($_.Exception.Message)")
    exit 1
}

$conn = 0; $dpad = 8; $btn = 0xFFFF
$lx = 128; $ly = 128; $rx = 128; $ry = 128
$beats = 0

function Bar([int]$v) {
    $pos = [math]::Min(20, [math]::Max(0, [math]::Round($v / 255.0 * 20)))
    return ('.' * $pos) + '@' + ('.' * (20 - $pos))
}

function Cell([string]$name, [bool]$on) {
    if ($on) { return '[' + $name + ']' }
    return ' ' + $name + ' '
}

function Draw {
    $sb = New-Object System.Text.StringBuilder
    if ($conn -eq 1) { $state = 'CONNECTED' } else { $state = 'WAITING  ' }
    [void]$sb.AppendLine(("PS5 -> PS2 bridge    {0}  dpad={1}  beats={2}" -f $state, $dpad, $beats))
    [void]$sb.AppendLine('=' * 60)
    [void]$sb.AppendLine(("LX [{0}] {1,3}" -f (Bar $lx), $lx))
    [void]$sb.AppendLine(("LY [{0}] {1,3}" -f (Bar $ly), $ly))
    [void]$sb.AppendLine(("RX [{0}] {1,3}" -f (Bar $rx), $rx))
    [void]$sb.AppendLine(("RY [{0}] {1,3}" -f (Bar $ry), $ry))
    [void]$sb.AppendLine('=' * 60)
    $keys = @(
        @('Cross', 14), @('Circ', 13), @('Squ', 15), @('Tri', 12),
        @('L1', 10), @('R1', 11), @('L2', 8), @('R2', 9),
        @('L3', 1), @('R3', 2), @('Start', 3), @('Sel', 0)
    )
    $line = ''
    foreach ($k in $keys) {
        $on = (($btn -band (1 -shl $k[1])) -eq 0)
        $line = $line + (Cell $k[0] $on)
    }
    [void]$sb.AppendLine($line)
    $dline = 'D-Pad:'
    foreach ($d in @(@('Up', 4), @('Right', 5), @('Down', 6), @('Left', 7))) {
        $on = (($btn -band (1 -shl $d[1])) -eq 0)
        $dline = $dline + (Cell $d[0] $on)
    }
    [void]$sb.AppendLine($dline)
    [void]$sb.AppendLine('=' * 60)
    [void]$sb.Append('Ctrl+C to exit')
    try { [Console]::SetCursorPosition(0, 0) } catch {}
    [Console]::Write($sb.ToString())
}

try { [Console]::Clear() } catch {}
$lastDraw = 0
try {
    while ($true) {
        $data = $port.ReadExisting()
        if ($data.Length -gt 0) {
            foreach ($ln in ($data -split "`n")) {
                if ($ln -match '\[beat\] conn=(\d) tx0=0x([0-9A-F]{2}) dpad=(\d) btn=([0-9A-F]{4}) lx=(\d+) ly=(\d+) rx=(\d+) ry=(\d+)') {
                    $conn = [int]$Matches[1]
                    $dpad = [int]$Matches[3]
                    $btn = [Convert]::ToInt32($Matches[4], 16)
                    $lx = [int]$Matches[5]; $ly = [int]$Matches[6]
                    $rx = [int]$Matches[7]; $ry = [int]$Matches[8]
                    $beats++
                }
            }
        }
        $now = [Environment]::TickCount
        if (($now - $lastDraw) -ge 33) {
            $lastDraw = $now
            Draw
        }
        Start-Sleep -Milliseconds 5
    }
} finally {
    if ($port.IsOpen) { $port.Close() }
}

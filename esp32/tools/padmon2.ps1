param(
    [string]$Port = "COM10",
    [int]$Baud = 115200
)

$ErrorActionPreference = "Stop"
$esc = [char]27
$cHome = "$esc[H"
$green = "$esc[32m"
$dim = "$esc[2m"
$reset = "$esc[0m"

function Get-HBar {
    param([int]$V, [int]$Width = 21)
    $p = [int][math]::Round(($V + 511) * ($Width - 1) / 1023.0)
    if ($p -lt 0) { $p = 0 }
    if ($p -gt ($Width - 1)) { $p = $Width - 1 }
    $mid = [int]($Width / 2)
    $s = ""
    for ($i = 0; $i -lt $Width; $i++) {
        if ($i -eq $p) {
            $s += "$green@$reset"
        } elseif ($i -eq $mid) {
            $s += "$dim|$reset"
        } else {
            $s += "."
        }
    }
    return $s
}

function Get-Bar {
    param([int]$V, [int]$Full = 255, [int]$Width = 10)
    $n = [int][math]::Round([math]::Max(0, [math]::Min($Full, $V)) * $Width / [double]$Full)
    return "[" + ("#" * $n) + ("-" * ($Width - $n)) + "]"
}

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$sp.ReadTimeout = 200
try {
    $sp.Open()
} catch {
    Write-Host "open $Port failed: $($_.Exception.Message)"
    exit 1
}

[Console]::Write("$esc[2J")
$pattern = 'buttons: (0x[0-9A-Fa-f]+), axis L: *(-?\d+), *(-?\d+), axis R: *(-?\d+), *(-?\d+), brake: *(\d+), throttle: *(\d+)'
$count = 0
$t0 = Get-Date
$rate = 0.0

try {
    while ($true) {
        $line = $sp.ReadLine()
        if ($line -match $pattern) {
            $count++
            $now = Get-Date
            $dt = ($now - $t0).TotalSeconds
            if ($dt -ge 1.0) {
                $rate = $count / $dt
                $count = 0
                $t0 = $now
            }
            $btn = [Convert]::ToInt32($Matches[1].Substring(2), 16)
            $lx = [int]$Matches[2]; $ly = [int]$Matches[3]
            $rx = [int]$Matches[4]; $ry = [int]$Matches[5]
            $brake = [int]$Matches[6]; $throttle = [int]$Matches[7]

            $rateStr = $rate.ToString("0.0").PadLeft(5)
            $sb = New-Object System.Text.StringBuilder
            [void]$sb.Append($cHome)
            [void]$sb.Append("+" + ("-" * 50) + "+`r`n")
            [void]$sb.Append("| RAINBOW 2 PRO  " + $Port.PadRight(6) + "  " + $rateStr + " pkt/s       |`r`n")
            [void]$sb.Append("+" + ("-" * 50) + "+`r`n")
            [void]$sb.Append("| LX " + (Get-HBar $lx) + " " + $lx.ToString().PadLeft(5) + "      |`r`n")
            [void]$sb.Append("| LY " + (Get-HBar $ly) + " " + $ly.ToString().PadLeft(5) + "      |`r`n")
            [void]$sb.Append("| RX " + (Get-HBar $rx) + " " + $rx.ToString().PadLeft(5) + "      |`r`n")
            [void]$sb.Append("| RY " + (Get-HBar $ry) + " " + $ry.ToString().PadLeft(5) + "      |`r`n")
            [void]$sb.Append("+" + ("-" * 50) + "+`r`n")
            [void]$sb.Append("| BTN 0x" + $btn.ToString("X4") + "   BRAKE " + (Get-Bar $brake) + "   |`r`n")
            [void]$sb.Append("|           THROTTLE " + (Get-Bar $throttle) + "   |`r`n")
            [void]$sb.Append("+" + ("-" * 50) + "+`r`n")
            [void]$sb.Append("  Ctrl+C to quit`r`n")
            [Console]::Write($sb.ToString())
        }
    }
} finally {
    $sp.Close()
}

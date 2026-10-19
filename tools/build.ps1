# =====================================================================
#  2024_H 固件一键构建脚本（不依赖 CCS GUI）
#  用法：  pwsh -File tools\build.ps1
#  产物：  firmware\build\2024_H.out / .map / .hex
#  依赖：  TI MSPM0 SDK 2.x + SysConfig + TI Arm Clang（CCS 自带即可）
# =====================================================================
param(
    [string]$Sdk        = "C:\ti\mspm0_sdk_2_11_00_07",
    [string]$SysConfig  = "",
    [string]$CcsRoot    = "D:\TI\CCS\ccs"
)

$ErrorActionPreference = "Continue"
$root = Split-Path -Parent $PSScriptRoot
$fw   = Join-Path $root "firmware"
$out  = Join-Path $root "build"

function Find-First($paths, $pattern) {
    foreach ($p in $paths) {
        $hit = Get-ChildItem -Path $p -Filter $pattern -Recurse -ErrorAction SilentlyContinue |
               Select-Object -First 1
        if ($hit) { return $hit.FullName }
    }
    return $null
}

if (-not (Test-Path $Sdk)) { throw "找不到 MSPM0 SDK: $Sdk" }

if (-not $SysConfig) {
    $SysConfig = Find-First @("C:\ti", "$CcsRoot\utils") "sysconfig_cli.bat"
}
if (-not $SysConfig) { throw "找不到 sysconfig_cli.bat，请用 -SysConfig 指定" }

$tiarmclang = Find-First @("$CcsRoot\tools\compiler") "tiarmclang.exe"
if (-not $tiarmclang) { throw "找不到 tiarmclang.exe（CCS 未安装？）" }
$tcRoot = Split-Path -Parent (Split-Path -Parent $tiarmclang)

Write-Host "SDK        : $Sdk"
Write-Host "SysConfig  : $SysConfig"
Write-Host "Compiler   : $tiarmclang"

New-Item -ItemType Directory -Force $out | Out-Null

# ---------- 1. SysConfig 生成 ti_msp_dl_config.* ----------
Write-Host "`n[1/3] SysConfig ..."
& $SysConfig -s "$Sdk\.metadata\product.json" --script "$fw\2024_H.syscfg" -o $out --compiler ticlang *> "$out\sysconfig.log"
if ($LASTEXITCODE -ne 0) { Get-Content "$out\sysconfig.log"; throw "SysConfig 失败" }

# ---------- 2. 编译 ----------
$common = @(
    "-march=thumbv6m", "-mcpu=cortex-m0plus", "-mfloat-abi=soft",
    "-mlittle-endian", "-mthumb", "-O2", "-g", "-Wall",
    "-I$fw", "-I$out",
    "-I$Sdk\source", "-I$Sdk\source\third_party\CMSIS\Core\Include",
    "@$out\device.opt"
)

$srcs = @(Get-ChildItem $fw -Filter *.c | Where-Object { $_.DirectoryName -eq $fw }) + @(Get-Item "$out\ti_msp_dl_config.c")
Write-Host "`n[2/3] Compile $($srcs.Count) files ..."
$objs = @()
foreach ($s in $srcs) {
    $o = Join-Path $out ($s.BaseName + ".o")
    Write-Host "  CC  $($s.Name)"
    & $tiarmclang @common -c $s.FullName -o $o
    if ($LASTEXITCODE -ne 0) { throw "编译失败: $($s.Name)" }
    $objs += $o
}

# ---------- 3. 链接 ----------
Write-Host "`n[3/3] Link ..."
$link = @(
    "-march=thumbv6m", "-mcpu=cortex-m0plus", "-mfloat-abi=soft",
    "-mlittle-endian", "-mthumb", "-O2", "-g",
    "@$out\device.opt",
    "-Wl,-m$out\2024_H.map",
    "-Wl,-i$Sdk\source",
    "-Wl,-i$fw",
    "-Wl,-i$out",
    "-Wl,-i$tcRoot\lib",
    "-Wl,--diag_wrap=off",
    "-Wl,--display_error_number",
    "-Wl,--warn_sections",
    "-Wl,--rom_model",
    "-o", "$out\2024_H.out"
) + $objs + @(
    "-Wl,-ldevice.cmd.genlibs",
    "-Wl,-llibc.a",
    "-Wl,-l$out\device_linker.cmd",
    "-Wl,--stack_size=1536"
)
& $tiarmclang @link
if ($LASTEXITCODE -ne 0) { throw "链接失败" }

# ---------- 4. hex ----------
$objcopy = Find-First @("$tcRoot\bin") "tiarmobjcopy.exe"
if ($objcopy) {
    & $objcopy -O ihex "$out\2024_H.out" "$out\2024_H.hex"
}

$size = (Get-Item "$out\2024_H.out").Length
Write-Host "`nOK -> $out\2024_H.out ($size bytes)"
Get-ChildItem $out -Filter "2024_H.*" | Select-Object Name, Length | Format-Table -AutoSize
exit 0

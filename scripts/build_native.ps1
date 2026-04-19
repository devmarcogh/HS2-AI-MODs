<# 
    Build script for StudioModsNative.dll (C++ SIMD PBD solver)
    Requires: CMake 3.15+, Visual Studio 2019+ with C++ Desktop workload
    
    Usage:  .\scripts\build_native.ps1 [-Config Release|Debug]
    Output: Native\build\bin\StudioModsNative.dll
#>
param(
    [ValidateSet("Release", "Debug")]
    [string]$Config = "Release"
)

$ErrorActionPreference = "Stop"

$nativeDir = Join-Path $PSScriptRoot "..\Native"
$buildDir  = Join-Path $nativeDir "build"

Write-Host "=== Building StudioModsNative ($Config) ===" -ForegroundColor Cyan

# Create build directory
if (!(Test-Path $buildDir)) {
    New-Item -ItemType Directory -Path $buildDir | Out-Null
}

Push-Location $buildDir
try {
    # Configure â€” auto-detect VS version
    Write-Host "Configuring with CMake..." -ForegroundColor Yellow
    
    $generators = @(
        "Visual Studio 18 2026",
        "Visual Studio 17 2022",
        "Visual Studio 16 2019"
    )
    
    $configured = $false
    foreach ($gen in $generators) {
        Write-Host "  Trying: $gen" -ForegroundColor Gray
        cmake .. -G $gen -A x64 2>&1 | Out-Null
        if ($LASTEXITCODE -eq 0) {
            Write-Host "  Using: $gen" -ForegroundColor Green
            $configured = $true
            break
        }
        # Clean failed cache before retrying
        Remove-Item CMakeCache.txt -ErrorAction SilentlyContinue
        Remove-Item CMakeFiles -Recurse -Force -ErrorAction SilentlyContinue
    }
    if (-not $configured) { throw "CMake configure failed â€” no supported Visual Studio found" }

    # Build
    Write-Host "Building..." -ForegroundColor Yellow
    cmake --build . --config $Config
    if ($LASTEXITCODE -ne 0) { throw "CMake build failed" }

    $dllPath = Join-Path $buildDir "bin\$Config\StudioModsNative.dll"
    if (!(Test-Path $dllPath)) {
        # Some generators put it directly in bin/
        $dllPath = Join-Path $buildDir "bin\StudioModsNative.dll"
    }

    if (Test-Path $dllPath) {
        Write-Host "Success: $dllPath" -ForegroundColor Green
        Write-Host ""
        Write-Host "To install, copy to your game's Plugins folder:" -ForegroundColor Cyan
        Write-Host "  Copy-Item `"$dllPath`" `"D:\Documents\CAARP\StudioNEOV2_Data\Plugins\`"" -ForegroundColor White
    }
    else {
        Write-Host "Warning: DLL not found at expected path. Check build output above." -ForegroundColor Yellow
    }
}
finally {
    Pop-Location
}

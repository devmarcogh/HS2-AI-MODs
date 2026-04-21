@echo off
REM Post-build script: Copia DLLs y ejecuta el EXE

REM Rutas de salida
setlocal
set NET_DLL=%~dp0bin\Debug\HS2StudioCharaModsMSG.dll
set CPP_DLL=%~dp0Native\build\bin\Debug\StudioModsNative.dll

REM Destinos
set NET_DEST="D:\Documents\CAARP\BepinEx"
set CPP_DEST="D:\Documents\CAARP\StudioNEOV2_Data\Plugins"
set EXE_PATH="D:\Documents\CAARP\StudioNEOV2.exe"

REM Copiar DLL .NET
copy /Y %NET_DLL% %NET_DEST%
REM Copiar DLL C++
copy /Y %CPP_DLL% %CPP_DEST%

REM Ejecutar el EXE
echo Lanzando StudioNEOV2.exe...
start "StudioNEOV2" %EXE_PATH%

REM (Opcional) Attach debugger: ver instrucciones en README
endlocal

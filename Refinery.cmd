@echo off
rem Double-click to play. Builds the simulator library Godot loads, prepares
rem Godot's extension cache on a fresh clone, then opens the chosen screen.
rem
rem   Refinery.cmd            the menu: every screen and every plant
rem   Refinery.cmd column     straight to one screen (column, reactor, pump,
rem                           furnace, plant or map)
rem
rem Needs godot (>= 4.7) on PATH; cargo too, unless the library is built already.
rem Set REFINERY_GODOT_ARGS to run a screen in this console instead of a window
rem of its own, e.g.  --headless --quit-after 2000 -- --auto
setlocal
pushd "%~dp0"

where godot >nul 2>&1
if errorlevel 1 goto no_godot

set "DLL=target\godot\godot\refinery_godot_ext.dll"

rem The build command is the one refinery.gdextension names: any other
rem target dir or feature set writes a library Godot cannot load.
where cargo >nul 2>&1
if errorlevel 1 goto no_cargo
echo Building the simulator (quick when nothing changed; minutes the first time)...
cmd /v:on /c "start /belownormal /b /wait cargo build -p refinery-godot-ext --features godot --target-dir target/godot --profile godot & exit !errorlevel!"
if errorlevel 1 goto build_failed
goto cache

:no_cargo
if not exist "%DLL%" goto no_library
echo cargo not found: using the simulator library built earlier.
goto cache

:build_failed
if not exist "%DLL%" goto no_library
echo.
echo The build failed - often because Godot is already open and holds the
echo library. Starting with the library built earlier.
timeout /t 5 >nul

:cache
rem Outside the editor Godot finds extensions only through this file, which
rem only the editor writes. Its shutdown exits nonzero; that is harmless.
if exist ".godot\extension_list.cfg" goto choose
echo First run: letting Godot index the project once...
godot --headless --path . --editor --quit >nul 2>&1

:choose
rem No screen named: the menu scene (demo\menu.tscn), which lists every
rem screen and every plant each opens. Esc in a screen comes back to it.
set "SCENE=%~1"
if "%SCENE%"=="" set "SCENE=menu"

:launch
if not exist "demo\%SCENE%.tscn" goto bad_scene
if defined REFINERY_GODOT_ARGS goto launch_here
start "" godot --path . res://demo/%SCENE%.tscn
popd
exit /b 0

:launch_here
godot --path . res://demo/%SCENE%.tscn %REFINERY_GODOT_ARGS%
set "RESULT=%errorlevel%"
popd
exit /b %RESULT%

:bad_scene
echo No screen called "%SCENE%". Choose column, reactor, pump, furnace, plant or map.
goto fail

:no_godot
echo Godot was not found. Install Godot 4.7 or newer and put godot on PATH.
goto fail

:no_library
echo The simulator library could not be built and none was built before.
echo Install Rust (https://rustup.rs), then run this file again.
goto fail

:fail
pause
popd
exit /b 1

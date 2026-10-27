@echo off
rem Build a native controller executable.
rem
rem   build_controller.bat [controller_source.c] [output.exe]
rem
rem Defaults: controller\placeholder_controller.c -> build\controller.exe
rem Uses MSVC (cl.exe) if it is on PATH or can be located with vswhere; otherwise gcc/clang (MinGW).
setlocal

set "ROOT=%~dp0"
set "SRC=%~f1"
if "%~1"=="" set "SRC=%ROOT%controller\placeholder_controller.c"
set "OUT=%~f2"
if "%~2"=="" set "OUT=%ROOT%build\controller.exe"

if not exist "%SRC%" (
  echo ERROR: controller source not found: %SRC%
  exit /b 1
)
for %%I in ("%OUT%") do set "OUTDIR=%%~dpI"
if not exist "%OUTDIR%" mkdir "%OUTDIR%"
if not exist "%ROOT%build\obj" mkdir "%ROOT%build\obj"

where cl >nul 2>nul
if not errorlevel 1 goto msvc

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" goto try_gcc
set "VSINSTALL="
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -property installationPath`) do set "VSINSTALL=%%i"
if not defined VSINSTALL goto try_gcc
if not exist "%VSINSTALL%\VC\Auxiliary\Build\vcvars64.bat" goto try_gcc
echo Using Visual Studio at %VSINSTALL%
call "%VSINSTALL%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>nul
where cl >nul 2>nul
if errorlevel 1 goto try_gcc

:msvc
echo Compiling %SRC% with MSVC ...
pushd "%ROOT%build\obj"
cl /nologo /O2 /W4 /WX /std:c11 /D_CRT_SECURE_NO_WARNINGS /I"%ROOT%controller" "%ROOT%controller\controller_host.c" "%SRC%" /Fe"%OUT%"
set "RC=%ERRORLEVEL%"
popd
if not "%RC%"=="0" (
  echo BUILD FAILED
  exit /b %RC%
)
goto done

:try_gcc
where gcc >nul 2>nul
if errorlevel 1 goto try_clang
echo Compiling %SRC% with gcc ...
gcc -O2 -Wall -Wextra -Werror -std=c11 -I"%ROOT%controller" "%ROOT%controller\controller_host.c" "%SRC%" -o "%OUT%" -lm
if errorlevel 1 (
  echo BUILD FAILED
  exit /b 1
)
goto done

:try_clang
where clang >nul 2>nul
if errorlevel 1 goto nocompiler
echo Compiling %SRC% with clang ...
clang -O2 -Wall -Wextra -Werror -std=c11 -D_CRT_SECURE_NO_WARNINGS -I"%ROOT%controller" "%ROOT%controller\controller_host.c" "%SRC%" -o "%OUT%"
if errorlevel 1 (
  echo BUILD FAILED
  exit /b 1
)
goto done

:nocompiler
echo ERROR: no C compiler found.
echo Install Visual Studio 2022 or later (or Build Tools) with the "Desktop development with C++" workload,
echo or MinGW-w64 gcc, or run this script from a "x64 Native Tools Command Prompt".
exit /b 1

:done
echo Built %OUT%
exit /b 0

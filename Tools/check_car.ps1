param([string]$Keil = '')
$ErrorActionPreference = 'Stop'
$carRoot = Split-Path -Parent $PSScriptRoot
$carOutput = Join-Path $carRoot 'build'
New-Item -ItemType Directory -Force -Path $carOutput | Out-Null
Push-Location -LiteralPath $carRoot
try {
    if ([string]::IsNullOrWhiteSpace($Keil)) {
        $keilCandidates = @(
            'C:\Keil_v5\UV4\UV4.exe',
            'D:\Keil_v5\UV4\UV4.exe',
            'D:\keil_v5\UV4\UV4.exe',
            'C:\Keil_v5\core\UV4\UV4.exe',
            'D:\Keil_v5\core\UV4\UV4.exe',
            'D:\keil_v5\core\UV4\UV4.exe'
        )
        $Keil = $keilCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
    }
    if (!$Keil -or !(Test-Path -LiteralPath $Keil)) {
        $tried = if ($keilCandidates) { " Tried: $($keilCandidates -join ', ')" } else { '' }
        throw "Keil executable missing: '$Keil'.$tried Pass -Keil <path-to-UV4.exe> to override."
    }

    # 代码目录按功能分类，头文件按文件名互相引用，因此每个子目录都要加进 -I。
    # 用数组而不是单个字符串：PowerShell 把带空格的字符串作为“一个”参数传给
    # clang.cmd 时，-I 与路径会被当成同一个参数，导致找不到头文件。
    $inc = @('-I', 'Tests/stubs', '-I', 'CODE')

    python -m unittest discover -s Tests -p 'test_*.py' -v
    if ($LASTEXITCODE -ne 0) { throw 'Python tests failed' }
    node --test Tests/test_car_console.cjs
    if ($LASTEXITCODE -ne 0) { throw 'Console tests failed' }

    foreach ($visionEnabled in 0, 1) {
        clang -std=c99 -Wall -Wextra -Werror "-DBOARD_VISION_ENABLE=$visionEnabled" $inc -I User -I Library/NEUQ_lib/NEUQ_lib_Driver Tests/test_car_startup.c Library/NEUQ_lib/NEUQ_lib_Driver/NEUQ_Delay.c User/NEUQ_Interrupts.c -o build/test_car_startup.exe
        if ($LASTEXITCODE -ne 0) { throw 'Startup test compilation failed' }
        & (Join-Path $carOutput 'test_car_startup.exe')
        if ($LASTEXITCODE -ne 0) { throw "Cold start / UART routing failed (vision=$visionEnabled)" }
    }

    # 传统故障停车模式的回归用 CAR_FAULT_STOP_ENABLE=1 编译；
    # 默认固件是记录模式（0），另由 test_car_fault_log 覆盖。
    clang -std=c99 -Wall -Wextra -Werror -DCAR_FAULT_STOP_ENABLE=1 $inc Tests/test_car_core.c CODE/Task_Fsm.c CODE/Pid.c CODE/Proto_Bluetooth.c -o build/test_car_core.exe
    if ($LASTEXITCODE -ne 0) { throw 'Core test compilation failed' }
    & (Join-Path $carOutput 'test_car_core.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Core tests failed' }

    clang -std=c99 -Wall -Wextra -Werror $inc Tests/test_car_fault_log.c CODE/Task_Fsm.c CODE/Pid.c CODE/Proto_Bluetooth.c -o build/test_car_fault_log.exe
    if ($LASTEXITCODE -ne 0) { throw 'Fault log test compilation failed' }
    & (Join-Path $carOutput 'test_car_fault_log.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Fault log tests failed' }

    clang -std=c99 -Wall -Wextra -Werror $inc Tests/test_car_pid.c CODE/Pid.c -o build/test_car_pid.exe
    if ($LASTEXITCODE -ne 0) { throw 'PID test compilation failed' }
    & (Join-Path $carOutput 'test_car_pid.exe')
    if ($LASTEXITCODE -ne 0) { throw 'PID tests failed' }

    clang -std=c99 -Wall -Wextra -Werror $inc Tests/test_car_vision.c CODE/Proto_Vision.c -o build/test_car_vision.exe
    if ($LASTEXITCODE -ne 0) { throw 'Vision test compilation failed' }
    & (Join-Path $carOutput 'test_car_vision.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Vision tests failed' }

    clang -std=c99 -Wall -Wextra -Werror $inc Tests/test_car_menu.c CODE/Menu_Task.c CODE/Task_Fsm.c CODE/Pid.c CODE/Proto_Bluetooth.c -o build/test_car_menu.exe
    if ($LASTEXITCODE -ne 0) { throw 'Menu test compilation failed' }
    & (Join-Path $carOutput 'test_car_menu.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Menu tests failed' }

    # 参数存储测试只编译记录编解码，不编译片内 Flash 读写（那部分要 HAL_FLASH）。
    clang -std=c99 -Wall -Wextra -Werror $inc '-DPID_STORAGE_WITH_FLASH=0' Tests/test_car_params.c CODE/Pid_Storage.c CODE/Task_Fsm.c CODE/Pid.c CODE/Proto_Bluetooth.c -o build/test_car_params.exe
    if ($LASTEXITCODE -ne 0) { throw 'PID storage compilation failed' }
    & (Join-Path $carOutput 'test_car_params.exe')
    if ($LASTEXITCODE -ne 0) { throw 'PID storage tests failed' }

    clang -std=c99 -Wall -Wextra -Werror $inc Tests/test_car_sudoku.c CODE/Sudoku.c -o build/test_car_sudoku.exe
    if ($LASTEXITCODE -ne 0) { throw 'Sudoku test compilation failed' }
    & (Join-Path $carOutput 'test_car_sudoku.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Sudoku tests failed' }

    clang -std=c99 -Wall -Wextra -Werror $inc Tests/test_car_race.c CODE/Track_Fsm.c CODE/Sudoku.c CODE/Task_Fsm.c CODE/Pid.c CODE/Proto_Bluetooth.c -o build/test_car_race.exe
    if ($LASTEXITCODE -ne 0) { throw 'Race test compilation failed' }
    & (Join-Path $carOutput 'test_car_race.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Race tests failed' }

    $carProject = Join-Path $carRoot 'MDK-ARM/NEUQ_STM32F407VET6_Opensource_Library.uvprojx'
    $carLog = Join-Path $carOutput 'keil_car.log'
    if (!(Test-Path -LiteralPath $Keil)) { throw "Keil executable missing: $Keil" }
    $carProcess = Start-Process -FilePath $Keil -ArgumentList @('-r', ('"{0}"' -f $carProject), '-o', ('"{0}"' -f $carLog)) -WindowStyle Hidden -Wait -PassThru
    Get-Content -LiteralPath $carLog
    if ($carProcess.ExitCode -ne 0) { throw "Keil build failed: $($carProcess.ExitCode)" }
    if (!(Select-String -LiteralPath $carLog -SimpleMatch '0 Error(s), 0 Warning(s).')) { throw 'Keil clean-build summary missing' }
    python Tools/check_pid_hex.py
    if ($LASTEXITCODE -ne 0) { throw 'PID Flash reservation check failed' }
} finally {
    Pop-Location
}

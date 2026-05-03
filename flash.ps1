$CMAKE   = "C:\Program Files\CMake\bin\cmake.exe"
$OBJCOPY = "C:\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe"

Remove-Item -Force -ErrorAction SilentlyContinue "build\pellicanus.bin", "build\pellicanus_final.uf2", "build\pellicanus.elf"

Write-Host "Building..."
& $CMAKE --build build --clean-first
if ($LASTEXITCODE -ne 0) { Write-Host "Build failed!"; exit 1 }

Write-Host "Converting to UF2..."
& $OBJCOPY -O binary "build\pellicanus.elf" "build\pellicanus.bin"
python build\bin2uf2.py build\pellicanus.bin build\pellicanus_final.uf2

Write-Host ""
Write-Host "==> Simdi Pico'yu USB'den cikart, BOOTSEL'e bas, tak, butonu birak."
Write-Host "==> D: diski gorunce ENTER'a bas."
Read-Host

Write-Host "Flashing..."
$src = (Resolve-Path "build\pellicanus_final.uf2").Path
$dst = "D:\pellicanus_final.uf2"
$fs = New-Object System.IO.FileStream($dst, [System.IO.FileMode]::Create, [System.IO.FileAccess]::Write, [System.IO.FileShare]::None, 512, [System.IO.FileOptions]::WriteThrough)
$data = [System.IO.File]::ReadAllBytes($src)
$fs.Write($data, 0, $data.Length)
$fs.Close()
Write-Host "Done! D: should disappear now."

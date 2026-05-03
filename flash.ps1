$CMAKE   = "C:\Program Files\CMake\bin\cmake.exe"
$OBJCOPY = "C:\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe"

Write-Host "Building..."
& $CMAKE --build build
if ($LASTEXITCODE -ne 0) { Write-Host "Build failed!"; exit 1 }

Write-Host "Converting to UF2..."
& $OBJCOPY -O binary "build\pellicanus.elf" "build\pellicanus.bin"
python build\bin2uf2.py build\pellicanus.bin build\pellicanus_final.uf2

Write-Host "Flashing to D:..."
cmd /c "copy /b build\pellicanus_final.uf2 D:\"
Write-Host "Done!"

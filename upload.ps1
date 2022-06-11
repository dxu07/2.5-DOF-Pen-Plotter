$drive = Get-Volume -FileSystemLabel "PYBFLASH"
$drive = $drive.DriveLetter

# rm -r $drive":\*"
cp -r src\pyb\* $drive":\"

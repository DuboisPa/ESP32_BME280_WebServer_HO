# Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
# Get-ExecutionPolicy -List
# 0x160000 = 1 441 792 en décimal

# C:/Users/%USERNAME%/AppData/Local/Arduino15/packages/esp32/hardware/esp32/3.1.3/tools/partitions
#../esptool.exe --chip esp32 --baud 921600 write_flash -z 0x290000 spiffs.bin

# default.csv
# Name,   Type, SubType, Offset,  Size, Flags
#nvs,      data, nvs,     0x9000,  0x5000,
#otadata,  data, ota,     0xe000,  0x2000,
#app0,     app,  ota_0,   0x10000, 0x140000,
#app1,     app,  ota_1,   0x150000,0x140000,
#spiffs,   data, spiffs,  0x290000,0x160000,
#coredump, data, coredump,0x3F0000,0x10000,


# minimal spiffs
# Name,   Type, SubType, Offset,  Size, Flags
#nvs,      data, nvs,     0x9000,  0x5000,
#otadata,  data, ota,     0xe000,  0x2000,
#app0,     app,  ota_0,   0x10000, 0x1E0000,
#app1,     app,  ota_1,   0x1F0000,0x1E0000,
#spiffs,   data, spiffs,  0x3D0000,0x20000,
#coredump, data, coredump,0x3F0000,0x10000,

# créer le fichier spiffs.bin à partir du répertoire data
../mkspiffs -c data -b 4096 -p 256 -s 0x20000 spiffs.bin
# transférer le fichier spiffs.bin par esptool. 
# le moniteur série doit être fermé. 
# on peut utiliser ElegantOTA s'il est installé
#../esptool.exe --chip esp32 -b 921600 -a hard_reset write_flash -z 0x3D0000 spiffs.bin

# default_8MB.csv
# Name,   Type, SubType, Offset,  Size, Flags
#nvs,      data, nvs,     0x9000,  0x5000,
#otadata,  data, ota,     0xe000,  0x2000,
#app0,     app,  ota_0,   0x10000, 0x330000,
#app1,     app,  ota_1,   0x340000,0x330000,
#spiffs,   data, spiffs,  0x670000,0x180000,
#coredump, data, coredump,0x7F0000,0x10000,

#../mkspiffs -c data -b 4096 -p 256 -s 0x180000 spiffs.bin
#../esptool.exe --chip esp32 -b 921600 -a hard_reset write_flash -z 0x670000 spiffs.bin

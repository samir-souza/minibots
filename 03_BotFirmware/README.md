
## Preparing the uController with the Minibot firmware

### Esptool

First, you need to get and install **esptool** from: https://github.com/espressif/esptool  

### FLashing the device
You will find the compiled firmware and the compiled LFS in this dir. So, if you don't need to modify the source code, just use the instruction bellow to flash and configure your Minibot.

Then, get the code:
```bash
git clone https://github.com/samir-souza/minibots
cd minibots/03_BotFirmware
```

Take a look on the directory **bin**. You will see the file: **lfs.img** and the dir **firmware**. The first is the Lua LFS image and the second the uController firmware (ESP8266). To flash the ESP8266 do:
```bash
PORT=<REPLACE THIS WITH THE CORRECT "ESP USB SERIAL PORT" STRING> # i.e: MAC: /dev/cu.usbserial-A700abYF
BAUD=115200

# First let's clean the flash
esptool.py --port $PORT -b $BAUD erase_flash

# Then, write the new one
esptool.py --port $PORT -b $BAUD write_flash  -fm qio 0x00000 bin/firmware/0x00000.bin 0x10000 bin/firmware/0x10000.bin
```

### Load the Lua image+files and configure the bot
  - Get **ESPlorer** (https://esp8266.ru/esplorer/) to finish the configuration of the device. 
  - Turn the device on and open ESPlorer. 
  - Then upload the Lua image to the device (there is a button on ESPlorer called upload): **bin/lua.img**
  - On the **ESPlorer** terminal, run the following Lua command: node.flashreload("lfs.img")
     - It will load LFS to the device
  - Then, upload the following files: lua/config.lua lua/init.lua lua/main.lua
  - Edit the file lua/config.lua on the device and update the **WIFI** and **BACKEND SERVER** config
  - That's it! :)

## Compiling the Firmware
If you need to customize the LFS, there is a Docker container available. After editing the lua files in the **lua/LFS** dir, just follow these instructions to get a new version of the **lfs.img**.

Get the code, build the docker image and run it:
```bash
git clone https://github.com/samir-souza/minibots
cd minibots/03_BotFirmware
docker build -t minibots-firmware:1.0 Container/

### Modify the files in lua/LFS, then
docker run -it --rm -v $PWD/bin:/out -v $PWD/lua/LFS:/in  minibots-firmware:1.0
```

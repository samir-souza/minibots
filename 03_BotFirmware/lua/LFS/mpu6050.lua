require 'class'
require 'vector'
require 'quaternion'

MPU6050 = class()

function MPU6050:__constants()
    self.GYRO_CONFIG = 0x1B
    self.ACCEL_CONFIG = 0x1C
    self.PWR_MGMT_1 = 0x6B
    self.RA_USER_CTRL = 0x6A
    self.DMP_EN_BIT = 7
    self.USERCTRL_DMP_RESET_BIT = 3
    self.PWR1_DEVICE_RESET_BIT = 7
    self.PWR1_SLEEP_BIT = 6
    self.RA_INT_STATUS = 0x3A
    self.DMP_CONFIG_SIZE = 192

    self.RA_FIFO_COUNTH = 0x72
    self.RA_MOT_THR = 0x1F
    self.RA_ZRMOT_THR = 0x21
    self.RA_MOT_DUR = 0x20
    self.RA_ZRMOT_DUR = 0x22
    self.RA_XG_OFFS_TC = 0x00
    self.RA_YG_OFFS_TC = 0x01
    self.RA_ZG_OFFS_TC = 0x02
    self.XA_OFFS_H = 0x06
    self.YA_OFFS_H = 0x08
    self.ZA_OFFS_H = 0x0A
    self.XG_OFFS_USRH = 0x13
    self.YG_OFFS_USRH = 0x15
    self.ZG_OFFS_USRH = 0x17
    self.BANK_SEL = 0x6D
    self.MEM_R_W = 0x6F

    self.OTP_BNK_VLD_BIT = 0
    self.I2C_SLV0_ADDR = 0x25
    self.USERCTRL_I2C_MST_EN_BIT = 5
    self.USERCTRL_I2C_MST_RESET_BIT = 1
    self.USERCTRL_FIFO_RESET_BIT = 2
    self.USERCTRL_FIFO_EN_BIT = 6
    self.MEM_START_ADDR = 0x6E
    self.MPU6050_DMP_CODE_SIZE = 1929
    self.DMP_MEMORY_CHUNK_SIZE = 16
    self.ACCEL_XOUT_H = 0x3B

    self.GYRO_FS_250 = 0x00
    self.GYRO_FS_500 = 0x01
    self.GYRO_FS_1000 = 0x02
    self.GYRO_FS_2000 = 0x03

    self.DLPF_BW_42 = 0x03
    self.EXT_SYNC_TEMP_OUT_L = 0x1
    
    self.RA_INT_ENABLE = 0x38
    self.RA_SMPLRT_DIV = 0x19
    self.RA_DMP_CFG_1 = 0x70
    self.RA_DMP_CFG_2 = 0x71
    
    self.WHO_AM_I = 0x75
    self.MEAN_SENSORS_ITERATIONS = 1000
    self.RA_CONFIG = 0x1A
    self.RA_FIFO_R_W = 0x74
    self.initialized = false
end

function MPU6050:init(sda, scl, dev_addr, bus)
    if not i2c.setup(bus, sda, scl, i2c.SLOW) then
        print('Failed to intilialize MPU')
        return nil
    end
    self:__constants()
    self.dev_addr = dev_addr
    self.bus = bus

    print('Initializing MPU...')
    self:write_byte( self.GYRO_CONFIG, self.GYRO_FS_250) -- GYRO_CONFIG set Full scale range +/- 250 degree/C
    self:write_byte( self.ACCEL_CONFIG, 0x00) -- ACCEL_CONFIG set +/- 2g full scale
    self:write_byte( self.PWR_MGMT_1, 0x01) -- PWR_MGMT_1 0x00000001

    tmr.delay(3000000)
    if file.open("calibration.txt", "r") then
        print('Loading calibration from file')
        local data = file.read()
        file.close()
        t={}
        for c in string.gmatch(data, "%S+") do table.insert(t,c) end
        
        self:set_imu_offset(t[1],t[2],t[3], t[4],t[5],t[6])
        print('IMU offsets: ', self:get_imu_offset())
    else
        self:auto_calibrate()
    end
    
    tmr.delay(1000000)
    if not self:check() then
        print('IMU initialization failed..')
        
    elseif self:dmp_initialize() == 0 then
        print('MPU6050 Initialized!')
        self.initialized = true
    else
        print('Something went wrong')
        tmr.delay(5000000)
        return nil
    end
    
    
end

function MPU6050:is_initialized()
    return self.initialized
end

function MPU6050:set_slave_address(num, address)
    if num > 3 then
        return self:write_byte(self.I2C_SLV0_ADDR + num*3, address)
    end
end


function MPU6050:write_memory_block(data, bank, address, verify)
    local dataSize = #data
    if dataSize < 1 then
        print('There is no data to write')
        return false
    end
    local pivot = 0
    while dataSize > 0 do
        local chunkSize = self.DMP_MEMORY_CHUNK_SIZE
        if dataSize < chunkSize then chunkSize = dataSize end
        
        local buffer = {}
        for i=1,chunkSize do buffer[i] = data[pivot+i] end

        self:set_memory_bank(bank)
        self:set_memory_start_address(address)
        self:write_bytes(self.MEM_R_W, buffer)
       
        -- verify data if needed
        if verify then
            self:set_memory_bank(bank)
            self:set_memory_start_address(address)
            -- I2Cdev::readBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);
            verifyBuffer = self:read_bytes(self.MEM_R_W, chunkSize)
            
            for j=1,chunkSize do 
                if buffer[j] ~= verifyBuffer[j] then 
                    print("Block write verification error, bank ",bank,", address ",address, " index ", j)
                    return false
                end 
            end
        end
        
        dataSize = dataSize - chunkSize
        address = address + chunkSize
        pivot = pivot + chunkSize
        collectgarbage()
    end
    print('Data loaded and verified successfully! :)')
    return true

end

function MPU6050:write_dmp_configuration_set(data)
    local dataSize = #data
    if dataSize < 1 then
        print('There is no data to write')
        return false
    end
    local pivot = 1
    local success = false
    while pivot < dataSize do
        local bank = data[pivot]
        local address = data[pivot+1]
        local chunkSize = data[pivot+2]
        pivot = pivot + 2

        if chunkSize > 0 then
            local buffer = {}
            print('Loading ', chunkSize, 'into bank', bank, 'address', address, 'pivot', pivot)
            for i=1,chunkSize do buffer[i] = data[pivot+i] end
            success = self:write_memory_block(buffer, bank, address, true)
            pivot = pivot + chunkSize + 1
        else
            -- special instruction
            -- NOTE: this kind of behavior (what and when to do certain things)
            -- is totally undocumented. This code is in here based on observed
            -- behavior only, and exactly why (or even whether) it has to be here
            -- is anybody's guess for now.
            local special = data[pivot+1]
            pivot = pivot + 2
            print( 'Special command, ', special)
            if special == 0x01 then
                -- enable DMP-related interrupts
                --I2Cdev::writeByte(devAddr, MPU6050_RA_INT_ENABLE, 0x32) -- single operation
                self:write_byte(self.RA_INT_ENABLE, 0x32)
                success = true;
            else
                -- unknown special command
                success = false
            end
        end
        if not success then break end
    end
    return success
end

function MPU6050:dmp_initialize()
    print("Resetting MPU6050...")
    self:write_bit(self.PWR_MGMT_1, self.PWR1_DEVICE_RESET_BIT, true)
    tmr.delay(30000)

    print("Disabling sleep mode...")
    --writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
    self:write_bit(self.PWR_MGMT_1, self.PWR1_SLEEP_BIT, false)

    print("Selecting user bank 16...")
    self:set_memory_bank(0x10, true, true)

    print("Selecting memory byte 6...")
    self:set_memory_start_address(0x06)

    print("Checking hardware revision...")
    local hwRevision  = self:read_byte(self.MEM_R_W)
    print("Revision @ user[16][6] = ")
    print(hwRevision)
    print("Resetting memory bank selection to 0...")
    self:set_memory_bank(0, false, false)

    -- check OTP bank valid
    print("Reading OTP bank valid flag...")
    local otpValid = self:read_bit(self.RA_XG_OFFS_TC, self.OTP_BNK_VLD_BIT)
    print("OTP bank is ")
    if otpValid then print('Valid') else print('Invalid') end

    -- get X/Y/Z gyro offsets
    print("Reading gyro offset values...")

    local Ax,Ay,Az,xgOffset, ygOffset, zgOffset = self:get_imu_offset()
    
    print("X gyro offset = "..xgOffset)
    print("Y gyro offset = "..ygOffset)
    print("Z gyro offset = "..zgOffset)

    -- setup weird slave stuff (?)
    print("Setting slave 0 address to 0x7F...")
    self:set_slave_address(0, 0x7F)
    print("Disabling I2C Master mode...")
    self:write_bit(self.RA_USER_CTRL, self.USERCTRL_I2C_MST_EN_BIT, false)
    print("Setting slave 0 address to 0x68 (self)...")
    self:set_slave_address(0, 0x68)
    print("Resetting I2C Master control...")
    self:write_bit(self.RA_USER_CTRL, self.USERCTRL_I2C_MST_RESET_BIT, true)
    tmr.delay(20000)

    -- load DMP code into memory banks
    print("Writing DMP code to MPU memory banks ("..self.MPU6050_DMP_CODE_SIZE.." bytes)")

    --local dmp_memory = require 'mpu6050_memory'
    --write_memory_block(data, bank, address, verify, useProgMem)
    local memory_loaded = true
    
    for bank=0,7 do
        collectgarbage()
        local pkg_name = 'mpu6050_memorybank'..bank
        local dmp_memory = require (pkg_name)
        print('Loading '..#dmp_memory..' into bank '..bank)
        if not self:write_memory_block(dmp_memory, bank, 0, true ) then
            memory_loaded = false
            break
        end
        package.loaded[pkg_name] = nil
        _G[pkg_name] = nil
        collectgarbage()
    end
    
    if memory_loaded then
        print("Success! DMP code written and verified.\n")

        -- write DMP configuration
        collectgarbage()
        local dmp_config = require 'mpu6050_config'
        print("Writing DMP configuration to MPU memory banks (",self.DMP_CONFIG_SIZE," bytes in config def)")
        if self:write_dmp_configuration_set(dmp_config) then
            package.loaded['mpu6050_config'] = nil
            _G['mpu6050_config'] = nil
            collectgarbage()

            print("Success! DMP configuration written and verified.\n")

            print("Setting clock source to Z Gyro...")
            --setClockSource(MPU6050_CLOCK_PLL_ZGYRO)
            self:write_byte( self.PWR_MGMT_1, 0x03 )

            print("Setting DMP and FIFO_OFLOW interrupts enabled...")
            -- setIntEnabled(0x12)
            self:write_byte( self.RA_INT_ENABLE, 0x12 )
            

            print("Setting sample rate to 200Hz...")
            --setRate(4) -- 1khz / (1 + 4) = 200 Hz
            self:write_byte( self.RA_SMPLRT_DIV, 4)

            print("Setting external frame sync to TEMP_OUT_L[0]...")
            --setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L)
            self:write_bits( self.RA_CONFIG, 5, 3, self.EXT_SYNC_TEMP_OUT_L ) -- 0x1 0b001

            print("Setting DLPF bandwidth to 42Hz...")
            --setDLPFMode(MPU6050_DLPF_BW_42)
            self:write_bits( self.RA_CONFIG, 2, 3, self.DLPF_BW_42 ) -- 0x03 0b011

            print("Setting gyro sensitivity to +/- 2000 deg/sec...")
            --setFullScaleGyroRange(MPU6050_GYRO_FS_2000)
            self:write_bits( self.GYRO_CONFIG, 4, 2, self.GYRO_FS_2000 ) -- 0x03 0b11

            print("Setting DMP configuration bytes (function unknown)...")
            --setDMPConfig1(0x03)
            --setDMPConfig2(0x00)
            self:write_byte( self.RA_DMP_CFG_1, 0x03)
            self:write_byte( self.RA_DMP_CFG_2, 0x00)
            

            print("Clearing OTP Bank flag...")
            --setOTPBankValid(false)
            self:write_bit( self.RA_XG_OFFS_TC, 0, true )
            
            print("Setting X/Y/Z gyro offsets to previous values...")
            --I2Cdev::writeBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
            self:write_bits( self.RA_XG_OFFS_TC, 6, 6, xgOffset )
            self:write_bits( self.RA_YG_OFFS_TC, 6, 6, ygOffset )
            self:write_bits( self.RA_ZG_OFFS_TC, 6, 6, zgOffset )
            --setXGyroOffset(xgOffset)
            --setYGyroOffset(ygOffset)
            --setZGyroOffset(zgOffset)

            print("Setting X/Y/Z gyro user offsets to zero...")
            --  I2Cdev::writeWord(devAddr, MPU6050_RA_XG_OFFS_USRH, offset);
            --setXGyroOffsetUser(0)
            --setYGyroOffsetUser(0)
            --setZGyroOffsetUser(0)
            self:set_imu_offset(Ax, Ay, Az, 0,0,0)

            print("Writing final memory update 1/7 (function unknown)...")
            --uint8_t dmpUpdate[16], j;
            --uint16_t pos = 0;
            --for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos])
            --writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1])
            collectgarbage()
            local dmpUpdate, bank, address, dataSize = nil,nil,nil,nil
            local dmp_updates = require 'mpu6050_updates'
            local pos = 1

            bank, address, dataSize = dmp_updates[pos],dmp_updates[pos+1],dmp_updates[pos+2]
            pos = pos+2
            dmpUpdate = {}
            for j=1,dataSize do dmpUpdate[j] = dmp_updates[pos+j] end
            pos = pos + dataSize + 1
            self:write_memory_block(dmpUpdate, bank, address, true )

            
            print("Writing final memory update 2/7 (function unknown)...")
            --for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos])
            --writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1])
            bank, address, dataSize = dmp_updates[pos],dmp_updates[pos+1],dmp_updates[pos+2]
            pos = pos+2
            dmpUpdate = {}
            for j=1,dataSize do dmpUpdate[j] = dmp_updates[pos+j] end
            pos = pos + dataSize + 1
            self:write_memory_block(dmpUpdate, bank, address, true )


            print("Resetting FIFO...")
            --resetFIFO()
            --writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
            self:reset_fifo()

            print("Reading FIFO count...")
            --uint8_t fifoCount = getFIFOCount()
            --uint8_t fifoBuffer[128];
            local fifoCount = self:get_fifo_count()
            print("Current FIFO count: ", fifoCount)
            
            if fifoCount > 0 then
                --getFIFOBytes(fifoBuffer, fifoCount)
                fifoBuffer = self:get_fifo_bytes(fifoCount)
            end
            print("Setting motion detection threshold to 2...")
            --setMotionDetectionThreshold(2)
            self:write_byte(self.RA_MOT_THR, 2)

            print("Setting zero-motion detection threshold to 156...")
            --setZeroMotionDetectionThreshold(156)
            self:write_byte(self.RA_ZRMOT_THR, 156)

            print("Setting motion detection duration to 80...")
            --setMotionDetectionDuration(80)
            self:write_byte(self.RA_MOT_DUR, 80)
            

            print("Setting zero-motion detection duration to 0...")
            --setZeroMotionDetectionDuration(0)
            self:write_byte(self.RA_ZRMOT_DUR, 0)
            
            print("Resetting FIFO...")
            --resetFIFO()
            --writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
            self:reset_fifo()

            print("Enabling FIFO...")
            --setFIFOEnabled(true)
            self:set_fifo_enabled(true)

            print("Enabling DMP...")
            --setDMPEnabled(true)
            self:set_dmp_enabled(true)

            print("Resetting DMP...")
            --resetDMP()
            self:reset_fifo()
            
            print("Writing final memory update 3/7 (function unknown)...")
            --for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos])
            --writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1])
            bank, address, dataSize = dmp_updates[pos],dmp_updates[pos+1],dmp_updates[pos+2]
            pos = pos+2
            dmpUpdate = {}
            for j=1,dataSize do dmpUpdate[j] = dmp_updates[pos+j] end
            pos = pos + dataSize + 1
            self:write_memory_block(dmpUpdate, bank, address, true )

            print("Writing final memory update 4/7 (function unknown)...")
            --for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos])
            --writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1])
            bank, address, dataSize = dmp_updates[pos],dmp_updates[pos+1],dmp_updates[pos+2]
            pos = pos+2
            dmpUpdate = {}
            for j=1,dataSize do dmpUpdate[j] = dmp_updates[pos+j] end
            pos = pos + dataSize + 1
            self:write_memory_block(dmpUpdate, bank, address, true )

            print("Writing final memory update 5/7 (function unknown)...")
            --for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos])
            --writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1])
            bank, address, dataSize = dmp_updates[pos],dmp_updates[pos+1],dmp_updates[pos+2]
            pos = pos+2
            dmpUpdate = {}
            for j=1,dataSize do dmpUpdate[j] = dmp_updates[pos+j] end
            pos = pos + dataSize + 1
            self:write_memory_block(dmpUpdate, bank, address, true )

            print("Waiting for FIFO count > 2...\n")
            --while ((fifoCount = getFIFOCount()) < 3)
            repeat 
                fifoCount = self:get_fifo_count()
            until fifoCount > 2

            print("Current FIFO count: ",fifoCount)
            
            print("Reading interrupt status...")
            --uint8_t mpuIntStatus __attribute__((__unused__)) = getIntStatus()
            local mpuIntStatus = self:read_byte(self.RA_INT_STATUS)
            print("Current interrupt status: ", mpuIntStatus)

            print("Reading final memory update 6/7 (function unknown)...")
            --for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos])
            --readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1])
            bank, address, dataSize = dmp_updates[pos],dmp_updates[pos+1],dmp_updates[pos+2]
            pos = pos+2
            dmpUpdate = {}
            for j=1,dataSize do dmpUpdate[j] = dmp_updates[pos+j] end
            pos = pos + dataSize + 1
            self:write_memory_block(dmpUpdate, bank, address, true )

            print("Waiting for FIFO count > 2...")
            --while ((fifoCount = getFIFOCount()) < 3)
            repeat 
                fifoCount = self:get_fifo_count()
            until fifoCount > 2

            print("Current FIFO count: ", fifoCount)

            --print("Reading FIFO data...")
            --fifoBuffer = self:read_bytes(self.RA_FIFO_R_W, fifoCount)

            print("Reading interrupt status...")
            mpuIntStatus = self:read_byte(self.RA_INT_STATUS)
            print("Current interrupt status: ", mpuIntStatus )
            

            print("Writing final memory update 7/7 (function unknown)...")
            --for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos])
            --writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1])
            bank, address, dataSize = dmp_updates[pos],dmp_updates[pos+1],dmp_updates[pos+2]
            pos = pos+2
            dmpUpdate = {}
            for j=1,dataSize do dmpUpdate[j] = dmp_updates[pos+j] end
            pos = pos + dataSize + 1
            self:write_memory_block(dmpUpdate, bank, address, true )
            package.loaded['mpu6050_updates'] = nil
            _G['mpu6050_updates'] = nil
            collectgarbage()

            print("DMP is good to go! Finally.")

            print("Disabling DMP (you turn it on later)...")
            --setDMPEnabled(false)
            self:set_dmp_enabled(false)
            
            print("Resetting FIFO and clearing INT status one last time...")
            --resetFIFO()
            self:reset_fifo()
            --getIntStatus()
            mpuIntStatus = self:read_byte(self.RA_INT_STATUS)
            print("Current interrupt status: ", mpuIntStatus )
        else
            print("ERROR! DMP configuration verification failed.")
            return 2 -- configuration block loading failed
        end
    else
        print("ERROR! DMP code verification failed.")
        return 1 -- main binary block loading failed
    end
    return 0 -- success
end
function MPU6050:set_fifo_enabled(flag)
    self:write_bit(self.RA_USER_CTRL, self.USERCTRL_FIFO_EN_BIT, flag)
end

function MPU6050:get_fifo_enabled()
    return self:read_bit(self.RA_USER_CTRL, self.USERCTRL_FIFO_EN_BIT)
end

function MPU6050:reset_fifo()
    local flag = self:get_fifo_enabled()
    if not flag then self:set_fifo_enabled(false) end
    self:write_bit(self.RA_USER_CTRL, self.USERCTRL_FIFO_RESET_BIT, true )
    self:set_fifo_enabled(flag)
end

function MPU6050:get_fifo_count()
    return self:read_word( self.RA_FIFO_COUNTH )
end

function MPU6050:get_fifo_bytes(fifoCount)
    return self:read_bytes(self.RA_FIFO_R_W, fifoCount)
end

function MPU6050:set_dmp_enabled(flag)
    self:write_bit( self.RA_USER_CTRL, self.DMP_EN_BIT, flag )
end

function MPU6050:set_memory_start_address(address)
    self:write_byte(self.MEM_START_ADDR, address)
end

function MPU6050:set_memory_bank(bank, prefetchEnabled, userBank)
    local bank = bit.band(bank, 0x1F)
    if userBank then bank = bit.bor(bank, 0x20 ) end
    if prefetchEnabled then bank = bit.bor(bank,0x40) end
    self:write_byte(self.BANK_SEL, bank)
end

function MPU6050:read_bits(reg, pos, len)
    if pos < 0 or pos > 7 or len < 1 or pos+1-len < 0 then
        print('Invalid pos or bits length. Should be between 0 and 7 or # > 0')
        return false
    end
    local b = self:read_byte(reg)
    local d = 0
    local j=len-1
    for i=pos,pos+1-bitsLength,-1 do
        if bit.isset(b,i) then d = bit.set(d, j) end
        j = j-1
    end
    return d
end

function MPU6050:write_bits(reg, pos, len, data)
    if pos < 0 or pos > 7 or len < 1 or pos+1-len < 0 then
        print('Invalid pos or bits length. Should be between 0 and 7 or # > 0')
        return false
    end
    local b = self:read_byte(reg)
    local j=len-1
    for i=pos,pos+1-len,-1 do
        if bit.isset(data,j) then b = bit.set(b,i) else b = bit.clear(b,i) end
        j = j-1
    end
    self:write_byte(reg,b)
end

function MPU6050:write_bit(reg, pos, val)
    if pos < 0 or pos > 7 then
        print('Invalid pos. Should be between 0 and 7')
        return false
    end
    
    local b = self:read_byte(reg)
    if val==true then
        b = bit.set(b,pos)
    else
        b = bit.clear(b,pos)
    end
    self:write_byte(reg,b)
end

function MPU6050:read_bit(reg, pos)
    if pos < 0 or pos > 7 then
        print('Invalid pos. Should be between 0 and 7')
        return false
    end
    local b = self:read_byte(reg)
    return bit.isset(b,pos)
end

-- write one byte to a given register
function MPU6050:write_byte(reg, val)
    self:write_bytes(reg,{val})
end

-- write an array of bytes to a given register
function MPU6050:write_bytes(reg, data)
    i2c.start(self.bus)
    i2c.address(self.bus, self.dev_addr, i2c.TRANSMITTER)
    i2c.write(self.bus, reg)
    for i=1,#data do
        i2c.write(self.bus, data[i])
        tmr.delay(2000)
    end
    i2c.stop(self.bus)
end

-- write two bytes to a given register
function MPU6050:write_word(reg, val)
    local msb = bit.band(bit.rshift(val,8),0xff)
    local lsb = bit.band(val,0xff)
    self:write_bytes(reg,{msb,lsb})
end


-- Reads multiple bytes
function MPU6050:read_bytes(reg, num_bytes)
    i2c.start(self.bus) 
    i2c.address(self.bus, self.dev_addr, i2c.TRANSMITTER)
    i2c.write(self.bus, reg)
    i2c.stop(self.bus)
    
    i2c.start(self.bus)
    i2c.address(self.bus, self.dev_addr, i2c.RECEIVER)
    local b=i2c.read(self.bus, num_bytes)
    i2c.stop(self.bus)

    local data = {}
    for i=1,num_bytes do
        data[i] = string.byte(b,i)
    end
    return data
end
-- Reads one byte (LSB) from the register
function MPU6050:read_byte(reg)
    return self:read_bytes(reg,1)[1]
end

-- Reads two bytes from the register
function MPU6050:read_word(reg)
    local data = self:read_bytes(reg,2)
    return self:uint2int16(bit.bor(bit.lshift(data[1], 8), data[2]))
end

-- convert unsigned 16-bit no. to signed 16-bit no.
function MPU6050:uint2int16(num)
    if num > 32768 then num = num - 65536 end
    return num
end

-- return the offset values set in the IMP
function MPU6050:get_imu_offset()
    return self:read_word( self.XA_OFFS_H),
           self:read_word( self.YA_OFFS_H),
           self:read_word( self.ZA_OFFS_H),
           
           self:read_word( self.XG_OFFS_USRH),
           self:read_word( self.YG_OFFS_USRH),
           self:read_word( self.ZG_OFFS_USRH)
end
-- set new offset values in the IMU
function MPU6050:set_imu_offset(ax,ay,az,gx,gy,gz)
    self:write_word( self.XA_OFFS_H, ax)
    self:write_word( self.YA_OFFS_H, ay)
    self:write_word( self.ZA_OFFS_H, az)
    
    self:write_word( self.XG_OFFS_USRH, gx)
    self:write_word( self.YG_OFFS_USRH, gy)
    self:write_word( self.ZG_OFFS_USRH, gz)
end

-- average the sensor readings
function MPU6050:mean_sensors()
    local Ax, Ay, Az, Gx, Gy, Gz, Tm = 0,0,0,0,0,0,0
    local n=self.MEAN_SENSORS_ITERATIONS
    for i=1,n+100,1 do
        local _Ax, _Ay, _Az, _Gx, _Gy, _Gz, _Tm = self:read_raw()
        if i > 100 then -- discard the first readings
            Ax = Ax + _Ax
            Ay = Ay + _Ay
            Az = Az + _Az
            Gx = Gx + _Gx
            Gy = Gy + _Gy
            Gz = Gz + _Gz
        end
        tmr.delay(2000)
    end
    return Ax/n,Ay/n,Az/n, Gx/n,Gy/n,Gz/n
end
-- auto calibrates the IMU, based on the measure averages
function MPU6050:auto_calibrate()
    local mean_ax,mean_ay,mean_az=0,0,0
    local mean_gx,mean_gy,mean_gz=0,0,0
    local ax_offset,ay_offset,az_offset=0,0,0
    local gx_offset,gy_offset,gz_offset=0,0,0
    local acel_deadzone=8
    local giro_deadzone=1
    local calibrated=false
    
    self:set_imu_offset(0,0,0,0,0,0)
               
    -- state 1
    print('Auto calibrating IMU...')
    mean_ax,mean_ay,mean_az, mean_gx,mean_gy,mean_gz = self:mean_sensors()
    tmr.delay(1000000)
    
    ax_offset=-mean_ax/8
    ay_offset=-mean_ay/8
    az_offset=(16384-mean_az)/8
    
    gx_offset=-mean_gx/4
    gy_offset=-mean_gy/4
    gz_offset=-mean_gz/4
    print("Now, lets start the final stage")
    
    while not calibrated do
        local ready = 0
        
        self:set_imu_offset(ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset)
        
        mean_ax,mean_ay,mean_az, mean_gx,mean_gy,mean_gz = self:mean_sensors()
        
        if math.abs(mean_ax) <= acel_deadzone then ready=ready+1
        else ax_offset=ax_offset-mean_ax/acel_deadzone end
        
        if math.abs(mean_ay)<=acel_deadzone then ready=ready+1
        else ay_offset=ay_offset-mean_ay/acel_deadzone end
        
        if math.abs(16384-mean_az)<=acel_deadzone then ready=ready+1
        else az_offset=az_offset+(16384-mean_az)/acel_deadzone end
        
        if math.abs(mean_gx)<=giro_deadzone then ready=ready+1
        else gx_offset=gx_offset-mean_gx/(giro_deadzone+3) end
        
        if math.abs(mean_gy)<=giro_deadzone then ready=ready+1
        else gy_offset=gy_offset-mean_gy/(giro_deadzone+3) end
        
        if math.abs(mean_gz)<=giro_deadzone then ready=ready+1
        else gz_offset=gz_offset-mean_gz/(giro_deadzone+3) end
        
        print("\n\nPredict: ",self:read_raw())
        print("Offset: ",ax_offset,ay_offset,az_offset, gx_offset, gy_offset,gz_offset)
        print("Mean: ",mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz)
        print("Ready?"..ready.."/6")
        
        if ready==6 then
            print('Calibration completed. Storing the offsets locally...')
            calibrated=true
            tmr.delay(1000000)
            if file.open("calibration.txt", "a+") then
                file.write(ax_offset.." "..ay_offset.." "..az_offset.." "..gx_offset.." "..gy_offset.." "..gz_offset)
                file.close()
            end
        end
    end
    
end

function MPU6050:status()
    i2c.start(self.bus)
    c=i2c.address(self.bus, self.dev_addr ,i2c.TRANSMITTER)
    i2c.stop(self.bus)
    if c==true then
        print("Device found at address: "..string.format("0x%X",self.dev_addr))
        return true
    else 
        print("Device not found: "..string.format("0x%X",self.dev_addr))
        return false
    end
end

function MPU6050:check()
    if not self:status() then return false end
    
    if self:read_byte(self.WHO_AM_I) == 104 then 
        print("     MPU6050 Device answered OK!")
    else 
        print("     Check Device - MPU6050 NOT available!")
        return false
    end
    
    if self:read_byte(self.PWR_MGMT_1)==64 then 
        print("     MPU6050 in SLEEP Mode !")
        return false
    else 
        print("     MPU6050 in ACTIVE Mode !")
        return true
    end
end

function MPU6050:read_raw()
    i2c.start(self.bus)
    i2c.address(self.bus, self.dev_addr, i2c.TRANSMITTER)
    i2c.write(self.bus, self.ACCEL_XOUT_H)
    i2c.stop(self.bus)
    
    i2c.start(self.bus)
    i2c.address(self.bus, self.dev_addr, i2c.RECEIVER)
    -- read 14 bytes
    data=i2c.read(self.bus, 14)
    i2c.stop(self.bus)
    
    Ax = self:uint2int16(bit.bor(bit.lshift(string.byte(data, 1), 8), string.byte(data, 2)))
    Ay = self:uint2int16(bit.bor(bit.lshift(string.byte(data, 3), 8), string.byte(data, 4)))
    Az = self:uint2int16(bit.bor(bit.lshift(string.byte(data, 5), 8), string.byte(data, 6)))
    Tm = self:uint2int16(bit.bor(bit.lshift(string.byte(data, 7), 8), string.byte(data, 8)))
    Gx = self:uint2int16(bit.bor(bit.lshift(string.byte(data, 9), 8), string.byte(data, 10)))
    Gy = self:uint2int16(bit.bor(bit.lshift(string.byte(data, 11), 8), string.byte(data, 12)))
    Gz = self:uint2int16(bit.bor(bit.lshift(string.byte(data, 13), 8), string.byte(data, 14)))

    return Ax, Ay, Az, Gx, Gy, Gz, Tm
end

-- /* ================================================================================================ *
--  | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
--  |                                                                                                  |
--  |  [QUAT W][   ]   [QUAT X][   ]   [QUAT Y][   ]  [QUAT Z][    ]  [GYRO X][    ]  [GYRO Y][    ]   |
--  |  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23   |
--  |                                                                                                  |
--  |  [GYRO Z][    ]  [ACC X ][    ]  [ACC Y ][    ]  [ACC Z ][    ]  [    ]                          |
--  |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
--  * ================================================================================================ */

function MPU6050:get_orientation(packet)
    local w = self:uint2int16(bit.bor(bit.lshift(packet[1],8), packet[2] )) / 16384.0
    local x = self:uint2int16(bit.bor(bit.lshift(packet[5],8), packet[6] )) / 16384.0
    local y = self:uint2int16(bit.bor(bit.lshift(packet[9],8), packet[10] )) / 16384.0
    local z = self:uint2int16(bit.bor(bit.lshift(packet[13],8), packet[14] )) / 16384.0
    return Quaternion(w,x,y,z)
end

function MPU6050:get_gyro(packet)
    local x = self:uint2int16(bit.bor(bit.lshift(packet[17],8), packet[18] )) / 16384.0
    local y = self:uint2int16(bit.bor(bit.lshift(packet[21],8), packet[22] )) / 16384.0
    local z = self:uint2int16(bit.bor(bit.lshift(packet[25],8), packet[26] )) / 16384.0
    return Vector(x,y,z)
end

function MPU6050:get_accel(packet)
    local x = self:uint2int16(bit.bor(bit.lshift(packet[29],8), packet[30] )) / 16384.0
    local y = self:uint2int16(bit.bor(bit.lshift(packet[33],8), packet[34] )) / 16384.0
    local z = self:uint2int16(bit.bor(bit.lshift(packet[37],8), packet[38] )) / 16384.0
    return Vector(x,y,z)
end

function MPU6050:get_gravity(q)
    return Vector(
        2.0 * (q.x*q.z - q.w*q.y),
        2.0 * (q.w*q.x + q.y*q.z),
        q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z
    )
end

function MPU6050:get_yaw_pitch_roll(q, gravity)
    -- yaw: (about Z axis)
    local yaw = math.atan2(2*q.x*q.y - 2*q.w*q.z, 2*q.w*q.w + 2*q.x*q.x - 1)
    -- pitch: (nose up/down, about Y axis)
    local pitch = math.atan2(gravity.x, math.sqrt(gravity.y*gravity.y + gravity.z*gravity.z))
    -- roll: (tilt left/right, about X axis)
    local roll = math.atan2(gravity.y, gravity.z)
    if gravity.z < 0 then
        if pitch > 0 then
            pitch = math.pi - pitch
        else
            pitch = -math.pi - pitch
        end
    end
    return yaw, pitch, roll
end

function MPU6050:get_linear_accel_in_world(vel, q)
    return vel:get_rotated(q)
end

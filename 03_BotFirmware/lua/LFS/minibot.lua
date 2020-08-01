require 'class'
require 'mpu6050'
require 'wificontroller'
require 'vector'

MiniBot = class()
MiniBot.PI_2 = math.pi*2

function MiniBot:init(config)
    self.server_user = config.server_user
    self.server_pwd = config.server_pwd
    self.crypto_key = config.crypto_key

    self.mpu_config = {
        scl_pin = 5,
        sda_pin = 6,
        bus = 0,
        dev_addr = 0x68
    }
    
    self.servo_left = { 
        pin = config.servo_left.pin
        
    }
    self.servo_right = { 
        pin = config.servo_right.pin
    }

    self.pos = Vector()
    self.cam_dir = Vector(1,0,0) -- top left corner is (0,0)
    self.seek = Vector()
    self.accel_world = {x=0, y=0, z=0}
    self.yaw = 0
    self.roll = 0
    self.pitch = 0
    self.voltage = adc.read(0)
    self.motion_status = 0

    -- server didn't accepted this bot yet
    self.active = false
    
    
    self.wifi = WifiController(config.wifi_ssid,config.wifi_pwd)
    self.wifi.got_ip_cb = function(t) self:main(t) end
    self.wifi.connected_cb = function(t) self:connected(t) end
    self.wifi.disconnected_cb = function(t) self:reconnect(t) end
        
    self.server = net.createUDPSocket()
    self.server_host = config.server_host
    self.server_port = config.server_port
    
    self.client = net.createUDPSocket()
    self.client_port = config.client_port
    
    self.client:on("receive", function(sck, data, port, ip) 
        self:on_receive(sck, data, port, ip) 
    end)
 
end

function MiniBot:start()
    self.sensors_timer = tmr.create()
    self.sensors_timer:register(250, tmr.ALARM_AUTO, function() self:read_sensors() end )
    
    self.sync_backend = tmr.create()
    self.sync_backend:register(250, tmr.ALARM_AUTO, function() self:send_telemetry() end )
    
    self.main_timer = tmr.create()
    self.main_timer:register(200, tmr.ALARM_AUTO, function() self:steering() end )
    
    self.wifi:start()
end

function MiniBot:send_telemetry()
    local msg = string.format("t;%d;%.03f;%.03f;%.03f;%.03f;%.03f;%.03f;%d;%d",
         tmr.now() / 1000.0, self.roll, self.pitch, self.yaw,
        self.accel_world.x, self.accel_world.y, self.accel_world.z,
        self.voltage, node.heap()
    )
    self:send_message(msg)

end

function MiniBot:get_delta_angle(angle1, angle2) -- in radians
    local delta = angle1 - angle2
    if math.abs(delta) > math.pi then
        delta = delta < 0 and delta+self.PI_2 or delta-self.PI_2
    end
    return delta
end

function MiniBot:read_sensors()
    local ts = tmr.now()
    --collectgarbage()
    self.voltage = adc.read(0)
    if self.mpu then
        -- 10825
        local fifo_count = self.mpu:get_fifo_count()
        
        if fifo_count > 42 then
            --32358
            local packet = self.mpu:get_fifo_bytes(42)
            -- 8664
            local ori = self.mpu:get_orientation(packet)
            -- 8664
            local gravity = self.mpu:get_gravity(ori)
            -- 8154
            self.yaw,self.pitch,self.roll = self.mpu:get_yaw_pitch_roll(ori,gravity)
            -- 881
            --local accel = self.mpu:get_accel(packet)
            -- 8533
            --self.accel_world = self.mpu:get_linear_accel_in_world(accel, ori) 
            -- 40790         
            self.mpu:reset_fifo()
            -- 35741
            --ts = tmr.now()
            
            --self.delta_yaw = self:get_delta_angle(self.yaw, self.previous_yaw)
        end

    end
    --print('read_sensors',(tmr.now()-ts)/1000)
end

function MiniBot:steering()
    local ts = tmr.now()

    local nextDir = self.seek - self.pos
    local distToTarget = nextDir:get_magnitude()
    
    -- TODO: Set threshold in config
    if distToTarget > 5 then -- we are far from the target, let's move
        --if self.new_seek == true then
        --    self.new_seek = false
            -- let's seek a target

            -- with this operation we get the angle between the bot dir
            -- and the desired target dir
            -- we need to minimize the target Angle.
        --    self.targetAngle = self.cam_dir:get_angle(nextDir)
            
            -- ok, since we have a angle shift of PI/2, lets adjust the angle
            -- based on curr yaw.
        --    self.targetAngle = self:get_delta_angle( self.yaw, -self.targetAngle )
        --end
        
        --local deltaAngle = self:get_delta_angle( self.yaw, self.targetAngle )
        --print(self.targetAngle, deltaAngle, self.yaw)
       local deltaAngle = nextDir:get_angle( self.cam_dir )
        --local deltaAngle = -self.cam_dir:get_angle(nextDir)
        
        
        
        --- RIGHT
        -- step1 : 0.116130
        -- step2 : 0.226470
        -- step3 : 0.301119
        -- step4 : 0.494105
        -- step5 : 0.634162
        --- LEFT
        -- step1 : 0.129935
        -- step2 : 0.280129
        -- step3 : 0.330724
        -- step4 : 0.466449  
        -- step5 : 0.629485
        local absDeltaAngle = math.abs(deltaAngle)
        local servo_left_duty, servo_right_duty
        
        if absDeltaAngle > 0.30 then -- step1
            local step = absDeltaAngle > math.pi/2 and self.speed2 or self.speed1
            if deltaAngle >= 0 then
                self.motion_status = 1 -- turn left
                servo_left_duty = self.servo_left.b - step
                servo_right_duty = self.servo_right.f - step
            else
                self.motion_status = 2 -- tern right
                servo_left_duty = self.servo_left.f + step
                servo_right_duty = self.servo_right.b + step
            end
        else
            local step = distToTarget > 30 and self.speed3 or self.speed1
            self.motion_status = 3 -- go straight
            servo_left_duty = self.servo_left.f + step
            servo_right_duty = self.servo_right.f - step
        end

        pwm2.set_duty( self.servo_left.pin, servo_left_duty )
        pwm2.set_duty( self.servo_right.pin, servo_right_duty )

        pwm2.start()
    else
        self.motion_status = 0
        pwm2.stop()
    end

end


function MiniBot:on_receive(sck, data, port, ip)
    local t={}
    for c in (data .. ";"):gmatch("([^;]*);") do table.insert(t,c) end

    if t[1] == "r" then -- server asked for login
        --self.server_host = ip
        self:send_message( "l;"..self.server_user..";"..self.server_pwd)
        
    elseif t[1] == "o" then -- server start message
        self.servo_left.freq = tonumber(t[4])
        self.servo_left.period = tonumber(t[5])

        self.servo_left.s = tonumber(t[6])
        self.servo_left.f = tonumber(t[7])
        self.servo_left.b = tonumber(t[8])

        self.servo_right.freq = tonumber(t[9])
        self.servo_right.period = tonumber(t[10])

        self.servo_right.s = tonumber(t[11])
        self.servo_right.f = tonumber(t[12])
        self.servo_right.b = tonumber(t[13])

        self.speed1 = tonumber(t[14])
        self.speed2 = tonumber(t[15])
        self.speed3 = tonumber(t[16])
        
        if self.active then
            if self.id == tonumber(t[2]) then
                pwm2.stop()
                
                pwm2.setup_pin_hz(self.servo_left.pin, self.servo_left.freq, self.servo_left.period, self.servo_left.s )
                pwm2.setup_pin_hz(self.servo_right.pin, self.servo_right.freq, self.servo_right.period, self.servo_right.s )
                
                print('Updating parameters: '..self.id)
            end

        else
            print( "Server registered me! :) Let's rock! \m/")
            
            self.id = tonumber(t[2])
            self.mpu_init = tonumber(t[3])

            print('Initializing MiniBot: '..self.id)
            
            
            if self.mpu_init == 1 then
                self.mpu = MPU6050( 
                    self.mpu_config.sda_pin, self.mpu_config.scl_pin,
                    self.mpu_config.dev_addr, self.mpu_config.bus
                )
                if self.mpu:is_initialized() then
                    self.mpu:set_dmp_enabled(true)
                    collectgarbage()
                end
            end
            self.active = true
            pwm2.setup_pin_hz(self.servo_left.pin, self.servo_left.freq, self.servo_left.period, self.servo_left.s )
            pwm2.setup_pin_hz(self.servo_right.pin, self.servo_right.freq, self.servo_right.period, self.servo_right.s )
        end
            
        
    elseif t[1] == "p" then -- new position
        self.pos.x = tonumber(t[2])
        self.pos.y = tonumber(t[3])
        
    elseif t[1] == "d" then -- new direction
        self.cam_dir.x = tonumber(t[2])
        self.cam_dir.y = tonumber(t[3])
        
    elseif t[1] == "s" then -- new seek
        self.seek.x = tonumber(t[2])
        self.seek.y = tonumber(t[3])
        
    elseif t[1] == "t" then -- stop
       self.seek = self.pos:copy()
       
    end
    
end

function MiniBot:send_message(msg)
    --local len = string.len(msg)
    --local mod = len - math.floor(len/16)*16

    --if mod > 0 then 
    --    local r = 16-mod
    --    msg = msg .. string.rep(string.char(r), r)
    --end
    
    --msg = crypto.encrypt('AES-CBC', self.crypto_key, msg, '0000000000000000')
    self.server:send(self.server_port, self.server_ip, msg)
end

function MiniBot:main(t)
    local ip,nm,gw = t["IP"],t["netmask"], t["gateway"]
    
    print("IP Address: ",ip)
    print("Netmask: ",nm)
    print("Gateway Addr: ",gw,'\n')
        
    print( "Initializing servos...")

    net.dns.resolve(self.server_host, function(conn, ip) self.server_ip=ip end)
    
    collectgarbage() collectgarbage()
    print(string.format("Heap: %d", node.heap()))

    print( "Listening for UDP messages. IP: ",ip, " Port: ", self.client_port)
    self.client:listen(self.client_port, ip)
    self.client_host = ip
    
    self.wait_for_server = tmr.create()
    self.wait_for_server:register(1000, tmr.ALARM_AUTO, function()
        if self.active then
            print( "Ok. After registered start the other threads" )
            self.wait_for_server:unregister()
            self.wait_for_server = nil
            
            if self.mpu then self.mpu:reset_fifo() end
            self.sensors_timer:start()
            self.sync_backend:start()
            self.main_timer:start()
        else

            if self.server_ip ~= nil then
                print( "Pinging server...")
                self:send_message( "i;"..ip..";"..self.client_port)
            end
        end
    end)
    self.wait_for_server:start()

end

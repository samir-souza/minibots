config = {}

config.server = {
    host = 'smartcity.spock.cloud',
    port = 5000,
    user = 'minibots',
    pwd = 'aws@123'
}
config.client = {
    port = 5000
}
config.wifi = {
    ssid = 'minibots',
    pwd = '29014996',
    auto = true
}
config.running = false
config.light_pole = { -- issue with 15 and 12-14
    { right = {pins={13,15}, status=0}, forward = {pins={2,12}, status=0} },
    { right = {pins={4,14}, status=0}, forward = {pins={5,27}, status=0} },
    { right = {pins={18,26}, status=0}, forward = {pins={19,25}, status=0} },
    { right = {pins={23,32}, status=0}, forward = {pins={33,21}, status=0} }
}

for k, i in pairs(config.light_pole) do
    gpio.config({gpio=i.forward.pins, dir=gpio.OUT, pull=gpio.PULL_DOWN})
    gpio.config({gpio=i.right.pins, dir=gpio.OUT, pull=gpio.PULL_DOWN})
    for k, pin in pairs(i.forward.pins) do
        gpio.write(pin, 1)
    end
    for k, pin in pairs(i.right.pins) do
        gpio.write(pin, 1)
    end
end


local server = net.createUDPSocket()
local client = net.createUDPSocket()

local wait_for_server = tmr.create()

function update(pole_id, dir)
    local light = config.light_pole[pole_id][dir]

    -- turn off everything
    for k, pin in pairs(light.pins) do gpio.write(pin, 1) end
     
    if light.status == 1 then -- red
        gpio.write(light.pins[1], 0)
    elseif light.status == 2 then -- green
        gpio.write(light.pins[2], 0)
    elseif light.status == 3 then -- yellow
        gpio.write(light.pins[1], 0)
        gpio.write(light.pins[2], 0)
    end
end

client:on("receive", function(sck, data, port, ip) 
    if ip ~= config.server.ip then
        print('Invalid Server IP', ip)
        return
    end
    
    local t={}
    for c in (data .. ";"):gmatch("([^;]*);") do table.insert(t,c) end
    
    if t[1] == "r" then -- server asked for login
        --self.server_host = ip
        local msg = "l;"..config.server.user..";"..config.server.pwd
        server:send(config.server.port, config.server.ip, msg )
        
    elseif t[1] == "o" then -- server start message
        if not config.running then
            print( "Server registered me! :) Let's rock! \m/")
            config.running = true
            wait_for_server:stop()
        end
    elseif t[1] == 't' then
        if config.running then
            
            --print('New status', t)
            config.light_pole[1].forward.status = tonumber(t[2])
            config.light_pole[1].right.status = tonumber(t[3])
            config.light_pole[2].forward.status = tonumber(t[4])
            config.light_pole[2].right.status = tonumber(t[5])
            config.light_pole[3].forward.status = tonumber(t[6])
            config.light_pole[3].right.status = tonumber(t[7])
            config.light_pole[4].forward.status = tonumber(t[8])
            config.light_pole[4].right.status = tonumber(t[9])

            for i=1,4 do
                update(i, 'forward')
                update(i, 'right')
            end
        end
    end
    
end)
    
wifi.sta.on('got_ip', function(ev, info)
    print("IP Address: ",info.ip)
    print("Netmask: ",info.netmask)
    print("Gateway Addr: ",info.gw,'\n')
        
    print( "Initializing Traffic light...")

    print( "Listening for UDP messages. IP: ",info.ip, " Port: ", config.client.port)
    client:listen(config.client.port, info.ip)

    print( 'Resolving DNS',config.server.host)
    net.dns.resolve(config.server.host, function(conn, ip)
        print(conn, ip)
        config.server.ip = conn ~= nil and conn or ip
        
    end)
    print(config.server.ip)
    
    wait_for_server:register(2000, tmr.ALARM_AUTO, function()
        if config.server.ip ~= nil then
            print( "Pinging server...")
            local msg = "i;"..info.ip..";"..config.client.port
            server:send( config.server.port, config.server.ip, msg)
        end
    end)
    wait_for_server:start()
end)

-- start wifi
wifi.mode(wifi.STATION)
wifi.start()
wifi.sta.config(config.wifi)

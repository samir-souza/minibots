require 'class'
WifiController = class()

function WifiController:init(ssid, password)
    self.config = {}
    self.config.ssid = ssid
    self.config.auto = true
    self.config.pwd = password
    self.ready = nil

end

function WifiController:start()
    if self.connected_cb == nil then
        print('You need inform a self.connected_cb function')
        return false
    end
    self.config.got_ip_cb = self.got_ip_cb
    self.config.connected_cb = self.connected_cb
    self.config.disconnected_cb = self.disconnected_cb
    
    print('Trying to connect to '..self.config.ssid)
    
    wifi.setmode(wifi.STATIONAP)
    wifi.sta.config(self.config)
end

function WifiController:stop()
    wifi.sta.disconnect(function (ssid,bssid,reason) 
        print(ssid, bssid, reason) 
    end)
end



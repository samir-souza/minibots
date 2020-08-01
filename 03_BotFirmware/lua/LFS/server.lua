require 'class'
Server = class()

function Server:init(host, port)
    self.host = host
    self.port = port

    --self.socket = tls.createConnection()
    self.socket = net.createUDPSocket()
end

function Server:start()
    print('Trying to connect to: ', self.host, self.port)
    if self.on_receive then self.socket:on('receive', self.on_receive) end
    if self.on_sent then self.socket:on('sent', self.on_sent) end
    
    --if self.on_connect then self.socket:on('connection', self.on_connect) end
    --if self.on_disconnect then self.socket:on('disconnection', self.on_disconnect) end
    --if self.on_reconnect then self.socket:on('reconnection', self.on_reconnect) end
    --self.socket:connect(self.port, self.host)
    self.socket:listen(self.port, self.host)
end

function Server:stop()
    self.socket:close()
end

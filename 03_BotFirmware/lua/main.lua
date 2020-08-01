require 'minibot'
require 'config'

for k,v in pairs(config) do print(k,v) end
local minibot = MiniBot(config)
minibot:start()

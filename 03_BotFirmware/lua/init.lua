pcall(node.flashindex("_init"))

function abortInit()
    timer = tmr.create()
    print("Press ENTER to abort startup")
    uart.on("data", "\r", function(data)
        timer:unregister() -- disable the start up timer
        uart.on("data") -- stop capturing the uart
        print("Startup aborted")
    end, 0)

    timer:register(5000, tmr.ALARM_SINGLE, function() 
        uart.on("data") -- stop capturing the uart
        print("Running startup")
        require "main" -- run the main program
    end)
    timer:start()
    
end

abort = tmr.create()
abort:register(1000, tmr.ALARM_SINGLE, abortInit)
abort:start()

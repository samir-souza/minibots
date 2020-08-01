config = {}

-- BOT ID -> must be the same of the april tag
config["bot_id"] = 2

-- WIFI
config["wifi_ssid"] = "minibots"
config["wifi_pwd"] = "<PASSWORD>"

-- MPU 6050
config["mpu_init"] = true
config["mpu_scl_pin"] = 5
config["mpu_sda_pin"] = 6
config["mpu_bus"] = 0
config["mpu_dev_addr"] = 0x68


-- SERVER CONFIG
config["server_host"] = '<BACKEND ADDRESS>'
config["server_port"] = 5000
config["server_user"] = "minibots"
config["server_pwd"] = "aws@123"
config["client_port"] = 5000

-- SERVOS

config["servo_left"] = {}
config["servo_left"]["pin"] = 7

config["servo_right"] = {}
config["servo_right"]["pin"] = 4

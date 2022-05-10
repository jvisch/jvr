import smbus2

print('Sensors')

# defines (from 'MPU-60X0 Registers Map and Descriptions Specification')
ADDRESS_MPU6050 = 0x68

# Registers
REGISTER_CONFIG = 0x1A      # 26
REGISTER_PWR_MGMT_1 = 0x6B  # 107
REGISTER_TEMP_OUT_H = 0x41
REGISTER_TEMP_OUT_L = 0x42
REGISTER_INT_PIN_CFG = 0x37
REGISTER_USER_CTRL = 0x6A

# Bitpositions

# INT_PIN_CFG
INT_LEVEL = 7
INT_OPEN = 6
LATCH_INT_EN = 5
INT_RD_CLEAR = 4
FSYNC_INT_LEVEL = 3
FSYNC_INT_EN = 2
I2C_BYPASS_EN = 1
# USER_CTRL
FIFO_EN = 6
I2C_MST_EN = 5
I2C_IF_DIS = 4
FIFO_RESET = 2
I2C_MST_RESET = 1
SIG_COND_RESET = 0

with smbus2.SMBus(1) as bus:

    # wake up (sensor starts in sleep mode)
    bus.write_byte_data(ADDRESS_MPU6050, REGISTER_PWR_MGMT_1, 0x00)

    # read temperature
    high_byte = bus.read_byte_data(ADDRESS_MPU6050, REGISTER_TEMP_OUT_H)
    low_byte = bus.read_byte_data(ADDRESS_MPU6050, REGISTER_TEMP_OUT_L)
    raw_value = int.from_bytes([high_byte, low_byte], "big", signed=True)
    temp_celcius = (raw_value / 340.0) + 36.53

    print(temp_celcius)

    # enable pass through (for compass)
    b = bus.read_byte_data(ADDRESS_MPU6050, ADDRESS_MPU6050)
    b = b | (1 << I2C_BYPASS_EN) # set to 1
    bus.write_byte_data(ADDRESS_MPU6050, REGISTER_INT_PIN_CFG, b)
    
    b = bus.read_byte_data(ADDRESS_MPU6050, REGISTER_USER_CTRL)
    b = b & ~(1 <<  I2C_MST_EN) # set to 0
    bus.write_byte_data(ADDRESS_MPU6050, REGISTER_USER_CTRL, b)
    
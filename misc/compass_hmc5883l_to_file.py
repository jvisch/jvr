import smbus2
import time
import math
import os


NR_OF_READS = 500

# i2c address of sensor
ADDRESS = 0x1E

# all registers
CONFIGURATION_REGISTER_A = 0
CONFIGURATION_REGISTER_B = 1
MODE_REGISTER = 2
DATA_OUTPUT_X_MSB_REGISTER = 3
DATA_OUTPUT_X_LSB_REGISTER = 4
DATA_OUTPUT_Z_MSB_REGISTER = 5
DATA_OUTPUT_Z_LSB_REGISTER = 6
DATA_OUTPUT_Y_MSB_REGISTER = 7
DATA_OUTPUT_Y_LSB_REGISTER = 8
STATUS_REGISTER = 9
IDENTIFICATION_REGISTER_A = 10
IDENTIFICATION_REGISTER_B = 11
IDENTIFICATION_REGISTER_C = 12

# uit de datasheet (single measurement)
#
# 1. Write CRA (00) – send 0x3C 0x00 0x70 (8-average, 15 Hz default or any other rate, normal measurement)
# 2. Write CRB (01) – send 0x3C 0x01 0xA0 (Gain=5, or any other desired gain)
# 3. For each measurement query:
#       Write Mode (02) – send 0x3C 0x02 0x01 (Single-measurement mode)
#       Wait 6 ms or monitor status register or DRDY hardware interrupt pin
#       Send 0x3D 0x06 (Read all 6 bytes. If gain is changed then this data set is using previous gain)
#       Convert three 16-bit 2’s compliment hex values to decimal values and assign to X, Z, Y, respectively.

# Write Configuration Register A ------------------------

# Number of samples avaeraged per measurement output
MEASUREMENT_AVARAGE = {
    1: 0b00, # default
    2: 0b01,
    4: 0b10,
    8: 0b11
}
MA = MEASUREMENT_AVARAGE[8]

# Output rate
DATA_OUTPUT_RATE = {
    0.75: 0b000,
    1.5: 0b001,
    3: 0b010,
    7.5: 0b011,
    15:  0b100,  # (Default)
    30: 0b101,
    75: 0b110
    # Reserved: 111
}
DO = DATA_OUTPUT_RATE[15]

# Measurement Mode
MEASUREMENT_MODE = {
    'normal': 0b00,
    'positive_bias': 0b01,
    'negative_bias': 0b10
    # reserved 0b11
}
MS = MEASUREMENT_MODE['normal']

# Configuration A byte layout
# 7 6 5 4 3 2 1 0
# X . . . . . . . RESERVED
# . X X . . . . . MEASUREMENT_AVARAGE (MA)
# . . . X X X . . DATA_OUTPUT_RATE (DO)
# . . . . . . X X MEASUREMENT_MODE (MS)

CRA_VALUE = 0
CRA_VALUE = CRA_VALUE | (MA << 5)
CRA_VALUE = CRA_VALUE | (DO << 2)
CRA_VALUE = CRA_VALUE | (MS << 0)

# -------------------------------------------------------

# Write Configuration Register B ------------------------

# Gain Configuration
# Gaus: (bitvalue, scale/resolution)
GAIN_CONFIGURATION = {
    0.88: (0b000, 0.73),
    1.30: (0b001, 0.92),  # default
    1.90: (0b010, 1.22),
    2.50: (0b011, 1.52),
    4.00: (0b100, 2.27),
    4.70: (0b101, 2.56),
    5.60: (0b110, 3.03),
    8.10: (0b111, 4.35)
}

GN_BITS, GN_SCALE = GAIN_CONFIGURATION[5.60]

# Configuration B byte layout
# 7 6 5 4 3 2 1 0
# X X X . . . . . Gain Configuration Bits (GN_BITS)
# . . . X X X X X reserved (must be 0)

CRB_VALUE = 0
CRB_VALUE = CRB_VALUE | (GN_BITS << 5)

# -------------------------------------------------------

# Write Mode Register -----------------------------------

OPERATING_MODE = {
    'continuous': 0b00,
    'single': 0b01,  # default
    'idle': 0b10,
    'idle2': 0b11
}
MD = OPERATING_MODE['single']

# Operation Mode byte layout
# 7 6 5 4 3 2 1 0
# X . . . . . . . High speed i2c (set to 1 for high speed)
# . . . . . . X X Operating mode (MD)

MODE_VALUE = 0
MODE_VALUE = MODE_VALUE | (MD << 0)

# -------------------------------------------------------

with smbus2.SMBus(1) as bus:
    # write configuration
    bus.write_byte_data(ADDRESS, CONFIGURATION_REGISTER_A, CRA_VALUE)
    bus.write_byte_data(ADDRESS, CONFIGURATION_REGISTER_B, CRB_VALUE)

    def convert(msb, lsb):
        return int.from_bytes([msb, lsb], "big", signed=True)

    def convert_scaled(msb, lsb):
        return convert(msb, lsb) * GN_SCALE

    with open('readings.csv', 'w') as file:
        file.write('i;x_msb;x_lsb;y_msb;y_lsb;z_msb;z_lsb;scale;x;y;z;x_scaled;y_scaled;z_scaled;rad;deg')
        file.write(os.linesep)

        for i in range(0, NR_OF_READS):
            bus.write_byte_data(ADDRESS, MODE_REGISTER, MODE_VALUE)
            time.sleep(0.100)  # at least 6ms sleep
            x_msb = bus.read_byte_data(ADDRESS, DATA_OUTPUT_X_MSB_REGISTER)
            x_lsb = bus.read_byte_data(ADDRESS, DATA_OUTPUT_X_LSB_REGISTER)

            y_msb = bus.read_byte_data(ADDRESS, DATA_OUTPUT_Y_MSB_REGISTER)
            y_lsb = bus.read_byte_data(ADDRESS, DATA_OUTPUT_Y_LSB_REGISTER)

            z_msb = bus.read_byte_data(ADDRESS, DATA_OUTPUT_Z_MSB_REGISTER)
            z_lsb = bus.read_byte_data(ADDRESS, DATA_OUTPUT_Z_LSB_REGISTER)

            print(i)
            print('Converted values')
            x_scaled = convert_scaled(x_msb, x_lsb)
            y_scaled = convert_scaled(y_msb, y_lsb)
            z_scaled = convert_scaled(z_msb, z_lsb)

            x = convert(x_msb, x_lsb)
            y = convert(y_msb, y_lsb)
            z = convert(z_msb, z_lsb)

            print(f'x {x_scaled}')
            print(f'y {y_scaled}')
            print(f'x {z_scaled}')
            print('heading')
            headingRad = math.atan2(y_scaled, x_scaled)
            print(f'Rad: {headingRad}')

            if headingRad < 0:
                headingRad += 2 * math.pi
            # Check for wrap and compensate
            elif headingRad > 2 * math.pi:
                headingRad -= 2 * math.pi
            headingDeg = headingRad * 180 / math.pi
            print('Deg: ' + str(headingDeg))
            file.write(
                f'{i};{x_msb};{x_lsb};{y_msb};{y_lsb};{z_msb};{z_lsb};{GN_SCALE};{x};{y};{z};{x_scaled};{y_scaled};{z_scaled};{headingRad};{headingDeg}')
            file.write(os.linesep)

            time.sleep(0.01)
            print('-----------')

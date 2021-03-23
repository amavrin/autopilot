""" PCL interface """

# import modbus_tk
import struct
import modbus_tk.defines as cst
import modbus_tk.modbus_tcp as modbus_tcp

#CONTROLLER = "83.219.159.195"
CONTROLLER = "78.107.114.6"
#CONTROLLER = "192.162.193.94"

MODBUS = {}

def init():
    """ Connect to MB slave """
    MODBUS['master'] = modbus_tcp.TcpMaster(host=CONTROLLER)
    MODBUS['master'].set_timeout(1.0)

def wordswap(_f):
    """ Swap bytes in float so modbus with Segnetics would work """
    _a1 = bytearray(struct.pack("f", _f))

    _a2 = bytearray()
    _a2.append(_a1[1])
    _a2.append(_a1[0])
    _a2.append(_a1[3])
    _a2.append(_a1[2])

    _a3 = struct.unpack('>f', _a2)
    return _a3[0]


def process_data(inputs):
    """ Write data to modbus """
    mb_speed = wordswap(float(inputs['Speed']))
    mb_heading = wordswap(float(inputs['Heading']))
    mb_altitude = wordswap(float(inputs['Altitude']))
    mb_bank = wordswap(float(inputs['Bank']))

    mb_rpm = int(inputs['RPM'])

    mb_climb = wordswap(float(inputs['Climb']))
    mb_pitch = wordswap(float(inputs['Pitch']))
    mb_latitude = wordswap(float(inputs['Latitude']))
    mb_longitude = wordswap(float(inputs['Longitude']))
    mb_elevation = wordswap(float(inputs['Elevation']))

    _m = MODBUS['master'].execute(1, cst.READ_INPUT_REGISTERS,
                 40962, 10,
                 data_format='>fffff')

    MODBUS['master'].execute(1, cst.WRITE_MULTIPLE_REGISTERS,
                   starting_address=41984,
                   output_value=[mb_speed, mb_heading, mb_altitude, mb_bank],
                   data_format='>ffff')

    MODBUS['master'].execute(1, cst.WRITE_MULTIPLE_REGISTERS,
                   starting_address=41992,
                   output_value=[mb_rpm],
                   data_format='>h')

    MODBUS['master'].execute(1, cst.WRITE_MULTIPLE_REGISTERS,
                   starting_address=41993,
                   output_value=[mb_climb, mb_pitch, mb_latitude, mb_longitude, mb_elevation],
                   data_format='>fffff')


    ret = {}
    ret['rudder'] = wordswap(_m[0])
    ret['elevator'] = wordswap(_m[1])
    ret['aileron'] = wordswap(_m[2])
    ret['flaps'] = wordswap(_m[3])
    ret['throttle'] = wordswap(_m[4])

    return ret

def test_modbus():
    """ test MB write """

    data = [10.0, 20.0, 30.0]

    MODBUS['master'].execute(1, cst.WRITE_MULTIPLE_REGISTERS,
                   starting_address=0,
                   output_value=[wordswap(f) for f in data],
                   data_format='>fff')

    _r1 = MODBUS['master'].execute(1, cst.READ_HOLDING_REGISTERS,
                   0, 6,
                   data_format='>fff')

    print(wordswap(_r1[0]))
    print(wordswap(_r1[1]))
    print(wordswap(_r1[2]))

    _r2 = MODBUS['master'].execute(1, cst.READ_INPUT_REGISTERS,
                   0, 2,
                   data_format='>f')

    print(wordswap(_r2[0]))

def main():
    """ test main function """
    init()
    test_modbus()

if __name__ == "__main__":
    main()

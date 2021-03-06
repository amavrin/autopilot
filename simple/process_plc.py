""" PCL interface """

# import modbus_tk
import struct
import modbus_tk.defines as cst
import modbus_tk.modbus_tcp as modbus_tcp

CONTROLLER = "78.107.114.6"

ModbusVars = {}
ModbusVars['speed'] = 0
ModbusVars['heading'] = 0
ModbusVars['altitude'] = 0

def init():
    """ Connect to MB slave """
    master = modbus_tcp.TcpMaster(host=CONTROLLER)
    master.set_timeout(1.0)
    return master

def wordswap(f):
    """ Swap bytes in float so modbus with Segnetics would work """
    _a1 = bytearray(struct.pack("f", f))

    _a2 = bytearray()
    _a2.append(_a1[1])
    _a2.append(_a1[0])
    _a2.append(_a1[3])
    _a2.append(_a1[2])

    _a3 = struct.unpack('>f', _a2)
    return _a3[0]


def test_modbus(master):
    """ test MB write """

    data = [10.0, 20.0, 30.0]

    master.execute(1, cst.WRITE_MULTIPLE_REGISTERS,
                   starting_address=0,
                   output_value=[wordswap(f) for f in data],
                   data_format='>fff')

    _r1 = master.execute(1, cst.READ_HOLDING_REGISTERS,
                   0, 6,
                   data_format='>fff')

    print(wordswap(_r1[0]))
    print(wordswap(_r1[1]))
    print(wordswap(_r1[2]))

    _r2 = master.execute(1, cst.READ_INPUT_REGISTERS,
                   0, 2,
                   data_format='>f')

    print(wordswap(_r2[0]))

def main():
    """ test main function """
    master = init()
    test_modbus(master)

if __name__ == "__main__":
    main()

#!env python
""" Autopilot """

import argparse
import importlib
import socket
import sys

LISTEN_PORT=10000
LISTEN_ADDR='127.0.0.1'
FG_PORT=10001
FG_ADDR='127.0.0.1'

def receiver(sock):
    """ Receive UDP stream """
    data, _ = sock.recvfrom(1024)
    res = {}
    pairs = data.decode().split(":")
    for _p in pairs:
        var, val = _p.split("=")
        res[var] = val

    sys.stdout.write("Speed: {}, Head: {}, Alt: {}, Climb: {}, Pitch: {}, Bank: {},"
          "RPM: {}, Lat: {}, Lon: {}, Elev: {} "
          .format(res['Speed'], res['Heading'], res['Altitude'], res['Climb'],
                  res['Pitch'], res['Bank'], res['RPM'], res['Latitude'], res['Longitude'],
                  res['Elevation']))
    return res

def sender(snd_sock, aileron, elevator, rudder, throttle, flaps):
    # pylint: disable=too-many-arguments
    """ UDP Sender """
    data_str = "{:.3f}:{:.3f}:{:.3f}:{:.3f}:{:.3f}".format(aileron,elevator,rudder,throttle,flaps)
    print(data_str)
    data = str.encode(data_str + "\n")
    snd_sock.sendto(data, (FG_ADDR, FG_PORT))

def main():
    """ Main function """
    parser = argparse.ArgumentParser(description='Initialize autopilot')
    parser.add_argument('-m', action='store', dest='module', help='module')
    args = parser.parse_args()

    processor = importlib.import_module(args.module)
    processor.init()

    recv_sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
    recv_sock.bind((LISTEN_ADDR, LISTEN_PORT))

    snd_sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

    while True:
        inputs = receiver(recv_sock)
        outputs = processor.process_data(inputs)
        sender(snd_sock,outputs['aileron'],
                        outputs['elevator'],
                        outputs['rudder'],
                        outputs['throttle'],
                        outputs['flaps'])

if __name__ == "__main__":
    main()

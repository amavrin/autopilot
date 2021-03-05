#!env python
""" Autopilot """

import socket
from simple_pid import PID


LISTEN_PORT=10000
LISTEN_ADDR='127.0.0.1'
FG_PORT=10001
FG_ADDR='127.0.0.1'

SetPoints = {}
PIDS = {}

def receiver(sock):
    """ Receive UDP stream """
    data, _ = sock.recvfrom(1024)
    res = {}
    pairs = data.decode().split(":")
    for _p in pairs:
        var, val = _p.split("=")
        res[var] = val

    return res

def sender(snd_sock, aileron, elevator, rudder, throttle):
    """ UDP Sender """
    data_str = '{}:{}:{}:{}'.format(aileron,elevator,rudder,throttle)
    print(data_str)
    data = str.encode(data_str + "\n")
    snd_sock.sendto(data, (FG_ADDR, FG_PORT))

def process_heading(heading):
    """ Sample heading processing """
    return PIDS['heading'](heading)

def process_data(inputs):
    """ Main processing function """
    out = {}

    heading = float(inputs['Heading'])

    if 'heading' not in SetPoints:
        SetPoints['heading'] = heading
        print("heading: {}".format(SetPoints['heading']))

    heading_dev = heading - SetPoints['heading']
    print("Heading deviation: {}".format(heading_dev))
    out['aileron'] = 0.0
    out['elevator'] = 0.0
    out['rudder'] = process_heading(heading_dev)
    out['throttle'] = 1.0

    return out

def main():
    """ Main function """
    recv_sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
    recv_sock.bind((LISTEN_ADDR, LISTEN_PORT))

    snd_sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

    PIDS['heading'] = PID(0.01, 0.003, 0.01, setpoint=0)

    while True:
        inputs = receiver(recv_sock)
        outputs = process_data(inputs)
        sender(snd_sock,outputs['aileron'],
                        outputs['elevator'],
                        outputs['rudder'],
                        outputs['throttle'])

if __name__ == "__main__":
    main()

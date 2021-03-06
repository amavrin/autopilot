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

    print("Speed: {}, Altitude: {}".format(res['Speed'], res['Altitude']))
    return res

def sender(snd_sock, aileron, elevator, rudder, throttle):
    """ UDP Sender """
    data_str = '{:.3f}:{:.3f}:{:.3f}:{:.3f}'.format(aileron,elevator,rudder,throttle)
    print(data_str)
    data = str.encode(data_str + "\n")
    snd_sock.sendto(data, (FG_ADDR, FG_PORT))

def process_heading(heading_dev):
    """ Sample heading processing """
    return PIDS['heading'](heading_dev)

def process_bank(bank):
    """ Sample bank processing """
    return PIDS['bank'](bank)

def process_altitude(altitude_dev):
    """ Sample altitude processing """
    return PIDS['altitude'](altitude_dev)

def process_speed(speed_dev):
    """ Sample speed processing """
    return PIDS['speed'](speed_dev)

def process_data(inputs):
    """ Main processing function """
    out = {}

    heading = float(inputs['Heading'])
    speed = float(inputs['Speed'])
    altitude = float(inputs['Altitude'])
    bank = float(inputs['Bank'])

    if 'heading' not in SetPoints:
        SetPoints['heading'] = heading
        print("heading: {}".format(SetPoints['heading']))

    heading_dev = heading - SetPoints['heading']
    out['rudder'] = process_heading(heading_dev)

    # Climb to preset altitude
    altitude_dev = SetPoints['targetalt'] - altitude
    if speed <= SetPoints['takeoffspeed']:
        out['elevator'] = 0.0
    else:
        out['elevator'] = process_altitude(altitude_dev)

    out['aileron'] = process_bank(bank)

    speed_dev = speed - SetPoints['targetspeed']
    out['throttle'] = process_speed(speed_dev)

    print("Deviations: Heading {:+06.2f}, altitude {:+06.2f}, bank {:+06.2f}, speed {:+06.2f}"
          .format(heading_dev, altitude_dev, bank, speed_dev))

    return out

def main():
    """ Main function """
    recv_sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
    recv_sock.bind((LISTEN_ADDR, LISTEN_PORT))

    snd_sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

    SetPoints['takeoffspeed'] = 40.0
    SetPoints['targetalt'] = 300.0
    SetPoints['targetspeed'] = 100.0
    PIDS['heading'] = PID(0.01, 0.003, 0.01, setpoint=0)
    PIDS['bank'] = PID(0.01, 0.003, 0.01, setpoint=0)
    PIDS['speed'] = PID(0.01, 0.003, 0.01, setpoint=0)
    PIDS['speed'].output_limits = (0.0, 1.0)
    PIDS['altitude'] = PID(0.005, 0.01, 0.1, setpoint=0)
    PIDS['altitude'].output_limits = (-0.05, 0.01)

    while True:
        inputs = receiver(recv_sock)
        outputs = process_data(inputs)
        sender(snd_sock,outputs['aileron'],
                        outputs['elevator'],
                        outputs['rudder'],
                        outputs['throttle'])

if __name__ == "__main__":
    main()

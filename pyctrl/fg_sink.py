""" FG data source """
import socket
import pyctrl.block

class FgSink(pyctrl.block.Sink, pyctrl.block.Block):
    """ FgSink """

    def __init__(self, **kwargs):
        """ Init UDP socket """
        self.send_port=10001
        self.send_addr='127.0.0.1'

        self.sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

        super().__init__(**kwargs)

    def write(self, *values):
        """ Send data over UDP sock """
        data_str = '{}:{}:{}:{}'.format(values[0],values[1],
                                        values[2],values[3])
        data = str.encode(data_str + "\n")
        self.sock.sendto(data, (self.send_addr, self.send_port))

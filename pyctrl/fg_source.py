""" FG data source """
import socket
import pyctrl.block

class FgSource(pyctrl.block.Source, pyctrl.block.Block):
    """ FgSource """

    def __init__(self, **kwargs):
        """ Open UDP socket """
        listen_port=10000
        listen_addr='127.0.0.1'

        self.sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
        self.sock.bind((listen_addr, listen_port))

        super().__init__(**kwargs)

    def read(self):
        """ Receive data over UDP sock """
        data, _ = self.sock.recvfrom(1024)
        res = []
        pairs = data.decode().split(":")
        for _p in pairs:
            _, val = _p.split("=")
            res.append(val)
        return res

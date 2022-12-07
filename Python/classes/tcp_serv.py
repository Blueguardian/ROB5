import socket
import select

class TCPServer:
    # If you pass an empty string, the server will accept connections on all available IPv4 interfaces.
    # HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
    __HOST = ""
    __PORT = 8998  # Port to listen on (non-privileged ports are > 1023)
    __SIZE = 512
    __FORMAT = "utf-8"
    clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


    def __init__(self):
            self.clientSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.clientSocket.bind((self.__HOST, self.__PORT))
            self.clientSocket.listen()

            while True:
                self.connection, self.address = self.clientSocket.accept()
                if self.connection != 0 and self.address != 0:
                    self.clientSocket.setblocking(False)
                    break
                else:
                    continue
            self.connection.setblocking(False)
    def __del__(self):
        self.clientSocket.close()

    def recieveData(self):
        try:
            data = self.connection.recv(self.__SIZE)
        except socket.error:

            return 'SOCKET', 'TIMEOUT'
        else:
            if str(data.decode(self.__FORMAT)) == "" or str(data.decode(self.__FORMAT)) == '':
                return 'DATA', 'INVALID'
            else:
                temp = data.decode(self.__FORMAT)
                identifier = temp[0:temp.index(':')]
                message = temp[temp.index(':')+1:]
                return identifier, message


    def sendData(self, identifier, messege):
        data = identifier+":"+messege
        data = str(data)
        self.connection.sendall(data.encode(self.__FORMAT))



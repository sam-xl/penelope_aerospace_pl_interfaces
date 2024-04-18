from multiprocessing import connection
import socket
import datetime
import os
import threading
import time
import pickle
import queue

class ClientHandler(threading.Thread):
    """
    Receive messages from multiple client sockets on cobot, handle the messages
    """

    wait_before_flushing = 2
    def __init__(self, connection, handle_cobot_message, send_message_queue = None, bytes_msg_length = 4) -> None:
        super().__init__()
        self.socket = connection
        self.handle_cobot_message = handle_cobot_message
        self.bytes_msg_length = bytes_msg_length
        self.send_message_queue = send_message_queue
        self.live = False

    def send_message(self, message):
        message = str(message)
        if self.send_message_queue != None:
            self.send_message_queue.put(message)
        else:
            print(message)

    def _recv_n_bytes(self,sock,n):
            """
            Convenience method for receiving exactly n bytes from socket
            Errors must be caught by the caller
            """
            data = ''
            while len(data) < n:
                bufsize = n - len(data)
                chunk = sock.recv(bufsize)
                if chunk == b'':
                    raise IOError("Did not receive first {} bytes of the message. Data received: '{}'".format(n, data.decode()) )
                data += chunk.decode()
            return data
    
    def _RECEIVE(self,sock):
        """.
        Method to receive messages from the socket connection
        Returns message string if message was received
        Return None otherwise
        Error must be caught by the caller
        """
        msg = ""
        bytes_recd = 0
        msg = self._recv_n_bytes(sock, self.bytes_msg_length) # receive first part of the message, that contains the message length
        bytes_recd = len(msg)
        MSGLEN = int( msg [ 0 : self.bytes_msg_length ] )
        while bytes_recd < MSGLEN:
            chunk = sock.recv(MSGLEN - bytes_recd)
            if chunk == b'':
                raise IOError ("Did not receive complete message from socket. Received message: " + msg)
            bytes_recd += len(chunk)
            msg += chunk.decode()
        return msg
    
    def _SEND(self, msg):
        """
        Method to send a message through the socket connection
        Returns None if send was successfull
        Raises Error if send was not successfull
        Errors must be caught by the caller
        """
        totalsent = 0
        MSGLEN = len(msg)
        while totalsent < MSGLEN:
            sent = self.sock.send(msg[totalsent:].encode())
            if sent == 0:
                raise IOError("Did not send complete message to SetiTec controller. Sent message: " + msg[:totalsent])
            totalsent = totalsent + sent

    def run(self):
        self.live = True

        while self.live:
            while True:
                message = self._RECEIVE(self.socket)

                # handle the message
                self.handle_cobot_message(message)

                # send any messages if there is a queue given
                if self.send_message_queue is not None:
                    try:
                        msg = self.send_message_queue.get(block = False)
                    except queue.Empty:
                        break

                self._SEND( msg )

    def stop(self):
        self.live = False

class CobotTCPServer(threading.Thread):

    def __init__(self, handle_cobot_message, send_message_queue = None, bindaddress='', portname=65432, maxqueue=5, save_to_file = True) -> None:
        super().__init__()
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.setblocking(True)
        self.server_address = (bindaddress, portname)
        self.server.bind(self.server_address)
        self.server.listen(maxqueue)
        self.client_list = dict()
        self.handle_cobot_message = handle_cobot_message
        self.send_message_queue = send_message_queue
        self.live = False

    def send_message(self, message):
        message = "Cobot TCP server," + str(message)
        if self.send_message_queue != None:
            self.send_message_queue.put(message)
        else:
            print(message)

    def run(self):
        self.live = True
        while self.live:
            connection, client_address = self.server.accept()
            self.send_message("connection from client address {}".format(client_address))
            new_client_handler = ClientHandler(connection, self.handle_cobot_message, self.send_message_queue)
            new_client_handler.start()

    def stop(self):
        self.live = False

if __name__ == "__main__":

    def handle_cobot_message(cobot_msg):
        bytes_msg_length = 4
        return pickle.dumps(cobot_msg[bytes_msg_length:], protocol=3)

    # Create the server
    cobot_server = CobotTCPServer(handle_cobot_message)

    # Activate the server; this will keep running until you interrupt the program with Ctrl-C
    cobot_server.start()
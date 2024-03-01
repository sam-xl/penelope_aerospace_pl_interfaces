import socket
import select
import threading
import time
import queue

# thread loop related stuff
max_wait_for_cmd_execution = 30 # [s] max time the objects wait for successfull execution of controller
max_wait_for_subscription = 10
sleep_duration = 0.1 # [s] time it sleeps when in a loop
keep_alive_interval = 10

# socket related stuff
ignore_last_character = True # last character is null. ignore otherwise #tp_popup throws an error
last_character_msg = '\x00'
message_end = "\x00"
bytes_msg_length = 4 # number of bytes at the beginning of the message where the total message length is given
msg_ending_characters = 1

length_EC_output_value = 5

class TCP_IP_Client(threading.Thread):

    #daemon = True # TBD difference to main code

    def __init__(self, log_message_queue = None, host = '10.237.20.101', port = 4545):
        super().__init__()
        self.host = host
        self.port = port
        self.send_cmd_q = queue.Queue()
        self.send_subscription_q = queue.Queue()
        self.received_q = queue.Queue() # FIFO queue
        self.stop_q = queue.Queue() # any message in this queue causes the thread to stop
        self.is_connected = False
        self.is_comm_started = False
        self.time_last_msg = time.time() # initialization
        self.last_msg_sent = "" # initialize
        self.last_cmd_sent = "" # initialize. a command is any message except acknowledgements
        self.is_last_cmd_acknowledged = True # initialize
        self.sleep_interval = 0.1
        self.method_raising_last_select_error = "" # can take read or write as values

        self.msg_handlers = {
                "0002": self._handle_msg_0002_COMMUNICATION_START,
                "0004": self._handle_msg_0004_COMMAND_ERROR,
                "0005": self._handle_msg_0005_COMMAND_ACCEPTED,
                "0071": self._handle_msg_0071_ALARM,
                "0217": self._handle_msg_0217_RELAY_FUNCTION,
                "0221": self._handle_msg_0221_DIGIN_FUNCTION,
                "7701": self._handle_msg_7701_EC_OUTPUT_FUNCTION,
                "7711": self._handle_msg_7711_MODULE_INFOS,
                "9999": self._handle_msg_9999_KEEP_ALIVE
        }

        self.log_message_queue = log_message_queue

    def log_message(self, message):
        message = "SetiTec EVO," + str(message)
        if self.log_message_queue != None:
            self.log_message_queue.put(message)
        else:
            print(message)
            
    def run(self):
        """Interfacing with the class is done through the following class attributes:
        send_cmd_q: queue.Queue()
            the main thread must put here messages to be sent to the Setitec controller
            messages must be in string format, ending with the nul characther \x00
        received_q: queue.Queue()
            messages received from the Setitec controller are put here, except from keep alive and communication start replies
        stop_q: queue.Queue()
            when any element is put in this queue, it signals the Setitec_Client_Thread to stop
        
        !!! When this code is run on a Doosan Cobot (controller v2.4, with Python 3.2.6), it is recommended to stop this thread
        by adding a element to the stop_q and by waiting for termination of the thread with the join() method !!!

        If there is a connection error, the thread automatically disconnects, resets its status attributes (including the send and receive queues), and attempt to reconnect and restart communication with the Setitec controller
        After three consecutive connection errors, the thread signals the problem to the user and stops attempting reconnection"""

        while True:

            time.sleep(self.sleep_interval)
            try:
                msg = self.stop_q.get(block = False)
                self.log_message("Exiting thread for communication with Setitec")
                break

            except queue.Empty as e: # no stop message received

                try:
                    if self.is_connected == True:

                        while True: # receive and handle all messages SetiTec wants to send
                            msg = self._RECEIVE()
                            if msg == None:
                                break
                            else:
                                self._HANDLE_MSG(msg)

                        if self.is_comm_started == True:
                            if time.time() - self.time_last_msg >= keep_alive_interval:
                                self._SEND(msg_9999_keep_alive)
                                continue

                            else:
                                if self.is_last_cmd_acknowledged == True:
                                    try:
                                        msg = self.send_subscription_q.get(block = False) # first make sure all subscription messages are sent
                                    except queue.Empty:
                                        try:
                                            msg = self.send_cmd_q.get(block = False) # is all subscriptions are sent, then send command messages if any
                                        except queue.Empty:
                                            continue
                                        except Exception as e:
                                            self.log_message("Error thrown when checking whether any command messages should be sent: " + str(e))
                                    except Exception as e:
                                        self.log_message("Error thrown when checking whether any subscription messages should be sent: " + str(e))
                                    self._SEND(msg, 1) # send only one message at the time

                        else: # communication not-started
                            self._START_COMMUNICATION()
                            self._SUBSCRIBE_TO_EVENT_MSGS()

                    else: # not connected
                        self._CONNECT()

                except IOError as e:
                    self.log_message("IO Error: " + str(e))
                    self._CLOSE()
                    self._RESET()

                except select.error as e:
                    if self.method_raising_last_select_error == "_RECEIVE":
                        # error is only logged and counted
                        # thread will continue from top of the while loop
                        self.log_message("Select error on receiving: " + str(e))
                        pass
                    else:
                        self._CLOSE()
                        self._RESET()

                except Exception as e:
                    self.log_message("Exception: " + str(e))
                    self._CLOSE()
                    self._RESET()

    def _CONNECT(self):
        """
        Method to connect to a TCP/IP server
        Errors must be caught by the caller
        """
        self.log_message("Connecting to EDU...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.sock.connect((self.host, self.port))
        self.is_connected = True
        self.time_last_msg = time.time()
        self.log_message("Connected to EDU")

    def _CLOSE(self):
        """
        Method to close a connection to a TCP/IP server
        Resets attribute is_connected to False
        Errors must be caught by the caller
        """
        self.sock.close()
        self.is_connected = False
        self.log_message("Disconnected")

    def _SEND(self, msg, is_cmd_msg = 0):
        """
        Method to send a message through the socket connection
        Returns None if send was successfull
        Raises Error if send was not successfull
        Errors must be caught by the caller
        """
        totalsent = 0
        try:
            ready_to_write = select.select( [], [self.sock], [], 0.1 )[1]
        except select.error as e:
            self.log_message("Select error while checking for sockets ready to WRITE: " + str(e))
            self.method_raising_last_select_error = "_SEND"
            raise
        else:
            if len(ready_to_write) > 0:
                MSGLEN = len(msg)
                while totalsent < MSGLEN:
                    sent = self.sock.send(msg[totalsent:].encode())
                    if sent == 0:
                        raise IOError("Did not send complete message to SetiTec controller. Sent message: " + msg[:totalsent])
                    totalsent = totalsent + sent
                self.time_last_msg = time.time()
                self.sleep_interval = 0.1
                self.last_msg_sent = EDU_msg(msg)
                if is_cmd_msg == 1:
                    self.last_cmd_sent = EDU_msg(msg)
                    self.is_last_cmd_acknowledged = False
                self.log_message("=> " +  self.last_msg_sent.MID + " " + self.last_msg_sent) 
            else:
                raise IOError('Socket was not ready to write')

    def _recv_n_bytes(self,n):
        """
        Convenience method for receiving exactly n bytes from self.socket
        Errors must be caught by the caller
        """
        data = ''
        while len(data) < n:
            bufsize = n - len(data)
            chunk = self.sock.recv(bufsize)
            if chunk == b'':
                raise IOError("Did not receive first %d bytes of the message. Data received:" + data + "\end" % (n))
            data += chunk.decode()
        return data

    def _RECEIVE(self):
        """
        Method to receive messages from the socket connection
        Returns message string if message was received
        Return None otherwise
        Error must be caught by the caller
        Input:
        timeout: flot or integer, optional
            timeout when checking if there are messages to be received
        """
        try:
            ready_to_read = select.select([self.sock], [], [], 0.1 )[0]
        except select.error as e:
            self.log_message("Select error while checking for sockets ready to READ: " + str(e))
            self.method_raising_last_select_error = "_RECEIVE"
            raise
        else:
            if len(ready_to_read) != 0:
                msg = ""
                bytes_recd = 0
                msg = self._recv_n_bytes(bytes_msg_length) # receive first part of the message, that contains the message length
                bytes_recd = len(msg)
                MSGLEN = int( msg [ 0 : bytes_msg_length ] ) + msg_ending_characters
                while bytes_recd < MSGLEN:
                    chunk = self.sock.recv(MSGLEN - bytes_recd)
                    if chunk == b'':
                        raise IOError ("Could not receive complete message from SetiTec controller. Received message: " + msg)
                    bytes_recd += len(chunk)
                    msg += chunk.decode()
                self.time_last_msg = time.time()
                message_received = EDU_msg(msg)
                self.log_message("<= " +  message_received.MID + " " + message_received)
                self.sleep_interval = 0.1
                return msg
            else:
                self.sleep_interval = 1
                return None

    def _START_COMMUNICATION(self):
        """
        Method to start the communication with the Setitec controller
        It sends a communication start message and checks the reply
        When successfull sets the attribute is_comm_started to True
        """
        self._SEND(msg_0001_comm_start)

    def _SUBSCRIBE_TO_EVENT_MSGS(self):
        # add subscription messages to send queue
        for msg in subscription_msgs:
            self.send_subscription_q.put(msg)

    def _RESET(self):
        """
        Method to reset the thread attributes describing its status
        Use this method after closing the socket connection and before re-connecting
        """
        self.send_cmd_q = queue.Queue()
        self.send_subscription_q = queue.Queue()
        self.received_q = queue.Queue()
        self.is_comm_started = False
        self.time_last_msg = time.time()
        self.reset()

    def _HANDLE_MSG(self,msg):
        msg = EDU_msg(msg) # make the message an object
        self.msg_handlers[msg.MID](msg)
    
    def _handle_msg_0002_COMMUNICATION_START(self, msg):
        self.is_comm_started = True

    def _handle_msg_0004_COMMAND_ERROR(self, msg):
        # TBD behavior. last command must be considered acknowledged, otherwise it blocks here
        self.is_last_cmd_acknowledged = True
        self.log_message("Command error message: " + msg + "\n" + "Last message sent: " + self.last_msg_sent)

    def _handle_msg_0005_COMMAND_ACCEPTED(self, msg):

        if msg.data[0] == self.last_cmd_sent.MID:

            self.is_last_cmd_acknowledged = True

            if self.last_msg_sent.MID == "0070":
                self.is_subscribed_to_alarms =  True
            elif self.last_msg_sent.MID == "0018":
                self.time_Pset_last_selected = time.time()
            elif self.last_msg_sent.MID == "0216":
                self.relays[self.last_cmd_sent.data[0]].is_subscribed = True
            elif self.last_msg_sent.MID == "0220":
                self.digINs[self.last_cmd_sent.data[0]].is_subscribed = True
            elif self.last_msg_sent.MID == "7700":
                self.EC_outputs[self.last_cmd_sent.data[0]].is_subscribed = True

        else:
            raise RuntimeError("Could not determine which command has been accepted. Last message sent: " + self.last_msg_sent + "\nLast message received: " + msg + ".")

    def _handle_msg_0071_ALARM(self, msg):
        # TBD behaviour
        self.log_message("Alarm message received: " + msg)
        self._SEND(msg_0071_alarm_ack)

    def _handle_msg_0217_RELAY_FUNCTION(self, msg):
        relay_no = msg.data[0]        
        relay_status = msg.data[1]
        relay = self.relays[relay_no]
        relay.change_status(relay_status)
        self._SEND(msg_0218_relay_ack)

    def _handle_msg_0221_DIGIN_FUNCTION(self, msg):
        digIN_no = msg.data[0]
        digIN_status = msg.data[1]
        digIN = self.digINs[digIN_no]
        digIN.change_status(digIN_status)
        self._SEND(msg_0222_digIN_ack)

    def _handle_msg_7701_EC_OUTPUT_FUNCTION(self,msg):
        EC_output_no = msg.data[0]
        EC_output_value = msg.data[1]
        EC_output = self.EC_outputs[EC_output_no]
        EC_output.value = EC_output_value
        self._SEND(msg_7702_EC_output_ack)

    def _handle_msg_7711_MODULE_INFOS(self,msg):
        module = EDU_module(msg.data)
        if module.head_name != '               ': # it means there is a module
            self.EDU_module = module
        else:
            self.EDU_module = None
        self._SEND(msg_7712_modules_infos_ack)

    def _handle_msg_9999_KEEP_ALIVE(self, msg):
        pass

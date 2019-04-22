import socket, time
import threading
import queue

globalVar = ""

class ClientSocket(threading.Thread):
    def __init__(self, IP, PORT):
        super(ClientSocket, self).__init__()
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((IP, PORT))
  
        print ('connected')
        self.alive = threading.Event()
        self.alive.set()

    def recieveData(self):
        global globalVar
        try:
            data = self.s.recv(105)
            print (data)
            globalVar = data
        except IOError as e:
            if e.errno == errno.EWOULDBLOCK:
                pass

    def sendData(self, sendingString):
        print ('sending')
        sendingString += "\n"
        self.s.send(sendingString.encode('UTF-8'))
        print ('done sending')

    def run(self):
        global globalVar
        while self.alive.isSet():
            data = self.s.recv(105)
            print (data)
            globalVar = data
            if(data == "0"):
                self.killSocket()
            
           
            
    def killSocket(self):
        self.alive.clear()
        self.s.close()
        print("Goodbye")
        exit()
            

# IP = '10.152.168.109'
# PORT = 5010
# client = ClientSocket(IP, PORT)
##client.start()

#while(True):
##    for i in ["hello human", "How are you", "Sorry, you must die now"]:
#    answer = input("Give a string: ")
#    if answer == "exit":
#        break;
#    time.sleep(1)
#    client.sendData(answer)
#
# client.killSocket()
# print("Exiting Sends")

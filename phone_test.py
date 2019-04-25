from client import ClientSocket
import time

IP = '10.200.39.59'
PORT = 5010
client = ClientSocket(IP, PORT)
client.sendData("This is a test")
time.sleep(1)
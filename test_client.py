import socket, pickle
import depthai as dai

HOST = '192.168.1.6'
PORT = 50007
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

CHOST = '192.168.1.3'
CPORT = 5008
cs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
cs.bind((CHOST, CPORT))
cs.listen()
conn, addr = cs.accept()

# pipeline = dai.Pipeline()
while True:
    message = input("your message: ")
    if message == 'q':
        break
    data_string = pickle.dumps(message)
    s.send(data_string)
    data = conn.recv(4096)
    message = pickle.loads(data)
    print("message received: " + message)
    

s.close()
conn.close()
print('Data sent to server')

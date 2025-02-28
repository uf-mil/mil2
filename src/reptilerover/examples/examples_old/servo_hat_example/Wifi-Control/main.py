import time
import threading
import socketserver 
from PCA9685 import PCA9685
from time import ctime  
import socket

print("start")

pwm = PCA9685(0x40)
pwm.setPWMFreq(50)

Pos0 = 1500  
Pos1 = 1500 
Pos2 = 1500 
Pos3 = 1500 
Step0 = 0  
Step1 = 0  
Step2 = 0  
Step3 = 0  
pwm.setServoPulse(0,Pos0)
pwm.setServoPulse(1,Pos1)
pwm.setServoPulse(2,Pos2)
pwm.setServoPulse(3,Pos3)

time.sleep(5)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(('8.8.8.8', 80))
host = s.getsockname()[0]
port = 8000
print(host + ':%d' %port)
addr = (host,port)  
	
class Servers(socketserver.StreamRequestHandler): 
    def handle(self): 
        global Step0,Step1,Step2,Step3
        print('got connection from ',self.client_address)  
        self.wfile.write(('connection %s:%s at %s succeed!' % (host,str,ctime())).encode('utf-8')) 
        while True:  
            data = self.request.recv(1024)
            data = data.decode('utf-8')
            data = str(data)
            if not data:   
                break  
            if data == "Stop":
                Step0 = 0
                Step1 = 0
                Step2 = 0
                Step3 = 0
            elif data == "Forward":
                Step0 = -5
            elif data == "Backward":
                Step0 = 5
            elif data == "TurnLeft":
                Step1 = -5
            elif data == "TurnRight":
                Step1 = 5
            elif data == "Up":
                Step2 = -5
            elif data == "Down":
                Step2 = 5
            elif data == "Left":
                Step3 = 5
            elif data == "Right":
                Step3 = -5
            print(data)   
            #print "recv from ", self.client_address[0]  
            self.request.send(data.encode('utf-8'))  
			
def timerfunc():
	global Step0,Step1,Step2,Step3,Pos0,Pos1,Pos2,Pos3,pwm

	if(Step0 != 0):
		Pos0 += Step0
		if(Pos0 >= 2500): 
			Pos0 = 2500
		if(Pos0 <= 500):
			Pos0 = 500
		#set channel 0
		pwm.setServoPulse(0,Pos0)    
		
	if(Step1 != 0):
		Pos1 += Step1
		if(Pos1 >= 2500): 
			Pos1 = 2500
		if(Pos1 <= 500):
			Pos1 = 500
		#set channel 1
		pwm.setServoPulse(1,Pos1)   
	
	if(Step2 != 0):
		Pos2 += Step2
		if(Pos2 >= 2500): 
			Pos2 = 2500
		if(Pos2 <= 500):
			Pos2 = 500
		#set channel 2
		pwm.setServoPulse(2,Pos2)   
		
	if(Step3 != 0):
		Pos3 += Step3
		if(Pos3 >= 2500): 
			Pos3 = 2500
		if(Pos3 <= 500):
			Pos3 = 500
		#set channel 3
		pwm.setServoPulse(3,Pos3)   
	
	global t        #Notice: use global variable!
	t = threading.Timer(0.02, timerfunc)
	t.start()

t = threading.Timer(0.02, timerfunc)
t.setDaemon(True)
t.start()

server = socketserver.ThreadingTCPServer(addr,Servers)  
print('server is running....')  
server.serve_forever()  

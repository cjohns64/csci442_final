import tkinter as tk
import maestro                                                     

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3
ELBOW = 7
SHOULDER = 5
SHOULDER_SIDE = 6
HAND = 10

# def sendCommand(x):
#    if(x == '8'):
#        tango.setTarget(MOTOR, 6800)
class KeyControl():
    def __init__(self):
        self.tango = maestro.Controller()
        self.body = 6000
        self.headTurn = 6000
        self.headTilt = 6000
        self.motors = 6000
        self.turn = 6000
        self.elbow = 5000
        self.shoulder = 6000
        self.shoulder_side = 7000
        self.hand = 4800
        self.tango.setTarget(ELBOW, self.elbow)
        self.tango.setTarget(SHOULDER, self.shoulder)
        self.tango.setTarget(SHOULDER_SIDE, self.shoulder_side)
        self.tango.setTarget(HAND, self.hand)
        
    def head(self,key):
        print(key.keycode)
        if key.keycode == 38:
            self.headTurn += 200
            if(self.headTurn > 7900):
                self.headTurn = 7900
            self.tango.setTarget(HEADTURN, self.headTurn)
        elif key.keycode == 52:
            self.headTurn -= 200
            if(self.headTurn < 1510):
                self.headTurn = 1510
            self.tango.setTarget(HEADTURN, self.headTurn)
        elif key.keycode == 25:
            self.headTilt += 200
            if(self.headTilt > 7900):
                self.headTilt = 7900
            self.tango.setTarget(HEADTILT, self.headTilt)
        elif key.keycode == 39:
            self.headTilt -= 200
            if(self.headTilt < 1510):
                self.headTilt = 1510
            self.tango.setTarget(HEADTILT, self.headTilt)

    def waist(self, key):
        print(key.keycode)
        self.tango.getPosition(ELBOW)
        if key.keycode == 54:
            self.body += 200
            if (self.body > 7900):
                self.body = 7900
            self.tango.setTarget(BODY, self.body)
            print("waist right")
        elif key.keycode == 52:
            self.body -= 200
            if (self.body < 1510):
                self.body = 1510
            self.tango.setTarget(BODY, self.body)
            print('waist left')
        elif key.keycode == 53:
            # self.elbow+=200
            self.shoulder_side-=200
            print (self.shoulder_side)
            # print(self.elbow)
            # self.shoulder-=200
            # print(self.shoulder)
            #self.hand += 200
            # print(self.hand)
            # self.tango.setTarget(HAND, self.hand)
            self.tango.setTarget(SHOULDER_SIDE, self.shoulder_side)
            # self.tango.setTarget(ELBOW, self.elbow)
            # self.tango.setTarget(SHOULDER,self.shoulder)
   
    
    def arrow(self, key):
        print(key.keycode)
        if key.keycode == 116:
            self.motors += 200
            if(self.motors > 7900):
                self.motors = 7900
            print(self.motors)
            self.tango.setTarget(MOTORS, self.motors)
        elif key.keycode == 111:
            self.motors -= 200
            if(self.motors < 1510):
                self.motors = 1510
            print(self.motors)
            self.tango.setTarget(MOTORS, self.motors)
        elif key.keycode == 114:
            self.turn += 200
            if(self.turn > 7400):
                self.turn = 7400
            print(self.turn)
            self.tango.setTarget(TURN, self.turn)
        elif key.keycode == 113:
            self.turn -= 200
            if(self.turn <2110):
                self.turn = 2110
            print(self.turn)
            self.tango.setTarget(TURN, self.turn)
        
        elif key.keycode == 65:
            self.motors = 6000
            self.turn = 6000
            self.tango.setTarget(MOTORS, self.motors)
            self.tango.setTarget(TURN, self.turn)

win = tk.Tk()
keys = KeyControl()

win.bind('<Up>', keys.arrow)
win.bind('<Left>', keys.arrow)
win.bind('<Down>', keys.arrow)
win.bind('<Right>', keys.arrow)
win.bind('<space>', keys.arrow)
win.bind('<z>', keys.waist)
win.bind('<c>', keys.waist)
win.bind('<w>', keys.head)
win.bind('<s>', keys.head)
win.bind('<a>', keys.head)
win.bind('<d>', keys.head)
win.bind('<x>', keys.waist)
win.mainloop()
keys = KeyControl()

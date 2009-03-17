"""
Creation of control window class
"""
import visual as v
from visual import controls


class controlwindow(object):
#constructor##################
    def __init__(self, visualizer, _rate = 100):
        #init self.variable
        self.x      = visualizer._scene.x + visualizer._scene.width
        self.y      = visualizer._scene.y
        self.width  = 400
        self.height = 100
        self.btnw = 30
        self.btnh = 15
        self.sldrl = 140
        self.offsetx = (2*self.btnw + self.sldrl)/2
        self.offsety = 0
        self.interactRate = _rate #in Hz, it's a frequency
        self.speedRate = 1
        self.isPlaying = False
        self.nbSnapShot = 100
        
        #creation of control btn and sldr 
        c = v.controls.controls(x = self.x, y = self.y, width = self.width, height = self.height, range=120,
            title='control window') # Create controls window
        self.btn_play = v.controls.button( pos = (0-self.offsetx,0-self.offsety), width = self.btnw, height = self.btnh,
            text='play', action = lambda: self.act_play())
        self.btn_speed = v.controls.button( pos = (self.btnw-self.offsetx,0-self.offsety), width = self.btnw,
            height = self.btnh, text='1', action = lambda:self.act_speed())
        self.slider   = v.controls.slider( pos = (2*self.btnw-self.offsetx,0-self.offsety), axis = (1,0,0),
            width=self.btnh, height=self.btnh, length=self.sldrl, min=0, max=self.nbSnapShot, action = lambda: self.act_slide())
            
        #interact
        while 1:
            v.rate(self.interactRate)
            c.interact()
            if self.isPlaying==True:
                self.displayNextSnapShot()
                
                
############################
#class functions
############################  
    def displayNextSnapShot(self):
        if (self.slider.value + self.speedRate < self.nbSnapShot):
            self.slider.value = self.slider.value + self.speedRate
        else:
            self.slider.value = self.nbSnapShot
            self.act_play()
            
        
    def act_play(self):
        if (self.btn_play.text == 'play'):
            self.btn_play.text = 'stop'
            self.isPlaying = True
            if self.slider.value == self.nbSnapShot:
                self.slider.value = 0
        else:
            self.btn_play.text = 'play'
            self.isPlaying = False
            
    def act_speed(self):
        if self.btn_speed.text == '1':
            self.btn_speed.text = '2'
            self.speedRate = 2
        elif self.btn_speed.text == '2':
            self.btn_speed.text = '4'
            self.speedRate = 4
        elif self.btn_speed.text == '4':
            self.btn_speed.text = '8'
            self.speedRate = 8
        elif self.btn_speed.text == '8':
            self.btn_speed.text = '16'
            self.speedRate = 16
        elif self.btn_speed.text == '16':
            self.btn_speed.text = '32'
            self.speedRate = 32
        elif self.btn_speed.text == '32':
            self.btn_speed.text = '64'
            self.speedRate = 64
        elif self.btn_speed.text == '64':
            self.btn_speed.text = '1'
            self.speedRate = 1
            
    def act_slide(self):
        print round(self.slider.value)

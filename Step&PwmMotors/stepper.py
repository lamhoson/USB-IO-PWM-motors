# -*- coding: utf-8 -*-
"""
Created on Mon Nov 23 15:09:07 2020

USB GPIO routines to drive Stepper Motor and PWM Motor

ver 1.0 - 24Nov2020
ver 1.1 - 1Dec2020

@author: 3Ddemo
"""

import ctypes; import os
from time import sleep
import PySimpleGUI as sg 
import cv2; import numpy as np
import winsound # beep speaker during very small motor movements.

DEBUG1=False
CAM0=0 # default camera ID for openCV

DIVIDER_DELAY={1:0.05, 8:0.00001, 128:0.0 } #delays for different angle dividers
PORT_CONFIG=0x00 # 0=O/P, 1=I/P
WRITE_EN=ctypes.c_byte(0x00) # 0 write enable. If 1=> readOnly.  

LOW_MASK=0xFE; HIGH_MASK=0x01 # for low-to-high pluse, bit0
CLK_WISE=0b101; ANTI_CLK_W=0b010 # set motor direction, bit1
MOTOR_EN=0b100; MOTOR_DISABLE=0b011 # enable motor, bit2


def c_byte(i): # convert to C-language byte format 
    return ctypes.c_byte(i)

class UsbDevice(object):
    name='USB-Device' # class string
    def __init__(self):        
        try:
            Objdll= ctypes.cdll.LoadLibrary(os.path.join(os.getcwd(), "usb2uis64.dll"))
            print('Successfully loaded 64bit driver.')   
        except OSError as error :  
            print(f'Try else 64/32bit version due to: {error}')
            Objdll= ctypes.cdll.LoadLibrary(os.path.join(os.getcwd(), "usb2uis32.dll"))
            print('Successfully loaded 32bit driver.')
        
        Objdll.USBIO_OpenDevice.restype=ctypes.c_byte

        Objdll.USBIO_CloseDevice.restype=ctypes.c_bool
        Objdll.USBIO_CloseDevice.argtypes=(ctypes.c_byte,)

        self.id=c_byte(Objdll.USBIO_OpenDevice())
        self.objdll=Objdll
        
        print(f"USB Device Open:{'Done!' if (self.id.value==0) else 'Wrong!'}")


class GpioStepper(UsbDevice):
    name='USB-GPIO-IO001' # class string
    def __init__(self, step_angle, p_divider): # p_divider=pulse division rate
        super().__init__()
        self.objdll.USBIO_GetGPIOConfig.restype=ctypes.c_bool
        self.objdll.USBIO_SetGPIOConfig.restype=ctypes.c_bool
        self.objdll.USBIO_GPIOWrite.restype=ctypes.c_bool
        
        self.objdll.USBIO_GetGPIOConfig.argtypes=(ctypes.c_byte,ctypes.c_void_p)
        self.objdll.USBIO_SetGPIOConfig.argtypes=(ctypes.c_byte,ctypes.c_byte)
        self.objdll.USBIO_GPIOWrite.argtypes=(ctypes.c_byte,ctypes.c_byte,ctypes.c_byte)

        self.step_angle=step_angle
        self.p_divider=p_divider
        self.delay=DIVIDER_DELAY[p_divider] # second between Lo-Hi, Hi-Lo, total delay X2


        p255=ctypes.byref(c_byte(255)) # point to a byte storage buffer.      
        print(f"GetGPIOConfig:{self.objdll.USBIO_GetGPIOConfig(c_byte(0),p255)}")
        print(f"SetGPIOConfig:{self.objdll.USBIO_SetGPIOConfig(c_byte(0),c_byte(PORT_CONFIG))}")


    def _lo_hi_preprocess(self, direction=CLK_WISE): # class's internal call
        """
        Mask corresponding bits to creat a high-signal and a low-signal patterns
         for driving the motor's controller

        Parameters
        ----------
        direction : default is CLK_WISE.

        Returns
        ------- (as C byte data format)
        low : low signal driving pattern
        high : high signal driving pattern

        """
        low=MOTOR_EN #init low-signal as 0b100. bit2=ENA, bit1=clockwise(0), bit0=pulse level
        low=low & CLK_WISE if  direction==CLK_WISE else low | ANTI_CLK_W # mask direction bit1       
        high=low | HIGH_MASK #creat hign-signal pattern. bitwise or with x01
        print('Low-Signal pattens:', bin(low), ' High-Signal is:', bin(high))
        # low=c_byte(low); high=c_byte(high) #pre-process for faster loop
        
        return c_byte(low), c_byte(high) # convert to C-byte data format
            

    def angle_to_pulses(self, angle):
        pulse=float(angle)/ (self.step_angle/self.p_divider); temp=pulse
        pulse=int(pulse )
        print(f'\nPulse for {angle} degrees is:{pulse}. StepAngle:{self.step_angle}, DivideRatio:{self.p_divider}\n')
        print(f"Pulse counts truncation error:{'{:.4f}'.format(pulse-temp)}")
        return pulse

    def run(self, pulse, direction=CLK_WISE): # drive the motor to run by pulses
        low, high=self._lo_hi_preprocess(direction) # ret in C byte format already
        # low=c_byte(low); high=c_byte(high) #convert to C byte data

        for i in range(pulse):
            self.objdll.USBIO_GPIOWrite(self.id, low, WRITE_EN) #;sleep(self.delay)#disable for highest motor speed
            # assert status, 'Write pulse to LOW error!'
            self.objdll.USBIO_GPIOWrite(self.id, high, WRITE_EN) #; sleep(self.delay)#disable for highest motor speed
            # assert status, 'Write pulse to High error!'

            #if DEBUG1: self._print_pulse_count(pulse, i) #print pulse counts to console
            
        return self.stop()
    
    def swing(self, pulse, count=10): # 1st clockwise, then anti-clockwise
        pulse=int(pulse)
        low, high=self._lo_hi_preprocess(CLK_WISE) # ret in C byte format already
        # low=c_byte(low_temp); high=c_byte(high_temp) #convert to C byte data

        low_antiClk, high_antiClk=self._lo_hi_preprocess(ANTI_CLK_W) # ret in C byte format already
        # low_antiClk=low_temp | ANTI_CLK_W; high_antiClk=high_temp | ANTI_CLK_W #set direction bit1 =
        # low_antiClk=c_byte(low_antiClk); high_antiClk=c_byte(high_antiClk) #convert to C byte data

        for _ in range(count):
            for i in range(pulse): # 1st clockwise direction
                self.objdll.USBIO_GPIOWrite(self.id, low, WRITE_EN) #;sleep(self.delay)#disable for highest motor speed
                self.objdll.USBIO_GPIOWrite(self.id, high, WRITE_EN) #; sleep(self.delay)#disable for highest motor speed
                
            for i in range(pulse): # 2nd anticlockwise direction
                self.objdll.USBIO_GPIOWrite(self.id, low_antiClk, WRITE_EN) #;sleep(self.delay)#disable for highest motor speed
                self.objdll.USBIO_GPIOWrite(self.id, high_antiClk, WRITE_EN) #; sleep(self.delay)#disable for highest motor speed
        
        # return self.objdll.USBIO_GPIOWrite(self.id, c_byte(0b000), WRITE_EN) #set low before exist
        return
    
    def swingCapture(self, pulse, divider=2, count=5):
        """
        Swing the motor and capture what the camera see before and after 
            the clockwise and the anti-clockwise drives

        Parameters
        ----------
        pulse : No of pulse to dirve the motor
        div   : divide pulse into finer sub-group steps
        count : cycle counts, default is 5.

        Returns
        -------
        Captured image frames as numpy array

        """
        pulse=int(pulse) #make sure is integer
        cap = cv2.VideoCapture(CAM0, cv2.CAP_DSHOW) # use camera to monitor the motor-mirror assemnbly        
        frames=[]  

        low, high=self._lo_hi_preprocess(CLK_WISE)
        low_antiClk, high_antiClk=self._lo_hi_preprocess(ANTI_CLK_W) # ret in C byte format already
        div=divider #divide pulses into groups
        for _ in range(count): #loop count
            sub_pulse=pulse//div
            for __ in range(div): #no of times to cap image
                for ___ in range(sub_pulse): # 1st clockwise direction
                    self.objdll.USBIO_GPIOWrite(self.id, low, WRITE_EN) #;sleep(self.delay)#disable for highest motor speed
                    self.objdll.USBIO_GPIOWrite(self.id, high, WRITE_EN) #; sleep(self.delay)#disable for highest motor speed
                ret, frame = cap.read() # Capture frame-by-frame  
                frames.append(frame) # store per group

            for __ in range(div):                
                for ___ in range(sub_pulse): # 1st anticlockwise direction
                    self.objdll.USBIO_GPIOWrite(self.id, low_antiClk, WRITE_EN) #;sleep(self.delay)#disable for highest motor speed
                    self.objdll.USBIO_GPIOWrite(self.id, high_antiClk, WRITE_EN) #; sleep(self.delay)#disable for highest motor speed
                ret, frame = cap.read() # Capture frame-by-frame                
                frames.append(frame) # store per group


    
        cap.release()
        cv2.destroyAllWindows()
        return np.asarray(frames)
    

    def _print_pulse_count(self, pulse, i):
        sample=30 if pulse//200 < 30 else  pulse//200 #disable for highest motor speed
        if i%sample==0:
            print('Pulse count at:', i)

    def test(self):
        """
        Run clockwise, anticlockwise and swing few times

        Returns
        -------
        True if test done and motor stopped normally.

        """
        winsound.PlaySound('SystemExclamation', winsound.SND_ALIAS)
        
        pulses=1000*3
        winsound.Beep(200, 1000) # .Beep(1650Hz, (XXXXms)) #e.g 1000ms=1second
        self.run(pulses); self.run(pulses, ANTI_CLK_W)
        sleep(1)

        winsound.Beep(400, 1000)
        self.swing(128, count=30); self.stop() #0.9 degrees
        sleep(1)

        winsound.Beep(800, 1000)
        print('Testing I.....')
        self.swing(32, count=120); self.stop() #0.225 degrees     
        sleep(1)

        winsound.Beep(1600, 1000)
        print('Testing II.....')
        self.swing(2, count=1800); self.stop() #0.05625 degrees
        
        winsound.PlaySound('SystemExclamation', winsound.SND_ALIAS)
        print('                Testings Done! ')
        return self.stop() #set low before exist   

    def stop(self):
        """
        bit2, ENAble=0. Motor disabled without any holding force and driving power.
        
        bit1, DIRection=0. bit0, Pluse=0 .
        Returns
        -------
        True if stopped normally

        """
        status=self.objdll.USBIO_GPIOWrite(self.id, c_byte(0b000), c_byte(0))  #ENA=0, DIR=0, bit0=0
        print(f"Set all ports to LOW and stopped the step-motor:{status}")

        return status

    def close(self):
        """
        Stop and then close the USB device

        Returns
        -------
        None.

        """
        self.stop()  #ENA=0, DIR=0, bit0=0
        status=self.objdll.USBIO_CloseDevice(self.id)
        print(f"Close USB-GPIO Device:{status}")
   
def cameraOn():
    """
    Turn-on the default camera, CAM0

    Returns
    -------
    None.

    """
    cap = cv2.VideoCapture(CAM0, cv2.CAP_DSHOW) # use camera to monitor the motor-mirror assemnbly by DirectShow
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Display the resulting frame
        cv2.imshow("                     Real-Time Video. Press 'q' to exist.",frame)
        if cv2.waitKey(8) & 0xFF == ord('q'): #display a frame for 8ms, ~120Hz
            break
    
    cap.release() # release the capture
    cv2.destroyAllWindows()

def show_frames(frame_buffer, m_sec=1):
    """
    Display image-pixel arrays as video with, m_sec, mini-seconds waitkey between them
    Parameters
    ----------
    frame_buffer : numpy array
        Image pixels arrays
    m_sec : mini-seconds 
        The default is 1.

    Returns
    -------
    None.

    """
    m_sec=int(m_sec)
    print('Shape of the frame buffer:', frame_buffer.shape)

    logic=True
    while logic:
        for frame in frame_buffer:
            cv2.imshow(f"                      Replay frames at {m_sec} ms, Press 'q' to exist.", frame)
            if cv2.waitKey(m_sec) & 0xFF == ord('q'): #display a frame for m_sec mSeconds
                logic=False # stop outer-loop
                break # break inner-loop
        
    cv2.destroyAllWindows()

def save_as_npy(filename: str, frames):
    assert filename.split('.')[1]=='npy', 'Error, file extension is not npy'
    with open(filename, 'wb') as f:
        np.save(f, frames)
        print('Save frames into:', filename, ' successfully.')

def load_from_npy(filename: str):
    assert filename.split('.')[1]=='npy', 'Error, file extension is not npy'
    with open(filename, 'rb') as f:
        temp=np.load(f) #return the numpy array
        print(f'File load from {filename} successfully.')

    return temp

def user_input():
    """
    Pop-up window to get user inputs

    Returns
    -------
    values : TYPE
        DESCRIPTION.

    """
    # Define the window's contents
    layout = [  [sg.Text("Angle to rotate, in degree:")],     # Part 2 - The Layout
                [sg.Input()],
                [sg.Button('Ok')] ]  
    
    window = sg.Window('Input Window', layout)      # Create the window   
    # Display and interact with the Window
    event, values = window.read()                   # Part 4 - Event loop or Window.read call
    window.close()
    return values

def user_output(angles, pulses):
    """
    Pop-up window to display angle and necessary pulses to drive 
        the motor to move the angles.

    Parameters
    ----------
    angles : TYPE
        DESCRIPTION.
    pulses : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    """
    sg.theme('DarkBrown1')    
    layout = [  [sg.Text('Target to Go', size=(20, 2), justification='center')],
                [sg.Text(size=(10, 2), font=('Helvetica', 12), justification='center', key='-OUTPUT-')],
                # [sg.T(' ' * 2), sg.Quit()]]
                [sg.T(' ' * 8), sg.Button('Go !!', focus=True)]]    
    window = sg.Window('Output Window', layout)     
    
    while True:                                 # Event Loop
        event, values = window.read(timeout=10) # Please try and use as high of a timeout value as you can
        window['-OUTPUT-'].update(f'Angle:{angles} , Pulse:{pulses}')
        if event == 'Go !!' or event == sg.WIN_CLOSED:             # if user closed the window using X or clicked Quit button
            break
    window.close()

if __name__ == '__main__': # do not run if this code is imported as lib instead run as main()
    ### MAIN ###
    motor=GpioStepper(step_angle=0.9, p_divider=128) #creat step motor obj instance
    motor.test() #Run clockwise, anticlockwise and swings


    motor.close() # stop the motor and close the USB Device



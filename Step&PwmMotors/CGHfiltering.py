# -*- coding: utf-8 -*-
"""
Created on Mon Nov 23 15:09:07 2020

ver 1.0 - 1Dec2020

@author: 3Ddemo
"""
# import numpy as np
import stepper as sp

motor=sp.GpioStepper(step_angle=0.9, p_divider=128) #creat step motor obj instance
print('Enlarge the pattern, turn the mirror to see the pattern.'); sp.cameraOn()

# motor.test() #Run clockwise, anticlockwise and swing 4 times
frames= motor.swingCapture(pulse=1024, divider=4, count=10); motor.close()
sp.show_frames(frames, m_sec=1000) # 1ms between two frames

filename='video.npy'
sp.save_as_npy(filename, frames) #save into numpy formated data file.
ld_frames=sp.load_from_npy(filename) #load from numpy formated data file.

print('Play the loaded video from:', filename)
sp.show_frames(ld_frames, m_sec=1)



""" Ref:
W1 is the original image, Num_recon and Opt_recon are the numerical, and optical reconstructed images, respectively. The optical reconstructed image is slightly tilted due to camera misalignment. You can use Photoshop to rotate it.

    """

#! /usr/bin/env python
from Tkinter import *
import tkMessageBox
import os
import subprocess

appRoot="./"

processes = []

class GUIFramework(Frame):
	
	def __init__(self,master=None):
		Frame.__init__(self,master)
		
		self.master.title("IHA GUI")
		
		"""Display the main window"
		with a little bit of padding"""
		self.grid(padx=10,pady=10)
		self.CreateWidgets()
	   
	def CreateWidgets(self):

		""" callback shim - to enable button callbacks that take arguments """
		class SimpleCallback:
			def __init__(self, callback, *args):
				self.__callback = callback
				self.__args = args

			def __call__(self, *args):
				return self.__callback(*(self.__args + args ) )


		def RunScript(scriptName, arg):
			print "Running script "+scriptName +" "+ arg
			processes.append( subprocess.Popen([appRoot + scriptName, arg]) )
			
		def checkbuttonCB(cbnum):
			print "CB ", cbnum, "state: ", self.checkstates[cbnum].get()

		self.scripts = ["facedetector.sh", "viewers.sh", "cameras.sh", "portaudio.sh", "soundserver.sh", "dynamics.sh"]
		self.labelNames = ["Face Detector", "Viewers", "Cameras", "Portaudio Grabber", "Sound Sensor", "Dynamics"]
		self.labels = []
		self.startButtons = []
		self.hasStart = [True, True, True, True, True, True]
		self.stopButtons = []
		self.hasStop = [True, True, True, True, True, True]
		self.connectButtons = []
		self.hasConnect = [True, True, False, True, True, True]
		self.checkboxes = []
		self.checkstates = []

		buttonNumber=0
		for script in self.scripts:
			callback = SimpleCallback(checkbuttonCB, buttonNumber)
			self.checkstates.append( BooleanVar() )
			c = Checkbutton(self,text="", onvalue=True, offvalue=False, variable=self.checkstates[buttonNumber], command=callback ) 
			c.var=self.checkstates[buttonNumber]
			c.grid(row=buttonNumber,column=0)
			#if self.checkstates[buttonNumber]:
			#	self.checkboxes[buttonNumber].select()
			self.checkboxes.append( c )
				
			if self.hasStart[buttonNumber]:
				callback = SimpleCallback(RunScript, script, "start")
				self.startButtons.append( Button(self, text="Start", command=callback) )
				self.startButtons[buttonNumber].grid(row=buttonNumber, column=1)
			else:
				self.startButtons.append("none")
			
			if self.hasStop[buttonNumber]:
				callback = SimpleCallback(RunScript, script, "stop")
				self.stopButtons.append( Button(self, text="Stop", command=callback) )
				self.stopButtons[buttonNumber].grid(row=buttonNumber, column=2)
			else:
				self.stopButtons.append("none")
			
			if self.hasConnect[buttonNumber]:
				callback = SimpleCallback(RunScript, script, "connect")
				self.connectButtons.append( Button(self, text="Connect", command=callback) )
				self.connectButtons[buttonNumber].grid(row=buttonNumber, column=3)
			else:
				self.connectButtons.append("none")

			self.labels.append( Label(self, text=self.labelNames[buttonNumber]) )
			self.labels[buttonNumber].grid(row=buttonNumber, column=4)

			buttonNumber=buttonNumber+1


if __name__ == "__main__":
	guiFrame = GUIFramework()
	guiFrame.mainloop()


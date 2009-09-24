#!/usr/bin/python

##Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
##Author Lorenzo Natale
##email:   <lorenzo.natale>@robotcub.org
##website: www.robotcub.org
##Permission is granted to copy, distribute, and/or modify this program
##under the terms of the GNU General Public License, version 2 or any
##later version published by the Free Software Foundation.
##
##A copy of the license can be found at
##http://www.robotcub.org/icub/license/gpl.txt
##
##This program is distributed in the hope that it will be useful, but
##WITHOUT ANY WARRANTY; without even the implied warranty of
##MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
##Public License for more details

import fnmatch
import os
import sys
import subprocess
from Tkinter import *

def isXML(x):
    return fnmatch.fnmatch(x, '*.xml')

def isPy(x):
    return fnmatch.fnmatch(x, '*.py')

def isManagerPy(x):
    return fnmatch.fnmatch(x, 'manager.py')

class appEntry:
    path=''
    context=''
    xmls=[]

class App:
    def __init__(self, master):
        frame = Frame(master)
        frame.pack()
        self.master=frame

    def setManager(self, manager):
        self.manager=manager
        
    def setAppList(self, appList):
        self.appList=appList;

        tmpFrame=Frame(self.master, relief=SUNKEN)

        tmpFrame.columnconfigure(0, minsize=150)
        tmpFrame.columnconfigure(1, minsize=150)
        tmpFrame.columnconfigure(2, minsize=40)
             
        Label(tmpFrame, text="Context:").grid(row=0,column=0, sticky=N+W)
        Label(tmpFrame, text="Detected xmls:").grid(row=0,column=1, sticky=N+W)

        r=1
        for app in appList:
             tmpFrame.grid(row=r, column=0, sticky=W)
             tmpApp=Label(tmpFrame, text=app.context, relief=SUNKEN).grid(row=r, column=0, sticky=N+W+E)
##             tmpFrame.columnconfigure(0, minsize=150)
##             tmpFrame.columnconfigure(1, minsize=150)
##             tmpFrame.columnconfigure(2, minsize=40)
##        
             rr=r
             #tmpFrame=Frame(tmpFrame)
             #tmpFrame.grid(row=1, column=1, sticky=N)
             for xml in app.xmls:
                 tmpXml=Label(tmpFrame,text=xml, relief=SUNKEN)
                 tmpXml.grid(row=r, column=1, sticky=N+W)
                 tmp=Button(tmpFrame, text="Run Manager", command=lambda i=os.path.join(app.path, xml):self.runManager(i))
                 tmp.grid(row=r, column=2, sticky=N)
                 r=r+1
             r=r+1

                      
    def runManager(self, fullPathToXml):
        print "running",
        print fullPathToXml
        
        cmd=['python', self.manager, fullPathToXml]
        #print cmd
        ret=subprocess.Popen(cmd);
        
def searchManager():
    contexts=os.listdir('.')
    for c in contexts:
        if (c=='default'):
            tmp=os.listdir(c)
            if 'scripts' in tmp:
                files=os.listdir(os.path.join(c, 'scripts'))
                for f in files:
                    if isManagerPy(f):
                        ret=os.path.join(c, 'scripts', f)
                        print ret
                        return ret
    
def searchApplications():
    contexts=os.listdir('.')

    applications=[]

    for c in contexts:
        if (os.path.isdir(c)):
            #print c
            tmp=os.listdir(c)
            if 'scripts' in tmp:
                #print c
                tmpApp=appEntry()
                files=os.listdir(os.path.join(c,'scripts'))
                tmpApp.path=os.path.join(c,'scripts')
                tmpApp.context=c
                tmpApp.xmls=filter(isXML, files)

                #print tmpApp.xmls
                if (tmpApp.xmls!=[]):
                    applications.append(tmpApp)

    return applications

if __name__ == '__main__':
    root = Tk()
    app = App(root)
    root.title("iCub application manager")

    applications=searchApplications()
    managerPath=searchManager()

    app.setManager(managerPath)
    app.setAppList(applications)
    
    root.mainloop()


  

                    

            
        

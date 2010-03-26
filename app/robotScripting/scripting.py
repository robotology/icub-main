#!/usr/bin/python

## Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
## Author Lorenzo Natale
## email:   <lorenzo.natale>@robotcub.org
## website: www.robotcub.org
## Permission is granted to copy, distribute, and/or modify this program
## under the terms of the GNU General Public License, version 2 or any
## later version published by the Free Software Foundation.
##
## A copy of the license can be found at
## http://www.robotcub.org/icub/license/gpl.txt
##
## This program is distributed in the hope that it will be useful, but
## WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
## Public License for more details

import fnmatch
import os
import sys
import subprocess
from Tkinter import *

def fileExists(f):
    try:
        file=open(f)
    except IOError:
        return 0
    else:
        return 1

def printUsage(scriptName):
    print "scripting.py: a python gui for scripting"
    print "Usage:"
    print scriptName, 
    print "dir"
     
class ApplicationDescriptor:
    title=''
    applications=[]

def pruneContexts(appDirs, allContexts):
    ret=[]
    for ap in appDirs:
        for c in allContexts:
            #print ap,
            #print c
            if (c==ap):
                ret.append(c)
    return ret
    
def isText(x):
    return fnmatch.fnmatch(x, '*.txt')

class appEntry:
    path=''
    context=''
    xmls=[]

class App:
    def __init__(self, master):
        frame = Frame(master)
        frame.pack()
        self.master=frame      

    def setScriptsList(self, scripts, directory):
        self.directory=directory
        self.scripts=scripts
        tmpFrame=Frame(self.master, relief=SUNKEN)

        tmpFrame.columnconfigure(0, minsize=150)
        tmpFrame.columnconfigure(1, minsize=150)

        Label(tmpFrame, text="Scripts:").grid(row=0,column=0, sticky=N+W)

        r=0
        c=1

        for sc in scripts:
            tmpFrame.grid(row=r, column=c, sticky=W)
            tmpApp=Label(tmpFrame, text=sc, relief=SUNKEN).grid(row=r, column=0, sticky=N+W+E)

            tmp=Button(tmpFrame, text="Run", command=lambda i=sc:self.runScript(i))
            tmp.grid(row=r, column=2, sticky=N)
            #tmp=Button(tmpFrame, text="Edit", command=lambda i=sc:self.editScript(i))
            #tmp.grid(row=r, column=3, sticky=N)
            #r=r+1

            r=r+1

    def runScript(self, script):
        f=open(os.path.join(directory, script),'r')
        for line in f:
            line=line.rstrip('\r\n')
            print line
            #ret=subprocess.Popen('cmd -c', line);
            os.system(line)
        f.close()

    def editScript(self, script):
        w=EditWindow(self.master, directory, script)

def searchScripts(scriptDir):
    all=os.listdir(scriptDir)
   
    ret=[]

    for c in all:
        if (not os.path.isdir(c)):
            ret.append(c)
    return ret

class EditWindow:
    def __init__(self, master, directory, script):
        frame=Toplevel()
        self.directory=directory
        self.script=script
        self.master=frame
    
        Label(frame, text=script).grid(row=0, column=0, sticky=W)
        bottom=Frame(frame).grid(row=1, column=0)
        f=open(os.path.join(directory, script),'r')
 
        self.lines=[]
        r=1
        for line in f:
            line=line.rstrip('\r\n')
            tmp=Entry(bottom)
            if (not line==[]):
                print line
                tmp.insert(END, 'ciao')
                self.lines.append(tmp)
                tmp.grid(row=r,column=0)
                r=r+1
        f.close()

        tmp=Button(bottom, text="Save", command=self.save())
        tmp.grid(row=r, column=0, sticky=N)

        tmp=Button(bottom, text="Add", command=self.addLine())
        tmp.grid(row=r, column=1, sticky=N)

        tmp=Button(bottom, text="Remove", command=self.removeLine())
        tmp.grid(row=r, column=2, sticky=N)

        #finally place window close to top left corner
        #frame.update_idletasks() #update current geometry
        width=frame.winfo_width()
        height=frame.winfo_height()
        rootx=master.winfo_rootx()
        rooty=master.winfo_rooty()
        #frame.geometry('%dx%d+%d+%d' %(width, height, rootx, rooty))
 
    def update(self):
        r=0
        for l in self.lines:
            tmp=Entry(self.master)
            tmp.insert(END, l)
            tmp.grid(row=r,column=0)
            r=r+1
            
    def addLine(self):
        tmp=Entry(self.master)
        self.lines.append(tmp)
        self.update()
        
    def removeLine(self):
        self.lines.pop()
        self.update()

    def save(self):
        f=open(os.path.join(directory, 'temp.txt'),'w')
   
        for l in self.lines:
            tmp=l.get()
            f.write(tmp)
        f.close()
            
if __name__ == '__main__':

    argc = len(sys.argv)

    if (argc==2):
        directory=sys.argv[1]
    else:
        printUsage(sys.argv[0])
        sys.exit(1)

    scripts=searchScripts(directory)
    print scripts
  
    root = Tk()
    app = App(root)
    app.setScriptsList(scripts, directory)

    root.mainloop()

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

# Recursively descent the working directory tree and detects all files
# with xml extension. Allows running an instance of the manager for each
# of them.
#
# Parameters: a file that contains a list of directories to search.
# By default look for app.txt
#
# Ths syntax of the file is:
# [NAME]
# name
#
# [APPLICATIONS]
# dir1
# dir2
#
# use wildcard * or an empty list to mean everyting

# Feb 2010 added scrollbar support.

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
    print "icubapp.py: a python gui for parsing application directories"
    print "Usage:"
    print scriptName, 
    print "app.txt"
    print "app.txt: configuration file, contains a list of directories that will be parsed"
    print "see app.txt.template in repository" 
    
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
    
def parseConfigFile(filename):
    f=open(filename,'r')
    group=''
    ret= ApplicationDescriptor()
    for line in f:
        line=line.rstrip('\r\n')

        if (group=='name' and line!='' and line[0]!='[' and line[0]!='#'):
            ret.title=line
                
        if (group=='app' and line!='' and line[0]!='[' and line[0]!='#'):
            #protect against empty lines
            if (line!=''):
                if (line[0]!='['):
                    ret.applications.append(line)

        if (line=='[NAME]'):
            group='name'

        if (line=='[APPLICATIONS]'):
            group='app'
  
    f.close()
    
    return ret            


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
	frame.grid()
	self.master=frame      
        
    def setManager(self, manager):
	self.manager=manager

    def setAppList(self, appList):
	self.appList=appList

	tmpFrame=Frame(self.master, relief=SUNKEN)

	tmpFrame.columnconfigure(0, minsize=150)
	tmpFrame.columnconfigure(1, minsize=150)
	tmpFrame.columnconfigure(2, minsize=40)

	Label(tmpFrame, text="Context:").grid(row=0,column=0, sticky=N+W)
	Label(tmpFrame, text="Found xmls:").grid(row=0,column=1, sticky=N+W)

	r=1
	c=1

	for app in appList:
	    tmpFrame.grid(row=r, column=c, sticky=W)

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
	ret=subprocess.Popen(cmd);

def searchManager():
    # first look in PATH
    path=os.getenv("PATH")
    
    if os.name == 'posix':
       path_dirs=path.split(':')
    elif os.name == 'nt':
        path_dirs=path.split(';')
    
    for pdir in path_dirs:
        files=os.listdir(pdir)
        for f in files:
		    if isManagerPy(f):
			ret=os.path.join(pdir, f)
			return ret

    contexts=os.listdir('.')
    for c in contexts:
	if (c=='default'):
	    tmp=os.listdir(c)
	    if 'scripts' in tmp:
		files=os.listdir(os.path.join(c, 'scripts'))
		for f in files:
		    if isManagerPy(f):
			ret=os.path.join(c, 'scripts', f)
			return ret

def searchApplications(appDirs):
    allContexts=os.listdir('.')
    allContexts.sort()
    contexts=[]
   
    # handle special cases
    if (appDirs==[]):
        contexts=allContexts
    elif (appDirs[0]=='*' or appDirs[0]==''):
        contexts=allContexts
    else:
        contexts=pruneContexts(appDirs, allContexts)

    applications=[]

    for c in contexts:
	if (os.path.isdir(c)):
	    tmp=os.listdir(c)
	    if 'scripts' in tmp:
		tmpApp=appEntry()
		files=os.listdir(os.path.join(c,'scripts'))
		tmpApp.path=os.path.join(c,'scripts')
		tmpApp.context=c
		tmpApp.xmls=filter(isXML, files)

		if (tmpApp.xmls!=[]):
		    applications.append(tmpApp)

    return applications


# From http://effbot.org/zone/tkinter-autoscrollbar.htm
class AutoScrollbar(Scrollbar):
    # a scrollbar that hides itself if it's not needed.  only
    # works if you use the grid geometry manager.
    def set(self, lo, hi):
        if float(lo) <= 0.0 and float(hi) >= 1.0:
            # grid_remove is currently missing from Tkinter!
            self.tk.call("grid", "remove", self)
        else:
            self.grid()
        Scrollbar.set(self, lo, hi)
    def pack(self, **kw):
        raise TclError, "cannot use pack with this widget"
    def place(self, **kw):
        raise TclError, "cannot use place with this widget"
    
if __name__ == '__main__':

    argc = len(sys.argv)

    confFile='app.txt'
    if (argc==1):
        print 'Using default ',
        print confFile
    elif (argc==2):
        confFile=sys.argv[1]
        print 'Using ',
        print confFile
    else:
        printUsage(sys.argv[0])
        sys.exit(1)

    found=fileExists(confFile)
    if (not found):
        print 'Error: file ',
        print confFile,
        print 'not found in local directory'
        sys.exit(1)

    config=parseConfigFile(confFile)
        
    applications=searchApplications(config.applications)
    managerPath=searchManager()
    print 'Manager was found at:',
    print managerPath    

    root = Tk()

    # create scrolled canvas
    vscrollbar = AutoScrollbar(root)
    vscrollbar.grid(row=0, column=1, sticky=N+S)
    hscrollbar = AutoScrollbar(root, orient=HORIZONTAL)
    hscrollbar.grid(row=1, column=0, sticky=E+W)
    
    canvas = Canvas(root,
                    yscrollcommand=vscrollbar.set,
                    xscrollcommand=hscrollbar.set)
    canvas.grid(row=0, column=0, sticky=N+S+E+W)
    
    vscrollbar.config(command=canvas.yview)
    hscrollbar.config(command=canvas.xview)
    
    # make the canvas expandable
    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=1)
    
    # create canvas contents
    frame = Frame(canvas)
    frame.rowconfigure(1, weight=1)
    frame.columnconfigure(1, weight=1)
    ##################################

    app = App(frame)
    app.setManager(managerPath)
    app.setAppList(applications)

    ######## scrollbar
    canvas.create_window(0, 0, anchor=NW, window=frame)
    frame.update_idletasks()
    canvas.config(scrollregion=canvas.bbox("all"))
    # resize canvas
    frame.update()
    canvas['width'] = frame.winfo_width() + 10
    height = frame.winfo_height()
    if height > 600:
        height = 600
    canvas['height'] = height
    ################

    root.title(config.title)

    root.mainloop()

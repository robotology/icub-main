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
	frame.pack()
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
#	    print app.context
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

	    if (r==10):
		r=1
		c=c+1


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
#			print ret
			return ret

def searchApplications(appDirs):
    allContexts=os.listdir('.')
    contexts=[]
   
    # handle special cases
    if (appDirs==[]):
        contexts=allContexts
    elif (appDirs[0]=='*' or appDirs[0]==''):
        contexts=allContexts
    else:
        contexts=pruneContexts(appDirs, allContexts)

#   print contexts
    applications=[]

    for c in contexts:
	if (os.path.isdir(c)):
#	    print c
	    tmp=os.listdir(c)
	    if 'scripts' in tmp:
#               print c
		tmpApp=appEntry()
		files=os.listdir(os.path.join(c,'scripts'))
		tmpApp.path=os.path.join(c,'scripts')
		tmpApp.context=c
		tmpApp.xmls=filter(isXML, files)

#              print tmpApp.xmls
		if (tmpApp.xmls!=[]):
		    applications.append(tmpApp)

    return applications

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


    root = Tk()
    app = App(root)
    root.title(config.title)
    app.setManager(managerPath)
    app.setAppList(applications)

    root.mainloop()

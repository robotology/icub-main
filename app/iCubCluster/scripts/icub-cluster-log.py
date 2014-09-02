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

# No longer looking for $ICUB_ROBOTNAME env variable
# Modified to accept DISPLAY value in xlm
	
import xml.dom.minidom
import subprocess
import os
import time
from Tkinter import *


class Util:
    @staticmethod
    def getSshCmd(user, host):
        cmd = ['ssh', '-f']
        if user == "":
            return cmd + [host]
        else:
            return cmd + [user + '@' + host]


class Node:
    def __init__(self, name, display, dispValue, user):
        self.name=name
        self.display=display
        self.displayValue=dispValue;
        self.user=user;

class RemoteExecWindow:
    def __init__(self, master, user, nodes):
        frame=Toplevel()
        self.master=frame
        self.nodes=nodes
        self.nodesChk=[]

        tmpFrame=Frame(frame)
        tmpFrame.pack()
        Label(tmpFrame, text="Select nodes:").pack()

        tmpFrame=Frame(frame)
        tmpFrame.pack()
        r=0
        for node in nodes:
            tmp=Entry(tmpFrame)
            tmp.insert(END, node.name)
            r=r+1
            tmp.grid(row=r, column=1)
            v=IntVar()
            check=Checkbutton(tmpFrame, variable=v)
#           check.config(foreground="#00A000")
            self.nodesChk.append(v)
            check.grid(row=r, column=2)

        tmpFrame=Frame(frame)
        tmpFrame.pack()
        Label(tmpFrame, text="User:").grid(row=0, column=0)
        self.user=Entry(tmpFrame)
        self.user.grid(row=0, column=1)
        self.user.insert(END, user)

        tmpFrame=Frame(frame)
        tmpFrame.pack()
        Label(tmpFrame, text="Type the command you wish to execute:").grid()

        tmpFrame=Frame(frame)
        tmpFrame.pack()

        self.command=Entry(tmpFrame)
        self.command.grid(row=0, column=0)

        Button(tmpFrame, text="Execute", command=self.executeCmd).grid(row=0, column=1)

        #finally place window close to top left corner
        frame.update_idletasks() #update current geometry
        width=frame.winfo_width()
        height=frame.winfo_height()
        rootx=master.winfo_rootx()
        rooty=master.winfo_rooty()
        frame.geometry('%dx%d+%d+%d' %(width, height, rootx, rooty))

    def executeCmd(self):
        remoteCmd=self.command.get()
        print 'Executing:',
        print remoteCmd

        i=0
        for chk in self.nodesChk:
            if (chk.get()):
                node=self.nodes[i].name
                user=self.nodes[i].user
                cmd = Util.getSshCmd(user, node) + [remoteCmd]
                print " ".join(cmd)
                subprocess.Popen(cmd).wait()
            i=i+1

class Cluster:
    def __init__(self, name, user, namespace, nsNode, nsType, nodes):
        self.name=name
        self.user=user
        self.namespace=namespace
        self.nsNode=nsNode
        self.nodes=nodes
        self.nsType=nsType

    def display(self):
        print "--- Cluster %s:" %self.name
        print "User is: %s" % self.user
        print "Nameserver %s " %self.namespace,
        print "type: %s " %self.nsType,
        print "is on %s" % self.nsNode

        print "Nodes are:"
        for node in self.nodes:
            print node.name,
            if (node.display):
                print "local display on",
                print node.displayValue
            else:
                print " local display disabled"


        print "------------"

class App:
    def __init__(self, master):
        frame = Frame(master)
        frame.pack()
        self.master=frame

    def setCluster(self, cluster):
        self.cluster=cluster

        Label(self.master, text="Cluster Management Window").pack()

        userFrame=Frame(self.master, relief="ridge", bd=1)
        userFrame.pack(fill=X)
        tmpFrame=Frame(userFrame)
        tmpFrame.pack()
        Label(tmpFrame, text="User:").grid(row=0, sticky=W)
        self.clusterUser = Entry(tmpFrame)
        self.clusterUser.insert(END, cluster.user)
        self.clusterUser.grid(row=0, column=1, sticky=E)
        Label(tmpFrame, text="Namespace:").grid(row=1, column=0, sticky=W)
        self.clusterNs = Entry(tmpFrame)
        self.clusterNs.insert(END, cluster.namespace)
        self.clusterNs.grid(row=1, column=1, sticky=E)

        nsFrame=Frame(self.master, relief="ridge", bd=1)
        nsFrame.pack(fill=X)
        tmpFrame=Frame(nsFrame)
        tmpFrame.pack()
        Label(tmpFrame, text="Nameserver node:").grid(row=0, sticky=W)
        self.clusterNsNode = Entry(tmpFrame)
        self.clusterNsNode.insert(END, cluster.nsNode)
        self.clusterNsNode.grid(row=1, column=0, sticky=W)
        self.nsFlag=IntVar()
        self.bCheckNs=Checkbutton(tmpFrame, variable=self.nsFlag)
        self.bCheckNs.config(state=DISABLED,disabledforeground="#00A000")
        self.bCheckNs.grid(row=1, column=1, sticky=W)

        tmpFrame=Frame(nsFrame)
        tmpFrame.pack()
        buttonCheckNs=Button(tmpFrame, text="Check", width=8, command=self.checkNs)
        buttonRunNs=Button(tmpFrame, text="Run", width=8, command=self.runNs)
        buttonStopNs=Button(tmpFrame, text="Stop", width=8, command=self.stopNs)
        buttonCheckNs.grid(row=5, column=0)
        buttonRunNs.grid(row=5, column=1)
        buttonStopNs.grid(row=5, column=2)

        nodesFrame=Frame(self.master, relief="ridge", bd=1)
        nodesFrame.pack()
        tmpFrame=Frame(nodesFrame)
        tmpFrame.pack()
        Label(tmpFrame, text="Nodes:").grid(row=0, column=0)

        tmpFrame1=Frame(nodesFrame)
        tmpFrame1.pack()

        tmpFrame=Frame(tmpFrame1)

        Label(tmpFrame, text="Name").grid(row=0, column=1)
        Label(tmpFrame, text="Display").grid(row=0, column=2)
        Label(tmpFrame, text="User").grid(row=0, column=3)
        Label(tmpFrame, text="On/Off").grid(row=0, column=4)
        Label(tmpFrame, text="Log").grid(row=0, column=5)
        Label(tmpFrame, text="Select").grid(row=0, column=6)
        

        tmpFrame.grid(row=1, column=0)
        self.clusterNodes=[]
        self.values=[]
        self.dispFlag=[]
        self.selected=[]
        self.logged=[]

        r=1
        for node in cluster.nodes:
            tmp=Entry(tmpFrame)
            tmp.insert(END, node.name)
            r=r+1
            tmp.grid(row=r, column=1)
            self.clusterNodes.append(tmp)

            #v=IntVar()

            v=StringVar()
            if node.display:
                v.set(node.displayValue)
                self.dispFlag.append(1)
            else:
                v.set("none")
                self.dispFlag.append(0)

            check=Entry(tmpFrame, textvariable=v, width=8, justify="center")
            #Checkbutton(tmpFrame, variable=v)
            check.config(state=DISABLED, disabledforeground="#000000")
            check.grid(row=r, column=2)

            u=StringVar()
            if node.user != cluster.user:
                u.set(node.user)
                self.dispFlag.append(1)
            else:
                u.set("")
                self.dispFlag.append(0)

            check=Entry(tmpFrame, textvariable=u, width=8, justify="center")
            check.config(state=DISABLED, disabledforeground="#000000")
            check.grid(row=r, column=3)

            v=IntVar()
            check=Checkbutton(tmpFrame, variable=v)
            check.config(state=DISABLED,disabledforeground="#00A000")
            self.values.append(v)

            check.grid(row=r, column=4)

            v=IntVar()
            v.set(0)

            check=Checkbutton(tmpFrame, variable=v)
#           check.config(state=ENABLED, disabledforeground="#00A000")
            self.logged.append(v)
            check.grid(row=r, column=5)

            v=IntVar()
            v.set(1)

            check=Checkbutton(tmpFrame, variable=v)
#           check.config(state=ENABLED, disabledforeground="#00A000")
            self.selected.append(v)
            check.grid(row=r, column=6)



        tmpFrame=Frame(tmpFrame1)
        tmpFrame.grid(row=1, column=1)
        buttonCheckNodes=Button(tmpFrame, text="Check All", width=10, command=self.checkNodes)
        buttonRunNodes=Button(tmpFrame, text="Run Selected", width=10, command=self.runNodes)
        buttonStopNodes=Button(tmpFrame, text="Stop Selected", width=10, command=self.stopNodes)
        buttonKillNodes=Button(tmpFrame, text="Kill Selected", width=10, command=self.killNodes)
        buttonCheckNodes.grid(row=1, column=0)
        buttonRunNodes.grid(row=2, column=0)
        buttonStopNodes.grid(row=3, column=0)
        buttonKillNodes.grid(row=4, column=0)

        tmpFrame=Frame(self.master)
        tmpFrame.pack()
        buttonExec=Button(tmpFrame, text="More...", width=10, height=1, command=self.executeWnd)
        buttonExec.grid(row=0, column=0, columnspan=4)

    def executeWnd(self):
        a=RemoteExecWindow(self.master, self.cluster.user, self.cluster.nodes)

    def checkNodes(self):
        print 'Checking nodes'
        i=iter(self.values)

        for node in self.cluster.nodes:
            cmd=['yarp', 'exists', '/'+node.name]
            print 'Running',
            print " ".join(cmd)
            ret=subprocess.Popen(cmd).wait()
            if ret==0:
                print 'setting',
                print node.name
                i.next().set(1)
            else:
                i.next().set(0)

    def runNodes(self):
        print 'Starting  nodes'
        self.checkNodes()
        i=iter(self.values)
        iS=iter(self.selected)
        iSS=iter(self.logged)
        for node in self.cluster.nodes:
            selected=iS.next().get()
            running=i.next().get()
            logged=iSS.next().get()

            if running==0 and selected==1:
                #cmd=['ssh', '-f', node.user+'@'+node.name, 'icub-cluster-run.sh', ' start ']
                cmd = Util.getSshCmd(node.user, node.name) + ['icub-cluster-run.sh', ' start ']

                if node.display:
                    if (node.displayValue == ""):
                        cmd.append('display')
                    else:
                        cmd.append('display ')
                        cmd.append(node.displayValue)

                if logged==1:
                    cmd.append('log')
                #else:
                    #cmd = Util.getSshCmd(node.user, node.name) + ['icub-cluster-run.sh', ' start ']
#                   cmd=['ssh', '-f', node.user+'@'+node.name, 'icub-cluster-run.sh', ' start ']

                print 'Running',
                print " ".join(cmd)
                ret=subprocess.Popen(cmd).wait()
            else:
                print node.name,
                print ' already running skipping'

        time.sleep(0.5)
        self.checkNodes()

    def stopNodes(self):
        print 'Stopping  nodes'
        self.checkNodes()

        i=iter(self.values)
        iS=iter(self.selected)
        for node in self.cluster.nodes:
            selected=iS.next().get()
            running=i.next().get()

            if running==1 and selected==1 :
                #cmd=['ssh', '-f', node.user+'@'+node.name, 'icub-cluster-run.sh' ' stop']
                cmd = Util.getSshCmd(node.user, node.name) + ['icub-cluster-run.sh', ' stop']
                print 'Running',
                print " ".join(cmd)
                ret=subprocess.Popen(cmd).wait()
            else:
                print node.name,
                print ' not running skipping'

        time.sleep(0.5)
        self.checkNodes()

    def killNodes(self):
        print 'Killing  nodes'
        iS=iter(self.selected)
        for node in self.cluster.nodes:
            selected=iS.next().get()
            if (selected):
                #cmd=['ssh', '-f', node.user+'@'+node.name, 'icub-cluster-run.sh' ' kill']
                cmd = Util.getSshCmd(node.user, node.name) + ['icub-cluster-run.sh', ' kill']
                print " ".join(cmd)
                ret=subprocess.Popen(cmd).wait()

        time.sleep(0.5)
        self.checkNodes()

    def checkNs(self):
        print 'Checking ns'
        cmd=['yarp', 'exists', self.cluster.namespace]
        print 'Running',
        print " ".join(cmd)
        ret=subprocess.Popen(cmd).wait()
        if ret==0:
            self.nsFlag.set(1)
        else:
            self.nsFlag.set(0)

    def runNs(self):
        print 'Running nameserver'
        self.checkNs()
        if self.nsFlag.get()==0:
            #cmd=['ssh', '-f', self.cluster.user+'@'+self.cluster.nsNode, 'icub-cluster-server.sh' ' start']
            cmd = Util.getSshCmd(self.cluster.user, self.cluster.nsNode) + ['icub-cluster-server.sh', ' start']
            if (self.cluster.nsType=='yarpserver3'):
                cmd.append('yarpserver3')
            elif(self.cluster.nsType=='yarpserver'):
                 cmd.append('yarpserver')

            print 'Running',
            print " ".join(cmd)
            ret=subprocess.Popen(cmd).wait()
            time.sleep(0.5)
            self.checkNs()
        else:
            print 'Nameserver already running'

    def stopNs(self):
        print 'Stopping nameserver'
        self.checkNs()
        if self.nsFlag.get()==1:
            #cmd=['ssh', '-f', self.cluster.user+'@'+self.cluster.nsNode, 'icub-cluster-server.sh' ' stop']
            cmd = Util.getSshCmd(self.cluster.user, self.cluster.nsNode) + ['icub-cluster-server.sh', ' stop']
            print 'Running',
            print " ".join(cmd)
            ret=subprocess.Popen(cmd).wait()
            time.sleep(0.5)
            self.checkNs()
        else:
            print 'Nameserver was not running'

def check_output(*popenargs, **kwargs):
    process = subprocess.Popen(stdout=subprocess.PIPE, *popenargs, **kwargs)
    output, unused_err = process.communicate()
    retcode = process.poll()
    if retcode:
        cmd = kwargs.get("args")
        if cmd is None:
            cmd = popenargs[0]
        error = subprocess.CalledProcessError(retcode, cmd)
        error.output = output
        raise error
    return output

def printUsage(scriptName):
    print scriptName, ": python gui for managing yarprun servers (Linux only)"
    print "Usage:"
    print scriptName, 
    print "[xml_configuration_file] [context]\n"
    print "  xml_configuration_file: cluster configuration file (default: cluster-config.xml)."
    print "  context: context where xml configuration file is sought through yarp's ResourceFinder (default: iCubCluster)."
    print "  To learn how to write a valid cluster-config.xml see example in app/iCubCluster/conf"


if __name__ == '__main__':

    #first check arguments
    argc = len(sys.argv)

    if (argc>2):
         contextName=sys.argv[2]
    else:    
         contextName="iCubCluster"
    if (argc>1):
         configFileName=sys.argv[1]
    else:    
         configFileName="cluster-config.xml"

    if (configFileName == "help"):
        printUsage("icub-cluster.py")
        sys.exit(1)

    print "Config file name: " + configFileName + ", context name: " + contextName

    if(sys.hexversion < 0x02070000):
        configFilePath=check_output(["yarp",  "resource",  "--find", configFileName, "--context", contextName])
    else:
        configFilePath=subprocess.check_output(["yarp",  "resource",  "--find", configFileName, "--context", contextName])
    if not configFilePath[1:-2] :  # "slicing" output variable to remove quotes and EOL
       print "Could not find file " + configFileName + " in context " + contextName + ", exiting."
       sys.exit(1)

    config = xml.dom.minidom.parse(configFilePath[1:-2])

    clusters=config.getElementsByTagName("cluster")

    for cluster in clusters:
        user=cluster.getAttribute("user")
        name=cluster.getAttribute("name")
        nameserver=cluster.getElementsByTagName("nameserver")[0]
        namespace=nameserver.getAttribute("namespace")
        namespaceNode=nameserver.getAttribute("node")
        namespaceType=nameserver.getAttribute("type")
        nodes=cluster.getElementsByTagName("node")

        nodeList=[]
        for node in nodes:

            if (node.hasAttributes()):
                if node.hasAttribute("user"):
                    nodeuser=node.getAttribute("user");
                else:
                    nodeuser=user;
                a=node.getAttribute("display")
                # handle default values for variable (for backward compatibility)
                if ( a=="true" or a=="True" or a=="TRUE" or a=="yes" or a=="YES"): 
                    newNode=Node(node.firstChild.data, True, ":0.0", nodeuser)
                else:
                    newNode=Node(node.firstChild.data, True, a, nodeuser)
            else:
                newNode=Node(node.firstChild.data, False, "", user)

            nodeList.append(newNode)

    cl=Cluster(name, user, namespace, namespaceNode, namespaceType, nodeList)
    cl.display()

    root = Tk()
    app = App(root)
    root.title("Cluster manager for:"+cl.name)

    app.setCluster(cl)
#   dont check when start, nameserver could be off
#   app.checkNs()
#   app.checkNodes()
    root.mainloop()

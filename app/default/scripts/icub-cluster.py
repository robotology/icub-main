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

#!/usr/bin/python

import xml.dom.minidom
import subprocess
import os
import time
from Tkinter import *

class Node:
    def __init__(self, name, display):
        self.name=name
        self.display=display
        
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
                cmd=['ssh', '-f', self.user.get()+'@'+node, remoteCmd]
                print cmd
                subprocess.Popen(cmd).wait()
            i=i+1

class Cluster:
    def __init__(self, name, user, namespace, nsNode, nodes):
        self.name=name
        self.user=user
        self.namespace=namespace
        self.nsNode=nsNode
        self.nodes=nodes

    def display(self):
        print "--- Cluster %s:" %self.name
        print "User is: %s" % self.user
        print "Nameserver %s " %self.namespace,
        print "is on %s" % self.nsNode

        print "Nodes are:"
        for node in self.nodes:
            if (node.display):
                print node.name,
                print " (enable local display)"
            else:
                print node.name
    
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
        Label(tmpFrame, text="On/Off").grid(row=0, column=2)
        Label(tmpFrame, text="Desktop").grid(row=0, column=3)
        Label(tmpFrame, text="Select").grid(row=0, column=4)
        
        tmpFrame.grid(row=1, column=0)
        self.clusterNodes=[]
        self.values=[]
        self.dispFlag=[]
        self.selected=[]
        
        r=1
        for node in cluster.nodes:
            tmp=Entry(tmpFrame)
            tmp.insert(END, node.name)
            r=r+1
            tmp.grid(row=r, column=1)
            self.clusterNodes.append(tmp)
            v=IntVar()
            check=Checkbutton(tmpFrame, variable=v)
            check.config(state=DISABLED,disabledforeground="#00A000")
            self.values.append(v)
            check.grid(row=r, column=2)

            v=IntVar()

            if node.display:
                v.set(1)
            else:
                v.set(0)
                
            check=Checkbutton(tmpFrame, variable=v)
            check.config(state=DISABLED, disabledforeground="#00A000")
            self.dispFlag.append(v)
            check.grid(row=r, column=3)

            v=IntVar()
            v.set(1)
                
            check=Checkbutton(tmpFrame, variable=v)
#           check.config(state=ENABLED, disabledforeground="#00A000")
            self.selected.append(v)
            check.grid(row=r, column=4)

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
            print cmd
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
        for node in self.cluster.nodes:
            selected=iS.next().get()
            running=i.next().get()
            
            if running==0 and selected==1:
                if node.display:
                    cmd=['ssh', '-f', self.cluster.user+'@'+node.name, '$ICUB_ROOT/scripts/yarprun.sh', ' start ', 'display']
                else:
                    cmd=['ssh', '-f', self.cluster.user+'@'+node.name, '$ICUB_ROOT/scripts/yarprun.sh', ' start ']
                    
                print 'Running',
                print cmd
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
                cmd=['ssh', '-f', self.cluster.user+'@'+node.name, '$ICUB_ROOT/scripts/yarprun.sh' ' stop']
                print 'Running',
                print cmd
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
                cmd=['ssh', '-f', self.cluster.user+'@'+node.name, '$ICUB_ROOT/scripts/yarprun.sh' ' kill']
                print cmd
                ret=subprocess.Popen(cmd).wait()

        time.sleep(0.5)
        self.checkNodes()

    def checkNs(self):
        print 'Checking ns'
        cmd=['yarp', 'exists', self.cluster.namespace]
        print 'Running',
        print cmd
        ret=subprocess.Popen(cmd).wait()
        if ret==0:
            self.nsFlag.set(1)
        else:
            self.nsFlag.set(0)

    def runNs(self):
        print 'Running nameserver'
        self.checkNs()
        if self.nsFlag.get()==0:
            cmd=['ssh', '-f', self.cluster.user+'@'+self.cluster.nsNode, '$ICUB_ROOT/scripts/yarpserver.sh' ' start']
            print 'Running',
            print cmd
            ret=subprocess.Popen(cmd).wait()
            time.sleep(0.5)
            self.checkNs()
        else:
            print 'Nameserver already running'

    def stopNs(self):
        print 'Stopping nameserver'
        self.checkNs()
        if self.nsFlag.get()==1:
            cmd=['ssh', '-f', self.cluster.user+'@'+self.cluster.nsNode, '$ICUB_ROOT/scripts/yarpserver.sh' ' stop']
            print 'Running',
            print cmd
            ret=subprocess.Popen(cmd).wait()
            time.sleep(0.5)
            self.checkNs()
        else:
            print 'Nameserver was not running'

if __name__ == '__main__':

    argc = len(sys.argv)

    if (argc!=2):
        robotName = os.environ.get("ICUB_ROBOTNAME")

        if (robotName!=None):
            configFilename='../../'+robotName+'/scripts/cluster-config.xml'
        else:
            configFilename='./cluster-config.xml'
    else:
        configFilename=sys.argv[1]
        
    print configFilename

    config = xml.dom.minidom.parse(configFilename)

    clusters=config.getElementsByTagName("cluster")

    for cluster in clusters:
        user=cluster.getAttribute("user")
        name=cluster.getAttribute("name")
        nameserver=cluster.getElementsByTagName("nameserver")[0]
        namespace=nameserver.getAttribute("namespace")
        namespaceNode=nameserver.getAttribute("node")
        nodes=cluster.getElementsByTagName("node")

        nodeList=[]
        for node in nodes:

            if (node.hasAttributes()):
                a=node.getAttribute("display")
                if a:
                    newNode=Node(node.firstChild.data, True)
                    
            else:
                newNode=Node(node.firstChild.data, False)

            nodeList.append(newNode)

    cl=Cluster(name, user, namespace, namespaceNode, nodeList)
    cl.display()

    root = Tk()
    app = App(root)
    root.title("Cluster manager for:"+cl.name)

    app.setCluster(cl)
#   dont check when start, nameserver could be off
#   app.checkNs()
#   app.checkNodes()
    root.mainloop()

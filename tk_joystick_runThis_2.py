'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   PROPRIETARY and CONFIDENTIAL

   This file contains source code that constitutes proprietary and
   confidential information created by David Zhu

   Kre8 Technology retains the title, ownership and intellectual property rights
   in and to the Software and all subsequent copies regardless of the
   form or media.  Copying or distributing any portion of this file
   without the written permission of Kre8 Technology is prohibited.

   Use of this code is governed by the license agreement,
   confidentiality agreement, and/or other agreement under which it
   was distributed. When conflicts or ambiguities exist between this
   header and the written agreement, the agreement supersedes this file.
   ========================================================================*/
'''

#--- CS123 Assignment 3-2 ---#
#--- Jamy Li 05442431 ---#
# 
# Summary:
# To test, please press the "Next Step" button in the GUI:
# (1) This button appends a bunch of subgoals into goal_list, then has them 'popped' immediately afterward
# create 2 motion primitives:
# (1) robot moves to (a,x,y). - implemented below
#     dead reackoning way to move it to (a,x,y) without sensors - vworld.goal_list.append("pose",a,x,y)
# (2) move forward until encountering a sensor signal that terminates primitive - still in development, as was having some issues with (1)
#     localize motion primitive: vworld.goal_list.append("direction", sensor, sensor, angle) # turn until sensor reads 40, 40 (i.e. detects object)
# Joystick (aswd keys on keywboard) are not used

# can use walls to localize using the direction primitive


import Tkinter as tk 
import time 
from HamsterAPI.comm_ble import RobotComm
import math
import threading
from tk_hamster_GUI import *
import numpy

UPDATE_INTERVAL = 30

gMaxRobotNum = 1; # max number of robots to control
# gRobotList = None
gQuit = False
m = None

class VirtualWorldGui:
    def __init__(self, vWorld, m):
        self.vworld = vWorld

        self.button0 = tk.Button(m,text="Grid")
        self.button0.pack(side='left')
        self.button0.bind('<Button-1>', self.drawGrid)

        self.button1 = tk.Button(m,text="Clear")
        self.button1.pack(side='left')
        self.button1.bind('<Button-1>', self.clearCanvas)

        self.button2 = tk.Button(m,text="Reset")
        self.button2.pack(side='left')
        self.button2.bind('<Button-1>', self.resetvRobot)

        self.button3 = tk.Button(m,text="Map")
        self.button3.pack(side='left')
        self.button3.bind('<Button-1>', self.drawMap)

        self.button4 = tk.Button(m,text="Trace")
        self.button4.pack(side='left')
        self.button4.bind('<Button-1>', self.toggleTrace)

        self.button5 = tk.Button(m,text="Prox Dots")
        self.button5.pack(side='left')
        self.button5.bind('<Button-1>', self.toggleProx)

        self.button6 = tk.Button(m,text="Floor Dots")
        self.button6.pack(side='left')
        self.button6.bind('<Button-1>', self.toggleFloor)

        self.button9 = tk.Button(m,text="Localize Inst")
        self.button9.pack(side='left')
        self.button9.bind('<Button-1>', self.drawLocalizeInstant)

        self.button8 = tk.Button(m,text="Next Step")
        self.button8.pack(side='left')
        self.button8.bind('<Button-1>', self.nextStep)

        self.button7 = tk.Button(m,text="Exit")
        self.button7.pack(side='left')
        self.button7.bind('<Button-1>', stopProg)

    def resetvRobot(self, event=None):
        self.vworld.vrobot.reset_robot()

    def toggleTrace(self, event=None):
        if self.vworld.trace:
            self.vworld.trace = False
            self.button4["text"] = "Trace"
        else:
            self.vworld.trace = True
            self.button4["text"] = "No Trace"

    def toggleProx(self, event=None):
        if self.vworld.prox_dots:
            self.vworld.prox_dots = False
            self.button5["text"] = "Prox Dots"
        else:
            self.vworld.prox_dots = True
            self.button5["text"] = "No Prox Dots"

    def toggleFloor(self, event=None):
        if self.vworld.floor_dots:
            self.vworld.floor_dots = False
            self.button6["text"] = "Floor Dots"
        else:
            self.vworld.floor_dots = True
            self.button6["text"] = "No Floor Dots"

    def drawMap(self, event=None):
        self.vworld.draw_map()

    def drawGrid(self, event=None):
        x1, y1 = 0, 0
        x2, y2 = self.vworld.canvas_width*2, self.vworld.canvas_height*2
        del_x, del_y = 20, 20
        num_x, num_y = x2 / del_x, y2 / del_y
        # draw center (0,0)
        self.vworld.canvas.create_rectangle(self.vworld.canvas_width-3,self.vworld.canvas_height-3,
                self.vworld.canvas_width+3,self.vworld.canvas_height+3, fill="red")
        # horizontal grid
        for i in range(num_y):
            y = i * del_y
            self.vworld.canvas.create_line(x1, y, x2, y, fill="yellow")
        # verticle grid
        for j in range(num_x):
            x = j * del_x
            self.vworld.canvas.create_line(x, y1, x, y2, fill="yellow")

    def clearCanvas(self, event=None):
        vcanvas = self.vworld.canvas
        vrobot = self.vworld.vrobot
        vcanvas.delete("all")
        poly_points = [0,0,0,0,0,0,0,0]
        vrobot.poly_id = vcanvas.create_polygon(poly_points, fill='blue')
        vrobot.prox_l_id = vcanvas.create_line(0,0,0,0, fill="red")
        vrobot.prox_r_id = vcanvas.create_line(0,0,0,0, fill="red")
        vrobot.floor_l_id = vcanvas.create_oval(0,0,0,0, outline="white", fill="white")
        vrobot.floor_r_id = vcanvas.create_oval(0,0,0,0, outline="white", fill="white")

    def updateCanvas(self, drawQueue):
        self.vworld.canvas.after(UPDATE_INTERVAL, self.updateCanvas, drawQueue)
        while (drawQueue.qsize() > 0):
            drawCommand = drawQueue.get()
            drawCommand()

    #--- go to next step
    #--- add goals depending on the state of the robot, then set the next state of the robot
    def nextStep(self, event=None):
        #--- draw detected point of the box

        sensorl = 50
        sensorr = 50
        angle = 0
        pi4 = 3.1415/4

        vrobot = self.vworld.vrobot
        state = vrobot.state

        print 'state:', state

        if state == 'gotoF':   # note - these letters assume the upper boxes are labeled E and D and the lower boxes are labeled C and B.
            vrobot.set_goal_list( ["direction", 0, 80, 80])
            #vrobot.set_goal_list( ["pose", 0, 80, 0] ) # move to (80,0) then turn to angle 0
        if state == 'gotoE':
            #vworld.goal_list.append( "pose", angle, xcoord, ycoord ) # move to angle a, position x and position y are reached
            vrobot.set_goal_list( ["pose", -pi4*2, 80, 100] )             
        if state == 'gotoD':
            vrobot.set_goal_list( ["pose", -pi4*4, -60, 100] )
        if state == 'gotoC':
            vrobot.set_goal_list( ["pose", -pi4*2, -60, 0] )
        if state == 'gotoA':
            vrobot.set_goal_list( ["direction", -pi4*2, 80, 80] )
        if state == 'done':
            print "done"

        #---- create a new thread to update the physical robot
        update_probot_thread = threading.Thread(target=self.executeGoalList)
        update_probot_thread.daemon = True
        update_probot_thread.start()
    
        #--- update states
        if state == 'start': vrobot.set_state( 'gotoF' )
        if state == 'gotoF': vrobot.set_state( 'gotoE' )
        if state == 'gotoE': vrobot.set_state( 'gotoD' )
        if state == 'gotoD': vrobot.set_state( 'gotoC' )
        if state == 'gotoC': vrobot.set_state( 'gotoA' )
        if state == 'gotoA': vrobot.set_state( 'done' )

        print 'update state to ', vrobot.state


    #--- execute the movements on the physical robot
    #--- send wheel speeds and angles, etc.
    def executeGoalList(self,event=None):
        vrobot = self.vworld.vrobot
        goal_list = vrobot.goal_list
        joystick = self.vworld.joystick

        if joystick.gRobotList:
            robot = joystick.gRobotList[0]
        else:
            print "error - no robot"
            return

        while goal_list:
            task = goal_list.pop(0)
            print 'task:', task

            if task[0] == 'direction':
                angle = task[1]
                sensorl = task[2]
                sensorr = task[3]
                adiff = angle - self.vworld.vrobot.a

                vrobot.sl = 30
                vrobot.sr = 30

                while( robot.get_proximity(0) < sensorl and robot.get_proximity(1) < sensorr ):
                    # robot does not see obj sensors, keep moving
                    robot.set_wheel(0,vrobot.sl)
                    robot.set_wheel(1,vrobot.sr)

                # stop robot
                vrobot.sl = 0
                vrobot.sr = 0
                robot.set_wheel(0,vrobot.sl)
                robot.set_wheel(1,vrobot.sr)

                # turn robot until both sensors see object
                while( robot.get_proximity(0) < sensorl ):
                    # turn right
                    vrobot.sl = 15
                    vrobot.sr = -15
                    robot.set_wheel(0,vrobot.sl)
                    robot.set_wheel(1,vrobot.sr)

                # stop robot
                vrobot.sl = 0
                vrobot.sr = 0
                robot.set_wheel(0,vrobot.sl)
                robot.set_wheel(1,vrobot.sr)

                while( robot.get_proximity(1) < sensorr ):
                    # turn left
                    vrobot.sl = -15
                    vrobot.sr = 15
                    robot.set_wheel(0,vrobot.sl)
                    robot.set_wheel(1,vrobot.sr)

                # stop robot
                vrobot.sl = 0
                vrobot.sr = 0
                robot.set_wheel(0,vrobot.sl)
                robot.set_wheel(1,vrobot.sr)

                # localize
                self.drawLocalizeInstant()

                # rotate robot
                vrobot.sl = 15 * int(numpy.sign(adiff))
                vrobot.sr = -15 * int(numpy.sign(adiff))

                #--- send message to physical robot to rotate
                robot.set_wheel(0,vrobot.sl)
                robot.set_wheel(1,vrobot.sr)
                print abs(adiff)

                a_factor_cw = 16.71
                a_factor2_ccw = 16
                if (vrobot.sl > 0): # rotating clockwise
                    time.sleep( abs(a_factor_cw * adiff / vrobot.sl) )
                if (vrobot.sl < 0): # rotating c-clockwise
                    time.sleep( abs(a_factor2_ccw * adiff / vrobot.sl) )
                print abs(a_factor_cw * adiff / vrobot.sl)
                print abs(a_factor2_ccw * adiff / vrobot.sl)

                # stop robot
                vrobot.sl = 0
                vrobot.sr = 0
                robot.set_wheel(0,vrobot.sl)
                robot.set_wheel(1,vrobot.sr)

            if task[0] == 'pose':
                # move to spot in a hard-coded manner, assume always vertical or horizontal but not both
                angle = task[1]
                xcoord = task[2]
                ycoord = task[3]

                xdist = xcoord - self.vworld.vrobot.x
                ydist = ycoord - self.vworld.vrobot.y
                adiff = angle - self.vworld.vrobot.a

                totaldist = xdist + ydist

                #--- move robot forward

                #--- virtual robot speed. sl = sr to get straight movement
                self.vworld.vrobot.sl = 30
                self.vworld.vrobot.sr = 30

                #---- send message to physical robot to move forward
                self.vworld.joystick.gRobotList[0].set_wheel(0,self.vworld.vrobot.sl)
                self.vworld.joystick.gRobotList[0].set_wheel(1,self.vworld.vrobot.sr)
                print abs(totaldist)

                #--- hard code the distance traveled using the virtual world characteristics
                #--- for example, if totaldist = 90 mm, time should equal 3 sec because speed = 30 mm/s
                # 
                # d_factor = distance / (time) / virtual speed = 1.0667
                # t = distance / virtual speed / 1.0667
                d_factor = 1.0667
                time.sleep( abs(totaldist / self.vworld.vrobot.sl / d_factor ) )


                #--- stop robot briefly
                self.vworld.vrobot.sl = 0
                self.vworld.vrobot.sr = 0
                print "stop briefly"

                #--- send message to physical robot to move
                self.vworld.joystick.gRobotList[0].set_wheel(0,self.vworld.vrobot.sl)
                self.vworld.joystick.gRobotList[0].set_wheel(1,self.vworld.vrobot.sr)

                #---- rotate robot

                #--- virtual robot angle turn speeds
                self.vworld.vrobot.sl = 15 * int(numpy.sign(adiff))
                self.vworld.vrobot.sr = -15 * int(numpy.sign(adiff))
                print "sl", 15 * int(numpy.sign(adiff))

                #--- send message to physical robot to rotate
                self.vworld.joystick.gRobotList[0].set_wheel(0,self.vworld.vrobot.sl)
                self.vworld.joystick.gRobotList[0].set_wheel(1,self.vworld.vrobot.sr)
                print abs(adiff)

                #--- hard code the angle moved
                # angular speed = 15 (wheel speed) / 2*pi rad * 7 sec = 16.71
                # t = 16.71 * angle / wheel speed

                a_factor_cw = 16.71
                a_factor2_ccw = 16
                if (self.vworld.vrobot.sl > 0): # rotating clockwise
                    time.sleep( abs(a_factor_cw * adiff / self.vworld.vrobot.sl) )
                if (self.vworld.vrobot.sl < 0): # rotating c-clockwise
                    time.sleep( abs(a_factor2_ccw * adiff / self.vworld.vrobot.sl) )
                print abs(a_factor_cw * adiff / self.vworld.vrobot.sl)
                print abs(a_factor2_ccw * adiff / self.vworld.vrobot.sl)

                print "vrobot a,x,y = ", vrobot.a, vrobot.x, vrobot.y

                #--- stop robot briefly
                self.vworld.vrobot.sl = 0
                self.vworld.vrobot.sr = 0
                print "stop briefly"

                #--- send message to physical robot to move
                self.vworld.joystick.gRobotList[0].set_wheel(0,self.vworld.vrobot.sl)
                self.vworld.joystick.gRobotList[0].set_wheel(1,self.vworld.vrobot.sr)

                print "vrobot a,x,y = ", vrobot.a, vrobot.x, vrobot.y


    #--------- Draws a linear fit just based on the last 2 proximity measurements taken -------#
    #------------------------------------------------------------------------------------------#
    # Less noisy maybe

    def drawLocalizeInstant(self, event=None):
        locationList = self.vworld.vrobot.location

        if locationList:
            locLast = locationList[-2:]    # get last 2 proximity measurements
            del self.vworld.vrobot.location
            self.vworld.vrobot.location = locLast    # save last 2 proximity measurements

            self.vworld.draw_localize()
            print "draw_localize"

            #--- perform regression to determine approximate position of box
            self.vworld.regr_localize()
            print "regr_localize"

            #--- delete all gathered localization information
            vrobot = self.vworld.vrobot
            for loc in vrobot.location:
                del loc
        else:
            print "no locationization data;"
        
class Joystick:
    def __init__(self, comm, m, rCanvas, vrobot):
        self.gMaxRobotNum = 1
        self.gRobotList = comm.robotList
        self.m = m
        self.vrobot = vrobot

        self.vrobot.t = time.time()

        rCanvas.bind_all('<w>', self.move_up)
        rCanvas.bind_all('<s>', self.move_down)
        rCanvas.bind_all('<a>', self.move_left)
        rCanvas.bind_all('<d>', self.move_right)
        rCanvas.bind_all('<x>', self.stop_move)  
        rCanvas.pack()

    # joysticking the robot 
    def move_up(self, event=None):
        if self.gRobotList:
            robot = self.gRobotList[0]
            self.vrobot.sl = 30
            self.vrobot.sr = 30   
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()
            print 'robot:', repr(robot)

    def move_down(self, event=None):
        if self.gRobotList:   
            robot = self.gRobotList[0]
            self.vrobot.sl = -30
            self.vrobot.sr = -30   
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_left(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            self.vrobot.sl = -15
            self.vrobot.sr = 15   
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()       

    def move_right(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            self.vrobot.sl = 15
            self.vrobot.sr = -15  
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr) 
            self.vrobot.t = time.time()      

    def stop_move(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            self.vrobot.sl = 0
            self.vrobot.sr = 0
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def update_virtual_robot(self):

        #---- model the robot
        #---- tests:
        # Location of test: Gates B21 paper-based grid
        # time = 5 second
        # distance robot moved = 160 mm
        # speed = 160 mm / 5 s = 32 mm / s   (v robot speed = 30 mm / s)
        # d_factor = distance / time / virtual speed = 1.0667
        # d_factor = 32 mm/s / 30 mm/s = 1.0667
        # 

        #---- a_factor_cw
        #---- tests:
        # Location of test: Gates B21 paper-based grid
        # time = 7 seconds
        # angle robot moved = 2*pi radians
        # angular speed = 15 (wheel speed) / (2*pi rad / 7 sec) = 16.71

        #---- a_factor2_ccw
        #---- tests:
        # Location of test: Gates B21 paper-based grid
        # time = 7.8 seconds
        # angle robot moved = 2*pi radians
        # angular speed = 15 (wheel speed) / (2*pi rad / 7.8 sec) = 18.6

        noise_prox = 25 # noisy level for proximity
        noise_floor = 20 #floor ambient color - if floor is darker, set higher noise
        p_factor = 1.6 #proximity conversion - assuming linear #orig = 1.4
        d_factor = 1.0667 #travel distance conversion  -  make bigger if fake robot is slower   # have robot move for 10 seconds. record distance.
        # d_factor = speed of the robot
        a_factor_cw = 16.71 # rotation conversion, assuming linear
        a_factor2_ccw = 16

        while not self.gRobotList:
            print "waiting for robot to connect"
            time.sleep(0.1)

        print "connected to robot"

        while not gQuit:
            if self.gRobotList is not None:
                robot = self.gRobotList[0]

                #--- update wheel balance
                self.gRobotList[0].set_wheel_balance(3)

                t = time.time()
                del_t = t - self.vrobot.t
                self.vrobot.t = t # update the tick
                if self.vrobot.sl == self.vrobot.sr: #---- speed of left wheel = speed of right wheel
                    #--- update x,y position
                    self.vrobot.x = self.vrobot.x + self.vrobot.sl * del_t * math.sin(self.vrobot.a) * d_factor
                    self.vrobot.y = self.vrobot.y + self.vrobot.sl * del_t * math.cos(self.vrobot.a) * d_factor
                if self.vrobot.sl == -self.vrobot.sr:
                    #--- update angle
                    #--- if sr > 0 and sl < 0, rotates clockwise
                    #--- if sr < 0 and sl > 0, rotates counter-clockwise
                    if (self.vrobot.sl > 0): self.vrobot.a = self.vrobot.a + (self.vrobot.sl * del_t)/a_factor_cw 
                    if (self.vrobot.sl < 0): self.vrobot.a = self.vrobot.a + (self.vrobot.sl * del_t)/a_factor2_ccw
                
                #---update sensors and get dist_l and dist_r
                prox_l = robot.get_proximity(0)
                prox_r = robot.get_proximity(1)
                if (prox_l > noise_prox):
                    self.vrobot.dist_l = (100 - prox_l)*p_factor
                else:
                    self.vrobot.dist_l = False
                if (prox_r > noise_prox):
                    self.vrobot.dist_r = (100 - prox_r)*p_factor
                else:
                    self.vrobot.dist_r = False
                
                floor_l = robot.get_floor(0)
                floor_r = robot.get_floor(1)
                if (floor_l < noise_floor):
                    self.vrobot.floor_l = floor_l
                else:
                    self.vrobot.floor_l = False
                if (floor_r < noise_floor):
                    self.vrobot.floor_r = floor_r
                else:
                    self.vrobot.floor_r = False

            time.sleep(0.1)


def stopProg(event=None):
    global gQuit
    global m
    m.quit()
    gQuit = True
    print "Exit"

def draw_virtual_world(virtual_world, joystick):
    time.sleep(1) # give time for robot to connect.
    while not gQuit:
        if joystick.gRobotList is not None:
            virtual_world.draw_robot()
            virtual_world.draw_prox("left")
            virtual_world.draw_prox("right")
            virtual_world.store_prox()
            virtual_world.draw_floor("left")
            virtual_world.draw_floor("right")
        time.sleep(0.1)

def main(argv=None): 
    global m
    comm = RobotComm(gMaxRobotNum)
    comm.start()
    print 'Bluetooth starts'
    m = tk.Tk() #root
    drawQueue = Queue.Queue(0)

    #creating the virtual appearance of the robot
    canvas_width = 700 # half width
    canvas_height = 380 # half height
    rCanvas = tk.Canvas(m, bg="white", width=canvas_width*2, height=canvas_height*2)

    vrobot = virtual_robot()
    pi4 = 3.1415 / 4
    #----- initialize position and angle of robot -----#
    vrobot.set_robot_a_pos(-pi4*2, 230, 0)

    joystick = Joystick(comm, m, rCanvas, vrobot)

    # visual elements of the virtual robot 
    poly_points = [0,0,0,0,0,0,0,0]
    joystick.vrobot.poly_id = rCanvas.create_polygon(poly_points, fill='blue') #robot
    joystick.vrobot.prox_l_id = rCanvas.create_line(0,0,0,0, fill="red") #prox sensors  ---- here
    joystick.vrobot.prox_r_id = rCanvas.create_line(0,0,0,0, fill="red")
    joystick.vrobot.floor_l_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white") #floor sensors
    joystick.vrobot.floor_r_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white")

    time.sleep(1)

    update_vrobot_thread = threading.Thread(target=joystick.update_virtual_robot)
    update_vrobot_thread.daemon = True
    update_vrobot_thread.start()

    #create the virtual worlds that contains the virtual robot, pass the joystick to reuse the move forward, stop, etc. functions
    # joystick should then be renamed to 'set of functions used to move robot forwards, stop robot, etc.' b/c joystick functionality is not used
    vWorld = virtual_world(drawQueue, joystick, joystick.vrobot, rCanvas, canvas_width, canvas_height)
    #objects in the world
    rectF = [0, -50, 40, 50]
    rectE = [-100, -180, 0, -140]
    rectD = [-140, -180, -100, -80]
    rectC = [-100, 140, 0, 180]
    rectB = [-140, 80, -100, 180]
    rectA = [-260, -20, -220, 20]
    vWorld.add_obstacle(rectF)
    vWorld.add_obstacle(rectE)
    vWorld.add_obstacle(rectD)
    vWorld.add_obstacle(rectC)
    vWorld.add_obstacle(rectB)
    vWorld.add_obstacle(rectA)

    draw_world_thread = threading.Thread(target=draw_virtual_world, args=(vWorld, joystick))
    draw_world_thread.daemon = True
    draw_world_thread.start()

    gui = VirtualWorldGui(vWorld, m)

    # #---- create a new thread to update the physical robot
    # update_probot_thread = threading.Thread(target=gui.nextStep)
    # update_probot_thread.daemon = True
    # update_probot_thread.start()

    gui.drawGrid()
    gui.drawMap()

    rCanvas.after(200, gui.updateCanvas, drawQueue)
    m.mainloop()


    #---------------- Automatically scan for box (not needed for assignment) --------------# 

    # start a state machine and set initial state (unnecessary)

    # set up event queue

    # define dispatcher

    # define handler


    for robot in joystick.gRobotList:
        robot.reset()
    comm.stop()
    comm.join()


if __name__ == "__main__":
    main()
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import sys

#  from pyquaternion import Quaternion    ## would be useful for 3D simulation
import numpy as np

window = 0     # number of the glut window
theta = np.pi/4
simTime = 0
dT = 0.01
simRun = True
RAD_TO_DEG = 180.0/3.1416
g = 10
contact = 0


#####################################################
#### Link class, i.e., for a rigid body
#####################################################

class Link:
        color=[0,0,0]    ## draw color
        size=[0.08,1,0.08]     ## dimensions
        mass = 1.0       ## mass in kg
        izz = 1/12       ## moment of inertia about z-axis
        theta=np.pi/4          ## 2D orientation  (will need to change for 3D)
        omega = np.array([0.0,0.0,0.0])        ## radians per second
        posn=np.array([0.0,0.0,0.0])     ## 3D position (keep z=0 for 2D)
        vel = np.array([0.0,0.0,0.0])      ## initial velocity
        
        def draw(self):      ### steps to draw a link
                glPushMatrix()                                            ## save copy of coord frame
                glTranslatef(self.posn[0], self.posn[1], self.posn[2])    ## move 
                glRotatef(self.theta*RAD_TO_DEG,  0,0,1)                             ## rotate
                glScale(self.size[0], self.size[1], self.size[2])         ## set size
                glColor3f(self.color[0], self.color[1], self.color[2])    ## set colour
                DrawCube()                                                ## draw a scaled cube
                glPopMatrix()                                             ## restore old coord frame

#####################################################
#### main():   launches app
#####################################################

def main():
        global window
        global link1, link2, link3, link4
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)     # display mode 
        glutInitWindowSize(640, 480)                                  # window size
        glutInitWindowPosition(0, 0)                                  # window coords for mouse start at top-left
        window = glutCreateWindow("CPSC 526 Simulation Template")
        glutDisplayFunc(DrawWorld)       # register the function to draw the world
        # glutFullScreen()               # full screen
        glutIdleFunc(SimWorld)          # when doing nothing, redraw the scene
        glutReshapeFunc(ReSizeGLScene)   # register the function to call when window is resized
        glutKeyboardFunc(keyPressed)     # register the function to call when keyboard is pressed
        InitGL(640, 480)                 # initialize window
        
        link1 = Link();
        link2 = Link();
        link3 = Link();
        link4 = Link();
        resetSim()
        
        glutMainLoop()                   # start event processing loop

#####################################################
#### keyPressed():  called whenever a key is pressed
#####################################################

def resetSim():
        global link1, link2, link3, link4
        global simTime, simRun
        global g

        printf("Simulation reset\n")
        simRun = True
        simTime = 0

        link1.size = [0.07, 1.0, 0.07]
        link1.color = [1/255, 103/255, 1]
        link1.posn = np.array([-1.0, 1.0, 0.0])
        link1.vel = np.array([0.0, 0.0, 0.0])
        link1.theta = 0.0
        link1.omega = np.array([0.0,0.0,0.0])  ## radians per second
        link1.mass = 1
        link1.izz = link1.mass*link1.size[1]**2/12
    
        link2.size = [0.07, 1.0, 0.07]
        link2.color = [1, 122/255, 112/255]
        link2.posn = np.array([-1.0, 0.0, 0.0])
        link2.vel = np.array([0.0, 0.0, 0.0])
        link2.theta = 0.0
        link2.omega = np.array([0.0,0.0,0.0])  ## radians per second
        link2.mass = 1
        link2.izz = link2.mass*link2.size[1]**2/12
    
        link3.size = [0.07, 1.0, 0.07]
        link3.color = [0, 250/255, 122/255]
        link3.posn = np.array([-1.0, -1.0, 0.0])
        link3.vel = np.array([0.0, 0.0, 0.0])
        link3.theta = 0.0
        link3.omega = np.array([0.0,0.0,0.0])  ## radians per second
        link3.mass = 1
        link3.izz = link3.mass*link3.size[1]**2/12

    
        link4.size = [0.07, 1.0, 0.07]
        link4.color = [228/255, 214/255, 1]
        link4.posn = np.array([-1+0.5*np.sin(np.pi/4), -(1.5 + 0.5*np.cos(np.pi/4)), 0.0])
        link4.vel = np.array([0.0, 0.0, 0.0])
        link4.theta = np.pi/4
        link4.omega = np.array([0.0,0.0,0.0])  ## radians per second
        link4.mass = 1
        link4.izz = link4.mass*link4.size[1]**2/12


#####################################################
#### keyPressed():  called whenever a key is pressed
#####################################################

def keyPressed(key,x,y):
    global simRun
    ch = key.decode("utf-8")
    if ch == ' ':                #### toggle the simulation
            if (simRun == True):
                 simRun = False
            else:
                 simRun = True
    elif ch == chr(27):          #### ESC key
            sys.exit()
    elif ch == 'q':              #### quit
            sys.exit()
    elif ch == 'r':              #### reset simulation
            resetSim()

#####################################################
#### SimWorld():  simulates a time step
#####################################################

def SimWorld():
        global simTime, dT, simRun
        global link1, link2, link3, link4
        global contact

        if (simRun==False):             ## is simulation stopped?
                return
        
        interia1 = np.array([[0, 0, 0],[0, 0, 0], [0, 0, link1.izz]])
        interia2 = np.array([[0, 0, 0],[0, 0, 0], [0, 0, link2.izz]])
        interia3 = np.array([[0, 0, 0],[0, 0, 0], [0, 0, link3.izz]])
        interia4 = np.array([[0, 0, 0],[0, 0, 0], [0, 0, link4.izz]])



        rw1 = np.array([-0.5*np.sin(link1.theta),0.5*np.cos(link1.theta),0])
        rtilda1 = np.array([[0,-rw1[2],rw1[1]],[rw1[2],0,-rw1[0]],[-rw1[1],rw1[0],0]])
        rw2 = np.array([-0.5*np.sin(link2.theta),0.5*np.cos(link2.theta),0])
        rtilda2 = np.array([[0,-rw2[2],rw2[1]],[rw2[2],0,-rw2[0]],[-rw2[1],rw2[0],0]])
        rw3 = np.array([-0.5*np.sin(link3.theta),0.5*np.cos(link3.theta),0])
        rtilda3 = np.array([[0,-rw3[2],rw3[1]],[rw3[2],0,-rw3[0]],[-rw3[1],rw3[0],0]])
        rw4 = np.array([-0.5*np.sin(link4.theta),0.5*np.cos(link4.theta),0])
        rtilda4 = np.array([[0,-rw4[2],rw4[1]],[rw4[2],0,-rw4[0]],[-rw4[1],rw4[0],0]])
        
        kd = 0.1



        rl = np.array([0 ,0.5, 0])
        delta_p1 = link1.posn + rw1 - np.array([-0.5*np.sin(np.pi/4),0.5*np.cos(np.pi/4),0])
        delta_v1 = link1.vel + np.cross(link1.omega,rl)

        delta_p2 = link2.posn + rw2 - (link1.posn-rw1)
        delta_v2 = link2.vel + np.cross(link2.omega,rw2) - (link1.vel + np.cross(link1.omega,-rw1))

        delta_p3 = link3.posn + rw3 - (link2.posn-rw2)
        delta_v3 = link3.vel + np.cross(link3.omega,rw3) - (link2.vel + np.cross(link2.omega,-rw2))

        delta_p4 = link4.posn + rw4 - (link3.posn-rw3)
        delta_v4 = link4.vel + np.cross(link4.omega,rw4) - (link3.vel + np.cross(link3.omega,-rw3))





        #wiw = np.cross(-link1.omega,np.matmul(interia,link1.omega))
        k_p = 0.5
        k_d = 0.5
        

        friction_wiw1 = np.cross(-link1.omega,np.matmul(interia1,link1.omega)) - kd*link1.omega
        friction_wiw2 = np.cross(-link2.omega,np.matmul(interia2,link2.omega)) - kd*link2.omega
        friction_wiw3 = np.cross(-link3.omega,np.matmul(interia3,link3.omega)) - kd*link3.omega
        friction_wiw4 = np.cross(-link4.omega,np.matmul(interia4,link4.omega)) - kd*link4.omega

        wwr1 = np.cross(link1.omega,np.cross(link1.omega,rw1))
        wwr2 = np.cross(link2.omega,np.cross(link2.omega,rw2)) + k_p*delta_p2 + k_d*delta_v2
        wwr3 = np.cross(link3.omega,np.cross(link3.omega,rw3)) + k_p*delta_p3 + k_d*delta_v3
        wwr4 = np.cross(link4.omega,np.cross(link4.omega,rw4)) + k_p*delta_p4 + k_d*delta_v4

        mg1 = -link1.mass*g
        mg2 = -link2.mass*g
        mg3 = -link3.mass*g
        mg4 = -link4.mass*g


            ####  for the constrained one-link pendulum, and the 4-link pendulum,
            ####  you will want to build the equations of motion as a linear system, and then solve that.
            ####  Here is a simple example of using numpy to solve a linear system.

        I = np.identity(3).astype(int)


        a = np.zeros((36, 36))

        a[0,0] = link1.mass
        a[1,1] = link1.mass
        a[2,2] = link1.mass
        a[3,3] = 0.1
        a[4,4] = 0.1
        a[5,5] = link1.izz

        a[6,6] = link2.mass
        a[7,7] = link2.mass
        a[8,8] = link2.mass
        a[9,9] = 0.1
        a[10,10] = 0.1
        a[11,11] = link2.izz

        a[12,12] = link3.mass
        a[13,13] = link3.mass
        a[14,14] = link3.mass
        a[15,15] = 0.1
        a[16,16] = 0.1
        a[17,17] = link3.izz

        a[18,18] = link4.mass
        a[19,19] = link4.mass
        a[20,20] = link4.mass
        a[21,21] = 0.1
        a[22,22] = 0.1
        a[23,23] = link4.izz

        a[0:3,24:27] = -I
        a[24:27,0:3] = -I
       
        a[0:3,27:30] = I
        a[27:30, 0:3] = I
        
        a[3:6,24:27] = -rtilda1
        a[24:27,3:6] = rtilda1

        a[3:6,27:30] = -rtilda1
        a[27:30,3:6] = rtilda1

        a[6:9,27:30] = -I
        a[27:30,6:9] = -I

        a[6:9,30:33] = I
        a[30:33,6:9] = I

        a[9:12,27:30] = -rtilda2
        a[27:30,9:12] = rtilda2

        a[9:12,30:33] = -rtilda2
        a[30:33,9:12] = rtilda2

        a[12:15,30:33] = -I
        a[30:33,12:15] = -I

        a[12:15,33:36] = I
        a[33:36,12:15] = I

        a[15:18,30:33] = -rtilda3
        a[30:33,15:18] = rtilda3

        a[15:18,33:36] = -rtilda3
        a[33:36,15:18] = rtilda3

        a[18:21,33:36] = -I
        a[33:36,18:21] = -I

        a[21:24,33:36] = -rtilda4
        a[33:36,21:24] = rtilda4




        b = np.array([0, mg1, 0,  friction_wiw1[0],  friction_wiw1[1],  friction_wiw1[2],
                      0, mg2, 0,  friction_wiw2[0],  friction_wiw2[1],  friction_wiw2[2],
                      0, mg3, 0,  friction_wiw3[0],  friction_wiw3[1],  friction_wiw3[2],
                      0, mg4 , 0,  friction_wiw4[0],  friction_wiw4[1],  friction_wiw4[2],
                      wwr1[0],wwr1[1],wwr1[2],
                      wwr1[0]+wwr2[0], wwr1[1]+wwr2[1],wwr1[2]+wwr2[2],
                      wwr2[0]+wwr3[0], wwr2[1]+wwr3[1],wwr2[2]+wwr3[2],
                      wwr3[0]+wwr4[0], wwr3[1]+wwr4[1],wwr3[2]+wwr4[2],
                      ])

        x = np.linalg.solve(a, b)
             #  print(x)   # [ -2.17647059  53.54411765  56.63235294]

            #### explicit Euler integration to update the state
        link1.posn += link1.vel*dT
        link1.vel += x[0:3]*dT
        link1.theta += link1.omega[2]*dT
        link1.omega += x[3:6]*dT

        link2.posn += link2.vel*dT
        link2.vel += x[6:9]*dT
        link2.theta += link2.omega[2]*dT
        link2.omega += x[9:12]*dT

        link3.posn += link3.vel*dT
        link3.vel += x[12:15]*dT
        link3.theta += link3.omega[2]*dT
        link3.omega += x[15:18]*dT
        
        link4.posn += link4.vel*dT
        link4.vel += x[18:21]*dT
        link4.theta += link4.omega[2]*dT
        link4.omega += x[21:24]*dT


        DrawWorld()

'''
        link2.posn += link2.vel*dT
        link2.vel += acc2*dT
        link2.theta += link2.omega*dT
        link2.omega += omega_dot2*dT

        simTime += dT

            #### draw the updated state
            
#printf("simTime=%.2f\n",simTime)
'''
#####################################################
#### DrawWorld():  draw the world
#####################################################

def DrawWorld():
        global link1, link2, link3, link4

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	# Clear The Screen And The Depth Buffer
        glLoadIdentity();
        gluLookAt(1,1,6,  0,0,0,  0,1,0)

#DrawOrigin()
        link1.draw()
        link2.draw()
        link3.draw()
        link4.draw()

        glutSwapBuffers()                      # swap the buffers to display what was just drawn

#####################################################
#### initGL():  does standard OpenGL initialization work
#####################################################

def InitGL(Width, Height):				# We call this right after our OpenGL window is created.
    glClearColor(1.0, 1.0, 0.9, 0.0)	# This Will Clear The Background Color To Black
    glClearDepth(1.0)					# Enables Clearing Of The Depth Buffer
    glDepthFunc(GL_LESS)				# The Type Of Depth Test To Do
    glEnable(GL_DEPTH_TEST)				# Enables Depth Testing
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);    glEnable( GL_LINE_SMOOTH );
    glShadeModel(GL_SMOOTH)				# Enables Smooth Color Shading
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()					# Reset The Projection Matrix
    gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)

#####################################################
#### ReSizeGLScene():    called when window is resized
#####################################################

def ReSizeGLScene(Width, Height):
    if Height == 0:						# Prevent A Divide By Zero If The Window Is Too Small 
	    Height = 1
    glViewport(0, 0, Width, Height)		# Reset The Current Viewport And Perspective Transformation
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)    ## 45 deg horizontal field of view, aspect ratio, near, far
    glMatrixMode(GL_MODELVIEW)

#####################################################
#### DrawOrigin():  draws RGB lines for XYZ origin of coordinate system
#####################################################

def DrawOrigin():
        glLineWidth(3.0);

        glColor3f(1,0.5,0.5)   ## light red x-axis
        glBegin(GL_LINES)
        glVertex3f(0,0,0)
        glVertex3f(1,0,0)
        glEnd()

        glColor3f(0.5,1,0.5)   ## light green y-axis
        glBegin(GL_LINES)
        glVertex3f(0,0,0)
        glVertex3f(0,1,0)
        glEnd()

        glColor3f(0.5,0.5,1)   ## light blue z-axis
        glBegin(GL_LINES)
        glVertex3f(0,0,0)
        glVertex3f(0,0,1)
        glEnd()

#####################################################
#### DrawCube():  draws a cube that spans from (-1,-1,-1) to (1,1,1)
#####################################################

def DrawCube():

	glScalef(0.5,0.5,0.5);                  # dimensions below are for a 2x2x2 cube, so scale it down by a half first
	glBegin(GL_QUADS);			# Start Drawing The Cube

	glVertex3f( 1.0, 1.0,-1.0);		# Top Right Of The Quad (Top)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Left Of The Quad (Top)
	glVertex3f(-1.0, 1.0, 1.0);		# Bottom Left Of The Quad (Top)
	glVertex3f( 1.0, 1.0, 1.0);		# Bottom Right Of The Quad (Top)

	glVertex3f( 1.0,-1.0, 1.0);		# Top Right Of The Quad (Bottom)
	glVertex3f(-1.0,-1.0, 1.0);		# Top Left Of The Quad (Bottom)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Bottom)
	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Bottom)

	glVertex3f( 1.0, 1.0, 1.0);		# Top Right Of The Quad (Front)
	glVertex3f(-1.0, 1.0, 1.0);		# Top Left Of The Quad (Front)
	glVertex3f(-1.0,-1.0, 1.0);		# Bottom Left Of The Quad (Front)
	glVertex3f( 1.0,-1.0, 1.0);		# Bottom Right Of The Quad (Front)

	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Back)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Back)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Right Of The Quad (Back)
	glVertex3f( 1.0, 1.0,-1.0);		# Top Left Of The Quad (Back)

	glVertex3f(-1.0, 1.0, 1.0);		# Top Right Of The Quad (Left)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Left Of The Quad (Left)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Left)
	glVertex3f(-1.0,-1.0, 1.0);		# Bottom Right Of The Quad (Left)

	glVertex3f( 1.0, 1.0,-1.0);		# Top Right Of The Quad (Right)
	glVertex3f( 1.0, 1.0, 1.0);		# Top Left Of The Quad (Right)
	glVertex3f( 1.0,-1.0, 1.0);		# Bottom Left Of The Quad (Right)
	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Right)
	glEnd();				# Done Drawing The Quad

            ### Draw the wireframe edges
	glColor3f(0.0, 0.0, 0.0);
	glLineWidth(1.0);
     
	glBegin(GL_LINE_LOOP);		
	glVertex3f( 1.0, 1.0,-1.0);		# Top Right Of The Quad (Top)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Left Of The Quad (Top)
	glVertex3f(-1.0, 1.0, 1.0);		# Bottom Left Of The Quad (Top)
	glVertex3f( 1.0, 1.0, 1.0);		# Bottom Right Of The Quad (Top)
	glEnd();				# Done Drawing The Quad

	glBegin(GL_LINE_LOOP);		
	glVertex3f( 1.0,-1.0, 1.0);		# Top Right Of The Quad (Bottom)
	glVertex3f(-1.0,-1.0, 1.0);		# Top Left Of The Quad (Bottom)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Bottom)
	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Bottom)
	glEnd();				# Done Drawing The Quad

	glBegin(GL_LINE_LOOP);		
	glVertex3f( 1.0, 1.0, 1.0);		# Top Right Of The Quad (Front)
	glVertex3f(-1.0, 1.0, 1.0);		# Top Left Of The Quad (Front)
	glVertex3f(-1.0,-1.0, 1.0);		# Bottom Left Of The Quad (Front)
	glVertex3f( 1.0,-1.0, 1.0);		# Bottom Right Of The Quad (Front)
	glEnd();				# Done Drawing The Quad

	glBegin(GL_LINE_LOOP);		
	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Back)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Back)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Right Of The Quad (Back)
	glVertex3f( 1.0, 1.0,-1.0);		# Top Left Of The Quad (Back)
	glEnd();				# Done Drawing The Quad

	glBegin(GL_LINE_LOOP);		
	glVertex3f(-1.0, 1.0, 1.0);		# Top Right Of The Quad (Left)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Left Of The Quad (Left)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Left)
	glVertex3f(-1.0,-1.0, 1.0);		# Bottom Right Of The Quad (Left)
	glEnd();				# Done Drawing The Quad

	glBegin(GL_LINE_LOOP);		
	glVertex3f( 1.0, 1.0,-1.0);		# Top Right Of The Quad (Right)
	glVertex3f( 1.0, 1.0, 1.0);		# Top Left Of The Quad (Right)
	glVertex3f( 1.0,-1.0, 1.0);		# Bottom Left Of The Quad (Right)
	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Right)
	glEnd();				# Done Drawing The Quad

####################################################
# printf()  
####################################################

def printf(format, *args):
    sys.stdout.write(format % args)

################################################################################
# start the app

print ("Hit ESC key to quit.")
main()
    	

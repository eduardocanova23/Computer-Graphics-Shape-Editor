import sys
import numpy as np
from OpenGL.GLUT import *
from OpenGL.GL import *
from OpenGL.GLU import *
from pyrr.matrix44 import *
import math

shapes = []
def euclidean_distance(point1, point2):
    squared_distance = sum((p1 - p2) ** 2 for p1, p2 in zip(point1, point2))
    return math.sqrt(squared_distance)

def apply_to_points(matrix, points):
    transformed_points = []
    for point in points:
        transformed_point = apply_to_vector(matrix, [point[0], point[1], 0, 1])
        transformed_points.append(transformed_point[:2])
    return transformed_points

class Circle:
    def __init__(self, center, radius, m=create_identity()):
        self.center = center
        self.radius = radius
        self.set_matrix(m)

    def set_radius(self, radius):
        self.radius = radius

    def set_matrix(self, t):
        self.m = t
        self.invm = inverse(t)

    def contains(self, p):
        p = apply_to_vector(self.invm, [p[0], p[1], 0, 1])
        cx, cy = self.center
        distance = math.sqrt((p[0] - cx) ** 2 + (p[1] - cy) ** 2)
        return distance <= self.radius

    def get_center(self):
        x_center = self.center[0]
        y_center = self.center[1]
        center_vector = apply_to_vector(self.m, [x_center, y_center, 0, 1])
        return center_vector[:2]

    def draw(self):
        glPushMatrix()
        glMultMatrixf(self.m)

        glColor3f(0.125,0.1640625,0.265625)
        glTranslatef(self.center[0], self.center[1], 0)
        gluDisk(gluNewQuadric(), 0, self.radius, 100, 1)
        glTranslatef(-self.center[0], -self.center[1], 0)
      
        glColor3f(1, 0.99, 0.82)
        glLineWidth(2.0)
        glBegin(GL_LINE_LOOP)
        num_segments = 100
        for i in range(num_segments):
            theta = 2.0 * np.pi * i / num_segments
            x = self.center[0] + self.radius * np.cos(theta)
            y = self.center[1] + self.radius * np.sin(theta)
            glVertex2f(x, y)
        glEnd()

        glPopMatrix()


class Rect(object):
    def __init__ (self, points, m = create_identity()):
        self.points = points
        self.set_matrix(m)
        self.center = [(points[0][0] + points[1][0]) / 2, (points[0][1] + points[1][1]) / 2]
    def set_point (self, i, p):
        self.points[i] = p

    def set_matrix(self,t):
        self.m = t
        self.invm = inverse(t)
    
    def get_center(self):
        #return [(self.points[0][0] + self.points[1][0]) / 2, (self.points[0][1] + self.points[1][1]) / 2]
        x_center = (self.points[0][0] + self.points[1][0]) / 2
        y_center = (self.points[0][1] + self.points[1][1]) / 2
        center_vector = apply_to_vector(self.m, [x_center, y_center, 0, 1])
        return center_vector[:2]

    def contains(self,p):
        p = apply_to_vector(self.invm, [p[0],p[1],0,1])
        xmin = min(self.points[0][0],self.points[1][0])
        xmax = max(self.points[0][0],self.points[1][0])
        ymin = min(self.points[0][1],self.points[1][1])
        ymax = max(self.points[0][1],self.points[1][1])
        return xmin <= p[0] <= xmax and ymin <=p[1] <= ymax
    def draw (self):
        glPushMatrix()
        glMultMatrixf(self.m)
        
        glRectf(*self.points[0],*self.points[1])
        glPopMatrix()

picked = None
modeConstants = ["CREATE RECTANGLE", "CREATE CIRCLE", "TRANSLATE", "ROTATE", "SCALE"]
mode = modeConstants[0]
current_mouse_pos = [0,0]
last_angle = 0
angle = 0

def mouse (button, state, x, y):
    global lastx,lasty,picked,lastx_whenclicked,lasty_whenclicked,angle, last_angle
    if button == GLUT_LEFT_BUTTON and state == GLUT_DOWN:
        lastx_whenclicked, lasty_whenclicked = (x, y)
        print(lastx_whenclicked,lasty_whenclicked)

    if state!=GLUT_DOWN: return
    if mode == "CREATE RECTANGLE":
        shapes.append(Rect([[x,y],[x,y]]))

    elif mode == "CREATE CIRCLE":
        shapes.append(Circle([x,y], 0))

    elif mode == "TRANSLATE":
        picked = None
        for s in shapes:
            if s.contains([x,y]): picked = s
        lastx,lasty = x,y
        
    elif mode == "ROTATE":
        picked = None
        for s in shapes:
            if s.contains([x, y]):
                picked = s
                angle = np.arctan2(picked.get_center()[1] - y, picked.get_center()[0] - x)
        lastx, lasty = x, y

    elif mode == "SCALE":
        picked = None
        for s in shapes:
            if s.contains([x, y]):
                picked = s
                break
        lastx, lasty = x, y

def mouse_drag(x, y):
    global current_mouse_pos, last_mouse_pos, angle, last_angle, lastx, lasty
    last_mouse_pos = current_mouse_pos
    current_mouse_pos = [x, y]
    if mode == "CREATE RECTANGLE":
        
        shapes[-1].set_point(1,[x,y])
    elif mode == "CREATE CIRCLE":
        shapes[-1].set_radius(euclidean_distance(shapes[-1].center,[x,y]))

    elif mode == "TRANSLATE":
        if picked:
            delta_x = x - lastx
            delta_y = y - lasty
            t = create_from_translation([x-lastx,y-lasty,0])
            picked.set_matrix(multiply(picked.m,t))
            lastx,lasty=x,y
    elif mode == "ROTATE":
         if picked:
            center = picked.get_center()
            obj_center = [center[0], center[1], 0]
            mouse_pos = [x, y, 0]
            delta_angle = angle - np.arctan2(obj_center[1] - mouse_pos[1], obj_center[0] - mouse_pos[0])
            t = create_from_translation([-obj_center[0], -obj_center[1], 0])
            r = create_from_eulers([0, delta_angle, 0])
            t_inv = create_from_translation([obj_center[0], obj_center[1], 0])
            rotation_matrix = multiply(multiply(t, r), t_inv)
            picked.set_matrix(multiply(picked.m, rotation_matrix))
            angle -= delta_angle

    elif mode == "SCALE":
        delta_x = x - lastx
        delta_y = y - lasty
        
        if picked:

            center = picked.get_center()
            sx = 1 + delta_x / abs(center[0]-delta_x)
            sy = 1 + delta_y / abs(center[1]-delta_y)

            S = np.array([[sx, 0, 0, 0],
                  [0, sy, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

            picked_matrix = picked.m
            updated_matrix = np.dot(picked_matrix, S)

            picked.set_matrix(updated_matrix)
            t = create_from_translation([-(x-lastx),-(y-lasty),0])
            picked.set_matrix(multiply(picked.m,t))
            lastx,lasty=x,y
    glutPostRedisplay()


def reshape( width, height):
    glViewport(0,0,width,height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluOrtho2D(0,width,height,0)
    glMatrixMode (GL_MODELVIEW)


def display():
    glClear(GL_COLOR_BUFFER_BIT)
    for s in shapes:
        glColor3f(0.125,0.1640625,0.265625)
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL)
        s.draw()
        glColor3f(1,0.9,0.82)
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE)
        s.draw()
    glutSwapBuffers()

def createMenu():
    global main_menu_id
    def domenu(item):
        global mode
        mode = modeConstants[item]
        return 0
    main_menu_id = glutCreateMenu(domenu)
    for i,name in enumerate(modeConstants):
        glutAddMenuEntry(name, i)

    glutSetMenu(main_menu_id)
    glutAttachMenu(GLUT_RIGHT_BUTTON)


glutInit(sys.argv);
glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
glutInitWindowSize (800, 600);
glutCreateWindow ("rectangle editor")
glutMouseFunc(mouse)
glutMotionFunc(mouse_drag)
glutDisplayFunc(display); 
glutReshapeFunc(reshape)
createMenu()

glutMainLoop();

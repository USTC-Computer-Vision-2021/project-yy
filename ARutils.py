import cv2
from SIFT import pysift
from matplotlib import pyplot as plt
import logging
import math
import sys
from pylab import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import pygame, pygame.image
from pygame.locals import *
from pcv.geometry import homography, camera
from pcv.localdescriptors import sift
from numpy import *
import pandas as pd
from PIL import Image

import os
os.chdir( os.path.split( os.path.realpath( sys.argv[0] ) )[0] ) 

logger = logging.getLogger()
logger.setLevel(logging.ERROR) 

# Get the points of cube
def cube_points(c, wid):  
    """ 
    Creates a list of points for plotting
    a cube with plot. (the first 5 points are
    the bottom square, some sides repeated). 
    """
    p = []
    # bottom
    p.append([c[0] - wid, c[1] - wid, c[2] - wid])
    p.append([c[0] - wid, c[1] + wid, c[2] - wid])
    p.append([c[0] + wid, c[1] + wid, c[2] - wid])
    p.append([c[0] + wid, c[1] - wid, c[2] - wid])
    p.append([c[0] - wid, c[1] - wid, c[2] - wid])  # back to the first point

    # top
    p.append([c[0] - wid, c[1] - wid, c[2] + wid])
    p.append([c[0] - wid, c[1] + wid, c[2] + wid])
    p.append([c[0] + wid, c[1] + wid, c[2] + wid])
    p.append([c[0] + wid, c[1] - wid, c[2] + wid])
    p.append([c[0] - wid, c[1] - wid, c[2] + wid])  # back to the first point

    # vertical edge
    p.append([c[0] - wid, c[1] - wid, c[2] + wid])
    p.append([c[0] - wid, c[1] + wid, c[2] + wid])
    p.append([c[0] - wid, c[1] + wid, c[2] - wid])
    p.append([c[0] + wid, c[1] + wid, c[2] - wid])
    p.append([c[0] + wid, c[1] + wid, c[2] + wid])
    p.append([c[0] + wid, c[1] - wid, c[2] + wid])
    p.append([c[0] + wid, c[1] - wid, c[2] - wid])

    return array(p).T

# Get the calibration matrix
def my_calibration(center, length):
    K = diag([length * 5, length * 5, 1])
    K[0, 2] = center[0]
    K[1, 2] = center[1]
    return K

# Load data or calculate the keypoints and descriptor
def load_data(file_source):
    f = file_source.split('/')
    filename = f[-1]
    if os.path.isfile('data/' + filename + '.csv'):
        data = pd.read_csv('data/' + filename + '.csv').values
        l = data[:,1:3]
        d = data[:,3:]
    else:
        img = cv2.imread(file_source, 0)
        l, d = pysift.computeKeypointsAndDescriptors(img)
        l = keypoints2location(l)
        h = l.shape[0]
        w = l.shape[1]+d.shape[1]
        data = np.zeros([h,w])
        data[:,:2] = l
        data[:,2:] = d
        pd.DataFrame(data).to_csv('data/' + filename + '.csv')
    return l, d

# Transform keypoints to location
def keypoints2location(keypoints):
    locations = []
    for key in keypoints:
        locations.append([key.pt[0], key.pt[1]])
    return np.array(locations)

# Plot the projection on the image
def plot_projection(filename0, filename1, box_cam1, box_trans, box_cam2, ndx=[], ndx2=[], l0=[], l1=[], obj_model=None):
    figure()
    im0 = cv2.imread(filename0)
    im0 = cv2.cvtColor(im0, cv2.COLOR_BGR2RGB)
    plot(box_cam1[0, :], box_cam1[1, :], linewidth=3)
    for i in ndx:
        cv2.circle(im0, (int(l0[i,0]), int(l0[i,1])), int(im0.shape[0]/200), (255,0,0), thickness=-1)
    title('2D square projection of bottom square')
    imshow(im0)

    figure()
    im1 = cv2.imread(filename1)
    im1 = cv2.cvtColor(im1, cv2.COLOR_BGR2RGB)
    plot(box_trans[0, :], box_trans[1, :], linewidth=3)
    title('2D square projection transfered with H')
    for i in ndx2:
        cv2.circle(im1, (int(l1[i,0]), int(l1[i,1])), int(im1.shape[0]/200), (255,0,0), thickness=-1)
    imshow(im1)

    figure()
    im1 = cv2.imread(filename1)
    im1 = cv2.cvtColor(im1, cv2.COLOR_BGR2RGB)
    if obj_model:
        for i in range(len(obj_model)):
            plot(obj_model[i][0,:], obj_model[i][1,:], linewidth=3)
    else:
        plot(box_cam2[0, :], box_cam2[1, :], linewidth=0.1)
    title('3D model projected in second image')
    for i in ndx2:
        cv2.circle(im1, (int(l1[i,0]), int(l1[i,1])), int(im1.shape[0]/200), (255,0,0), thickness=-1)
    imshow(im1)
    show()

# Get view
def set_projection_from_camera(K, height, width): 
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    fx = K[0, 0]
    fy = K[1, 1]
    fovy = 2 * math.atan(0.5 * height / fy) * 180 / math.pi
    aspect = (width * fy) / (height * fx)
    # Define near and far clipping planes
    near = 0.1
    far = 100.0
    # Set perspective
    gluPerspective(fovy, aspect, near, far)
    glViewport(0, 0, width, height)

# Get matrix
def set_modelview_from_camera(Rt, height, width, center):
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    # Rotate the teapot 90 degrees around the x-axis so that the z-axis is up
    Rx = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    # Obtain the best approximation of rotation
    R = Rt[:, :3]
    # Singular value decomposition
    U, S, V = np.linalg.svd(R)
    R = np.dot(U, V)
    # Change the symbol of the x-axis
    R[0, :] = -R[0, :]
    # Obtain the amount of translation
    t = Rt[:, 3]
    # The value t of t are x-axis, y-axis, z-axis, 
    # When the value of t is positive, x turn right, y turn up
    t[0] = (center[0] - width / 2) * 0.001
    t[1] = (height / 2 - center[1]) * 0.001
    # Get an analog view of a 4x4 matrix
    M = np.eye(4)
    M[:3, :3] = np.dot(R, Rx)
    M[:3, 3] = t
    # Transpose and flattening to get column order value
    M = M.T
    m = M.flatten()
    # Replace the analog view matrix with a new matrix
    glLoadMatrixf(m)


# Load background image
def draw_background(imname):
    img =  Image.open(imname)
    img = img.transpose(Image.FLIP_TOP_BOTTOM)
    imgData = np.array(list(img.getdata()), np.uint8)
    glMatrixMode(GL_MODELVIEW)  # Specifies the current matrix as the projection matrix
    glLoadIdentity()  # Set the matrix as the unit matrix

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)  # Clear color, depth buffer
    glEnable(GL_TEXTURE_2D)  # Map Texture 
    glBindTexture(GL_TEXTURE_2D, glGenTextures(1))
    if len(imgData.shape) == 1:
        glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, img.size[0], img.size[1], 
                    0, GL_LUMINANCE, GL_UNSIGNED_BYTE, imgData)
    elif imgData.shape[1] == 4:
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.size[0], img.size[1], 
                    0, GL_RGBA, GL_UNSIGNED_BYTE, imgData)
    elif imgData.shape[1] == 3:
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.size[0], img.size[1], 
                    0, GL_RGB, GL_UNSIGNED_BYTE, imgData)
    
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    
    # Bind texture
    glBegin(GL_QUADS)
    glTexCoord2f(0.0, 0.0)
    glVertex3f(-1.0, -1.0, -1.0)
    glTexCoord2f(1.0, 0.0)
    glVertex3f(1.0, -1.0, -1.0)
    glTexCoord2f(1.0, 1.0)
    glVertex3f(1.0, 1.0, -1.0)
    glTexCoord2f(0.0, 1.0)
    glVertex3f(-1.0, 1.0, -1.0)
    glEnd()
    glDeleteTextures(1) # Clear texture



def draw_teapot(size):  # Red teapot
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_DEPTH_TEST)
    glClear(GL_DEPTH_BUFFER_BIT)
    # Draw red teapot
    glMaterialfv(GL_FRONT, GL_AMBIENT, [0, 0, 0, 0])
    glMaterialfv(GL_FRONT, GL_DIFFUSE, [0.5, 0.0, 0.0, 0.0])
    glMaterialfv(GL_FRONT, GL_SPECULAR, [0.7, 0.6, 0.6, 0.0])
    glMaterialf(GL_FRONT, GL_SHININESS, 0.25 * 128.0)
    glutSolidTeapot(size)
    glFlush()


def drawFunc(size):  # White teapot
    glRotatef(0.5, 5, 5, 0)
    glutWireTeapot(size)
    # Refresh display
    glFlush()



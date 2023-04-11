#!/usr/bin/env python
import sys, time

import numpy as np
import vispy
from vispy import scene

import esp32imu

# https://ahrs.readthedocs.io/en/latest/index.html
import ahrs
from ahrs.filters import Tilt
from ahrs.filters import Complementary
from ahrs.filters import Madgwick
from ahrs.filters import Mahony
from ahrs.filters import EKF

from scipy.spatial.transform import Rotation as Rot

import os
import queue
import threading

# https://vispy.org/gallery/scene/index.html
# https://vispy.org/gallery/scene/turntable_box.html#sphx-glr-gallery-scene-turntable-box-py
canvas = scene.SceneCanvas(keys='interactive', size=(800, 600), show=True)

# Set up a viewbox to display the cube with interactive arcball
view = canvas.central_widget.add_view()
view.bgcolor = '#efefef'
view.camera = 'turntable'
view.padding = 100

# make axes in corner
axes = scene.visuals.XYZAxis(width=5, antialias=True, parent=view.scene)
axes.transform = vispy.visuals.transforms.MatrixTransform()
axes.transform.matrix[3,:3] = (-1,-1,-1)

def mk_cube(pos):
    global view
    nf = 12
    fcolor = np.ones((nf, 4), dtype=np.float32)

    # -z
    fcolor[0,:] = [0.7, 0.7, 1, 1]
    fcolor[1,:] = [0.7, 0.7, 1, 1]

    # +z
    fcolor[2,:] = [0, 0, 1, 1]
    fcolor[3,:] = [0, 0, 1, 1]

    # -y
    fcolor[4,:] = [0.7, 1, 0.7, 1]
    fcolor[5,:] = [0.7, 1, 0.7, 1]

    # +y
    fcolor[6,:] = [0, 1, 0, 1]
    fcolor[7,:] = [0, 1, 0, 1]

    # -x
    fcolor[8,:] = [1, 0.7, 0.7, 1]
    fcolor[9,:] = [1, 0.7, 0.7, 1]

    # +x
    fcolor[10,:] = [1, 0, 0, 1]
    fcolor[11,:] = [1, 0, 0, 1]

    cube = scene.visuals.Box(1, 1, 1, face_colors=fcolor, edge_color="black", parent=view.scene)
    cube.transform = vispy.visuals.transforms.MatrixTransform()
    cube.transform.matrix[3,:3] = pos
    return cube

# q -> (w,x,y,z)
# from_quat -> (x,y,z,w)
def set_cube_rot(cube,q):
    T = np.array(cube.transform.matrix)
    T[:3,:3] = Rot.from_quat((q[1],q[2],q[3],q[0])).as_matrix().T
    cube.transform.matrix = T

################################################################################

# https://ahrs.readthedocs.io/en/latest/filters/tilt.html
class myTilt:
    def __init__(self,pos):
        self.q = np.array([1., 0, 0, 0])
        self.cube = mk_cube(pos)
        self.filt = Tilt()

    def update(self,accel,gyro):
        self.q = self.filt.estimate(accel)
        set_cube_rot(self.cube, self.q)

# https://ahrs.readthedocs.io/en/latest/filters/complementary.html
class myComplementary:
    def __init__(self,pos):
        self.q = np.array([1., 0, 0, 0])
        self.cube = mk_cube(pos)
        self.filt = Complementary()

    def update(self,accel,gyro):
        self.q = self.filt.update(self.q, gyro, accel)
        set_cube_rot(self.cube, self.q)

# https://ahrs.readthedocs.io/en/latest/filters/madgwick.html
class myMadgwick:
    def __init__(self,pos):
        self.q = np.array([1., 0, 0, 0])
        self.cube = mk_cube(pos)
        self.filt = Madgwick()

    def update(self,accel,gyro):
        self.q = self.filt.updateIMU(self.q, gyr=gyro, acc=accel)
        set_cube_rot(self.cube, self.q)

# https://ahrs.readthedocs.io/en/latest/filters/mahony.html
class myMahony:
    def __init__(self,pos):
        self.q = np.array([1., 0, 0, 0])
        self.cube = mk_cube(pos)
        self.filt = Mahony()

    def update(self,accel,gyro):
        self.q = self.filt.updateIMU(self.q, gyr=gyro, acc=accel)
        set_cube_rot(self.cube, self.q)

# https://ahrs.readthedocs.io/en/latest/filters/ekf.html
class myEKF:
    def __init__(self,pos):
        self.q = np.array([1., 0, 0, 0])
        self.cube = mk_cube(pos)
        self.filt = EKF(frame='ENU')

    def update(self,accel,gyro):
        self.q = self.filt.update(self.q, gyro, accel)
        set_cube_rot(self.cube, self.q)

# https://docs.python.org/3/library/queue.html
que = queue.Queue()
def imu_callback(msg):
    gyro = np.array([msg.gyro_x, msg.gyro_y, msg.gyro_z])
    accel = np.array([msg.accel_x, msg.accel_y, msg.accel_z])
    que.put((accel,gyro))

def worker():
    while True:
        accel,gyro = que.get()
        # print(que.qsize())
        for cube in cubes:
            cube.update(accel,gyro)


################################################################################

driver = esp32imu.SerialDriver('/dev/ttyUSB0', 2000000)
driver.registerCallbackIMU(imu_callback)
# time.sleep(0.2)
driver.sendRate(100)

#myClasses = [myTilt, myComplementary, myMadgwick, myMahony, myEKF]
myClasses = [myTilt, myMadgwick, myEKF]
cubes = [myclass((i*2,0,0)) for i,myclass in enumerate(myClasses)]

# https://vispy.org/api/vispy.scene.cameras.base_camera.html#vispy.scene.cameras.base_camera.BaseCamera.set_range
# https://vispy.org/api/vispy.scene.cameras.base_camera.html#vispy.scene.cameras.base_camera.BaseCamera.center
if len(cubes) > 1:
    view.camera.set_range(x=(-2,2*(len(cubes))))

threading.Thread(target=worker, daemon=True).start()

if __name__ == '__main__' and sys.flags.interactive == 0:
    canvas.app.run()

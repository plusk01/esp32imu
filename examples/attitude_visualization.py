import sys, time

import numpy as np
import vispy
from vispy import scene

import esp32imu

import ahrs

from scipy.spatial.transform import Rotation as Rot

# filt = ahrs.filters.AngularRate()
filt = ahrs.filters.EKF()
q = np.array([1, 0, 0, 0])





canvas = scene.SceneCanvas(keys='interactive', size=(800, 600), show=True)

# Set up a viewbox to display the cube with interactive arcball
view = canvas.central_widget.add_view()
view.bgcolor = '#efefef'
view.camera = 'turntable'
view.padding = 100

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

# import ipdb; ipdb.set_trace()

cube = scene.visuals.Box(1, 1, 1, face_colors=fcolor, edge_color="black", parent=view.scene)
cube.transform = vispy.visuals.transforms.MatrixTransform()


axes = scene.visuals.XYZAxis(width=5, antialias=True, parent=view.scene)
axes.transform = vispy.visuals.transforms.MatrixTransform()

axes.transform.matrix[3,:3] = (-1,-1,-1)



def imu_cb(msg):
    global q
    q = filt.update(q, (msg.gyro_x, msg.gyro_y, msg.gyro_z), (msg.accel_x, msg.accel_y, msg.accel_z))

    global cube
    T = np.eye(4)
    T[:3,:3] = Rot.from_quat((q[1],q[2],q[3],q[0])).as_matrix().T
    cube.transform.matrix = T



driver = esp32imu.UDPDriver()

msg = esp32imu.RGBLedCmdMsg()
msg.r = 255
msg.g = 0
msg.b = 0
msg.brightness = 50
driver.sendRGBLedCmd(msg)

driver.registerCallbackIMU(imu_cb)

time.sleep(0.2)
driver.sendRate(100)

# time.sleep(5)



# import ipdb; ipdb.set_trace()

if __name__ == '__main__' and sys.flags.interactive == 0:
    canvas.app.run()
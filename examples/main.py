
import time, sys

import numpy as np

import pyqtgraph as pg
from pyqtgraph.console import ConsoleWidget
from pyqtgraph.dockarea.Dock import Dock
from pyqtgraph.dockarea.DockArea import DockArea
import pyqtgraph.parametertree.parameterTypes as pTypes
from pyqtgraph.parametertree import Parameter, ParameterTree
from pyqtgraph.Qt import QtWidgets

import esp32imu

class ViewControl:
    def __init__(self, callbacks, params, plot_update_hz):
        self.plot_update_hz = plot_update_hz
        self.params = params
        self.callbacks = callbacks
        self.__setup_ui()
        self.__create_plot_update_timer()

    def __setup_ui(self):
        self.app = pg.mkQApp("ESP32 Data Viewer")

        # docks
        self.dock_connection = Dock("Connection", size=(10,1), hideTitle=True, closable=False)
        self.dock_controls = Dock("Controls", size=(10,3), hideTitle=True, closable=False)
        self.dock_rgbled = Dock("RGB LED", size=(10,2), hideTitle=True, closable=False)
        self.dock_params = Dock("Parameters", size=(10,4), hideTitle=True, closable=False)
        self.dock_hzplot = Dock("Debug Plots", size=(10,4), hideTitle=True, closable=False)
        self.dock_plots = Dock("Plots", size=(400,10), hideTitle=True, closable=False)
        self.__setup_docks()

        # add docks
        self.area = DockArea()
        self.area.addDock(self.dock_connection, 'left')
        self.area.addDock(self.dock_controls, 'bottom', self.dock_connection)
        self.area.addDock(self.dock_rgbled, 'bottom', self.dock_controls)
        self.area.addDock(self.dock_params, 'bottom', self.dock_rgbled)
        self.area.addDock(self.dock_hzplot, 'bottom', self.dock_params)
        self.area.addDock(self.dock_plots, 'right')

        # set up window
        self.win = QtWidgets.QMainWindow()
        self.win.setCentralWidget(self.area)
        self.win.resize(1600, 800)
        self.win.setWindowTitle("ESP32 Data Viewer")

    def __setup_docks(self):
        # Connection dock
        layout = pg.LayoutWidget()
        self.btn_con_serial = QtWidgets.QPushButton('Connect via Serial')
        self.btn_dis_serial = QtWidgets.QPushButton('Disconnect via Serial', enabled=False)
        self.btn_con_udp = QtWidgets.QPushButton('Connect via UDP')
        self.btn_dis_udp = QtWidgets.QPushButton('Disconnect via UDP', enabled=False)
        layout.addWidget(self.btn_con_serial, row=0, col=0)
        layout.addWidget(self.btn_dis_serial, row=0, col=1)
        layout.addWidget(self.btn_con_udp, row=1, col=0)
        layout.addWidget(self.btn_dis_udp, row=1, col=1)
        self.dock_connection.addWidget(layout)

        # Button setup
        self.btn_con_serial.clicked.connect(self.callbacks['connect_serial'])
        self.btn_dis_serial.clicked.connect(self.callbacks['disconnect_serial'])
        self.btn_con_udp.clicked.connect(self.callbacks['connect_udp'])
        self.btn_dis_udp.clicked.connect(self.callbacks['disconnect_udp'])

        # Controls dock
        layout = pg.LayoutWidget()
        self.chk_stream = QtWidgets.QCheckBox('stream online data', checked=True)
        btn_enable_log = QtWidgets.QPushButton('Start SD Logging')
        btn_disable_log = QtWidgets.QPushButton('Stop SD Logging', enabled=True)
        btn_read_log = QtWidgets.QPushButton('Read SD Log')
        label = QtWidgets.QLabel(text='0 samples copied from SD card')
        layout.addWidget(self.chk_stream, row=0, col=0, colspan=1)
        layout.addWidget(btn_enable_log, row=1, col=0)
        layout.addWidget(btn_disable_log, row=1, col=1)
        layout.addWidget(btn_read_log, row=2, col=0, colspan=2)
        layout.addWidget(label, row=3, col=0, colspan=1)
        self.dock_controls.addWidget(layout)

        # Button setup
        self.chk_stream.toggled.connect(self.callbacks['toggle_stream'])
        btn_enable_log.clicked.connect(self.callbacks['start_logging'])
        btn_disable_log.clicked.connect(self.callbacks['stop_logging'])
        btn_read_log.clicked.connect(self.callbacks['read_log'])

        # RGB LED dock
        layout = pg.LayoutWidget()
        self.rslider = QtWidgets.QSlider(pg.QtCore.Qt.Orientation.Horizontal)
        self.rslider.setRange(0, 255)
        self.gslider = QtWidgets.QSlider(pg.QtCore.Qt.Orientation.Horizontal)
        self.gslider.setRange(0, 255)
        self.bslider = QtWidgets.QSlider(pg.QtCore.Qt.Orientation.Horizontal)
        self.bslider.setRange(0, 255)
        self.bslider.setValue(255)
        self.islider = QtWidgets.QSlider(pg.QtCore.Qt.Orientation.Horizontal)
        self.islider.setRange(0, 100)
        self.islider.setValue(5)
        layout.addWidget(QtWidgets.QLabel(text='RGB LED Control'), row=0, col=0, colspan=10)
        layout.addWidget(QtWidgets.QLabel(text='Red'), row=1, col=0)
        layout.addWidget(self.rslider, row=1, col=1, colspan=9)
        layout.addWidget(QtWidgets.QLabel(text='Green'), row=2, col=0)
        layout.addWidget(self.gslider, row=2, col=1, colspan=9)
        layout.addWidget(QtWidgets.QLabel(text='Blue'), row=3, col=0)
        layout.addWidget(self.bslider, row=3, col=1, colspan=9)
        layout.addWidget(QtWidgets.QLabel(text='Intensity'), row=4, col=0)
        layout.addWidget(self.islider, row=4, col=1, colspan=9)
        self.dock_rgbled.addWidget(layout)

        # Slider setup
        self.rslider.sliderReleased.connect(self.callbacks['update_rgbled'])
        self.gslider.sliderReleased.connect(self.callbacks['update_rgbled'])
        self.bslider.sliderReleased.connect(self.callbacks['update_rgbled'])
        self.islider.sliderReleased.connect(self.callbacks['update_rgbled'])

        # Parameter tree dock
        layout = pg.LayoutWidget()
        t = self.__create_param_tree()
        layout.addWidget(t, row=0, col=0)
        self.dock_params.addWidget(layout)

        # Hz plot dock
        self.plot_hz = pg.PlotWidget(title='Data Transfer Rate [Hz]')
        layout = pg.LayoutWidget()
        layout.addWidget(self.plot_hz, row=0, col=0)
        self.dock_hzplot.addWidget(layout)

        # Plots dock
        self.plot_acc = pg.PlotWidget(title='Accel [m/s/s]')
        self.plot_gyr = pg.PlotWidget(title='Gyro [rad/s]')
        layout = pg.LayoutWidget()
        layout.addWidget(self.plot_acc, row=0, col=0)
        layout.addWidget(self.plot_gyr, row=1, col=0)
        self.dock_plots.addWidget(layout)

        # plot handles to update data with
        self.curve_hz = self.plot_hz.plot(pen=pg.mkPen('g'))
        self.curve_accx = self.plot_acc.plot(pen=pg.mkPen('r'))
        self.curve_accy = self.plot_acc.plot(pen=pg.mkPen('g'))
        self.curve_accz = self.plot_acc.plot(pen=pg.mkPen('b'))
        self.curve_gyrx = self.plot_gyr.plot(pen=pg.mkPen('r'))
        self.curve_gyry = self.plot_gyr.plot(pen=pg.mkPen('g'))
        self.curve_gyrz = self.plot_gyr.plot(pen=pg.mkPen('b'))

    def disable_connection_buttons(self, enable):
        self.btn_con_serial.setEnabled(False)
        self.btn_dis_serial.setEnabled(False)
        self.btn_con_udp.setEnabled(False)
        self.btn_dis_udp.setEnabled(False)
        enable.setEnabled(True)

    def reset_connection_buttons(self):
        self.btn_con_serial.setEnabled(True)
        self.btn_dis_serial.setEnabled(False)
        self.btn_con_udp.setEnabled(True)
        self.btn_dis_udp.setEnabled(False)

    def __create_plot_update_timer(self):
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.callbacks['update_plot'])
        self.timer.start(int(1e3/self.plot_update_hz)) # ms

    def __create_param_tree(self):
        t = ParameterTree()
        params = []
        for group, groupchildren in self.params.items():
            children = []
            for child, opts in groupchildren.items():
                type, limits, default, suffix = opts
                children.append(Parameter.create(name=child, type=type, limits=limits, value=default, suffix=suffix))
            params.append(Parameter.create(name=group, type='group', children=children))
        p = Parameter.create(name='params', type='group', children=params)
        p.sigTreeStateChanged.connect(self.callbacks['update_params'])
        t.setParameters(p, showTop=False)
        return t

    def start(self):
        self.win.show()
        pg.exec()



class ESP32DataViewer:
    IMU_SAMPLE_RATE = 500

    # How many seconds of time-domain samples to plot
    SAMPLE_WINDOW_SEC = 5

    # Plotting frequency for time-domain signals
    SAMPLE_PLOT_FREQ_HZ = 20

    def __init__(self):
        self.driver = None

        callbacks = {
            'connect_serial': self._serial_connect,
            'disconnect_serial': self._serial_disconnect,
            'connect_udp': self._serial_connect,
            'disconnect_udp': self._serial_disconnect,
            'update_plot': self._update_plot,
            'update_params': self._update_params,
            'update_rgbled': self._rgbled_update,
            'toggle_stream': self._toggle_stream,
            'start_logging': self._start_logging,
            'stop_logging': self._stop_logging,
            'read_log': self._read_log,
        }

        params = {
            'IMU Parameters': {
                'Sample Rate': ('int', [0, 8000], self.IMU_SAMPLE_RATE, 'Hz'),
            },
            'Plotting Parameters': {
                'Window Length': ('float', [1, None], self.SAMPLE_WINDOW_SEC, 's')
            }
        }

        self.view = ViewControl(callbacks, params, plot_update_hz=self.SAMPLE_PLOT_FREQ_HZ)
        self.view.start()

    def _clear_plots(self):
        self.buf_t = None

    def _update_plot(self):
        if self.driver is None or self.buf_t is None:
            return

        # use this as a naive synchronization barrier
        if (len(self.buf_t) != len(self.buf_hz) or
                len(self.buf_t) != len(self.buf_acc) or len(self.buf_t) != len(self.buf_gyr)):
            return

        self.view.curve_hz.setData(self.buf_t, np.round(self.buf_hz))

        self.view.curve_accx.setData(self.buf_t, self.buf_acc[:,0])
        self.view.curve_accy.setData(self.buf_t, self.buf_acc[:,1])
        self.view.curve_accz.setData(self.buf_t, self.buf_acc[:,2])

        self.view.curve_gyrx.setData(self.buf_t, self.buf_gyr[:,0])
        self.view.curve_gyry.setData(self.buf_t, self.buf_gyr[:,1])
        self.view.curve_gyrz.setData(self.buf_t, self.buf_gyr[:,2])

    def _update_params(self, p, changes):
        for param, change, data in changes:
            if param.name() == 'Window Length':
                if param.value() < self.SAMPLE_WINDOW_SEC:
                    self.buf_t = None
                self.SAMPLE_WINDOW_SEC = param.value()

            if param.name() == 'Sample Rate':
                self.IMU_SAMPLE_RATE = param.value()
                if self.driver:
                    self.driver.sendRate(self.IMU_SAMPLE_RATE)

    def _serial_connect(self):
        self.last_t_us = 0
        self.buf_t = None
        self.buf_acc = None
        self.buf_gyr = None
        self.buf_hz = None

        # initialize serial communications
        self.driver = esp32imu.SerialDriver('/dev/ttyUSB0', 2000000)
        time.sleep(0.1) # wait for everything to initialize
        # self.driver.sendRate(self.IMU_SAMPLE_RATE)

        # Connect an IMU callback that will fire when a sample arrives
        self.driver.registerCallbackIMU(self._imu_cb)

        self.view.disable_connection_buttons(enable=self.view.btn_dis_serial)

    def _serial_disconnect(self):
        # clean up to prevent error or resource deadlock
        self.driver.unregisterCallbacks()
        del self.driver
        self.driver = None

        self.view.reset_connection_buttons()

    def _rgbled_update(self):
        if self.driver is None:
            return

        msg = esp32imu.RGBLedCmdMsg()
        msg.r = self.view.rslider.value()
        msg.g = self.view.gslider.value()
        msg.b = self.view.bslider.value()
        msg.brightness = self.view.islider.value()
        self.driver.sendRGBLedCmd(msg)

    def _toggle_stream(self):
        if self.driver is None:
            return

        if self.view.chk_stream.isChecked():
            self._clear_plots()

        msg = esp32imu.ConfigMsg()
        msg.stream = self.view.chk_stream.isChecked()
        msg.logging = False
        msg.readlog = False
        self.driver.sendConfig(msg)

    def _start_logging(self):
        if self.driver is None:
            return

        msg = esp32imu.ConfigMsg()
        msg.stream = self.view.chk_stream.isChecked()
        msg.logging = True
        msg.readlog = False
        self.driver.sendConfig(msg)

    def _stop_logging(self):
        if self.driver is None:
            return

        msg = esp32imu.ConfigMsg()
        msg.stream = self.view.chk_stream.isChecked()
        msg.logging = False
        msg.readlog = False
        self.driver.sendConfig(msg)

    def _read_log(self):
        if self.driver is None:
            return

        self._clear_plots()

        msg = esp32imu.ConfigMsg()
        msg.stream = False
        msg.logging = False
        msg.readlog = True
        self.driver.sendConfig(msg)

        self.view.chk_stream.setChecked(False)

    def _imu_cb(self, msg):
        dt = (msg.t_us - self.last_t_us) * 1e-6 # us to s
        if dt == 0:
            return
        self.last_t_us = msg.t_us
        hz = 1./dt
        self.Fs = hz
        # print('Got IMU {} at {} us ({:.0f} Hz): {:.2f}, {:.2f}, {:.2f}, \t {:.2f}, {:.2f}, {:.2f}'
        #         .format(msg.id, msg.t_us, hz,
        #                 msg.accel_x, msg.accel_y, msg.accel_z,
        #                 msg.gyro_x, msg.gyro_y, msg.gyro_z))

        if dt < 0:
            self._clear_plots()

        t = np.array([msg.t_us * 1e-6])
        acc = np.array([msg.accel_x, msg.accel_y, msg.accel_z]).reshape((1,3))
        gyr = np.array([msg.gyro_x, msg.gyro_y, msg.gyro_z]).reshape((1,3))

        if self.buf_t is None:
            self.buf_t = t
            self.buf_acc = acc
            self.buf_gyr = gyr
            self.buf_hz = np.array([hz])
        elif len(self.buf_t) > hz * self.SAMPLE_WINDOW_SEC and self.view.chk_stream.isChecked():
            self.buf_t = np.roll(self.buf_t, -1)
            self.buf_t[-1] = t

            self.buf_acc = np.roll(self.buf_acc, -1, axis=0)
            self.buf_acc[-1,:] = acc

            self.buf_gyr = np.roll(self.buf_gyr, -1, axis=0)
            self.buf_gyr[-1,:] = gyr

            self.buf_hz = np.roll(self.buf_hz, -1)
            self.buf_hz[-1] = hz
        else:
            self.buf_t = np.r_[self.buf_t, t]
            self.buf_acc = np.r_[self.buf_acc, acc]
            self.buf_gyr = np.r_[self.buf_gyr, gyr]
            self.buf_hz = np.r_[self.buf_hz, hz]




if __name__ == '__main__':
    viewer = ESP32DataViewer()
    # pg.exec()

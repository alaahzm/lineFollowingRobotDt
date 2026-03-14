from __future__ import annotations
import sys
from collections import deque
from typing import Deque
import numpy as np

try:
    from PyQt6 import QtWidgets, QtCore
except ModuleNotFoundError:
    from PySide6 import QtWidgets, QtCore

import pyqtgraph as pg

pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')


class RealTimePlotter:

    def __init__(self, buffer_size: int = 2000, update_frequency: int = 5):
        self.buffer_size      = buffer_size
        self.update_frequency = update_frequency
        self._counter         = 0

        self.t_buf   : Deque[float] = deque(maxlen=buffer_size)
        self.x_buf   : Deque[float] = deque(maxlen=buffer_size)
        self.y_buf   : Deque[float] = deque(maxlen=buffer_size)
        self.lat_buf : Deque[float] = deque(maxlen=buffer_size)
        self.vc_buf  : Deque[float] = deque(maxlen=buffer_size)

        self._app = (QtWidgets.QApplication.instance()
                     or QtWidgets.QApplication(sys.argv))
        pg.setConfigOptions(antialias=True)

        self.win = pg.GraphicsLayoutWidget(title="Line Follower - Real-Time Dashboard")
        self.win.resize(1200, 800)

        # Plot 1 top-left: XY trajectory
        self.traj_plot = self.win.addPlot(title="Robot Trajectory vs Path")
        self.traj_plot.setLabel('bottom', 'X position (m)')
        self.traj_plot.setLabel('left',   'Y position (m)')
        self.traj_plot.setAspectLocked(True)
        self.traj_plot.addLegend()
        self.curve_robot = self.traj_plot.plot(pen=pg.mkPen('b', width=2), name="Robot")
        self.curve_ref   = self.traj_plot.plot(
            pen=pg.mkPen('g', style=QtCore.Qt.PenStyle.DashLine, width=1),
            name="Reference path")
        ref_x = np.linspace(0, 35, 500)
        self.curve_ref.setData(ref_x, np.zeros(500))

        # Plot 2 top-right: lateral error vs time
        self.win.nextColumn()
        self.lat_plot = self.win.addPlot(title="Lateral Error vs Time")
        self.lat_plot.setLabel('bottom', 'Time (s)')
        self.lat_plot.setLabel('left',   'Lateral error (m)')
        self.lat_plot.addLine(y=0,     pen=pg.mkPen('k', style=QtCore.Qt.PenStyle.DotLine))
        self.lat_plot.addLine(y= 0.05, pen=pg.mkPen('g', style=QtCore.Qt.PenStyle.DashLine))
        self.lat_plot.addLine(y=-0.05, pen=pg.mkPen('g', style=QtCore.Qt.PenStyle.DashLine))
        self.curve_lat = self.lat_plot.plot(pen=pg.mkPen('r', width=1.5))

        # Plot 3 bottom-left: vel_cmd vs time
        self.win.nextRow()
        self.vc_plot = self.win.addPlot(title="Angular Velocity Command vs Time")
        self.vc_plot.setLabel('bottom', 'Time (s)')
        self.vc_plot.setLabel('left',   'vel_cmd (rad/s)')
        self.vc_plot.addLine(y=0, pen=pg.mkPen('k', style=QtCore.Qt.PenStyle.DotLine))
        self.curve_vc = self.vc_plot.plot(pen=pg.mkPen('m', width=1.5))

        # Plot 4 bottom-right: heading error vs time
        self.win.nextColumn()
        self.hd_plot = self.win.addPlot(title="Heading Error vs Time")
        self.hd_plot.setLabel('bottom', 'Time (s)')
        self.hd_plot.setLabel('left',   'Heading error (rad)')
        self.hd_plot.addLine(y=0, pen=pg.mkPen('k', style=QtCore.Qt.PenStyle.DotLine))
        self.curve_hd = self.hd_plot.plot(pen=pg.mkPen(color=(255, 140, 0), width=1.5))

        self.win.show()
        self._app.processEvents()

    def update_data(self,
                    t:         float,
                    x_current: float,
                    x_desired: float,
                    angle_deg: float,
                    force:     float) -> None:
        self.t_buf.append(t)
        self.x_buf.append(x_current)
        self.y_buf.append(x_desired)
        self.lat_buf.append(angle_deg)
        self.vc_buf.append(force)

        self._counter += 1
        if self._counter >= self.update_frequency:
            self._counter = 0
            self._redraw()

    def close(self) -> None:
        self.win.close()

    @staticmethod
    def _dq(dq: Deque[float]) -> np.ndarray:
        return np.fromiter(dq, dtype=float, count=len(dq))

    def _redraw(self) -> None:
        t_arr   = self._dq(self.t_buf)
        x_arr   = self._dq(self.x_buf)
        y_arr   = self._dq(self.y_buf)
        lat_arr = self._dq(self.lat_buf)
        vc_arr  = self._dq(self.vc_buf)

        self.curve_robot.setData(x_arr, y_arr)
        self.curve_lat.setData(t_arr, lat_arr)
        self.curve_vc.setData(t_arr, vc_arr)
        self.curve_hd.setData(t_arr, y_arr)

        self._app.processEvents()
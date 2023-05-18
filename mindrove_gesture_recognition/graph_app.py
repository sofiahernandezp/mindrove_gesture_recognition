import argparse
import logging

import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph.Qt import QtGui
from PyQt5 import QtGui

import sys
from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds
from mindrove.data_filter import DataFilter, FilterTypes, DetrendOperations, NoiseTypes
from PyQt5 import QtGui, QtWidgets


# ...



class Graph:
    def __init__(self, board_shim):
        self.app = QtWidgets.QApplication(sys.argv)
        self.board_id = board_shim.get_board_id()
        self.board_shim = board_shim
        self.emg_channels = BoardShim.get_emg_channels(self.board_id)
        self.sampling_rate = BoardShim.get_sampling_rate(self.board_id)
        self.update_speed_ms = 50
        self.window_size = 4
        self.num_points = self.window_size * self.sampling_rate
        #self.app = QtGui.QApplication([])
        self.win = pg.GraphicsLayoutWidget(title='Mindrove Plot',size=(800, 600))
        self.win.show()
        self._init_timeseries()

        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(self.update_speed_ms)
        #QtGui.QApplication.instance().exec_()
        self.app.instance().exec_()


    def _init_timeseries(self):
        self.plots = list()
        self.curves = list()
        for i in range(len(self.emg_channels)):
            p = self.win.addPlot(row=i,col=0)
            p.showAxis('left', True)
            p.setMenuEnabled('left', False)
            p.showAxis('bottom', False)
            p.setMenuEnabled('bottom', False)
            if i == 0:
                p.setTitle('Real-time TimeSeries Plot')
            self.plots.append(p)
            curve = p.plot()
            self.curves.append(curve)

    def update(self):
        data = self.board_shim.get_current_board_data(self.num_points)
        board_id = BoardIds.SYNTHETIC_BOARD.value
        for count, channel in enumerate(self.emg_channels):
            DataFilter.remove_environmental_noise(data[channel], BoardShim.get_sampling_rate(board_id), NoiseTypes.FIFTY.value)
            self.curves[count].setData(data[channel].tolist())

        self.app.processEvents()


def main():

    BoardShim.enable_dev_board_logger()
    logging.basicConfig(level=logging.DEBUG)


    params = MindRoveInputParams()

    try:
        board_shim = BoardShim(BoardIds.MINDROVE_WIFI_BOARD, params)
        board_shim.prepare_session()
        board_shim.start_stream(450000)
        Graph(board_shim)
    except BaseException:
        logging.warning('Exception', exc_info=True)
    finally:
        logging.info('End')
        if board_shim.is_prepared():
            logging.info('Releasing session')
            board_shim.release_session()


if __name__ == '__main__':
    main()
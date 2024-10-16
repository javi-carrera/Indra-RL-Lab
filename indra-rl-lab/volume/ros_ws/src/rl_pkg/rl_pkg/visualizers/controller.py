import queue
import time
import numpy as np

from typing import List
from queue import Queue
import pyqtgraph as pg


def clear_queue(q: Queue):
    with q.mutex:
        q.queue.clear()
        q.all_tasks_done.notify_all()
        q.unfinished_tasks = 0

class RewardPlot:
    def __init__(self, *args, **kwargs):
        self.cum_reward_plt = pg.PlotItem(*args, **kwargs)
        self.cum_reward_plt.setTitle("Reward")
        self.cum_reward_plt.setLabel("left", "Value")
        self.cum_reward_plt.setLabel("bottom", "Time")

        self.total_reward_plt = pg.PlotItem(*args, **kwargs)
        self.total_reward_plt.setTitle("Total Reward")
        self.total_reward_plt.setLabel("left", "Value")
        self.total_reward_plt.setLabel("bottom", "Time")
        
    def init(self):
        self.total_reward = 0
        self.data = np.zeros(100)
        self.tot_reward_data = np.array([0])
        self.curve = self.cum_reward_plt.plot(self.data, pen=pg.mkPen(color=(0, 255, 0)))
        self.total_curve = self.total_reward_plt.plot(self.tot_reward_data, pen=pg.mkPen(color=(0, 255, 0)))

    def clear(self):    
        self.curve.clear()
        self.total_curve.clear()

    def update(self, shift, reward):
        self.data = np.roll(self.data, -shift)
        self.data[-shift:] = reward 
        self.curve.setData(self.data, downsample=0)
        self.tot_reward_data = np.append(self.tot_reward_data, self.total_reward)
        self.total_curve.setData(self.tot_reward_data, downsample=0)


class ObservationPlot:
    def __init__(self, *args, **kwargs):
        self.plt = pg.PlotItem(*args, **kwargs) 
            
        self.scaterplot = pg.ScatterPlotItem(pen="w")
        self.plt.addItem(self.scaterplot)

        self.pointList = []

        self.agents_pos = pg.Point(0.0, 0.0)
        self.pointList.append(self.agents_pos)

        startpoint = self.agents_pos
        player_vel = np.array([-1.0, 0.5])  # Moving right
        player_vel += 1e-6
        vel_mag = np.sqrt(player_vel[0]**2 + player_vel[1]**2)
        endpoint = np.array([startpoint[0] + player_vel[0], startpoint[1] + player_vel[1]])

        self.line = pg.PlotCurveItem(x=[startpoint[0], endpoint[0]], y=[startpoint[1], endpoint[1]], pen=pg.mkPen('g', width=2))
        self.plt.addItem(self.line)

        self.player_arrow = pg.ArrowItem(pos=endpoint, angle=180 + np.degrees(np.arctan2(player_vel[1], player_vel[0])), headLen=vel_mag / 5, headWidth=vel_mag / 20, pen=pg.mkPen('g', width=2), pxMode=False,)
        self.plt.addItem(self.player_arrow)

        self.bot_pos = pg.Point(1.0, 1.0)
        self.pointList.append(self.bot_pos)

        self.scaterplot.addPoints(pos=self.pointList, size=20, symbol='o', brush=['g', 'r'])
        
        startpoint = self.bot_pos
        bot_vel = np.array([0.0, 0.0])     # Moving diagonally down-left
        bot_vel += 1e-6
        vel_mag = np.sqrt(bot_vel[0]**2 + bot_vel[1]**2)
        endpoint = np.array([startpoint[0] + bot_vel[0], startpoint[1] + bot_vel[1]])
        self.botLine = pg.PlotCurveItem(x=[startpoint[0], endpoint[0]], y=[startpoint[1], endpoint[1]], pen=pg.mkPen('r', width=2))
        self.bot_arrow = pg.ArrowItem(pos=endpoint, angle=180 + np.degrees(np.arctan2(bot_vel[1], bot_vel[0])), headLen=vel_mag / 5, headWidth=vel_mag / 20, pen=pg.mkPen('r', width=2), pxMode=False)
        # bot_arrow.setPos(self.bot_pos[0], self.bot_pos[1])
        # bot_arrow.setAngle(np.degrees(np.arctan2(bot_vel[1], bot_vel[0])))  # Set arrow direction
        self.plt.addItem(self.botLine)
        self.plt.addItem(self.bot_arrow)

        self.turret_line = pg.PlotCurveItem(x=[0, 0], y=[0, 0], pen=pg.mkPen('b', width=2))
        self.plt.addItem(self.turret_line)
        

    def init(self):
        self.plt.getViewBox().enableAutoRange(False)
        self.plt.getViewBox().setAspectLocked(True)
        self.plt.getViewBox().setXRange(-1, 1, padding=0.5)
        self.plt.getViewBox().setYRange(-1, 1, padding=0.5)

    def clear(self):
        pass

    def update(self, observation=None):
        if observation is None:
            return
        target_relative_position_normalized = observation[0:2]
        linear_velocity_normalized = observation[2]
        target_linear_velocity_normalized_x = observation[3]
        target_linear_velocity_normalized_y = observation[4]
        # angular_velocity_normalized = observation[5]
        # lidar_ranges_normalized = observation[6:6+20]
        # curremt_health_normalized = observation[26]
        # current_target_health_normalized = observation[27]
        turret_angle_sin = observation[28]
        turret_angle_cos = observation[29]
        # turret_cooldown_normalized = observation[30]
        # turret_has_fired = observation[31]

        
        self.bot_pos.setX(target_relative_position_normalized[0])
        self.bot_pos.setY(target_relative_position_normalized[1])
        self.scaterplot.clear()
        self.scaterplot.addPoints(pos=self.pointList, size=20, symbol='o', brush=['g', 'r'])
        
        vel = np.array([0, linear_velocity_normalized]) + 1e-6
        vel_mag = np.sqrt(vel[0]**2 + vel[1]**2)
        startpoint = self.agents_pos
        endpoint = np.array([startpoint[0] + vel[0], startpoint[1] + vel[1]])
        self.line.setData(x=[startpoint[0], endpoint[0]], y=[startpoint[1], endpoint[1]])
        self.player_arrow.setPos(endpoint[0], endpoint[1])
        self.player_arrow.setStyle(angle=180 + np.degrees(np.arctan2(vel[1], vel[0])), headLen=vel_mag / 5, headWidth=vel_mag / 20)

        target_vel = np.array([target_linear_velocity_normalized_x, target_linear_velocity_normalized_y]) + 1e-6
        target_vel_mag = np.sqrt(target_vel[0]**2 + target_vel[1]**2) + 1e-3
        startpoint = self.bot_pos
        endpoint = np.array([startpoint[0] + target_vel[0], startpoint[1] + target_vel[1]])
        self.botLine.setData(x=[startpoint[0], endpoint[0]], y=[startpoint[1], endpoint[1]])
        self.bot_arrow.setPos(endpoint[0], endpoint[1])
        self.bot_arrow.setStyle(angle=180 + np.degrees(np.arctan2(target_vel[1], target_vel[0])), headLen=target_vel_mag / 5, headWidth=target_vel_mag / 20)


        # turret_vector = np.array([turret_angle_sin, turret_angle_cos]) * 
        self.turret_line.setData(x=[0, turret_angle_sin], y=[0, turret_angle_cos])



class PlotController:
    def __init__(self, data_queue: Queue = None, *args, **kwargs):
        self.data_queue = data_queue

        self.observation_plot = ObservationPlot()
        self.reward_plot = RewardPlot()

        self.plots: List[pg.PlotItem] = []
        # self.plots.extend([self.observation_plot.plt, self.reward_plot.cum_reward_plt, self.reward_plot.total_reward_plt])
        self.plots.extend([self.reward_plot.cum_reward_plt, self.reward_plot.total_reward_plt])

    def start(self):
        self.clear_and_init_plot()

    def stop(self):
        pass

    def update(self):
        try:
            observation = None
            reward_buffer = np.array([])
            while True:
                try:
                    observation, reward, dones = self.data_queue.get_nowait()
                    self.reward_plot.total_reward += reward
                    reward = np.array(reward).reshape(-1)
                    reward_buffer = np.concatenate((reward_buffer, reward), axis=0)
                except queue.Empty:
                    break
            
            self.observation_plot.update(observation)

            shift = len(reward_buffer)
            if shift == 0:
                return
            elif shift > len(self.reward_plot.data):
                shift = len(self.reward_plot.data)
            self.reward_plot.update(shift, reward_buffer[-shift:])

            if any(dones):
                self.clear_and_init_plot()

        except Exception as e:
            print("PlotController Error: ", e)

    def init(self):
        self.observation_plot.init()
        self.reward_plot.init()

    def clear(self):
        self.observation_plot.clear()
        self.reward_plot.clear()

    def clear_and_init_plot(self):
        self.clear()
        self.init()
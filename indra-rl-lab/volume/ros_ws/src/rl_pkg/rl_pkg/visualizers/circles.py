import sys
import numpy as np
import pyqtgraph as pg
from PySide6.QtWidgets import QApplication

# Function to plot agents and their velocities
def plot_agents(player_pos, bot_pos, player_vel, bot_vel):
    # Clear previous items
    plot_widget.clear()

    # Plot the player agent (tank) at (0,0)
    player_circle = pg.CircleROI([[-0.05, -0.05]], pen=pg.mkPen('g', width=2), movable=False)
    plot_widget.addItem(player_circle)

    # Plot the bot agent (tank) at (x, y)
    # bot_circle = pg.CircleROI([[bot_pos[0]-0.05, bot_pos[1]-0.05], [0.1, 0.1]], pen=pg.mkPen('r', width=2), movable=False)
    # plot_widget.addItem(bot_circle)

    # Draw velocity arrows
    player_arrow = pg.ArrowItem(angle=90, tipAngle=30, tipLength=0.5, pen=pg.mkPen('g', width=2))
    player_arrow.setPos(player_pos[0], player_pos[1])
    player_arrow.setAngle(np.degrees(np.arctan2(player_vel[1], player_vel[0])))  # Set arrow direction
    plot_widget.addItem(player_arrow)

    bot_arrow = pg.ArrowItem(angle=90, tipAngle=30, tipLength=0.5, pen=pg.mkPen('r', width=2))
    bot_arrow.setPos(bot_pos[0], bot_pos[1])
    bot_arrow.setAngle(np.degrees(np.arctan2(bot_vel[1], bot_vel[0])))  # Set arrow direction
    plot_widget.addItem(bot_arrow)

    # Update plot limits
    plot_widget.setXRange(-10, 10, padding=0)
    plot_widget.setYRange(-10, 10, padding=0)
    plot_widget.setAspectLocked(True)  # Keep aspect ratio

# Initialize the QApplication
app = QApplication(sys.argv)

# Create a plot widget
plot_widget = pg.plot(title="Agent Positions and Velocities")

# Example positions and velocities
player_position = np.array([0, 0])
bot_position = np.array([5, 5])
player_velocity = np.array([1, 0])  # Moving right
bot_velocity = np.array([-1, 1])     # Moving diagonally down-left

# Plot agents and velocities
plot_agents(player_position, bot_position, player_velocity, bot_velocity)

# Start the Qt event loop
sys.exit(app.exec())

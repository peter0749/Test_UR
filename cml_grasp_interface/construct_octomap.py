import sys
from .pyrobot_handler import PyRobotHandler

if __name__ == '__main__':
    bot_name = sys.argv[1]
    camera_name = sys.argv[2]
    bot = PyRobotHandler(bot=bot_name, camera=camera_name)
    bot.disable_octomap()
    bot.update_octomap()
    bot.enable_octomap()
    bot.save_octomap()

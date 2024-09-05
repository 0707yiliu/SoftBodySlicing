import yaml
import numpy as np

with open('config.yml', 'r', encoding="utf-8") as f:
    config = yaml.safe_load(f) # import config from yaml

traj_name = config['traj_name']
traj_pos = np.load('tele_control/data/' + '20240905150350' + 'slaveposcutoff' + '.npy')
traj_force = np.load('tele_control/data/' + '20240905150350' + 'slaveforcecutoff' + '.npy')


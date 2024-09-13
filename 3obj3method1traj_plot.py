import matplotlib.pyplot as plt
import numpy as np

import yaml
with open('config.yml', 'r', encoding="utf-8") as f:
    config = yaml.safe_load(f) # import config from yaml

def traj_load(slaveforce, slavepos):
    # loading tele-trajectory from slave datasets for DMPs
    traj_force = np.load('tele_control/data/' + slaveforce + '.npy')
    traj_pos = np.load('tele_control/data/' + slavepos + '.npy')
    traj_pos = np.delete(traj_pos, 0, 0)
    traj_force = np.delete(traj_force, 0, 0)
    return traj_force, traj_pos

def traj_load_cut_data(slaveforce, slavepos):
    # loading tele-trajectory from slave datasets for DMPs
    traj_force = np.load('cut_data/' + slaveforce + '.npy')
    traj_pos = np.load('cut_data/' + slavepos + '.npy')
    traj_pos = np.delete(traj_pos, [0, 1, 2, 3], 0)
    traj_force = np.delete(traj_force, [0, 1, 2, 3], 0)
    return traj_force, traj_pos

high, width = 3, 15
my_dpi=100
row = 2
col = 3
_title = ['X', 'Y', 'Z', 'X', 'Y', 'Z']
_colors = ['#DB4D6D', '#7B90D2', '#6A4028', '#BEC23F']
# -----------------------------
cutoff_pos = '20240912133152slaveposcutoff'
cutoff_force = '20240912133152slaveforcecutoff'
cutoff_force, cutoff_pos = traj_load(cutoff_force, cutoff_pos)
fig, axs = plt.subplots(row, col, figsize=(1500/my_dpi, 300/my_dpi),dpi=my_dpi, sharex=True, sharey=False)
cutoff_time = np.linspace(0, cutoff_pos.shape[0]/100, cutoff_pos.shape[0])
#3objs3models
orange_y4_ = '20240912170153poscutofforange'
apple_y4 = '20240912172445poscutoffapple'
cucumber_y4 = '20240912173029poscutoffcucumber'
obj = ['20240912170153', '20240912172445', '20240912173029']
states = ['armforce', 'armpos', 'force', 'pos']
act = 'cutoff'
objname = ['orange', 'apple', 'cucumber']# relate to obj
obj_datas = []
time_datas = []
for i in range(3):
    obj_ftsensordata, obj_qdata = traj_load_cut_data(
        obj[i] + states[0] + act + objname[i],
        obj[i] + states[1] + act + objname[i]
    )
    obj_ftcompensate, obj_qcompensate = traj_load_cut_data(
        obj[i] + states[2] + act + objname[i],
        obj[i] + states[3] + act + objname[i]
    )
    timelen = np.linspace(0, obj_ftsensordata.shape[0]/100, obj_ftsensordata.shape[0])
    time_datas.append(timelen)
    obj_datas.extend([obj_ftsensordata, obj_qdata, obj_ftcompensate, obj_qcompensate])
#plot pos
# for k in range(3): # 3objs
#     for i in range(row):
#         for j in range(col):
#             index = (i + 1) * j if i == 0 else i * j + col
#
for i in range(row):
    for j in range(col):
        if i == 0:
            axs[i, j].set_title(_title[j])
            index = (i + 1) * j
            if j == 0:
                axs[i, j].set_ylabel(r'position($m$)')
        if j == 0 and i == 1:
            axs[i, j].set_ylabel(r'rotation($rad$)')
        if i == 1:
            index = i * j + col
            axs[i, j].set_xlabel(r'time($s$)')
        demo = axs[i, j].plot(cutoff_time, cutoff_pos[:, index], label=r"$y_1$", color=_colors[0], ls="-", lw=5, alpha=0.5)
        o1_q = axs[i, j].plot(time_datas[0], obj_datas[1][:, index], label=objname[0], ls="-", lw=2)
        # o1_qc = axs[i, j].plot(time_datas[0], obj_datas[3][:, index], label=r"obj1_qcompen", ls="-", lw=2)
        o2_q = axs[i, j].plot(time_datas[1], obj_datas[5][:, index], label=objname[1], ls="-", lw=2)
        # o2_qc = axs[i, j].plot(time_datas[1], obj_datas[7][:, index], label=r"obj2_qcompen", ls="-", lw=2)
        o3_q = axs[i, j].plot(time_datas[2], obj_datas[9][:, index], label=objname[2], ls="-", lw=2)
        # o3_qc = axs[i, j].plot(time_datas[2], obj_datas[11][:, index], label=r"obj3_qcompen", ls="-", lw=2)

axs[-1, -1].legend()

#force

plt.show()

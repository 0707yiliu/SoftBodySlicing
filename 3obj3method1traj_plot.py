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


#force ----------------------------------------
fig, axs = plt.subplots(row, col, figsize=(1500/my_dpi, 300/my_dpi),dpi=my_dpi, sharex=True, sharey=False)
cutoff_time = np.linspace(0, cutoff_force.shape[0]/100, cutoff_force.shape[0])
# 20240916175334poscutofforange.npy
# obj = ['20240912170153', '20240912172445', '20240912173029']
obj = ['20240916175334', '20240912172445', '20240912173029']
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
# initial filter param

from scipy.signal import savgol_filter
from scipy import signal
def butter_lowpass_filtfilt(data, order, cutoff, fs):
    wn = 2 * cutoff / fs
    b, a = signal.butter(order, wn, 'lowpass', analog=False)
    output = signal.filtfilt(b, a, data, axis=0)
    return output
def savgol_smooth(data):
    col, row = data.shape
    windows = 123
    order = 5
    filtereddata = np.zeros_like(data)
    for i in range(row):
        filtereddata[:, i] = savgol_filter(data[:, i], windows, order, mode= 'nearest')
        filtereddata[:, i] = butter_lowpass_filtfilt(filtereddata[:, i], 4, 4, 50)
    return filtereddata
cutoff_force = savgol_smooth(cutoff_force)
for i in range(len(obj_datas)):
    obj_datas[i] = savgol_smooth(obj_datas[i])

for i in range(row):
    for j in range(col):
        if i == 0:
            axs[i, j].set_title(_title[j])
            index = (i + 1) * j
            if j == 0:
                axs[i, j].set_ylabel(r'force($N$)')
        if j == 0 and i == 1:
            axs[i, j].set_ylabel(r'torque($N\cdot m$)')
        if i == 1:
            index = i * j + col
            axs[i, j].set_xlabel(r'time($s$)')
        demo = axs[i, j].plot(cutoff_time, cutoff_force[:, index], label=r"$y_1$", color=_colors[0], ls="-", lw=2, alpha=0.5)
        o1_qc = axs[i, j].plot(time_datas[0], obj_datas[0][:, index], label='pure DMP', ls="-", lw=2)
        o1_q = axs[i, j].plot(time_datas[0], -obj_datas[2][:, index], label=r"pure DMP ForceComp", ls="-", lw=2)
        # o2_qc = axs[i, j].plot(time_datas[1], obj_datas[4][:, index], label=objname[1], ls="-", lw=2)
        # o2_q = axs[i, j].plot(time_datas[1], -obj_datas[6][:, index], label=r"obj2ForceComp", ls="-", lw=2)
        o3_qc = axs[i, j].plot(time_datas[2], obj_datas[8][:, index], label='cr-DMP', ls="-", lw=2)
        o3_q = axs[i, j].plot(time_datas[2], -obj_datas[10][:, index], label=r"cr-DMP ForceComp", ls="-", lw=2)

axs[-1, -1].legend(bbox_to_anchor=(1, 1.5))

# #filter and smooth test -----------------------------------------------
# from scipy import signal
# def butter_lowpass_filtfilt(data, order, cutoff, fs):
#     wn = 2 * cutoff / fs
#     b, a = signal.butter(order, wn, 'lowpass', analog=False)
#     output = signal.filtfilt(b, a, data, axis=0)
#     return output
# order=2
# cutoff=4
# fs=100
# def sample_from_data(data, index):
#     get = int(data.shape[0]) // index
#     sampled_data = np.array([])
#     for i in range(get):
#         sampled_data = np.append(sampled_data, data[i*index])
#     return sampled_data
# sampleindex = 1
# sampledata = sample_from_data(cutoff_force[:, 2], sampleindex)
# sampledatac = sample_from_data(obj_datas[4][:, 2], sampleindex)
# from scipy.signal import savgol_filter
# sampledata = savgol_filter(sampledata, 123, 5, mode= 'nearest')
# sampledatac = savgol_filter(sampledatac, 123, 5, mode= 'nearest')# smooth func
# filterforce = butter_lowpass_filtfilt(sampledata, order, cutoff, fs)
# o1forcefilter = butter_lowpass_filtfilt(sampledatac, 6, 4, 50)
# o1forcefilter2 = butter_lowpass_filtfilt(sampledatac, 4, 4, 50)
# plt.figure(3)
# # plt.plot(filterforce, label=r"$y_1$",)
# plt.plot(filterforce, label=r"1", lw=4, alpha=0.5)
# plt.plot(o1forcefilter, label=r"2",)
# plt.plot(o1forcefilter2, label=r"3",)
# plt.legend()
# ----------------------------------------------------------------------------
plt.show()

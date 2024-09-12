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

traj_name = config['traj_name']
master_force = config['slave_file_date'] + 'masterforce'# for virtual DMP virtual force
master_pos = config['slave_file_date'] + 'masterpos'
slave_force = config['slave_file_date'] + 'slaveforce' + traj_name# for virtual DMP virtual force
slave_pos = config['slave_file_date'] + 'slavepos' + traj_name

traj_force_m, traj_pos_m = traj_load(master_force, master_pos)
traj_force_s, traj_pos_s = traj_load(slave_force, slave_pos)

diffnum = traj_pos_s.shape[0] - traj_force_m.shape[0]
if diffnum > 0:
    timelen = traj_force_m.shape[0] / 100
    traj_pos_s = np.delete(traj_pos_s, np.s_[:diffnum],0)
    traj_force_s = np.delete(traj_force_s, np.s_[:diffnum], 0)
elif diffnum < 0:
    timelen = traj_pos_s.shape[0] / 100
    traj_pos_m = np.delete(traj_pos_m, np.s_[:diffnum], 0)
    traj_force_m = np.delete(traj_force_m, np.s_[:diffnum], 0)
else:
    timelen = traj_pos_s.shape[0] / 100

newtick = np.linspace(0, timelen, traj_pos_s.shape[0])

high, width = 3, 15
my_dpi=100
row = 2
col = 3
fig, axs = plt.subplots(row, col, figsize=(1500/my_dpi, 300/my_dpi),dpi=my_dpi, sharex=True, sharey=False)
# plt.figure(1, figsize=(width, high))
_title = ['X', 'Y', 'Z', 'X', 'Y', 'Z']
_colors = ['#DB4D6D', '#7B90D2', '#6A4028', '#BEC23F']
for i in range(row):
    for j in range(col):
        axs2 = axs[i, j].twinx()
        if i == 0:
            axs[i, j].set_title(_title[j])
            index = (i + 1) * j
            if j == 0:
                axs[i, j].set_ylabel(r'position($m$)')
            if j == col-1:
                axs2.set_ylabel(r'force($N$)')
        if j == 0 and i == 1:
            axs[i, j].set_ylabel(r'rotation($rad$)')
        if i == 1:
            index = i * j + col
            axs[i, j].set_xlabel(r'time($s$)')
            if j == col-1:
                axs2.set_ylabel(r'torque($N\cdot m$)')
        l1 = axs[i, j].plot(newtick, traj_pos_s[:, index], label=r"$q_s$", color=_colors[0], ls="-", lw=2)
        l2 = axs[i, j].plot(newtick, traj_pos_m[:, index], label=r"$q_m$", color=_colors[1], lw=5, alpha=0.5)
        l3 = axs2.plot(newtick, traj_force_s[:, index], label=r"$F_s$", color=_colors[2], ls="-", lw=2)
        l4 = axs2.plot(newtick, traj_force_m[:, index], label=r"$F_m$", color=_colors[3], lw=2, alpha=0.5)
lns = l1 + l2 + l3 + l4
labs = [l.get_label() for l in lns]
axs2.legend(lns, labs, loc=0)


# -----------------------------
cutoff_pos = '20240909180249slaveposcutoff'
cutoff_force = '20240909180249slaveforcecutoff'
cutin_pos = '20240906133602slaveposcutin'
cutin_force = '20240906133602slaveforcecutin'
slice_pos = '20240906134635slaveposslice'
slice_force = '20240906134635slaveforceslice'
dig_pos = '20240909120507slaveposcutoff' # fake
dig_force = '20240909120507slaveforcecutoff'
cutoff_force, cutoff_pos = traj_load(cutoff_force, cutoff_pos)
cutin_force, cutin_pos = traj_load(cutin_force, cutin_pos)
slice_force, slice_pos = traj_load(slice_force, slice_pos)
dig_force, dig_pos = traj_load(dig_force, dig_pos)
fig, axs = plt.subplots(row, col, figsize=(1500/my_dpi, 300/my_dpi),dpi=my_dpi, sharex=True, sharey=False)
cutoff_time = np.linspace(0, cutoff_pos.shape[0]/100, cutoff_pos.shape[0])
cutin_time = np.linspace(0, cutin_pos.shape[0]/100, cutin_pos.shape[0])
slice_time = np.linspace(0, slice_pos.shape[0]/100, slice_pos.shape[0])
dig_time = np.linspace(0, dig_pos.shape[0]/100, dig_pos.shape[0])
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
        l1 = axs[i, j].plot(cutoff_time, cutoff_pos[:, index], label=r"$y_1$", color=_colors[0], ls="-", lw=2)
        l2 = axs[i, j].plot(cutin_time, cutin_pos[:, index], label=r"$y_2$", color=_colors[1], ls="-", lw=2)
        l3 = axs[i, j].plot(slice_time, slice_pos[:, index], label=r"$y_3$", color=_colors[2], ls="-", lw=2)
        l4 = axs[i, j].plot(dig_time, dig_pos[:, index], label=r"$y_4$", color=_colors[3], ls="-", lw=2)
axs[-1, -1].legend()
plt.show()

from tele_control.register_robot import RegisterRobot
import yaml
with open('config.yml', 'r', encoding="utf-8") as f:
    config = yaml.safe_load(f)  # import config from yaml
myrobot = RegisterRobot(config['controlled_robot_ip'])
myrobot.zeroforce()
curr_pos, curr_euler = myrobot.getToolPos()
print(curr_pos, curr_euler)
# q = [0.5611623525619507, -1.3226483774236222, 2.1882827917682093, -2.4357482395567835, -1.559420410786764, 1.3036015033721924]
# myrobot.servoJ(q, 5, 0.03, 500)
# without knife
# [-0.16992376 -0.26399856 -0.01603487] [ 3.13025205e+00 -2.89415282e-04 -2.46277454e-05]
# take knife
# [-0.24890595 -0.18501712 -0.01696009] [ 3.13382234 -0.00819066 -0.78536004]


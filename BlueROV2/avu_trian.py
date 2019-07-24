import os
import argparse
import time
import torch
import numpy as np
from pymavlink import mavutil
from math import sin, cos, pi

import TD3
import utils
import pid_controller
from reward_function import reward_three_dof
from pid_controller import PID, depth_data, angle_data, arm, disarm, force_distribute, pid_for_init, change_mode


def save_reward(filename, r):
    with open(filename, 'a') as f:
        f.writelines(r+'\n')


def save_data(filename, depth_data, roll_data, pitch_data, pwm):
    with open(filename, 'a') as f:
        f.writelines(str(depth_data)[:5] + '  ' + str(roll_data)[:7] + '  ' + str(pitch_data)[:7] + '  ' + str(pwm[0])
                     + '  ' + str(pwm[1])+'  '+str(pwm[2])+'  '+str(pwm[3])+'\n')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy_name", default="TD3", help='Policy name')
    parser.add_argument("--seed", default=0, type=int, help='PyTorch and Numpy seeds')
    parser.add_argument('--max-steps', type=int, default=600, help='the max time steps per episode')
    parser.add_argument('--max-episode', type=int, default=1200, help='the max length of episode to train the agent')
    parser.add_argument("--start_time_steps", default=60000, type=int, help="how many time steps random policy run for")
    parser.add_argument("--explore_noise", default=0.1, type=float, help='Std of Gaussian exploration noise')
    parser.add_argument("--batch_size", default=100, type=int, help='Batch size for both actor and critic')
    parser.add_argument('--replay-size', type=int, default=120000, help='the size of replay buffer')
    parser.add_argument('--replay_size2', type=int, default=60000, help='the size of replay buffer2')
    parser.add_argument("--discount", default=0.99, type=float, help='Discount factor')
    parser.add_argument("--tau", default=0.005, type=float, help='Target network update rate')
    parser.add_argument("--policy_noise", default=0.3, type=float, help='Noise added to target policy')
    parser.add_argument("--noise_clip", default=0.5, type=float, help='Range to clip target policy noise')
    parser.add_argument("--policy_freq", default=2, type=int, help='Frequency of delayed policy updates')
    parser.add_argument('--save-dir1', type=str, default='saved_models_actor/', help='actor model')
    parser.add_argument('--save-dir2', type=str, default='saved_models_critic/', help='critic model')
    args = parser.parse_args()

    if not os.path.exists(args.save_dir1):
        os.mkdir(args.save_dir1)
    if not os.path.exists(args.save_dir2):
        os.mkdir(args.save_dir2)
    model_path_actor = args.save_dir1 + 'BlueRovTD3_actor' + '/'
    model_path_critic = args.save_dir2 + 'BlueRovTD3_critic' + '/'
    if not os.path.exists(model_path_actor):
        os.mkdir(model_path_actor)
    if not os.path.exists(model_path_critic):
        os.mkdir(model_path_critic)

    # Set seeds
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    state_dim = 9
    action_dim = 4
    max_action = 34.3

    model_path1 = model_path_actor + '/model.pt'
    model_path2 = model_path_critic + '/model.pt'
    policy = TD3.TD3(state_dim, action_dim, max_action)

    replay_buffer = utils.ReplayBuffer(args.replay_size)
    replay_buffer2 = utils.ReplayBuffer2(args.replay_size2)
    total_steps = 0
    episode_num = 0

    master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
    master.wait_heartbeat()

    # file names
    reward_file = 'reward_blue.txt'
    data_file = 'data.txt'
    save_data(data_file, 'Depth', 'Roll', 'Pitch', ['F1', 'F2', 'F3', 'F4'])
    state_exp = np.array([2.0, 0.000, 0.000])
    depth_hold = PID(65, 0.9, 0.01)
    pitch_hold = PID(10, 0.9, 0.01)
    roll_hold = PID(10, 0.9, 0.01)
    arm()
    # change_mode('STABILIZE')
    for episode_idx in range(args.max_episode):
        arm()
        episode_reward = 0
        counter = 0
        # dep_init = np.random.uniform(0.5, 4.0)
        # r_init = np.random.uniform(-0.1, 0.1)
        # p_init = np.random.uniform(-0.1, 0.1)
        # pid_for_init(dep_init, p_init, r_init)
        # arm()
        # print('1')
        # change_mode('ALT_HOLD')
        f_start = [0, 0, 0, 0]
        depth_start = depth_data()
        pitch_start, roll_start = angle_data()
        state = np.array([depth_start, roll_start, pitch_start, 0, 0, 0, 2.50-depth_start, 0.00-roll_start, 0.00-pitch_start])
        print('\n' + '.......................................................................')
        print('State_init: {}'.format([state[:3]]))
        for ep_step in range(args.max_steps):
            global depth, p_angle, r_angle
            t1 = time.time()
            # if len(replay_buffer.storage) > args.start_time_steps:
            action = policy.select_action(state)
            action = (action + np.random.normal(0, args.explore_noise, size=action_dim)).clip(-max_action, max_action)
            # else:
            #     depth_hold.set_target(state_exp[0])
            #     pitch_hold.set_target(state_exp[1])
            #     roll_hold.set_target(state_exp[2])
            #     depth = depth_data()
            #     p_angle, r_angle = angle_data()
            #     depth_hold.calcu(depth)
            #     pitch_hold.calcu(p_angle)
            #     roll_hold.calcu(r_angle)
            #     f = force_distribute(depth_hold.output, pitch_hold.output, roll_hold.output)
            #     thruster = []
            #     for i in range(4):
            #         thruster.append(f[i, 0])
            #     thruster = np.clip(thruster, -34.3, 34.3)
            #     action = [thruster[0], thruster[1], thruster[2], thruster[3]]
            pwm = [a / 34.3 * 300 + 1500 for a in action]
            pid_controller.set_motor_pwm(4, pwm[0])
            pid_controller.set_motor_pwm(5, pwm[1])
            pid_controller.set_motor_pwm(6, pwm[2])
            pid_controller.set_motor_pwm(7, pwm[3])
            f_law = f_start
            f_start = action

            depth2 = depth_data()
            p_angle2, r_angle_2 = angle_data()
            t2 = time.time()
            delta_t = t2 - t1
            v_z, v_theta, v_phi = (depth2-depth_start)/delta_t, (p_angle2-pitch_start)/delta_t, (r_angle_2-roll_start)/delta_t
            state2 = np.array([depth2, r_angle_2, p_angle2, v_z, v_phi, v_theta, 2.50-depth2, 0.000-r_angle_2,
                               0.000-p_angle2])
            save_data(data_file, depth2, r_angle_2, p_angle2, action)  # 保存数据
            # 计算误差
            d_z = state2[0] - state_exp[0]
            d_phi = state2[1] - state_exp[1]
            d_theta = state2[2] - state_exp[2]

            w, p, q = state2[3], state2[4], state2[5]

            # 奖励设置
            reward = reward_three_dof(d_z, d_phi, d_theta, w, p, q, law=f_law, tau=f_start)
            episode_reward += reward

            replay_buffer.add((state, state2, action, reward))
            depth_start = depth2
            roll_start = r_angle_2
            pitch_start = p_angle2
            # if len(replay_buffer.storage) <= args.start_timesteps:
            #     replay_buffer2.add((state, state2, action, reward))

            if len(replay_buffer.storage) >= args.start_time_steps:
                policy.train(replay_buffer, 10, args.batch_size, args.discount, args.tau,
                             args.policy_noise, args.noise_clip, args.policy_freq)
            counter += 1
            total_steps += 1
            state = state2

            if abs(d_z) >= 2.5 or abs(d_phi) >= 0.5 or abs(d_theta) >= 0.5:
                break
            else:
                continue

        print('Episode: {}, Average_Reward: {:.4f}, state: {}, Force: {}, Steps: {}'.format(episode_idx,
                                                                episode_reward / counter, state[:3], f_start, counter))
        if len(replay_buffer.storage) > args.start_time_steps:
            save_reward(reward_file, r=episode_reward/counter)
        if episode_reward/counter >= -10.0:
            policy.save(model_path_actor, model_path_critic)

    # Final evaluation
    policy.save(model_path_actor, model_path_critic)

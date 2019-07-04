#!/usr/bin/env python

import rospy
from openai_ros.task_envs.f1tenth import f1tenth_turning
from openai_ros.robot_envs import f1tenth_env
import time
from rl_agent.input_preprocessor import InputPreprocessor
from rl_agent.ddpg_agent import ddpgAgent
import numpy as np
from data_collector import DataCollector

EPISODE = 2000
EXPLORE = 100000.
epsilon = 1.0
TESTING = True
COLLECT = True

if __name__=='__main__':
    rospy.init_node('f1tenth_turning', anonymous=True, log_level=rospy.DEBUG)
    env = f1tenth_turning.F1TenthTurningEnv()
    input_preprocessor = InputPreprocessor()
    agent = ddpgAgent(Testing=TESTING)
    if COLLECT:
        collector = DataCollector()
    time.sleep(2)
    
    for episode in range(EPISODE):
        s = env.reset()
        s = input_preprocessor(s)
        while True:
            epsilon -= 1.0 / EXPLORE
            a = agent.getAction(s, epsilon)
            s_, r, done, info = env.step(a[0][0])
            s_ = input_preprocessor(s_)

            if TESTING is False:
                agent.storeTrajectory(s, a, r, s_, done)
                agent.learn()

            if COLLECT:
                collector.storeDataStep(s, a, s_)

            s = s_
            if done:
                if COLLECT:
                    collector.writeDataEpisode('episode_' + str(episode) + '.csv')
                break

        if TESTING is False:
            if np.mod(episode, 10) == 0:
                agent.save_model()
        
        """
        while True:
            pass
        """
#! /usr/bin/env python3

import random
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates

class Utility:
    def __init__(self):
        self.plotting  = False
        self.color     = 'tab:orange'
        self.episode   = 0
        self.main_rate = rospy.Rate(10)

        # Subscriber
        rospy.Subscriber('/gazebo/model_states',   ModelStates, self.model_callback)
        rospy.Subscriber('/pioneer/dragging/plot', Bool,        self.plot_callback)

    def model_callback(self, msg):
        models_name      = msg.name
        models_pose      = msg.pose
        thormang3_idx    = models_name.index('thormang3')
        thormang3_pose   = models_pose[thormang3_idx]
        self.thormang3_x = thormang3_pose.position.x

    def plot_callback(self, msg):
        if msg.data == True:
            self.plotting = True
            rospy.loginfo('[Util] Robot Distance: {}'.format(self.thormang3_x))
    
    def run(self):
        # plotting
        style_plot = random.choice(plt.style.available)
        plt.style.use(style_plot)
        
        plt.ion()

        fig = plt.figure(figsize=(12,5))
        ax1 = fig.add_subplot(1,1,1)
        ax1.set_title('Distance/Episode')
        ax1.set_xlabel('Episode')
        ax1.set_ylabel('meter')

        while not rospy.is_shutdown():
            if self.plotting:
                dist = self.thormang3_x * -1
                
                ax1.bar(self.episode, dist, color=self.color)
                self.episode += 1
                plt.draw()
                plt.pause(0.1)

                self.plotting = False
            
            self.main_rate.sleep()
        
if __name__ == "__main__":
    rospy.init_node('pioneer_drag_utils')
    drag_util = Utility()
    drag_util.run()
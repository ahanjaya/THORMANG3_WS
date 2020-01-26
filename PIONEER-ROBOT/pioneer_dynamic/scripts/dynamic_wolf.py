#!/usr/bin/env python3

import rospy
from pioneer_dynamic.ddynamic_reconfigure import DDynamicReconfigure

class Dynamic(object):
    def __init__(self):
        rospy.init_node('initial_pose')
        rospy.loginfo("[Dynamic] Pioneer Dynamic Wolf - Running")

        # Add variables (name, description, default value, min, max, edit_method)
        self.cob = DDynamicReconfigure("centre_of_body")
        self.cob.add_variable("apply", "", False)
        self.cob.add_variable("cob_x_offset_m",  "",  -0.015, -10.0, 10.0)
        self.cob.add_variable("cob_y_offset_m",  "",  -0.00,  -10.0, 10.0)

        # self.add_variables_to_self(self.cob)
        self.cob.start(self.dyn_rec_callback)

    def add_variables_to_self(self, object_item):
        var_names = object_item.get_variable_names()
        for var_name in var_names:
            self.__setattr__(var_name, None)

    def dyn_rec_callback(self, config, level):
        # rospy.loginfo("Received reconf call: " + str(config))
        # # Update all variables
        # var_names = self.ddr.get_variable_names()
        # for var_name in var_names:
        #     self.__dict__[var_name] = config[var_name]
        return config

if __name__ == '__main__':
    dynamic = Dynamic()
    rospy.spin()
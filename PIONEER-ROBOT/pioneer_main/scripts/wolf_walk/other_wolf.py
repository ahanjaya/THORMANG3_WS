## main_wolf.py

 def set_initial_pose(self):
        r_foot_x     = rospy.get_param("/initial_pose/right_foot/x")
        r_foot_y     = rospy.get_param("/initial_pose/right_foot/y")
        r_foot_z     = rospy.get_param("/initial_pose/right_foot/z")
        r_foot_roll  = rospy.get_param("/initial_pose/right_foot/roll")
        r_foot_pitch = rospy.get_param("/initial_pose/right_foot/pitch")
        r_foot_yaw   = rospy.get_param("/initial_pose/right_foot/yaw")

        l_foot_x     = rospy.get_param("/initial_pose/left_foot/x")
        l_foot_y     = rospy.get_param("/initial_pose/left_foot/y")
        l_foot_z     = rospy.get_param("/initial_pose/left_foot/z")
        l_foot_roll  = rospy.get_param("/initial_pose/left_foot/roll")
        l_foot_pitch = rospy.get_param("/initial_pose/left_foot/pitch")
        l_foot_yaw   = rospy.get_param("/initial_pose/left_foot/yaw")

        cob_x        = rospy.get_param("/initial_pose/centre_of_body/x")
        cob_y        = rospy.get_param("/initial_pose/centre_of_body/y")
        cob_z        = rospy.get_param("/initial_pose/centre_of_body/z")
        cob_roll     = rospy.get_param("/initial_pose/centre_of_body/roll")
        cob_pitch    = rospy.get_param("/initial_pose/centre_of_body/pitch")
        cob_yaw      = rospy.get_param("/initial_pose/centre_of_body/yaw")

        self.walking.set_robot_pose( r_foot_x, r_foot_y, r_foot_z, r_foot_roll, r_foot_pitch, r_foot_yaw,\
                                     l_foot_x, l_foot_y, l_foot_z, l_foot_roll, l_foot_pitch, l_foot_yaw,\
                                     cob_x,    cob_y,    cob_z,    cob_roll,    cob_pitch,    cob_yaw)
        rospy.loginfo('[WW] Finish set initial pose')
        print(r_foot_x, r_foot_y, r_foot_z, r_foot_roll, r_foot_pitch, r_foot_yaw)
        print(l_foot_x, l_foot_y, l_foot_z, l_foot_roll, l_foot_pitch, l_foot_yaw)
        print(cob_x, cob_y, cob_z, cob_roll, cob_pitch, cob_yaw)

        if self.save_data:
            right_foot     = { 'x': r_foot_x, 'y': r_foot_y, 'z': r_foot_z, 'roll': r_foot_roll, 'pitch': r_foot_pitch, 'yaw': r_foot_yaw }
            left_foot      = { 'x': l_foot_x, 'y': l_foot_y, 'z': l_foot_z, 'roll': l_foot_roll, 'pitch': l_foot_pitch, 'yaw': l_foot_yaw }
            centre_of_body = { 'x': cob_x,    'y': cob_y,    'z': cob_z,    'roll': cob_roll,    'pitch': cob_pitch,    'yaw': cob_yaw    }

            initial_config = {}
            initial_config['right_foot']     = left_foot
            initial_config['left_foot']      = right_foot
            initial_config['centre_of_body'] = centre_of_body
            
            with open(self.yaml_file, 'w') as f:
                yaml.dump(initial_config, f, default_flow_style=False)


## dynamic_wolf.py

# Add variables (name, description, default value, min, max, edit_method)
self.r_foot = DDynamicReconfigure("right_foot")
self.r_foot.add_variable("x",     "",  0.0,   -1.0, 1.0)
self.r_foot.add_variable("y",     "", -0.093, -1.0, 1.0)
self.r_foot.add_variable("z",     "", -0.63,  -1.0, 1.0)
self.r_foot.add_variable("roll",  "",  0.0,   -1.0, 1.0)
self.r_foot.add_variable("pitch", "",  0.0,   -1.0, 1.0)
self.r_foot.add_variable("yaw",   "",  0.0,   -1.0, 1.0)

self.l_foot = DDynamicReconfigure("left_foot")
self.l_foot.add_variable("x",     "",  0.0,   -1.0, 1.0)
self.l_foot.add_variable("y",     "",  0.093, -1.0, 1.0)
self.l_foot.add_variable("z",     "", -0.63,  -1.0, 1.0)
self.l_foot.add_variable("roll",  "",  0.0,   -1.0, 1.0)
self.l_foot.add_variable("pitch", "",  0.0,   -1.0, 1.0)
self.l_foot.add_variable("yaw",   "",  0.0,   -1.0, 1.0)

self.cob = DDynamicReconfigure("centre_of_body")
self.cob.add_variable("apply", "", False)
self.cob.add_variable("x",     "",  0.0,  -10.0, 10.0)
self.cob.add_variable("y",     "",  0.0,  -10.0, 10.0)
self.cob.add_variable("z",     "",  0.0,  -10.0, 10.0)
self.cob.add_variable("roll",  "",  0.0,  -1.0, 1.0)
self.cob.add_variable("pitch", "",  0.0,  -1.0, 1.0)
self.cob.add_variable("yaw",   "",  0.0,  -1.0, 1.0)

# self.add_variables_to_self(self.r_foot)
self.r_foot.start(self.dyn_rec_callback)

# self.add_variables_to_self(self.l_foot)
self.l_foot.start(self.dyn_rec_callback)

# self.add_variables_to_self(self.cob)
self.cob.start(self.dyn_rec_callback)



cob_x = -0.6 #-0.06 #-0.02 # -0.015 -0.1

big_suitcase:
  1:
    l_arm_el_y: -30.01
    l_arm_grip: 59.96
    l_arm_sh_p1: 24.99
    l_arm_sh_p2: 4.98
    l_arm_sh_r: 75.0
    l_arm_wr_p: -0.0
    l_arm_wr_r: -0.0
    l_arm_wr_y: 0.0
    r_arm_el_y: 0.42
    r_arm_grip: 29.98
    r_arm_sh_p1: -19.33
    r_arm_sh_p2: -5.65
    r_arm_sh_r: -62.27
    r_arm_wr_p: 21.28
    r_arm_wr_r: -16.91
    r_arm_wr_y: 1.37
    time: 0
    velocity: 5
  2:
    l_arm_el_y: -30.01
    l_arm_grip: 59.96
    l_arm_sh_p1: 24.98
    l_arm_sh_p2: 4.98
    l_arm_sh_r: 75.0
    l_arm_wr_p: -0.0
    l_arm_wr_r: -0.0
    l_arm_wr_y: -0.0
    r_arm_el_y: 29.13
    r_arm_grip: 8.62
    r_arm_sh_p1: 29.78
    r_arm_sh_p2: -49.59
    r_arm_sh_r: -79.1
    r_arm_wr_p: -45.73
    r_arm_wr_r: 74.05
    r_arm_wr_y: 55.36
    time: 0
    velocity: 5
  3:
    l_arm_el_y: -30.01
    l_arm_grip: 59.96
    l_arm_sh_p1: 24.98
    l_arm_sh_p2: 4.98
    l_arm_sh_r: 75.0
    l_arm_wr_p: -0.0
    l_arm_wr_r: -0.0
    l_arm_wr_y: 0.0
    r_arm_el_y: 64.05
    r_arm_grip: 8.62
    r_arm_sh_p1: 31.05
    r_arm_sh_p2: -32.83
    r_arm_sh_r: -105.7
    r_arm_wr_p: -37.27
    r_arm_wr_r: 51.72
    r_arm_wr_y: 44.13
    time: 0
    velocity: 5
  4:
    l_arm_el_y: -30.01
    l_arm_grip: 59.96
    l_arm_sh_p1: 24.99
    l_arm_sh_p2: 4.98
    l_arm_sh_r: 75.0
    l_arm_wr_p: -0.0
    l_arm_wr_r: -0.0
    l_arm_wr_y: 0.0
    r_arm_el_y: 64.06
    r_arm_grip: 55.0
    r_arm_sh_p1: 31.05
    r_arm_sh_p2: -32.84
    r_arm_sh_r: -105.69
    r_arm_wr_p: -37.27
    r_arm_wr_r: 51.72
    r_arm_wr_y: 44.13
    time: 0
    velocity: 5

release_human:
  1:
    l_arm_el_y: -64.0
    l_arm_grip: 10.0
    l_arm_sh_p1: -31.0
    l_arm_sh_p2: 33.0
    l_arm_sh_r: 100.0
    l_arm_wr_p: 37.0
    l_arm_wr_r: -50.0
    l_arm_wr_y: -45.0
    r_arm_el_y: 64.0
    r_arm_grip: 10.0
    r_arm_sh_p1: 31.0
    r_arm_sh_p2: -33.0
    r_arm_sh_r: -100.0
    r_arm_wr_p: -37.0
    r_arm_wr_r: 50.0
    r_arm_wr_y: 45.0
    time: 3
    velocity: 5
  2:
    l_arm_el_y: -30.0
    l_arm_grip: 59.96
    l_arm_sh_p1: 24.99
    l_arm_sh_p2: 4.99
    l_arm_sh_r: 75.0
    l_arm_wr_p: -0.0
    l_arm_wr_r: 0.0
    l_arm_wr_y: 0.0
    r_arm_el_y: 30.01
    r_arm_grip: 59.96
    r_arm_sh_p1: -24.99
    r_arm_sh_p2: -5.0
    r_arm_sh_r: -74.99
    r_arm_wr_p: -0.0
    r_arm_wr_r: 0.0
    r_arm_wr_y: -0.0
    time: 0
    velocity: 10
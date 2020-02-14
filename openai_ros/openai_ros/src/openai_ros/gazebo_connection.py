#!/usr/bin/env python

import rospy
import rospkg
from pathlib import Path
from std_srvs.srv import Empty
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, Pose
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest, SpawnModel, GetWorldProperties, DeleteModel, GetLinkState

class GazeboConnection():
    def __init__(self, start_init_physics_parameters, reset_world_or_sim, max_retry = 20):
        self.home                   = str(Path.home())
        rospack                     = rospkg.RosPack()
        self.model_folder           = rospack.get_path("pioneer_dragging") + '/config/'

        self._max_retry             = max_retry
        self.unpause                = rospy.ServiceProxy('/gazebo/unpause_physics',      Empty)
        self.pause                  = rospy.ServiceProxy('/gazebo/pause_physics',        Empty)
        self.reset_simulation_proxy = rospy.ServiceProxy('/gazebo/reset_simulation',     Empty)
        self.reset_world_proxy      = rospy.ServiceProxy('/gazebo/reset_world',          Empty)
        self.spawn_model_prox       = rospy.ServiceProxy('/gazebo/spawn_sdf_model',      SpawnModel)
        self.get_model              = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self.delete_model           = rospy.ServiceProxy('/gazebo/delete_model',         DeleteModel)
        self.get_link_state         = rospy.ServiceProxy('/gazebo/get_link_state',       GetLinkState)

        # Setup the Gravity Controle system
        service_name = '/gazebo/set_physics_properties'
        rospy.logdebug("Waiting for service " + str(service_name))
        rospy.wait_for_service(service_name)
        rospy.logdebug("Service Found " + str(service_name))

        self.set_physics                   = rospy.ServiceProxy(service_name, SetPhysicsProperties)
        self.start_init_physics_parameters = start_init_physics_parameters
        self.reset_world_or_sim            = reset_world_or_sim
        self.init_values()
        # We always pause the simulation, important for legged robots learning
        self.pauseSim()

    def getLinkState(self, link_name):
        rospy.logdebug("GET LINK STATE")
        rospy.wait_for_service('/gazebo/get_link_state')
        rospy.logdebug("GET LINK STATE service found...")

        get_link_done = False
        counter = 0
        z_link  = None

        while not get_link_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    rospy.logdebug("GET LINK STATE service calling...")

                    resp   = self.get_link_state(link_name, '')
                    z_link = resp.link_state.pose.position.z

                    get_link_done = True
                    rospy.logdebug("GET LINK STATE service calling...DONE")

                except rospy.ServiceException as e:
                    counter += 1
                    rospy.logerr("gazebo/get_link_state service call failed")
            else:
                error_message = "Maximum retries done"+str(self._max_retry)+", please check Gazebo spawned service"
                rospy.logerr(error_message)
                assert False, error_message

        rospy.logdebug("GET LINK STATE FINISH")
        return z_link

    def spawnSDFModel(self, object_name, pose):
        rospy.logdebug("SPAWNING SDF MODEL")
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.logdebug("SPAWN SDF service found...")

        spawned_done = False
        counter = 0

        while not spawned_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    rospy.logdebug("SPAWN service calling...")

                    # file_path = '{}/.gazebo/models/{}/model.sdf'.format(self.home, object_name) 
                    file_path = '{}/models/{}/model.sdf'.format(self.model_folder, object_name) 
                    f         = open(file_path,'r')
                    sdff      = f.read()

                    self.spawn_model_prox(object_name, sdff, "thormang3_name_space", pose, "world")

                    spawned_done = True
                    rospy.logdebug("SPAWN service calling...DONE")

                except rospy.ServiceException as e:
                    counter += 1
                    rospy.logerr("gazebo/spawn_sdf_model service call failed")
            else:
                error_message = "Maximum retries done"+str(self._max_retry)+", please check Gazebo spawned service"
                rospy.logerr(error_message)
                assert False, error_message

        rospy.logdebug("SPAWNING FINISH")

    def checkModel(self, object_name):
        rospy.logdebug("CHECK MODEL")
        rospy.wait_for_service('/gazebo/get_world_properties')
        rospy.logdebug("GAZEBO model service found...")

        checked_done = False
        counter      = 0

        while not checked_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    rospy.logdebug("GET WORLD service calling...")
                    resp = self.get_model().model_names

                    if object_name in resp:
                        return True
                   
                    checked_done = True
                    rospy.logdebug("GET WORLD service calling...DONE")
                except rospy.ServiceException as e:
                    counter += 1
                    rospy.logerr("/gazebo/get_world_properties service call failed")
            else:
                error_message = "Maximum retries done"+str(self._max_retry)+", please check Gazebo get world service"
                rospy.logerr(error_message)
                assert False, error_message

        rospy.logdebug("CHECK MODEL FINISH")
        return False

    def deleteModel(self, object_name):
        rospy.logdebug("DELETE MODEL")
        rospy.wait_for_service('/gazebo/delete_model')
        rospy.logdebug("GAZEBO model service found...")

        delete_done = False
        counter      = 0

        while not delete_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    rospy.logdebug("DELETE MODEL service calling...")
                    self.delete_model(object_name)
                    delete_done = True
                    rospy.logdebug("DELETE MODEL service calling...DONE")
                except rospy.ServiceException as e:
                    counter += 1
                    rospy.logerr("/gazebo/delete_model service call failed")
            else:
                error_message = "Maximum retries done"+str(self._max_retry)+", please check Gazebo delete model service"
                rospy.logerr(error_message)
                assert False, error_message

        rospy.logdebug("DELETE MODEL FINISH")

    def pauseSim(self):
        rospy.logdebug("PAUSING START")
        rospy.wait_for_service('/gazebo/pause_physics')
        rospy.logdebug("PAUSING service found...")
        paused_done = False
        counter = 0
        while not paused_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    rospy.logdebug("PAUSING service calling...")
                    self.pause()
                    paused_done = True
                    rospy.logdebug("PAUSING service calling...DONE")
                except rospy.ServiceException as e:
                    counter += 1
                    rospy.logerr("/gazebo/pause_physics service call failed")
            else:
                error_message = "Maximum retries done"+str(self._max_retry)+", please check Gazebo pause service"
                rospy.logerr(error_message)
                assert False, error_message

        rospy.logdebug("PAUSING FINISH")

    def unpauseSim(self):
        rospy.logdebug("UNPAUSING START")
        rospy.wait_for_service('/gazebo/unpause_physics')
        rospy.logdebug("UNPAUSING service found...")
        unpaused_done = False
        counter = 0
        while not unpaused_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    rospy.logdebug("UNPAUSING service calling...")
                    self.unpause()
                    unpaused_done = True
                    rospy.logdebug("UNPAUSING service calling...DONE")
                except rospy.ServiceException as e:
                    counter += 1
                    rospy.logerr("/gazebo/unpause_physics service call failed...Retrying "+str(counter))
            else:
                error_message = "Maximum retries done"+str(self._max_retry)+", please check Gazebo unpause service"
                rospy.logerr(error_message)
                assert False, error_message

        rospy.logdebug("UNPAUSING FiNISH")


    def resetSim(self):
        """
        This was implemented because some simulations, when reseted the simulation
        the systems that work with TF break, and because sometime we wont be able to change them
        we need to reset world that ONLY resets the object position, not the entire simulation
        systems.
        """
        if self.reset_world_or_sim == "SIMULATION":
            rospy.logdebug("SIMULATION RESET")
            self.resetSimulation()
        elif self.reset_world_or_sim == "WORLD":
            rospy.logdebug("WORLD RESET")
            self.resetWorld()
        elif self.reset_world_or_sim == "NO_RESET_SIM":
            rospy.logdebug("NO RESET SIMULATION SELECTED")
        else:
            rospy.logdebug("WRONG Reset Option:"+str(self.reset_world_or_sim))

    def resetSimulation(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_simulation_proxy()
        except rospy.ServiceException as e:
            print ("/gazebo/reset_simulation service call failed")

    def resetWorld(self):
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_world_proxy()
        except rospy.ServiceException as e:
            print ("/gazebo/reset_world service call failed")

    def init_values(self):

        self.resetSim()

        if self.start_init_physics_parameters:
            rospy.logdebug("Initialising Simulation Physics Parameters")
            self.init_physics_parameters()
        else:
            rospy.logerr("NOT Initialising Simulation Physics Parameters")

    def init_physics_parameters(self):
        """
        We initialise the physics parameters of the simulation, like gravity,
        friction coeficients and so on.
        """
        self._time_step = Float64(0.001)
        self._max_update_rate = Float64(1000.0)

        self._gravity = Vector3()
        self._gravity.x = 0.0
        self._gravity.y = 0.0
        self._gravity.z = -9.81

        # self._ode_config = ODEPhysics()
        # self._ode_config.auto_disable_bodies = False
        # self._ode_config.sor_pgs_precon_iters = 0
        # self._ode_config.sor_pgs_iters = 50
        # self._ode_config.sor_pgs_w = 1.3
        # self._ode_config.sor_pgs_rms_error_tol = 0.0
        # self._ode_config.contact_surface_layer = 0.001
        # self._ode_config.contact_max_correcting_vel = 0.0
        # self._ode_config.cfm = 0.0
        # self._ode_config.erp = 0.2
        # self._ode_config.max_contacts = 20

        self._ode_config = ODEPhysics()
        self._ode_config.auto_disable_bodies = False
        self._ode_config.sor_pgs_precon_iters = 0
        self._ode_config.sor_pgs_iters = 100 # 200 # 50
        self._ode_config.sor_pgs_w = 1.4 # 1.3
        self._ode_config.sor_pgs_rms_error_tol = 0.0
        self._ode_config.contact_surface_layer = 0.01 # 0.001
        self._ode_config.contact_max_correcting_vel = 2000.0 # 0.0
        self._ode_config.cfm = 0.0
        self._ode_config.erp = 0.8 # 0.2
        self._ode_config.max_contacts = 20

        self.update_gravity_call()

    def update_gravity_call(self):
        self.pauseSim()

        set_physics_request = SetPhysicsPropertiesRequest()
        set_physics_request.time_step = self._time_step.data
        set_physics_request.max_update_rate = self._max_update_rate.data
        set_physics_request.gravity = self._gravity
        set_physics_request.ode_config = self._ode_config

        rospy.logdebug(str(set_physics_request.gravity))

        result = self.set_physics(set_physics_request)
        rospy.logdebug("Gravity Update Result==" + str(result.success) + ",message==" + str(result.status_message))

        self.unpauseSim()

    def change_gravity(self, x, y, z):
        self._gravity.x = x
        self._gravity.y = y
        self._gravity.z = z

        self.update_gravity_call()
import os
import math
import torch
import rospy
import pickle
import rospkg
import random
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

class NeuralNetwork(nn.Module):
    def __init__(self, n_states, n_actions):
        super(NeuralNetwork, self).__init__()
        self.linear1    = nn.Linear(n_states, 128)
        self.linear2    = nn.Linear(128, 64)
        self.linear3    = nn.Linear(64, n_actions)
        # self.activation = nn.Tanh()
        self.activation = nn.ReLU()

    def forward(self, x):
        x = self.linear1(x)
        x = self.activation(x)
        x = self.linear2(x)
        x = self.activation(x)
        x = self.linear3(x)
        return x

class DuelingNetwork(nn.Module):
    def __init__(self, n_states, n_actions):
        super(DuelingNetwork, self).__init__()
        self.linear1    = nn.Linear(n_states, 128)
        self.linear2    = nn.Linear(128, 64)

        self.advantage  = nn.Linear(64, 64)
        self.advantage2 = nn.Linear(64, n_actions)

        self.value      = nn.Linear(64, 64)
        self.value2     = nn.Linear(64, 1)

        self.activation = nn.Tanh()
        # self.activation = nn.ReLU()

    def forward(self, x):
        output1 = self.linear1(x)
        output1 = self.activation(output1)
        output1 = self.linear2(output1)
        output1 = self.activation(output1)

        output_advantage = self.advantage(output1)
        output_advantage = self.activation(output_advantage)
        output_advantage = self.advantage2(output_advantage)

        output_value = self.value(output1)
        output_value = self.activation(output_value)
        output_value = self.value2(output_value)

        output_final = output_value + output_advantage - output_advantage.mean()
        return output_final

class ExperienceReplay(object):
    def __init__(self, capacity):
        self.capacity = capacity
        self.memory   = []
        self.position = 0
        self.file_mem = None
                
    def push(self, state, action, new_state, reward, done):
        transition = (state, action, new_state, reward, done)

        if self.position >= len(self.memory):
            self.memory.append(transition)
        else:
            self.memory[self.position] = transition

        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        return zip(*random.sample(self.memory, batch_size))

    def load(self):
        if self.file_mem is not None:
            with open(self.file_mem, 'rb') as filehandle:
                # read the data as binary data stream
                self.memory = pickle.load(filehandle)
            rospy.loginfo('[DQN] Loaded memory: {}'.format(self.file_mem))

    def save(self):
        if self.file_mem is not None:
            with open(self.file_mem, 'wb') as filehandle:
                # store the data as binary data stream
                pickle.dump(self.memory, filehandle)
            rospy.loginfo('[DQN] Save memory: {}'.format(self.file_mem))

    def __len__(self):
        return len(self.memory)

class DQN(object):
    def __init__(self, n_states, n_actions):
        rospack             = rospkg.RosPack()

        self.alpha          = rospy.get_param("/alpha") 
        self.gamma          = rospy.get_param("/gamma") 
        self.epsilon        = rospy.get_param("/epsilon") 
        self.epsilon_final  = rospy.get_param("/epsilon_final")
        self.epsilon_decay  = rospy.get_param("/epsilon_decay")
        self.testing        = rospy.get_param("/testing")
        self.mode_action    = rospy.get_param('/mode_action')
        self.clip_err       = rospy.get_param("/clip_error")
        self.update_fre     = rospy.get_param("/update_fre")
        self.mode_optimize  = rospy.get_param('/mode_optimize')
        self.mode_loss      = rospy.get_param('/loss_func')
        self.m_optimizer    = rospy.get_param('/optimizer')
        
        self.steps_done     = 0
        self.file_models    = None

        self.device         = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.n_states       = n_states
        self.n_actions      = n_actions
        self.list_actions   = list(range(self.n_actions))
        self.lr_rate        = rospy.get_param("/learning_rate")

        
        if self.mode_optimize == 'dueling_dqn':
            self.policy_net = DuelingNetwork(self.n_states, self.n_actions).to(self.device)
            self.target_net = DuelingNetwork(self.n_states, self.n_actions).to(self.device)
        else:
            self.policy_net = NeuralNetwork(self.n_states, self.n_actions).to(self.device)
            self.target_net = NeuralNetwork(self.n_states, self.n_actions).to(self.device)

         if self.mode_loss == 'MSE_loss':
            self.loss_func  = nn.MSELoss()
        elif self.mode_loss == 'SmoothL1Loss':
            self.loss_func  = nn.SmoothL1Loss()

        if self.m_optimizer == 'Adam':
            self.optimizer  = optim.Adam(params=self.policy_net.parameters(), lr=self.lr_rate)
        elif self.m_optimizer == 'RMS':
            self.optimizer  = optim.RMSprop(params=self.policy_net.parameters(), lr=self.lr_rate)

        if self.testing:
            if self.mode_action == 'Discrete-Action':
                self.file_models = rospack.get_path("pioneer_dragging") + '/data/1-discrete_cob.pth'
            elif self.mode_action == 'Step-Action':
                # self.file_models = rospack.get_path("pioneer_dragging") + '/data/dragging_thormang3.pth'
                self.file_models = rospack.get_path("pioneer_dragging") + '/data/5-Step-Action.pth'

            if os.path.exists(self.file_models):
                self.load_model()

    def save_model(self):
        if self.file_models is not None:
            torch.save({
            'policy_model_state_dict': self.policy_net.state_dict(),
            'target_model_state_dict': self.target_net.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict()
            }, self.file_models)

            # torch.save(self.policy_net.state_dict(), self.file_models )
            rospy.loginfo('[DQN] Save model: {}'.format(self.file_models))

    def load_model(self):
        check_point = torch.load(self.file_models)
        self.policy_net.load_state_dict(check_point['policy_model_state_dict'])
        self.target_net.load_state_dict(check_point['target_model_state_dict'])
        self.optimizer.load_state_dict(check_point['optimizer_state_dict'])

        rospy.loginfo('[DQN] Loaded model: {}'.format(self.file_models))

    def calculate_epsilon(self, epsilon, i_episode):
        # epsilon = self.epsilon_final + (epsilon - self.epsilon_final) * \
        #             math.exp(-1. * self.steps_done / self.epsilon_decay)
        # self.steps_done += 1

        epsilon = self.epsilon_final + (epsilon - self.epsilon_final) * \
                    math.exp(-1. * i_episode / self.epsilon_decay)
        return epsilon

    def select_action(self, state, i_episode):
        if not self.testing:
            self.epsilon = self.calculate_epsilon(self.epsilon, i_episode)

            if random.random() > self.epsilon:
                with torch.no_grad():
                    state     = torch.Tensor(state).to(self.device)
                    action_nn = self.policy_net(state)
                    action    = torch.max(action_nn, 0)[1].item()
            else:
                action = random.choice(self.list_actions)
        else:
            with torch.no_grad():
                state     = torch.Tensor(state).to(self.device)
                action_nn = self.policy_net(state)
                action    = torch.max(action_nn, 0)[1].item()

            self.epsilon = self.epsilon_final

        return action, self.epsilon

    ################################
    # without experience replay memory

    def optimize(self, state, action, new_state, reward, done):
        if self.testing:
            return

        state     = torch.Tensor(state).to(self.device)
        new_state = torch.Tensor(new_state).to(self.device)
        reward    = torch.Tensor([reward]).to(self.device)

        if done:
            target_value = reward
        else:
            new_state_values     = self.policy_net(new_state).detach() # turn off autograd
            max_new_state_values = torch.max(new_state_values)
            target_value         = reward + self.gamma * max_new_state_values  # bellman equation

        predicted_value = self.policy_net(state)[action].unsqueeze(0)

        loss = self.loss_func(predicted_value, target_value)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

    ################################
    # experience replay memory

    def optimize_with_replay_memory(self, state, action, new_state, reward, done):
        if self.testing:
            return

        state     = torch.Tensor(state).to(self.device)
        action    = torch.LongTensor(action).to(self.device)
        new_state = torch.Tensor(new_state).to(self.device)
        reward    = torch.Tensor(reward).to(self.device)
        done      = torch.Tensor(done).to(self.device)

        new_state_values     = self.policy_net(new_state).detach() # turn off autograd
        max_new_state_values = torch.max(new_state_values, 1)[0]
        target_value         = reward + (1 - done) * self.gamma * max_new_state_values  # bellman equation

        predicted_value = self.policy_net(state).gather(1, action.unsqueeze(1))
        predicted_value = predicted_value.squeeze(1)

        loss = self.loss_func(predicted_value, target_value)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

    ################################
    # experience replay memory

    def optimize_with_target_net(self, state, action, new_state, reward, done):
        if self.testing:
            return

        state     = torch.Tensor(state).to(self.device)
        action    = torch.LongTensor(action).to(self.device)
        new_state = torch.Tensor(new_state).to(self.device)
        reward    = torch.Tensor(reward).to(self.device)
        done      = torch.Tensor(done).to(self.device)

        new_state_values     = self.target_net(new_state).detach() # turn off autograd
        # new_state_values     = self.policy_net(new_state).detach() # turn off autograd
        max_new_state_values = torch.max(new_state_values, 1)[0]
        target_value         = reward + (1 - done) * self.gamma * max_new_state_values  # bellman equation

        predicted_value = self.policy_net(state).gather(1, action.unsqueeze(1))
        predicted_value = predicted_value.squeeze(1)

        loss = self.loss_func(predicted_value, target_value)
        self.optimizer.zero_grad()
        loss.backward()

        if self.clip_err:
            for param in self.policy_net.parameters():
                param.grad.data.clamp_(-1, 1)

        self.optimizer.step()

    ################################
    # dueling DQN

    def optimize_with_dueling_DQN(self, state, action, new_state, reward, done):
        if self.testing:
            return

        state     = torch.Tensor(state).to(self.device)
        action    = torch.LongTensor(action).to(self.device)
        new_state = torch.Tensor(new_state).to(self.device)
        reward    = torch.Tensor(reward).to(self.device)
        done      = torch.Tensor(done).to(self.device)

        new_state_indexes     = self.policy_net(new_state).detach()
        max_new_state_indexes = torch.max(new_state_indexes, 1)[1]  
        new_state_values      = self.target_net(new_state).detach()
        max_new_state_values  = new_state_values.gather(1, max_new_state_indexes.unsqueeze(1)).squeeze(1)

        target_value         = reward + (1 - done) * self.gamma * max_new_state_values  # bellman equation
        predicted_value      = self.policy_net(state).gather(1, action.unsqueeze(1))
        predicted_value      = predicted_value.squeeze(1)

        loss = self.loss_func(predicted_value, target_value)
        self.optimizer.zero_grad()
        loss.backward()

        if self.clip_err:
            for param in self.policy_net.parameters():
                param.grad.data.clamp_(-1, 1)

        self.optimizer.step()

    def update_param(self, i_episode):
        if i_episode % self.update_fre == 0:
            self.target_net.load_state_dict( self.policy_net.state_dict() )

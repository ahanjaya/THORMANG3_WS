import rospy
import math
import torch
import random
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

class NeuralNetwork(nn.Module):
    def __init__(self, n_states, n_actions):
        super(NeuralNetwork, self).__init__()
        self.linear1    = nn.Linear(n_states, 64)
        self.linear2    = nn.Linear(64, n_actions)
        self.activation = nn.Tanh()
        # self.activation = nn.ReLU()

    def forward(self, x):
        x = self.linear1(x)
        x = self.activation(x)
        x = self.linear2(x)
        return x

class ExperienceReplay(object):
    def __init__(self, capacity):
        self.capacity = capacity
        self.memory   = []
        self.position = 0
                
    def push(self, state, action, new_state, reward, done):
        transition = (state, action, new_state, reward, done)

        if self.position >= len(self.memory):
            self.memory.append(transition)
        else:
            self.memory[self.position] = transition

        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        return zip(*random.sample(self.memory, batch_size))

    def __len__(self):
        return len(self.memory)

class DQN(object):
    def __init__(self, n_states, n_actions):

        self.alpha         = rospy.get_param("/alpha") 
        self.gamma         = rospy.get_param("/gamma") 
        self.epsilon       = rospy.get_param("/epsilon") 
        self.epsilon_final = rospy.get_param("/epsilon_final")
        self.epsilon_decay = rospy.get_param("/epsilon_decay")
        self.file2save     = rospy.get_param("/file2save")

        self.clip_err     = rospy.get_param("/clip_error")
        self.update_target_frequency = rospy.get_param("/update_target_frequency")
        self.save_model_frequency    = rospy.get_param("/save_model_frequency")
        self.steps_done    = 0
        self.update_target_counter = 0


        self.device    = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.n_states  = n_states
        self.n_actions = n_actions
        self.lr_rate   = rospy.get_param("/learning_rate")
        self.nn        = NeuralNetwork(self.n_states, self.n_actions).to(self.device)
        self.target_nn = NeuralNetwork(self.n_states, self.n_actions).to(self.device)
        
        # self.loss_func = nn.MSELoss()
        self.loss_func = nn.SmoothL1Loss()

        # self.optimizer = optim.Adam(params=self.nn.parameters(), lr=self.lr_rate)
        self.optimizer = optim.RMSprop(params=self.nn.parameters(), lr=self.lr_rate)

    def save_model(self, model):
        torch.save(model.state_dict(), self.file2save )

    def calculate_epsilon(self):
        self.steps_done += 1
        epsilon = self.epsilon_final + (self.epsilon - self.epsilon_final) * \
                    math.exp(-1. * self.steps_done / self.epsilon_decay)
        return epsilon

    def select_action(self, state):
        epsilon = self.calculate_epsilon()

        if random.random() > epsilon:
            with torch.no_grad():
                state     = torch.Tensor(state).to(self.device)
                action_nn = self.nn(state)
                action    = torch.max(action_nn, 0)[1].item()
        else:
            action = random.randrange(self.n_actions)

        return action, epsilon

    ################################
    # without experience replay memory

    def optimize(self, state, action, new_state, reward, done):
        state     = torch.Tensor(state).to(self.device)
        new_state = torch.Tensor(new_state).to(self.device)
        reward    = torch.Tensor([reward]).to(self.device)

        if done:
            target_value = reward
        else:
            new_state_values     = self.nn(new_state).detach() # turn off autograd
            max_new_state_values = torch.max(new_state_values)
            target_value         = reward + self.gamma * max_new_state_values  # bellman equation

        predicted_value = self.nn(state)[action].unsqueeze(0)

        loss = self.loss_func(predicted_value, target_value)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

    ################################
    # experience replay memory

    def optimize_with_replay_memory(self, state, action, new_state, reward, done):
        state     = torch.Tensor(state).to(self.device)
        action    = torch.LongTensor(action).to(self.device)
        new_state = torch.Tensor(new_state).to(self.device)
        reward    = torch.Tensor(reward).to(self.device)
        done      = torch.Tensor(done).to(self.device)

        new_state_values     = self.nn(new_state).detach() # turn off autograd
        max_new_state_values = torch.max(new_state_values, 1)[0]
        target_value         = reward + (1 - done) * self.gamma * max_new_state_values  # bellman equation

        predicted_value = self.nn(state).gather(1, action.unsqueeze(1))
        predicted_value = predicted_value.squeeze(1)

        loss = self.loss_func(predicted_value, target_value)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

    ################################
    # experience replay memory

    def optimize_with_target_net(self, state, action, new_state, reward, done):
        state     = torch.Tensor(state).to(self.device)
        action    = torch.LongTensor(action).to(self.device)
        new_state = torch.Tensor(new_state).to(self.device)
        reward    = torch.Tensor(reward).to(self.device)
        done      = torch.Tensor(done).to(self.device)

        new_state_values     = self.target_nn(new_state).detach() # turn off autograd
        # new_state_values     = self.nn(new_state).detach() # turn off autograd
        max_new_state_values = torch.max(new_state_values, 1)[0]
        target_value         = reward + (1 - done) * self.gamma * max_new_state_values  # bellman equation

        predicted_value = self.nn(state).gather(1, action.unsqueeze(1))
        predicted_value = predicted_value.squeeze(1)

        loss = self.loss_func(predicted_value, target_value)
        self.optimizer.zero_grad()
        loss.backward()

        if self.clip_err:
            for param in self.nn.parameters():
                param.grad.data.clamp_(-1, 1)

        self.optimizer.step()

        if self.update_target_counter % self.update_target_frequency == 0:
            self.target_nn.load_state_dict(self.nn.state_dict())

        if self.update_target_counter % self.save_model_frequency == 0:
            self.save_model(self.nn)

        self.update_target_counter += 1
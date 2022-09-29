#!/usr/bin/env python
from click import command
import rospy
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

map_obj = None
x1 = np.ones(20)
y1 = np.ones(20)
y2 = np.ones(20)


class my_map():
  global x1,y1

  def __init__(self):
      self.map = np.zeros((20,20))
      
      self.prob_plot = None  
      self.prob = np.zeros((20,20))
      self.prob1 = np.zeros((20,20))
      self.prob2 = np.zeros((20,20))
      self.prob3 = np.zeros((20,20))
      self.robot_pos = [9,9]

      self.map1dx = np.zeros(20)
      
      # self.prob_plot1d = None  
      self.prob1d = 0.001*np.ones(20)
      self.prob11d = 0.001*np.ones(20)
      self.prob21d = 0.001*np.ones(20)
      self.prob31d = 0.001*np.ones(20)
      self.robot_pos1d = 9

      self.prob11d[self.robot_pos1d-1] = 0.981
      self.prob21d[self.robot_pos1d+0] = 0.981
      self.prob31d[self.robot_pos1d+1] = 0.981

      self.prob1dy = 0.001*np.ones(20)
      self.prob11dy = 0.001*np.ones(20)
      self.prob21dy = 0.001*np.ones(20)
      self.prob31dy = 0.001*np.ones(20)
      self.robot_pos1dy = 9

      self.prob11dy[self.robot_pos1dy-1] = 0.981
      self.prob21dy[self.robot_pos1dy+0] = 0.981
      self.prob31dy[self.robot_pos1dy+1] = 0.981

      # self.my_act = None


  def prediction(self, action, steps):
    # self.my_act = [action,steps]
    my_step = 0
    my_stepy = 0

    if steps == 0:
      return

    if action == 'D':
      self.robot_pos1dy += steps
      my_stepy = steps
    elif action == 'U':
      self.robot_pos1dy -= steps
      my_stepy = -steps
    elif action == 'R':
      self.robot_pos1d += steps
      my_step = steps
    elif action == 'L':
      self.robot_pos1d -= steps
      my_step = -steps

    self.robot_pos1d = np.clip(self.robot_pos1d, 0, 19)
    self.robot_pos1dy = np.clip(self.robot_pos1d, 0, 19)

    if action == 'R' or action == 'L':
      if self.robot_pos1d-1 >= 0 and self.robot_pos1d-1 <= 19:
        
        self.prob11d = np.roll(self.prob1d,my_step-1)
        self.prob21d = np.roll(self.prob1d,my_step+0)  
        self.prob31d = np.roll(self.prob1d,my_step+1)

        # update probablilty map
        self.prob1d = 0.2*self.prob11d + 0.6*self.prob21d + 0.2*self.prob31d

        # normalization
        norm = np.sum(self.prob1d)
        self.prob1d = self.prob1d/norm

      elif self.robot_pos1d-1 < 0:
        # self.prob11d[self.robot_pos1d-1] = 1
        self.prob21d[self.robot_pos1d+0] = 1
        self.prob31d[self.robot_pos1d+1] = 1

        # update probablilty map
        # self.prob = 0.8*self.prob2 + 0.2*self.prob3
        self.prob1d = 0.8*self.prob21d + 0.2*self.prob31d
        


      elif self.robot_pos1d-1 > 19:
        self.prob11d[self.robot_pos1d-1] = 1
        self.prob21d[self.robot_pos1d+0] = 1
        # self.prob31d[self.robot_pos1d+1] = 1

        # update probablilty map
        # self.prob = 0.2*self.prob1 + 0.8*self.prob2
        self.prob1d = 0.2*self.prob11d + 0.8*self.prob21d


    # update y position
    if action == 'U' or action == 'D':
      if self.robot_pos1dy-1 >= 0 and self.robot_pos1dy-1 <= 19:
        
        self.prob11dy = np.roll(self.prob1dy,my_stepy-1)
        self.prob21dy = np.roll(self.prob1dy,my_stepy+0)  
        self.prob31dy = np.roll(self.prob1dy,my_stepy+1)

        # update probablilty map
        self.prob1dy = 0.2*self.prob11dy + 0.6*self.prob21dy + 0.2*self.prob31dy

        # normalization
        norm = np.sum(self.prob1dy)
        self.prob1dy = self.prob1dy/norm

      # elif self.robot_pos1d-1 < 0:
      #   # self.prob11d[self.robot_pos1d-1] = 1
      #   self.prob21d[self.robot_pos1d+0] = 1
      #   self.prob31d[self.robot_pos1d+1] = 1

      #   # update probablilty map
      #   # self.prob = 0.8*self.prob2 + 0.2*self.prob3
      #   self.prob1d = 0.8*self.prob21d + 0.2*self.prob31d
        


      # elif self.robot_pos1d-1 > 19:
      #   self.prob11d[self.robot_pos1d-1] = 1
      #   self.prob21d[self.robot_pos1d+0] = 1
      #   # self.prob31d[self.robot_pos1d+1] = 1

      #   # update probablilty map
      #   # self.prob = 0.2*self.prob1 + 0.8*self.prob2
      #   self.prob1d = 0.2*self.prob11d + 0.8*self.prob21d   



  def correction(self, dir, pos):
    obs_arr = np.zeros(20)
    obs_arr[0] = 0.1
    obs_arr[1] = 0.2
    obs_arr[2] = 0.4
    obs_arr[3] = 0.2
    obs_arr[4] = 0.1

    if dir == 'X':
      obs_arr = np.roll(obs_arr,pos-2)
      self.prob1d = np.multiply(obs_arr,self.prob1d)

      # normalization
      norm = np.sum(self.prob1d)
      self.prob1d = self.prob1d/norm


    elif dir == 'Y':
      obs_arr = np.roll(obs_arr,pos-2)
      self.prob1dy = np.multiply(obs_arr,self.prob1dy)

      # normalization
      norm = np.sum(self.prob1dy)
      self.prob1dy = self.prob1dy/norm




  def initial_belief(self):
      self.prob[self.robot_pos[0],self.robot_pos[1]] = 1
      self.map[self.robot_pos[0],self.robot_pos[1]] = 1
      self.prob1d[self.robot_pos1d] = 0.981
      self.prob1dy[self.robot_pos1dy] = 0.981

  def render(self):
    global x1,y1,y2
    print('')
    print('Pose believe in X direction:')
    print('')
    print(self.prob1d)
    print('')
    print('Pose believe in Y direction:')
    print('')
    print(self.prob1dy)

    x1 = np.array(range(20))
    y1 = self.prob1d
    y2 = self.prob1dy

# initializing a figure in
# which the graph will be plotted
fig = plt.figure()

 
# marking the x-axis and y-axis
axis = plt.axes(xlim =(0, 20),
                ylim =(0, 1))
 
# initializing a line variable
line, = axis.plot([], [], lw = 3)

def init():
    line.set_data([], [])
    return line,

ax = plt.gca()
ax.text(0.5, 1.100, "Pose belief in X", bbox={'facecolor': 'red',
                                       'alpha': 0.5, 'pad': 5},
         transform=ax.transAxes, ha="center")


def animate(i):
    global x1,y1
    
    ax = plt.gca()
    ax.text(0.5, 1.100, "y=sin(x)", bbox={'facecolor': 'red',
                                          'alpha': 0.5, 'pad': 5},
            transform=ax.transAxes, ha="center")

    line.set_data(x1, y1)
    return line,

# initializing a figure in
# which the graph will be plotted
fig1 = plt.figure()
 
# marking the x-axis and y-axis
axis1 = plt.axes(xlim =(0, 20),
                ylim =(0, 1))
 
# initializing a line variable
line1, = axis1.plot([], [], lw = 3)

def init1():
    line1.set_data([], [])
    return line1,

ax1 = plt.gca()
ax1.text(0.5, 1.100, "Pose belief in Y", bbox={'facecolor': 'red',
                                       'alpha': 0.5, 'pad': 5},
         transform=ax.transAxes, ha="center")

def animate1(i):
    global x1,y2

    ax1.text(0.5, 1.100, "Pose belief in Y", bbox={'facecolor': 'red',
                                       'alpha': 0.5, 'pad': 5},
         transform=ax.transAxes, ha="center")

    line1.set_data(x1, y2)
    return line1,

def update_localization_plot(data):
    global map_obj

    commands = data.data
    
    print('')
    print('Input: ',commands)
    print('')
    my_char = commands[0]
    my_num = commands[1:len(commands)]
    
    if my_char == 'X' or my_char == 'Y':
      map_obj.correction(my_char,int(my_num))
    else:  
      map_obj.prediction(my_char,int(my_num))
    
    # plt.close()
    map_obj.render()
    
def lab9():
    global map_obj,x1,y1
    
    rospy.init_node('lab9', anonymous=True)

    map_obj = my_map()
    map_obj.initial_belief()
    map_obj.render()


    rospy.Subscriber("robot", String, update_localization_plot)

    if x1[0] != 1:
      
      anim = FuncAnimation(fig, animate,
                    init_func = init,
                    frames = 200,
                    interval = 20,
                    blit = True)
      
      anim1 = FuncAnimation(fig1, animate1,
                    init_func = init1,
                    frames = 200,
                    interval = 20,
                    blit = True)
      plt.show()
      
    rospy.spin()

if __name__ == '__main__':
    lab9()
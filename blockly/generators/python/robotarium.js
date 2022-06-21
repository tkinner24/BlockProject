/**
 * @license
 * Copyright 2012 Google LLC
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @fileoverview Generating Python for variable blocks.
 * @author q.neutron@gmail.com (Quynh Neutron)
 */
 'use strict';


 Blockly.Python['initialize'] = function(block) {
   var code = `
import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
import numpy as np
import time
import math
from copy import copy
N = 0
a = np.array([[],[],[]])
names = np.array([])

def createPoint(x,y,angle):
  pose = np.zeros((3, N))
  pose[0, 0] = x
  pose[1, 0] = y
  pose[2, 0] = angle

  return pose

def actualTurn(point,x, error,rightBool):
  while (point[2,0] > x[2,0] + error or point[2,0] < x[2,0] - error):
      print(point[2,0])
      print(x[2,0])
      print("---------")
      # Get poses of agents
      x = r.get_poses()

      # Create single-integrator control inputs
      dxu = unicycle_position_controller(x, point[:2][:])

      # Create safe control inputs (i.e., no collisions)
      dxu = uni_barrier_cert(dxu, x)

      if(rightBool):
          dxu[1,0] = -0.5
      else:
          dxu[1,0] = 0.5
      
      print(dxu)

      # Set the velocities by mapping the single-integrator inputs to unciycle inputs
      r.set_velocities(np.arange(N), dxu)

      # Iterate the simulation
      r.step()

def turnAngle(angle,rightBool):
  if(rightBool):
      angle*=-1
  x = r.get_poses()
  r.step()
  newX = copy(x)
  newX[2,0] = x[2,0] + angle
  if(newX[2,0] > math.pi):
      newX[2,0] = newX[2,0] - 2*math.pi
  elif(newX[2,0] < -1*math.pi):
      newX[2,0] = newX[2,0] + 2*math.pi
  actualTurn(newX,x,0.01,rightBool)

def moveToPoint(point,x):
  while (np.size(at_pose(x, point, rotation_error=100)) != N):

      # Get poses of agents
      x = r.get_poses()

      # Create single-integrator control inputs
      dxu = unicycle_position_controller(x, point[:2][:])

      # Create safe control inputs (i.e., no collisions)
      dxu = uni_barrier_cert(dxu, x)

      # Set the velocities by mapping the single-integrator inputs to unciycle inputs
      r.set_velocities(np.arange(N), dxu)

      # Iterate the simulation
      r.step()

def moveForward(stepSize):
  x = r.get_poses()
  r.step()
  xMovement = stepSize*math.cos(x[2,0])
  yMovement = stepSize*math.sin(x[2,0])
  point = createPoint(x[0,0]+xMovement,x[1,0]+yMovement,0)
  moveToPoint(point,x)
   `;
   return code;
 };
 
 Blockly.Python['new_robot'] = function(block) {
   var text_name = block.getFieldValue('name');
   var number_x_val = block.getFieldValue('x_val');
   var number_y_val = block.getFieldValue('y_val');
   var angle_theta = block.getFieldValue('theta');
   var code = `
names = np.append(names,["`+String(text_name)+`"],axis = 0)
a = np.append(a,[[`+String(number_x_val)+`],[`+String(number_y_val)+`],[`+String(angle_theta * 3.14/180)+`]], axis = 1)
N = N + 1
   `;
   return code;
 };
 
 Blockly.Python['end_initialize'] = function(block) {
   var code = `
initial_conditions = np.array(a)
r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=False)
unicycle_position_controller = create_clf_unicycle_position_controller()
uni_barrier_cert = create_unicycle_barrier_certificate()
x=r.get_poses()
target_positions = x
r.step()
`;
   return code;
 };
 
 Blockly.Python['run_to_point'] = function(block) {
   var text_name = block.getFieldValue('name');
   var number_target_x = block.getFieldValue('target_x');
   var number_target_y = block.getFieldValue('target_y');
   var code = `
target_values = np.array([[`+number_target_x+`],[`+number_target_y+`],[0]])

while (np.size(at_pose(x, target_values, rotation_error=100)) != N):
  x = r.get_poses()
  dxu = unicycle_position_controller(x, target_values[:2][:])
  r.set_velocities(np.arange(N), dxu)    
  r.step()
`;
   return code;
 };
 
 Blockly.Python['display_image'] = function(block) {
  var value_image_name = block.getFieldValue('image name');
  // TODO: Assemble Python into code variable.
  var code = `gt_img = plt.imread("`+ String(value_image_name) +`.png")
x_img = np.linspace(-1.0, 1.0, gt_img.shape[1])
y_img = np.linspace(-1.0, 1.0, gt_img.shape[0])
gt_img_handle = r.axes.imshow(gt_img, extent=(-1, 1, -1, 1))
`;
  return code;
 };

 Blockly.Python['move_forward'] = function(block) {
  var value_name = Blockly.Python.valueToCode(block, 'NAME', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = `
moveForward(`+value_name+`)`;
  return code;
};

Blockly.Python['turn'] = function(block) {
  var value_angle = Blockly.Python.valueToCode(block, 'Angle', Blockly.Python.ORDER_ATOMIC);
  value_angle = value_angle * 3.14/180
  var value_direction = Blockly.Python.valueToCode(block, 'Direction', Blockly.Python.ORDER_ATOMIC);

  // TODO: Assemble Python into code variable.
  var code = `
turnAngle(`+value_angle+`,`+value_direction+`)
`;
  return code;
};

Blockly.Python['end'] = function(block) {
  var code = `
r.call_at_scripts_end()
`;
  return code;
};

Blockly.Python['pen_down'] = function(block) {
  // TODO: Assemble Python into code variable.
  var code = `line = r.axes.plot(target_values[0],terget_values[1],color='green', marker='o', linestyle='dashed', linewidth=1, markersize=6)
  `;
  return code;
};
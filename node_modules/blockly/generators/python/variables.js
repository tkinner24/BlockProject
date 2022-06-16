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
N = 0
a = np.array([[],[],[]])
names = np.array([])
  `;
  return code;
};

Blockly.Python['new_robot'] = function(block) {
  var text_name = block.getFieldValue('name');
  var number_x_val = block.getFieldValue('x_val');
  var number_y_val = block.getFieldValue('y_val');
  var angle_theta = block.getFieldValue('theta');
  var code = `
names = np.append(names,[[`+String(text_name)+`]],axis = 0)
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
r.get_poses()
r.step()
  `;
  return code;
};

Blockly.Python['run_to_point'] = function(block) {
  var text_name = block.getFieldValue('name');
  var number_target_x = block.getFieldValue('target_x');
  var number_target_y = block.getFieldValue('target_y');
  var code = `

  
  
  
  `;
  return code;
};
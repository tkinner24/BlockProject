/**
 * @license
 * Copyright 2012 Google LLC
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @fileoverview Variable blocks for Blockly.

 * This file is scraped to extract a .json file of block definitions. The array
 * passed to defineBlocksWithJsonArray(..) must be strict JSON: double quotes
 * only, no outside references, no functions, no trailing commas, etc. The one
 * exception is end-of-line comments, which the scraper will remove.
 * @author fraser@google.com (Neil Fraser)
 */
'use strict';

// goog.provide('Blockly.Blocks.variables');  // Deprecated.
// goog.provide('Blockly.Constants.Variables');

// goog.require('Blockly');
// goog.require('Blockly.Blocks');
// goog.require('Blockly.FieldLabel');
// goog.require('Blockly.FieldVariable');


/**
 * Unused constant for the common HSV hue for all blocks in this category.
 * @deprecated Use Blockly.Msg['VARIABLES_HUE']. (2018 April 5)
 */
Blockly.Constants.Variables.HUE = 330;

Blockly.defineBlocksWithJsonArray([  // BEGIN JSON EXTRACT
  // Block for variable getter


  {
    "type": "initialize",
    "message0": "Initialize Robotarium",
    "nextStatement": "robot",
    "colour": 135,
    "tooltip": "Starts entire process",
    "helpUrl": ""
  },
  {
    "type": "new_robot",
    "message0": "Name of Robot:  %1 %2 Starting X Value: %3 %4 Starting Y Value: %5 %6 Starting Heading %7",
    "args0": [
      {
        "type": "field_input",
        "name": "name",
        "text": "name"
      },
      {
        "type": "input_dummy"
      },
      {
        "type": "field_number",
        "name": "x_val",
        "value": 0
      },
      {
        "type": "input_dummy"
      },
      {
        "type": "field_number",
        "name": "y_val",
        "value": 0
      },
      {
        "type": "input_dummy"
      },
      {
        "type": "field_angle",
        "name": "theta",
        "angle": 90
      }
    ],
    "previousStatement": "robot",
    "nextStatement": [
      "robot",
      "end_int"
    ],
    "colour": 230,
    "tooltip": "Create a robot at X and Y with heading given",
    "helpUrl": ""
  }, 
  {
    "type": "end_initialize",
    "message0": "End Initialization",
    "previousStatement": "robot",
    "nextStatement": "command",
    "colour": 135,
    "tooltip": "End the initialization process, prepares to create commands",
    "helpUrl": ""
  },
  {
    "type": "run_to_point",
    "message0": "Move %1 to: x: %2 y: %3",
    "args0": [
      {
        "type": "field_input",
        "name": "name",
        "text": "Name"
      },
      {
        "type": "field_number",
        "name": "target_x",
        "value": 0
      },
      {
        "type": "field_number",
        "name": "target_y",
        "value": 0
      }
    ],
    "previousStatement": "command",
    "nextStatement": "command",
    "colour": 230,
    "tooltip": "Enter name of robot and move to x y point",
    "helpUrl": ""
  },
  {
    "type": "display_image",
    "message0": "Disply image %1.png",
    "args0": [
      {
        "type": "field_input",
        "name": "image name",
        "check": "String"
      }
    ],
    "inputsInline": true,
    "previousStatement": null,
    "nextStatement": null,
    "colour": 230,
    "tooltip": "Put in file name of image",
    "helpUrl": ""
  },

  {
    "type": "move_forward",
    "message0": "move forward %1",
    "args0": [
      {
        "type": "input_value",
        "name": "NAME",
        "check": "Number"
      }
    ],
    "inputsInline": false,
    "previousStatement": null,
    "nextStatement": null,
    "colour": 230,
    "tooltip": "",
    "helpUrl": ""
  },
  {
    "type": "turn",
    "message0": "Turn (degrees) %1 to the right %2",
    "args0": [
      {
        "inputsInline": true,
        "type": "input_value",
        "name": "Angle",
        "check": "Number"
      },
      {
        "inputsInline": false,
        "type": "input_value",
        "name": "Direction",
        "check": "Boolean"
      }
    ],
    "previousStatement": null,
    "nextStatement": null,
    "colour": 230,
    "tooltip": "",
    "helpUrl": ""
  },
  {
    "type": "end",
    "message0": "End Program",
    "previousStatement": null,
    "colour": 0,
    "tooltip": "",
    "helpUrl": ""
  },
  {
    "type": "pen_down",
    "message0": "Pen Down",
    "previousStatement": null,
    "nextStatement": null,
    "colour": 230,
    "tooltip": "",
    "helpUrl": ""
  }


]);  // END JSON EXTRACT (Do not delete this comment.)

/**
 * Mixin to add context menu items to create getter/setter blocks for this
 * setter/getter.
 * Used by blocks 'variables_set' and 'variables_get'.
 * @mixin
 * @augments Blockly.Block
 * @package
 * @readonly
 */
Blockly.Constants.Variables.CUSTOM_CONTEXT_MENU_VARIABLE_GETTER_SETTER_MIXIN = {
  /**
   * Add menu option to create getter/setter block for this setter/getter.
   * @param {!Array} options List of menu options to add to.
   * @this {Blockly.Block}
   */
  customContextMenu: function(options) {
    if (!this.isInFlyout) {
      // Getter blocks have the option to create a setter block, and vice versa.
      if (this.type == 'variables_get') {
        var opposite_type = 'variables_set';
        var contextMenuMsg = Blockly.Msg['VARIABLES_GET_CREATE_SET'];
      } else {
        var opposite_type = 'variables_get';
        var contextMenuMsg = Blockly.Msg['VARIABLES_SET_CREATE_GET'];
      }

      var option = {enabled: this.workspace.remainingCapacity() > 0};
      var name = this.getField('VAR').getText();
      option.text = contextMenuMsg.replace('%1', name);
      var xmlField = Blockly.utils.xml.createElement('field');
      xmlField.setAttribute('name', 'VAR');
      xmlField.appendChild(Blockly.utils.xml.createTextNode(name));
      var xmlBlock = Blockly.utils.xml.createElement('block');
      xmlBlock.setAttribute('type', opposite_type);
      xmlBlock.appendChild(xmlField);
      option.callback = Blockly.ContextMenu.callbackFactory(this, xmlBlock);
      options.push(option);
      // Getter blocks have the option to rename or delete that variable.
    } else {
      if (this.type == 'variables_get' || this.type == 'variables_get_reporter') {
        var renameOption = {
          text: Blockly.Msg.RENAME_VARIABLE,
          enabled: true,
          callback: Blockly.Constants.Variables.RENAME_OPTION_CALLBACK_FACTORY(this)
        };
        var name = this.getField('VAR').getText();
        var deleteOption = {
          text: Blockly.Msg.DELETE_VARIABLE.replace('%1', name),
          enabled: true,
          callback: Blockly.Constants.Variables.DELETE_OPTION_CALLBACK_FACTORY(this)
        };
        options.unshift(renameOption);
        options.unshift(deleteOption);
      }
    }
  }
};

/**
  * Callback for rename variable dropdown menu option associated with a
  * variable getter block.
  * @param {!Blockly.Block} block The block with the variable to rename.
  * @return {!function()} A function that renames the variable.
  */
Blockly.Constants.Variables.RENAME_OPTION_CALLBACK_FACTORY = function(block) {
  return function() {
    var workspace = block.workspace;
    var variable = block.getField('VAR').getVariable();
    Blockly.Variables.renameVariable(workspace, variable);
  };
};

/**
 * Callback for delete variable dropdown menu option associated with a
 * variable getter block.
 * @param {!Blockly.Block} block The block with the variable to delete.
 * @return {!function()} A function that deletes the variable.
 */
Blockly.Constants.Variables.DELETE_OPTION_CALLBACK_FACTORY = function(block) {
  return function() {
    var workspace = block.workspace;
    var variable = block.getField('VAR').getVariable();
    workspace.deleteVariableById(variable.getId());
    workspace.refreshToolboxSelection();
  };
};

Blockly.Extensions.registerMixin('contextMenu_variableSetterGetter',
    Blockly.Constants.Variables.CUSTOM_CONTEXT_MENU_VARIABLE_GETTER_SETTER_MIXIN);

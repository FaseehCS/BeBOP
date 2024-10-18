"""Implement various py trees behaviors for robosuite."""
# Copyright (c) 2023, ABB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with
# or without modification, are permitted provided that
# the following conditions are met:
#
#   * Redistributions of source code must retain the
#     above copyright notice, this list of conditions
#     and the following disclaimer.
#   * Redistributions in binary form must reproduce the
#     above copyright notice, this list of conditions
#     and the following disclaimer in the documentation
#     and/or other materials provided with the
#     distribution.
#   * Neither the name of ABB nor the names of its
#     contributors may be used to endorse or promote
#     products derived from this software without
#     specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
from math import sqrt
from operator import add
from typing import Any
import re
import numpy as np
import py_trees as pt
from behaviors import common_behaviors
from simulation.maple_benchmarks.robosuite_interface import ActionTypes, rotate_frame_z



def get_node(node_descriptor, world_interface, verbose=False):
    """Return a py trees behavior or composite given the string."""
    return common_behaviors.get_node(node_descriptor, world_interface, verbose)


class AtPos(pt.behaviour.Behaviour):
    """
    Check if object is at position relative some other object
    """
    def __init__(self, name, parameters, world_interface, _verbose: bool = False):
        self.world_interface = world_interface
        self.target_object = parameters[0]
        self.relative_object = parameters[1]
        self.offset = parameters[2]
        self.threshold = parameters[3]
        super().__init__(name)

    @staticmethod
    def is_parameter_valid(parameters, index):
        """ Checks to make sure the two object parameters are different """
        if index == 1 and parameters[0] == parameters[1]:
            return False
        return True

    def update(self):
        if self.world_interface.object_at_pos(self.target_object,
                                              self.world_interface.get_object_position(self.relative_object) +
                                              self.offset,
                                              self.threshold):
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Parse behavior parameters from string """
        parameters = []
        at_pos = node_descriptor.find(' at ')
        if at_pos >= 0:
            target_object = node_descriptor[0:at_pos]
            parenthesis_pos = node_descriptor.find('(')
            if parenthesis_pos > 0:
                relative_object = node_descriptor[at_pos + 4:parenthesis_pos - 1]
            else:
                relative_object = node_descriptor[at_pos + 4:]
        else:
            target_object = ""
            relative_object = ""
        parameters.append(target_object)
        parameters.append(relative_object)
        numbers = list(map(float, re.findall(r'-?\d+\.\d+|-?\d+', node_descriptor)))
        if len(numbers) > 2:
            position = tuple(numbers[0:3])
        else:
            position = (0.0, 0.0, 0.0)
        parameters.append(position)
        if len(numbers) > 3:
            parameters.append(numbers[3])
        else:
            parameters.append(0.0)

        return parameters

class CheckLoc(pt.behaviour.Behaviour):
    """
    Check if a location is free or not
    """
    # Save the failure info 
    def __init__(self, name, parameters, world_interface, _verbose: bool = False):
        # print ("Parameters:",parameters)
        self.world_interface = world_interface
        self.target_object = parameters[0]
        self.relative_object = parameters[1]
        self.offset = parameters[2]
        self.threshold = 0.1 # In future add it as an optional parameters
        super().__init__(name)

    @staticmethod
    def is_parameter_valid(parameters, index):
        # """ Checks to make sure the two object parameters are different """
        # if index == 1 and parameters[0] == parameters[1]:
        #     return False
        return True

    def update(self):
        # Ensure graspable_objects is always a list
        if isinstance(self.world_interface.graspable_objects, str):
            self.world_interface.graspable_objects = [self.world_interface.graspable_objects]

        # Now proceed with the modified graspable_objects
        graspable_objects = self.world_interface.graspable_objects
        # print("graspable_objects", graspable_objects)

        for obj in graspable_objects:
            # print("obj", obj)
            # Check if the object is not 'peg' and is at the calculated position relative to the offset and threshold
            if obj == 'peg':
                continue
            if self.world_interface.object_at_pos(
                obj,
                self.world_interface.get_object_position(self.relative_object) + self.offset,
                self.threshold
            ):
                # print("FAIL CheckLoc")
                return pt.common.Status.FAILURE

        return pt.common.Status.SUCCESS


    @staticmethod
    def parse_parameters(node_descriptor):
        """Parse behavior parameters from string."""
        parameters = []
        # print ("TARGET OBJECT",node_descriptor)
        # Split the descriptor into parts by spaces
        parts = node_descriptor.split()
        if len(parts) < 4:
            # If format is unexpected, provide defaults
            return ["", "", (0.0, 0.0, 0.0)]

        # Extract target_object and relative_object
        target_object = ""#parts[1]  # Second part is target object
        relative_object = parts[1]  # Third part is relative object
        parameters.append(target_object)
        parameters.append(relative_object)

        # Use regex to extract position numbers, ensuring we handle both spaced and non-spaced numbers
        position_match = re.search(r'\((-?\d*\.?\d+),\s*(-?\d*\.?\d+),\s*(-?\d*\.?\d+)\)', node_descriptor)
        if position_match:
            position = tuple(map(float, position_match.groups()))
        else:
            position = (0.0, 0.0, 0.0)  # Default if no match found

        # Add the position to parameters
        parameters.append(position)

        return parameters

class IsGraspable(pt.behaviour.Behaviour):
    """
    Check if an object is graspable in the world.
    """

    def __init__(self, name, parameters, world_interface, _verbose: bool = False):
        """
        Initialize the IsGraspable behavior.
        Args:
            name: Name of the behavior node.
            parameters: List of parameters for the node, which includes the target object.
            world_interface: Interface to interact with the simulation or real world.
            _verbose: Verbosity level.
        """
        self.world_interface = world_interface
        self.target_object = parameters[0]
        super().__init__(name)

    @staticmethod
    def is_parameter_valid(parameters, index):
        # """ Checks to make sure the two object parameters are different """
        # if index == 1 and parameters[0] == parameters[1]:
        #     return False
        return True
    
    def update(self):
        """
        Check if the target object is graspable.
        Returns:
            SUCCESS if the object is graspable, FAILURE otherwise.
        """

        # Check if the target object is in the list of graspable objects in the world
        if self.world_interface.is_graspable(self.target_object):
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

    @staticmethod
    def parse_parameters(node_descriptor):
        """
        Parse the behavior parameters from a string descriptor.
        Args:
            node_descriptor: String describing the node, e.g., "is graspable target_object"
        Returns:
            List of parsed parameters, which include the target object.
        """
        # Split the descriptor by spaces and extract the target object name
        parts = node_descriptor.split()
        if len(parts) < 3:
            # If the format is unexpected, return a default value
            return ["unknown"]
        target_object = parts[2]  # The third part is the target object
        return [target_object]

class AtPosFree(AtPos):
    """
    Check if object is at position relative some other object. Object must not be grasped
    """
    def update(self):
        if self.world_interface.object_at_pos(self.target_object,
                                              self.world_interface.get_object_position(self.relative_object) +
                                              self.offset,
                                              self.threshold) and \
                self.world_interface.get_grasped_object() == "none":
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE


class PosCheck(common_behaviors.ComparisonCondition):
    """Check position of target object relative to the origin."""
    def __init__(
        self, name: str,
        parameters: list,
        world_interface: Any,
        verbose: bool = False
    ):
        self.target_object = parameters.pop(0)
        self.axis = parameters.pop(0)
        super().__init__(name, parameters, world_interface, verbose)

    def compare(self, variable: Any) -> pt.common.Status:
        """Compare input variable to stored value."""
        if (self.larger_than and variable > self.value) or \
           (not self.larger_than and variable < self.value):
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

    def update(self):
        """Executes behavior and performs pos check. """
        origin = self.world_interface.get_object_position('none')[self.axis]
        object_position = self.world_interface.get_object_position(self.target_object)[self.axis]
        return self.compare(object_position - origin)

    @staticmethod
    def to_string(name, parameters):
        """ Returns a string description of the node """
        string_node = name + " " + parameters[0] + '.' + str(parameters[1])

        if parameters[3]:  # larger_than
            string_node += ' > '
        else:
            string_node += ' < '
        string_node += str(parameters[2])
        string_node += '?'
        return string_node

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Parse behavior parameters from string """
        parameters = []
        dot_pos = node_descriptor.find('.')
        target_object = node_descriptor[4:dot_pos]
        parameters.append(target_object)
        axis = int(node_descriptor[dot_pos + 1])
        parameters.append(axis)
        value = float(re.findall(r'-?\d+\.\d+|-?\d+', node_descriptor[dot_pos + 5:])[0])
        parameters.append(value)

        return parameters


class HandleAngle(pt.behaviour.Behaviour):
    """ Check angle of handle. """
    def __init__(
            self, name: str,
            parameters: list,
            world_interface: Any,
            _verbose: bool = False):
        self.world_interface = world_interface
        self.value = float(parameters[0])
        super().__init__(name)

    def update(self):
        """Compare input variable to stored value."""
        if self.world_interface.get_handle_angle() > self.value:
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Parse behavior parameters from string """
        parameters = []
        values = re.findall(r'-?\d+\.\d+|-?\d+', node_descriptor[13:])
        if len(values) > 0:
            parameters.append(float(values[0]))
        else:
            parameters.append(0.0)
        return parameters


class DoorAngle(pt.behaviour.Behaviour):
    """ Check angle of door. """
    def __init__(
            self, name: str,
            parameters: list,
            world_interface: Any,
            _verbose: bool = False):
        self.world_interface = world_interface
        self.value = float(parameters[0])
        super().__init__(name)

    def update(self):
        """Compare input variable to stored value."""
        if self.world_interface.get_door_angle() > self.value:
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Parse behavior parameters from string """
        parameters = []
        values = re.findall(r'-?\d+\.\d+|-?\d+', node_descriptor[12:])
        if len(values) > 0:
            parameters.append(float(values[0]))
        else:
            parameters.append(0.0)
        return parameters


class AngleCheck(pt.behaviour.Behaviour):
    """Check angle of target object."""
    def __init__(
        self, name: str,
        parameters: list,
        world_interface: Any,
        _verbose: bool = False
    ):
        self.target_object = parameters[0]
        self.world_interface = world_interface
        self.value = float(parameters[1])
        super().__init__(name)

    def update(self):
        if self.world_interface.object_at_yaw(self.value, self.target_object):
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Parse behavior parameters from string """
        parameters = []
        eq_pos = node_descriptor.find('=')
        target_object = node_descriptor[0:eq_pos - 7]
        parameters.append(target_object)
        values = re.findall(r'-?\d+\.\d+|-?\d+', node_descriptor[eq_pos + 3:])
        if len(values) > 0:
            parameters.append(float(values[0]))
        else:
            parameters.append(0.0)

        return parameters


class Aligned(pt.behaviour.Behaviour):
    """Check if target object is aligned."""
    def __init__(
        self, name: str,
        parameters: list,
        world_interface: Any,
        _verbose: bool = False
    ):
        self.target_object = parameters[0]
        self.world_interface = world_interface
        super().__init__(name)

    def update(self):
        if self.world_interface.is_object_aligned(self.target_object):
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Parse behavior parameters from string """
        parameters = []
        question_mark_pos = node_descriptor.find('?')
        if question_mark_pos > 0:
            target_object = node_descriptor[8:question_mark_pos]
        else:
            target_object = node_descriptor[8:]
        parameters.append(target_object)

        return parameters


class Atomic(common_behaviors.Behavior):
    """ Execute atomic action. This behavior uses an offset to current position in pos and yaw"""

    def __init__(
        self,
        name: str,
        parameters: list,
        world_interface: Any,
        verbose: bool = False
    ):
        super().__init__(name, world_interface, verbose, max_ticks=30)
        self.target_object = parameters[0]
        self.pos = parameters[1]
        self.yaw = parameters[2]
        self.gripper = parameters[3]

    def check_for_success(self):
        """Check if at target position."""
        if self.world_interface.object_at_pos(self.target_object,
                                              list(map(add, self.world_interface.get_object_position('none'),
                                                       self.pos))) and \
           self.world_interface.object_at_yaw(self.yaw, self.target_object) and \
           self.world_interface.gripper_at_ref(self.gripper):
            self.success()

    def update(self):
        """Executes behavior. Scales position offset by 5 to make it reasonably fast """
        self.check_for_success()
        super().update()
        if self.state is pt.common.Status.RUNNING:
            self.world_interface.set_action_type(ActionTypes.ATOMIC)
            self.world_interface.set_target_and_offset(None,
                                                       [5 * sum(x) for x in
                                                        zip(self.world_interface.get_object_position('none'),
                                                            self.pos,
                                                            -self.world_interface.get_object_position(self.target_object))],
                                                       normalize_around_zero=True)
            self.world_interface.set_yaw((self.yaw - self.world_interface.get_object_yaw(self.target_object)))
            self.world_interface.set_gripper(self.gripper)

        return self.state

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Parse behavior parameters from string """
        parameters = []
        parenthesis_pos = node_descriptor.find('(')
        if parenthesis_pos > 0:
            target_object = node_descriptor[7:parenthesis_pos - 1]
        else:
            target_object = node_descriptor[7:]
        parameters.append(target_object)
        numbers = list(map(float, re.findall(r'-?\d+\.\d+|-?\d+', node_descriptor)))
        if len(numbers) > 2:
            offset = tuple(numbers[0:3])
        else:
            offset = (0.0, 0.0, 0.0)
        parameters.append(offset)
        if len(numbers) > 3:
            angle = numbers[3]
        else:
            angle = 0.0
        parameters.append(angle)
        if len(numbers) > 4:
            gripper = numbers[4]
        else:
            gripper = 0.0
        parameters.append(gripper)
        return parameters

PREDEFINED_OBS_LOCATIONS = [(-0.1,0.4,0.02), (-0.05,0.4,0.02), (0.0,0.4,0.02), (0.05,0.4,0.02), (0.1,0.4,0.02),
                            (-0.1,0.3,0.02), (-0.05,0.3,0.02), (0.0,0.3,0.02), (0.05,0.3,0.02), (0.1,0.3,0.02)]
RANDOM_LOCATION_LIMITS = [(-0.1, 0.1), (0.2, 0.4), (0.01, 0.03)]  # (x, y, z) limits for random generation

class Reach(common_behaviors.Behavior):
    """
    Reach for target position
    Will add to the target position also the vector between robot end effector
    and currently grasped object so that the grasped object (if any) is the one reaching
    the target position
    """

    def __init__(
        self,
        name: str,
        parameters: list,
        world_interface: Any,
        verbose: bool = False
    ):
        super().__init__(name, world_interface, verbose, max_ticks=1)
        self.target_object = parameters[0]
        self.offset = parameters[1]
        self.target_pos = np.array([0.0, 0.0, 0.0])
        self.target_pos_global = None
        self.last_grasped_object = "none"

    def calculate_target_pos(self):
        """
        Calculates the target position by adding also the vector between robot end effector
        and currently grasped object so that the grasped object (if any) is the one reaching
        the target position
        """
        grasped_object = self.world_interface.get_grasped_object()
        if self.last_grasped_object != grasped_object:
            # New object grasped, treat it as starting from scratch
            self.last_grasped_object = grasped_object
            self.counter = 0
        grasp_to_robot_vector = np.array([0.0, 0.0, 0.0])
        if grasped_object != "none":
            grasp_to_robot_vector += self.world_interface.get_object_position("robot_ee")
            grasp_to_robot_vector -= self.world_interface.get_object_position(grasped_object)
            frame_yaw = self.world_interface.get_object_yaw(self.target_object)
            if frame_yaw != 0.0:
                # Rotate so that offset in in target_object frame
                grasp_to_robot_vector = rotate_frame_z(grasp_to_robot_vector, -frame_yaw)

        self.target_pos = grasp_to_robot_vector + self.offset
        # Check if there is an object at the target position
        if self.world_interface.is_object_at_position(self.target_pos, threshold=0.1) is not None:
            # If there is a collision, try predefined locations
            for predefined_loc in PREDEFINED_OBS_LOCATIONS:
                test_target_pos = grasp_to_robot_vector + np.array(predefined_loc)
                if self.world_interface.is_object_at_position(test_target_pos, threshold=0.1) is None:
                    # Found a collision-free location
                    self.target_pos = test_target_pos
                    # print(f"Using predefined location {predefined_loc} due to collision at initial target position.")
                    return

            # If no collision-free predefined location is found, try random positions up to 100 times
            for _ in range(100):
                random_loc = np.array([
                    np.random.uniform(*RANDOM_LOCATION_LIMITS[0]),
                    np.random.uniform(*RANDOM_LOCATION_LIMITS[1]),
                    np.random.uniform(*RANDOM_LOCATION_LIMITS[2]),
                ])
                test_target_pos = grasp_to_robot_vector + random_loc
                if self.world_interface.is_object_at_position(test_target_pos, threshold=0.1) is None:
                    # Found a collision-free random location
                    self.target_pos = test_target_pos
                    # print(f"Using random location {random_loc} due to collision at predefined locations.")
                    return

            # If no collision-free location is found, raise an error
            raise ValueError("Unable to find a collision-free target position after 100 random attempts.")

    def initialise(self) -> None:
        super().initialise()
        self.calculate_target_pos()
        self.target_pos_global = None

    def check_for_success(self):
        """Check if at target position."""
        if self.last_grasped_object == self.target_object and self.target_pos_global is not None:
            # If we are making a movement relative the grasped object, do not move the goalpost
            if self.world_interface.at_pos(None, self.target_pos_global):
                self.success()
        else:
            if self.world_interface.at_pos(self.target_object, self.target_pos):
                self.success()

    def update(self):
        """Executes behavior """
        self.check_for_success()
        super().update()

        if self.state == pt.common.Status.RUNNING:
            self.world_interface.set_action_type(ActionTypes.REACH)
            self.calculate_target_pos()
            self.target_pos_global = self.world_interface.set_target_and_offset(self.target_object, self.target_pos)
        return self.state

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Parse behavior parameters from string """
        parameters = []
        parenthesis_pos = node_descriptor.find('(')
        if parenthesis_pos > 0:
            target_object = node_descriptor[6:parenthesis_pos - 1]
        else:
            target_object = node_descriptor[6:]
        parameters.append(target_object)
        numbers = list(map(float, re.findall(r'-?\d+\.\d+|-?\d+', node_descriptor)))
        if len(numbers) > 2:
            offset = tuple(numbers[0:3])
        else:
            offset = (0.0, 0.0, 0.0)
        parameters.append(offset)
        return parameters


class Grasp(common_behaviors.Behavior):
    """
    Grasp target object.
    """

    def __init__(
        self,
        name: str,
        parameters: list,
        world_interface: Any,
        verbose: bool = False
    ):
        super().__init__(name, world_interface, verbose, max_ticks=1)
        self.target_object = parameters[0]
        self.offset = parameters[1]
        self.yaw = parameters[2]

    def check_for_success(self):
        """Check if at target position."""
        if self.world_interface.at_pos(self.target_object, self.offset) and \
           self.world_interface.at_yaw(self.yaw, self.target_object) and \
           self.world_interface.gripper_at_ref(1.0):
            self.success()
            self.world_interface.set_grasped_object(self.target_object)
            self.world_interface.set_graspable_objects(self.target_object)
    def update(self):
        """Executes behavior """
        self.check_for_success()
        super().update()

        if self.state == pt.common.Status.RUNNING:
            self.world_interface.set_action_type(ActionTypes.GRASP)
            self.world_interface.set_target_and_offset(self.target_object, self.offset)
            self.world_interface.set_yaw(self.yaw, self.target_object)
        # print ("GRASP State", self.state)
        return self.state

    @staticmethod
    def parse_parameters(node_descriptor):
        # """ Parse behavior parameters from string. Old implementation """
        # parameters = []
        # parenthesis_pos = node_descriptor.find('(')
        # if parenthesis_pos > 0:
        #     target_object = node_descriptor[6:parenthesis_pos - 1]
        # else:
        #     target_object = node_descriptor[6:]
        # parameters.append(target_object)
        # numbers = list(map(float, re.findall(r'-?\d+\.\d+|-?\d+', node_descriptor)))
        # if len(numbers) > 2:
        #     offset = tuple(numbers[0:3])
        # else:
        #     offset = (0.0, 0.0, 0.0)
        # parameters.append(offset)
        # if len(numbers) > 3:
        #     angle = numbers[3]
        # else:
        #     angle = 0.0
        # parameters.append(angle)
        # return parameters

        """Parse behavior parameters from string. New implementation """
        parameters = []

        # Split by spaces while keeping parts inside parentheses intact
        parts = node_descriptor.strip().split(' ', 2)

        if len(parts) == 3 and '(' in parts[2]:  # Handles case 1: 'grasp obstacle none (0.0, -0.15, 0.1)'
            target_object = parts[1]
            remaining = parts[2].strip()
            # Split remaining by the first occurrence of '(' to separate the relative object and offset
            relative_object, offset_str = remaining.split('(', 1)
            relative_object = relative_object.strip()
            # Extract offset as a tuple of floats
            offset = tuple(map(float, re.findall(r'-?\d*\.?\d+', f"({offset_str}")))
            if len(offset) < 3:
                offset = (0.0, 0.0, 0.0)  # Default if offset is not parsed correctly
            elif len(offset) > 3:
                offset = offset[0:3]
            angle = 0.0  # Default angle
            # Check if there is an angle part after the offset
            if ')' in offset_str:
                angle_part = offset_str.split(')')[1].strip()
                if angle_part:
                    angle = float(angle_part)
            parameters.extend([target_object, offset, angle])
        elif len(parts) == 2:  # Handles case 2: 'grasp peg'
            target_object = parts[1]
            relative_object = 'none'  # Set relative object as 'none'
            offset = f'{target_object}_INITIAL'  # Set offset as 'peg_INITIAL'
            angle = 0.0  # Default angle if not specified
            parameters.extend([target_object, offset, angle])

        return parameters
    
class Push(common_behaviors.Behavior):
    """ Push to an offset"""

    def __init__(
        self,
        name: str,
        parameters: list,
        world_interface: Any,
        verbose: bool = False
    ):
        super().__init__(name, world_interface, verbose, max_ticks=1)
        self.target_object = parameters[0]
        self.offset = parameters[1]
        self.yaw = parameters[2]
        self.delta_offset = list(parameters[3])
        self.delta_offset[2] -= 0.02  # Default press downwards
        self.success_on_next = False

    def initialise(self) -> None:
        super().initialise()
        self.success_on_next = False

    def check_for_success(self):
        """ Push always runs once and then reports success."""
        if self.success_on_next:
            self.success()
        self.success_on_next = True

    def update(self):
        """ Executes behavior """
        self.check_for_success()
        super().update()

        if self.state == pt.common.Status.RUNNING:
            self.world_interface.set_action_type(ActionTypes.PUSH)
            self.world_interface.set_target_and_offset(self.target_object, self.offset, True)
            self.world_interface.set_yaw(self.yaw, None)
            self.world_interface.set_delta_offset(self.delta_offset)
        return self.state

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Parse behavior parameters from string """
        parameters = []
        parenthesis_pos = node_descriptor.find('(')
        if parenthesis_pos > 0:
            target_object = node_descriptor[5:parenthesis_pos - 1]
        else:
            target_object = node_descriptor[5:]
        parameters.append(target_object)
        numbers = list(map(float, re.findall(r'-?\d+\.\d+|-?\d+', node_descriptor)))
        if len(numbers) > 2:
            offset = tuple(numbers[0:3])
        else:
            offset = (0.0, 0.0, 0.0)
        parameters.append(offset)
        if len(numbers) > 3:
            angle = numbers[3]
        else:
            angle = 0.0
        parameters.append(angle)
        if len(numbers) > 6:
            delta_offset = tuple(numbers[4:8])
        else:
            delta_offset = (0.0, 0.0, 0.0)
        parameters.append(delta_offset)
        return parameters


class PushTowards(common_behaviors.Behavior):
    """Push towards a position"""

    def __init__(
        self,
        name: str,
        parameters: list,
        world_interface: Any,
        verbose: bool = False
    ):
        super().__init__(name, world_interface, verbose, max_ticks=1)
        self.target_object = parameters[0]
        self.goal_position = parameters[1]
        self.distance = parameters[2]
        self.start_offset = np.array([0.0, 0.0, 0.0])
        self.end_offset = np.array([0.0, 0.0, -0.02])  # Default press downwards
        self.success_on_next = False

    def calculate_start_position(self):
        """
        Calculates the start position by drawing a line from the goal position
        through the object. Start is offset by half the distance
        """
        object_position = self.world_interface.get_object_position(self.target_object)
        self.start_offset[0] = object_position[0] - self.goal_position[0]
        self.start_offset[1] = object_position[1] - self.goal_position[1]

        # Scaling to get to start offset length distance / 2
        start_offset_scaling = (self.distance / 2) / sqrt(self.start_offset[0] * self.start_offset[0] +
                                                          self.start_offset[1] * self.start_offset[1])
        self.start_offset[0] *= start_offset_scaling
        self.start_offset[1] *= start_offset_scaling

        self.end_offset[0] = self.goal_position[0] - object_position[0] - self.start_offset[0]
        self.end_offset[1] = self.goal_position[1] - object_position[1] - self.start_offset[1]

        # Scaling to get to correct distance
        end_offset_scaling = self.distance / sqrt(self.end_offset[0] * self.end_offset[0] +
                                                  self.end_offset[1] * self.end_offset[1])
        if end_offset_scaling < 1:
            self.end_offset[0] *= end_offset_scaling
            self.end_offset[1] *= end_offset_scaling

    def initialise(self) -> None:
        super().initialise()
        self.success_on_next = False

    def check_for_success(self):
        """Check push always runs once and then reports success."""
        if self.success_on_next:
            self.success()
        self.success_on_next = True

    def update(self):
        """Executes behavior """
        self.check_for_success()
        super().update()

        if self.state == pt.common.Status.RUNNING:
            self.calculate_start_position()
            self.world_interface.set_action_type(ActionTypes.PUSH)
            self.world_interface.set_target_and_offset(self.target_object, self.start_offset, True)
            self.world_interface.set_yaw(1.57, None) #TODO needs to be parameterized
            self.world_interface.set_delta_offset(self.end_offset)

        return self.state

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Parse behavior parameters from string """
        parameters = []
        parenthesis_pos = node_descriptor.find('(')
        if parenthesis_pos > 0:
            target_object = node_descriptor[5:parenthesis_pos - 1]
        else:
            target_object = node_descriptor[5:]
        parameters.append(target_object)
        numbers = list(map(float, re.findall(r'-?\d+\.\d+|-?\d+', node_descriptor)))
        if len(numbers) > 2:
            goal_position = tuple(numbers[0:3])
        else:
            goal_position = (0.0, 0.0, 0.0)
        parameters.append(goal_position)
        if len(numbers) > 3:
            distance = numbers[3]
        else:
            distance = 0.0
        parameters.append(distance)
        return parameters


class Open(common_behaviors.Behavior):
    """Opens gripper."""

    def __init__(
        self,
        name: str,
        _parameters: list,
        world_interface: Any,
        verbose: bool = False
    ):
        super().__init__(name, world_interface, verbose, max_ticks=1)

    def check_for_success(self):
        """Check if at target position."""
        if self.world_interface.gripper_at_ref(-1.0):
            self.success()

    def update(self):
        """Executes behavior """
        self.check_for_success()
        super().update()

        if self.state == pt.common.Status.RUNNING:
            self.world_interface.set_action_type(ActionTypes.OPEN)
            self.world_interface.set_gripper(-1.0)
            self.world_interface.set_grasped_object("none")
        return self.state

    @staticmethod
    def parse_parameters(_node_descriptor):
        """ Parse behavior parameters from string """
        return []

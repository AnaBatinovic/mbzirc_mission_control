#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import os.path
import logging
from copy import deepcopy, copy
from collections import defaultdict
from itertools import islice, izip
from math import radians

import networkx as nx
import numpy as np
from geographic_msgs.msg import GeoPose, GeoPoint
from geometry_msgs.msg import Quaternion, Pose
from multiset import Multiset
from tf.transformations import quaternion_from_euler

# ******************************************************************************
# *********************** MODIFY CODE STARTING HERE ****************************
def direction_score_function(object, layer, direction=0):
    simplicity = {'R': 6, 'G': 3, 'B': 1.5, 'O': 1}
    object.brick_score = {'R': 6, 'G': 8, 'B': 10, 'O': 0}

    # Simplicity based score
    lr_score = 0
    rl_score = 0
    position = 1
    for brick in layer:
        lr_score += (object.num_of_bricks - position) * simplicity[brick.color]
        rl_score += (position - 1) * simplicity[brick.color]
        position += 1

    dir = 1 if lr_score >= rl_score else -1

    if direction == 0:
        return dir, max(lr_score, rl_score)
    elif direction == 1:
        return direction, lr_score
    elif direction == -1:
        return direction, rl_score

def brick_priority_function(object, node):
    imidiate_score = object.nodes[node]['points']
    child_score = sum([object.nodes[succ]['points'] for succ in object.successors(node)])
    layer = int(node.split('.')[1])

    # Total score has 5 digits: 12345.
    # Digits 1 and 2 are reserved for layer score.
    # Digits 3, 4, and 5 are reserved for points score
    p1 = 0.7
    p2 = 0.3
    layer_score = (-0.2 * layer + 1.2) * 10000
    point_score = (p1 * imidiate_score + p2 * child_score) * 100
    total_score = layer_score + point_score

    return total_score
# ******************************************************************************
# ******************************************************************************

class Brick(object):
    """
    Class that represent a single brick in the wall.

    Attributes:
        color (str): Color of the brick (R, G, B, O).
        size (float): Length of the brick.
        begin (float): Starting position of the brick.
        end (float): Ending position of the brick.
        ID (str): Identifier of the brick.
    """

    def __init__(self, color, wall_num, begin, size, ID=None):
        """Initialize class variables."""
        self.color = color
        self.size = size
        self.begin = begin
        self.end = begin + size
        self.wall_num = wall_num
        self.ID = ID

    def is_on_top_of(self, other):
        """
        Check for overlap between two bricks.

        Args:
            other (Brick): Other brick.

        Returns:
            True if bricks overlap, False otherwise.
        """
        self.end = self.begin + self.size
        other.end = other.begin + other.size

        conversion_scale = 10
        begin1 = int(round(self.begin * conversion_scale))
        end1 = int(round(self.end * conversion_scale))
        begin2 = int(round(other.begin * conversion_scale))
        end2 = int(round(other.end * conversion_scale))

        if begin2 < begin1 < end2:
            return True
        if begin2 < end1 < end2:
            return True
        if begin1 <= begin2 and end1 >= end2:
            return True
        return False

    def __str__(self):
        return 'ID: {}, span: {} - {}'.format(self.ID, self.begin, self.end)

    def __repr__(self):
        return self.color


class WallParser(object):
    """
    Class for parsing a wall description file into a TAEMS mission structure.

    Attributes:
        wall_pattern (dict): Everything needed to describe the wall.
        tasks (OrderedDict): Holds every TAEMS task.
        IRs (OrderedDict): Holds every TAEMS IR.
        resources (OrderedDict): Holds every TAEMS resource.
        imidiate_subtasks (list): Subtasks of root task.
        root_label (str): Label of the root task.
        use_simple_IRs (bool): Select complex or simplified version of IRs.
        wall_structure (list): Wall structure described wiht Brick objects.
        brick_positions (dict): Positions of the bricks relative to the origin.
    """

    def __init__(self, wall_pattern_file, file_options, construction_options):
        """
        Initialize class variables.

        Args:
            wall_pattern_file (str): Path to the file containing the desired wall pattern.
            file_options (dict): Key-value pairs for defining the properties of the wall pattern file.
            construction_options (dict): Key-value pairs for defining the properties of the wall.
            yaml_string (str): If set, wall pattern is read directly from the string instead of the provided file.
        """
        # Load data describing the desired wall structure.
        self.wall_pattern = {}

        if os.path.exists(wall_pattern_file):
            with open(wall_pattern_file, 'r') as stream:
                wall_pattern = stream.readlines()
        else:
            wall_pattern = wall_pattern_file.strip('\n').split('\n')

        # Options and parameters
        self.construction_options = construction_options
        self.pattern_options = file_options
        self.wall_directions = deepcopy(construction_options['build_directions'])
        self.brick_score = construction_options['brick_points']

        self.num_of_bricks = self.pattern_options['num_bricks']  # Number of bricks in a single layer of the single segment
        self.num_of_segments = self.pattern_options['num_segments']  # Number of wall segments

        layer_num = 1
        for layer in reversed(wall_pattern):
            layer = layer.replace('\n', '')
            for segment in range(self.num_of_segments):
                sequence = layer[segment * self.num_of_bricks * 2:(segment + 1) * self.num_of_bricks * 2]
                self.wall_pattern[(segment + 1, layer_num)] = sequence
            layer_num += 1

        # Variables for generating wall diagram within GUI.
        self.wall_structure = defaultdict(list)  # Dict of lists of lists of Brick elements.

        # Variables for generating building specification.
        self.brick_positions = {}  # {ID: [x, y, z] position relative to the origin of the wall.

        # Additional variables for reporting to other parts of the system.
        self.report_relations = defaultdict(list)
        self.report_tasks = defaultdict(list)
        self.report_score = dict()

    def reinitialize(self, wall_pattern_file, file_options, construction_options):
        """
        Reinitialize the WallParser object.

        Args:
            wall_pattern_file (str): Path to the file containing the desired wall pattern.
            file_options (dict): Key-value pairs for defining the properties of the wall pattern file.
            construction_options (dict): Key-value pairs for defining the properties of the wall.
            yaml_string (str): If set, wall pattern is read directly from the string instead of the provided file.
        """
        self.__init__(wall_pattern_file, file_options, construction_options)

    def parse_wall(self):
        """
        Parse data from loaded .yaml specification file and store it in class variables.

        - Save Brick objects to self.wall_structure -> this will be used for drawing.
        - Calculate position of each brick within in the wall and save to self.brick_positions -> this will be used to
          generate building instructions.
        """
        # TODO: put this in a configuration file
        brick_lengths = self.construction_options['brick_lengths']
        brick_height = self.construction_options['brick_height']
        horizontal_spacing = self.construction_options['horizontal_spacing']

        # Start from the bottom - the way bricks are built.
        for key, row in sorted(self.wall_pattern.items(), key=lambda x: x[0]):
            this_row = []
            position = 0
            brick_index = 1
            wall_index = int(key[0])
            row_index = int(key[1])
            if row is not None:
                for color in row.split():
                    if color not in ['R', 'G', 'B', 'O']:
                        raise ValueError('Unknown brick color: {}'.format(color))

                    begin = position
                    size = brick_lengths[color]
                    ID = '{}_{}.{}.{}'.format(color, wall_index, row_index, brick_index)
                    self.report_tasks[wall_index].append(ID)
                    this_row.append(Brick(color, wall_index, begin, size, ID))
                    brick_index += 1

                    # Save position of each brick. This will be used for building the wall.
                    self.brick_positions[ID] = [round(begin + size / 2.0, 2),
                                                0,
                                                round((row_index - 0.5) * brick_height, 2)]

                    position += size + horizontal_spacing

            self.wall_structure[wall_index].append(this_row)


        self.determine_optimal_directions()
        self.generate_relations()

    def generate_relations(self):
        print("###### WALL PATTERN ######")
        for key, single_wall_structure in self.wall_structure.items():
            score = self.report_score[key]
            print('WALL {}: score {}'.format(key, score))
            for layer in reversed(single_wall_structure):
                print(layer)
            print
        print('##########################')


        # Interrelationships based on vertical placement.
        for wall_index, single_wall_structure in self.wall_structure.items():
            for current_row, row_below in izip(islice(single_wall_structure, 1, None), single_wall_structure):
                for current_brick in current_row:
                    for brick_below in row_below:
                        # TODO: optimizirati prolzak kroz petlju
                        if current_brick.end <= brick_below.begin:
                            break
                        if current_brick.is_on_top_of(brick_below):
                            self.report_relations[wall_index].append((brick_below.ID, current_brick.ID))

        # Interrelationships based on horizontal placement.
        for wall_index, single_wall_structure in self.wall_structure.items():
            for current_row in single_wall_structure:
                for i in range(len(current_row) - 1):
                    j = 1
                    while current_row[i + j].ID is None:
                        j += 1
                        if i + j >= len(current_row):
                            break
                    if current_row[i].ID is None or current_row[i + j].ID is None:
                        continue
                    # If the order of placement is reversed, reverse the horizontal interrelationships.
                    if self.wall_directions['segment_{}'.format(wall_index)] == 1:
                        self.report_relations[wall_index].append((current_row[i].ID, current_row[i + j].ID))
                    else:
                        self.report_relations[wall_index].append((current_row[i + j].ID, current_row[i].ID))

    def modify_pattern(self, wall_range, layer, color, fixed_tasks):
        horizontal_spacing = self.construction_options['horizontal_spacing']

        for wall_index in wall_range:
            move = []
            hold = []
            for brick in self.wall_structure[wall_index][layer - 1]:
                if brick.color == color and brick.ID not in fixed_tasks:
                    move.append(brick)
                else:
                    hold.append(brick)

            if self.wall_directions['segment_' + str(wall_index)] == 1:
                self.wall_structure[wall_index][layer - 1] = hold + move
            else:
                self.wall_structure[wall_index][layer - 1] = move + hold

            position = 0
            for brick in self.wall_structure[wall_index][layer - 1]:
                brick.begin = position
                self.brick_positions[brick.ID][0] = round(brick.begin + brick.size / 2.0, 2)
                position += brick.size + horizontal_spacing

        self.report_relations = defaultdict(list)
        self.generate_relations()

        return map(lambda x: x.ID, move)

    def determine_optimal_directions(self):
        self.wall_directions = deepcopy(self.construction_options['build_directions'])
        for key in self.wall_directions:
            # 0 means direction should be automatically determined, -1 and 1 mean that the user decides.
            specified_direction = self.wall_directions[key]
            if int(key[-1]) in self.wall_structure:
                wall_segment = self.wall_structure[int(key[-1])]
                if wall_segment:
                    direction, score = self.calculate_dir_score(wall_segment[0], specified_direction)
                    self.wall_directions[key] = direction
                    self.report_score[int(key[-1])] = score

    def calculate_dir_score(self, layer, direction=0):
        return direction_score_function(self, layer, direction)


class PrecedenceGraph(nx.DiGraph):
    def __init__(self, relations, tasks=None, **kwargs):
        super(PrecedenceGraph, self).__init__()

        if isinstance(tasks, dict):
            for task_name, value in tasks.items():
                self.add_node(task_name, points=value)
        elif isinstance(tasks, list):
            self.add_nodes_from(tasks, points=1)
        elif tasks is None:
            pass
        else:
            raise NotImplementedError('Unsupported type {} for argument tasks!'.format(type(tasks)))

        for rel in relations:
            self.add_edge(rel[0], rel[1])

        self.Tf = []
        self.Tl = []
        self.Th = []

    def __str__(self):
        return str(nx.to_numpy_matrix(self))

    @property
    def tasks(self):
        return self.nodes.keys()

    @property
    def Tf_sorted(self):
        return sorted(self.Tf, key=self.calc_priority, reverse=True)

    def update_layers(self):
        # TODO: optimize
        self.Tf = set()
        self.Tl = set()
        self.Th = set()
        temp = deepcopy(self)
        for node in self.nodes:
            num_predecessors = len(list(self.predecessors(node)))
            if num_predecessors == 0:
                self.Tf.add(node)
                temp.remove_node(node)
        for node in temp.nodes:
            num_predecessors = len(list(temp.predecessors(node)))
            if num_predecessors == 0:
                self.Tl.add(node)
        self.Th = set(self.nodes) - self.Tl - self.Tf

        self.Tf = list(self.Tf)
        self.Tl = list(self.Tl)
        self.Th = list(self.Th)

    def get_priorities(self, nodes):
        if nodes:
            priorities = [(node, self.calc_priority(node)) for node in nodes]
            best = max(priorities, key=lambda x: x[1])
            return best, priorities
        return (None, 0), []

    def calc_priority(self, node):
        return brick_priority_function(self, node)

    def get_enabled_nodes_continuous(self, n, in_place, filter=None, priority_function=None):
        """
        Return N nodes without predecessors, in order defined by their priority.

        If desired number of nodes (N) is larger than number of nodes without
        predecessors (M), update the graph by removing M nodes and try to return
        remaining (N-M) nodes whos predecessors where in the original M nodes.
        Repeat the process if needed.
        Nodes are ordered in a list based on their priority.

        Args:
            n (int): Number of nodes to return.
            in_place (bool): If true, operations are done in place. THIS WILL REMOVE NODES FROM GRAPH!
                             If false, a copy is created. THIS IS INCONSISTENT BETWEEN TWO CALLS!
            filter (dict): Filter for discarding unwanted nodes.
            priority_function: Function for calculating priority.

        Returns:
            List of enabled nodes.
        """
        assert n > 0, 'Number of nodes to return must be greater than zero!'

        if in_place:
            temp = self
        else:
            temp = deepcopy(self)

        enabled = []
        if priority_function is None:
            priority_function = self.calc_priority

        while len(enabled) < n and len(temp.nodes) > 0:
            temp.update_layers()
            temp.Tf.sort(key=priority_function, reverse=True)
            self.filter_out(temp.Tf, filter)
            if not temp.Tf:
                break
            enabled.append(temp.Tf[0])
            temp.remove_node(temp.Tf[0])

        return enabled

    def get_enabled_nodes(self, n, in_place, filter=None, priority_function=None):
        """
        Return N nodes without predecessors, in order defined by their priority.

        If desired number of nodes (N) is larger than number of nodes without
        predecessors (M), return only M nodes. If N = -1, also return all M
        nodes.
        Nodes are ordered in a list based on their priority.

        Args:
            n (int): Number of nodes to return.
            in_place (bool): If true, operations are done in place. THIS WILL REMOVE NODES FROM GRAPH!
                             If false, a copy is created. THIS IS INCONSISTENT BETWEEN TWO CALLS!
            filter (dict): Filter for discarding unwanted nodes.
            priority_function: Function for calculating priority.

        Returns:
            List of enabled nodes.
        """
        if in_place:
            temp = self
        else:
            temp = deepcopy(self)

        enabled = []
        if priority_function is None:
            priority_function = self.calc_priority

        temp.update_layers()
        temp.Tf.sort(key=priority_function, reverse=True)
        self.filter_out(temp.Tf, filter)
        if temp.Tf:
            enabled.extend(temp.Tf)
            temp.remove_nodes_from(temp.Tf)

        return enabled


    def filter_out(self, nodes, filter):
        nodes_orig = deepcopy(nodes)
        if filter is not None:
            for node in nodes:
                if 'color' in filter:
                    for item in list(filter['color']):
                        if node[0] == item:
                            try:
                                nodes.remove(node)
                            except ValueError as e:
                                print('### FILTER_OUT FAILED WITH ERROR: %s' % e)
                                print('### ORIGINAL NODE LIST: %s' % nodes_orig)
                                print('### CURRENT NODE LIST: %s' % nodes)
                                print('### NODE TO REMOVE: %s' % node)


class ColoredFormatter(logging.Formatter):
    # These are the sequences need to get colored ouput
    RESET_SEQ = "\033[0m"
    BOLD_SEQ = "\033[1m"

    COLORS = {
        'WARNING': "\033[38;5;130m",
        'INFO': "\033[96m",
        'DEBUG': "\033[38;5;2m",
        'CRITICAL': "\033[31m",
        'ERROR': "\033[31m",
    }

    LEVELS = {
        'DEBUG': logging.DEBUG,
        'INFO': logging.INFO,
        'WARN': logging.WARNING,
    }

    def __init__(self, msg):
        logging.Formatter.__init__(self, msg)

    def format(self, record):
        skip_line = False
        if record.msg[0] == '\n':
            skip_line = True
            record.msg = record.msg[1:]
        if record.msg[0] == '!':
            bold = ColoredFormatter.BOLD_SEQ
            record.msg = record.msg[1:]
        else:
            bold = ''
        result = logging.Formatter.format(self, record)
        result = bold + ColoredFormatter.COLORS[record.levelname] + result + ColoredFormatter.RESET_SEQ
        if skip_line:
            result = '\n' + result
        return result

class CustomLogger(logging.Logger):
    def __init__(self, name='default', level='DEBUG'):
        super(CustomLogger, self).__init__(name, ColoredFormatter.LEVELS[level])

        # create console handler and set level to debug
        ch = logging.StreamHandler()
        ch.setLevel(level)

        # create formatter
        formatter = ColoredFormatter('[%(levelname)s] [%(name)s]> %(message)s')

        # add formatter to ch
        ch.setFormatter(formatter)

        # add ch to logger
        self.addHandler(ch)


class Basket(object):
    def __init__(self, name='Basket'):
        self.name = name
        self.combinations = [Multiset('BG'),
                             Multiset('GG'),
                             Multiset('BRR'),
                             Multiset('GRR'),
                             Multiset('RRRR'),
                             Multiset('O')]
        self.bricks = Multiset()
        self.inside = []

    def __str__(self):
        return str(self.inside)

    def __repr__(self):
        return self.__str__()

    def add(self, brick):
        if brick in self.inside:
            print("BRICK {} IS ALREADY IN THE BASKET {}".format(brick, self.name))
            return True

        for option in self.combinations:
            if self.bricks.issubset(option):
                remaining = option - self.bricks
                if brick[0] in remaining:
                    self.bricks.add(brick[0])
                    self.inside.append(brick)
                    return True
        return False

    def add_from(self, bricks):
        for brick in bricks:
            self.add(brick)

    def update_state(self, ohter_basket):
        self.bricks = deepcopy(ohter_basket.bricks)

    def remove(self, brick):
        if brick in self.inside:
            self.bricks.remove(brick[0], multiplicity=1)
            self.inside.remove(brick)
            return True
        else:
            return False

    def remove_from(self, bricks):
        failed = []
        for brick in bricks:
            if not self.remove(brick):
                failed.append(brick)
        return failed if failed else True

    def empty(self):
        self.bricks.clear()
        self.inside = []

    def is_full(self):
        for option in self.combinations:
            if self.bricks == option:
                return True
        return False

    def is_empty(self):
        return len(self.bricks) == 0


def list_to_msg(type, array, add=None, override=None):
    array = copy(array)
    if add is not None:
        assert len(array) == len(add)
        array += add

    if override is not None:
        assert len(array) == len(override)
        for i in range(len(array)):
            if not np.isnan(override[i]):
                array[i] = override[i]

    if type == 'Pose':
        msg = Pose()
        msg.position.x = array[0]
        msg.position.y = array[1]
        msg.position.z = array[2]
        if len(array) == 4:
            msg.orientation = Quaternion(*quaternion_from_euler(0, 0, radians(array[3])))
        else:
            msg.orientation.w = 1
    elif type == 'GeoPoint':
        msg = GeoPoint()
        msg.latitude = array[0]
        msg.longitude = array[1]
        msg.altitude = array[2]
    elif type == 'GeoPose':
        msg = GeoPose()
        msg.position.latitude = array[0]
        msg.position.longitude = array[1]
        msg.position.altitude = array[2]
        if len(array) == 4:
            msg.orientation = Quaternion(*quaternion_from_euler(0, 0, radians(array[3])))
        else:
            msg.orientation.w = 1
    else:
        raise ValueError('Unknown type: {}'.format(type))

    return msg

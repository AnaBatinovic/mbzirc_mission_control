#!/usr/bin/env python
# -*- coding: utf-8 -*-
import itertools
from copy import deepcopy

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from mbzirc_mission_control import WallParser, PrecedenceGraph, CustomLogger, Basket
from mbzirc_mission_control.srv import PickupUGV, PickupUGVRequest, PickupUGVResponse, PlaceUGV, PlaceUGVRequest, PlaceResponse

logger = CustomLogger()


class WallCoordinatorUGV(object):
    def __init__(self):
        # Create predefined class variables.
        self.brick_colors = {'R': 'red', 'G': 'green', 'B': 'blue', 'O': 'orange'}
        self.failed_tasks = []
        self.completed_tasks = []
        self.basket = Basket('main')
        self.fail_monitor = {'R': 0, 'G': 0, 'B': 0, 'O': 0}  # brick_color: number of recent fails

        self.should_restart = False

        # Get all necessary parameters for parsing the wall.
        self.wall_pattern_file = rospy.get_param('CH2/wall_pattern')
        self.pattern_file_options = rospy.get_param('CH2/config/pattern_file_options')
        self.construction_options = rospy.get_param('CH2/config/construction_options')
        self.max_pick_attempts = self.construction_options['max_pick_attempts']

        # Create a WallParser object. Wall info is immediately parsed from file.
        self.wp = WallParser(self.wall_pattern_file, self.pattern_file_options, self.construction_options)
        self.wp.parse_wall()
        rospy.loginfo('Done parsing wall pattern.')

        # Make a schedule, i.e. determine the possible order of the tasks.
        self.make_schedule()

        # Create services.
        rospy.Service('brick_pickup', PickupUGV, self.brick_pickup_srv)
        rospy.Service('brick_place', PlaceUGV, self.brick_place_srv)
        rospy.Service('/restart', Trigger, self.restart_srv)

        # Create subscribers.
        rospy.Subscriber('brick_placed', String, self.brick_place_srv, queue_size=1)

        while not rospy.is_shutdown():
            if self.should_restart:
                self.restart()
            rospy.sleep(1)

    def brick_pickup_srv(self, req):
        """
        Service handler for brick pickup.

        Args:
            req (PickupUGVRequest): Service request.
        """
        for_pickup = Basket('pickup')
        # print('Brick pickup request:\n{}'.format(req))
        if req.success:
            self.basket.add_from(req.success)
            for brick in req.success:
                self.fail_monitor[brick[0]] = 0

        if req.failed:
            # If pickup failed, add the brick to the list of failed attempts, remove it from the list of bricks
            # to be picked up and try to get the next available brick. Don't do it in place, we don't want to
            # remove precedence constraints before the brick is picked up.

            self.fail_monitor[req.failed[0]] += 1

            # Request new precedence graph to determine which bricks can be picked up.
            if not self.reschedule(int(req.failed.split('.')[1]), req.failed[0], self.completed_tasks + self.basket.inside):
                return False
            pick_graph = deepcopy(self.precedence_graph)
            pick_graph.remove_nodes_from(self.basket.inside)
            # Get new bricks that can be picked up in this turn. First remove the original bricks for pickup,
            # add the ones that are already picked up and then add the remaining ones.
            for_pickup.update_state(self.basket)
        else:
            pick_graph = deepcopy(self.precedence_graph)

        color_filter = {k: v >= self.max_pick_attempts for k, v in self.fail_monitor.items()}
        if not self.fill_basket(for_pickup, pick_graph,
                                excludes=[req.failed] + self.failed_tasks, color_filter=color_filter):
            if self.precedence_graph.tasks:
                rospy.logerr('No bricks to pick up and tasks are still available. - %s', self.precedence_graph.tasks)
                self.reschedule_all()
                pick_graph = deepcopy(self.precedence_graph)
                pick_graph.remove_nodes_from(self.basket.inside)
                for_pickup.update_state(self.basket)
                self.fill_basket(for_pickup, pick_graph,
                                 excludes=[req.failed] + self.failed_tasks, color_filter=color_filter)
            else:
                rospy.loginfo('Mission completed')
                self.should_restart = True
        print("$$$ {}".format(for_pickup))

        # Package the bricks for pickup and their positions into the service response.
        resp = PickupUGVResponse()
        resp.brick_ids = for_pickup.inside
        resp.brick_pos = [self.wp.brick_positions[brick][0] for brick in for_pickup.inside]
        return resp

    def brick_place_srv(self, req):
        if req.success:
            rospy.loginfo('Registering successfuly placed brick %s', req.brick)
            self.completed_tasks.append(req.brick)
            self.precedence_graph.remove_node(req.brick)
        else:
            rospy.loginfo('Registering fail placing brick %s', req.brick)
        self.basket.remove(req.brick)
        return True

    def make_schedule(self):
        # Combine relations of each segment in one list
        self.new_task_relations = []
        for task_relations_segment in self.wp.report_relations.values():
            self.new_task_relations.extend(task_relations_segment)

        # Combine tasks of each segment in one list.
        self.new_tasks = list(itertools.chain.from_iterable(self.wp.report_tasks.values()))
        # Add score points to tasks.
        self.new_tasks = {task: self.construction_options['brick_points'][task[0]] for task in self.new_tasks}

        # Create a precedence graph out of combined list of task relations.
        self.precedence_graph = PrecedenceGraph(self.new_task_relations, self.new_tasks)
        self.precedence_graph.update_layers()

    def reschedule(self, layer, color, fixed_tasks):
        # Take the current layer.
        wall_segment_range = self.wp.report_tasks.keys()
        current_layer = ''.join([''.join(str(self.wp.wall_structure[seg][layer - 1])) for seg in wall_segment_range])

        # Modify the wall pattern to include new constraints
        moved_tasks = self.wp.modify_pattern(self.wp.report_tasks.keys(), layer, color, fixed_tasks)
        self.make_schedule()

        # Take the new layer and compare it to the old one.
        new_layer = ''.join([''.join(str(self.wp.wall_structure[seg][layer - 1])) for seg in wall_segment_range])
        if new_layer == current_layer:

            rospy.logwarn('Rescheduling had no effect. Removing tasks: %s', moved_tasks)
            self.failed_tasks.extend(moved_tasks)

        self.precedence_graph.remove_nodes_from(self.completed_tasks)
        self.precedence_graph.update_layers()
        return True

    def reschedule_all(self):
        for layer in range(1, self.pattern_file_options['num_layers'] + 1):
            for color in self.fail_monitor:
                if self.fail_monitor[color] >= self.max_pick_attempts:
                    self.reschedule(layer, color, self.completed_tasks + self.basket.inside)

    def fill_basket(self, basket, graph, excludes=None, color_filter=None):
        """
        Add bricks to the basket.

        Bricks to be added are determined by the enabled nodes from the passed
        precedence graph. Arguments are changed by reference.

        Args:
            basket (Basket): Basket in which bricks need to be added.
            graph (PrecedenceGraph): Precedence graph with tasks related to the bricks.
            excludes (list[string]): List of bricks that should not be added  to the basket.
            color_filter (dict): {< R | G | B | O > : < True | False >} - True if color should be excluded.
        """
        if excludes is None:
            excludes = []

        graph = deepcopy(graph)
        outcome = True  # outcome will be false if none of the enabled bricks can be added to the basket.
        while not basket.is_full() and outcome:
            outcome = False
            enabled_nodes = graph.get_enabled_nodes(-1, in_place=False)
            for brick in enabled_nodes:
                if brick not in excludes:
                    if not color_filter[brick[0]]:
                        if basket.add(brick):
                            outcome = True
                            graph.remove_node(brick)
                            break
                    else:
                        rospy.logwarn("Will not add brick {} because of the color filter!".format(brick))

        if basket.is_empty():
            return False
        else:
            return True

    def restart_srv(self, req):
        rospy.loginfo('Restart request - new wall pattern')
        self.restart()
        return TriggerResponse()

    def restart(self):
        self.should_restart = False
        self.wp.reinitialize(self.wall_pattern_file, self.pattern_file_options, self.construction_options)
        self.wp.parse_wall()
        rospy.loginfo('Done parsing wall pattern.')

        self.completed_tasks = []
        self.failed_tasks = []
        self.fail_monitor = {'R': 0, 'G': 0, 'B': 0, 'O': 0}  # brick_color: number of recent fails
        self.basket = Basket('main')

        self.make_schedule()

        rospy.loginfo('-------------- RESTART ---------------')

if __name__ == "__main__":
    rospy.init_node("wall_coordinator")

    try:
        logger.name = rospy.get_name()
        node = WallCoordinatorUGV()
    except rospy.ROSInterruptException:
        pass

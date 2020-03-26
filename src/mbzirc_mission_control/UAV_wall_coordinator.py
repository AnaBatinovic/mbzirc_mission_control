#!/usr/bin/env python
# -*- coding: utf-8 -*-
import itertools

import rospy

from mbzirc_mission_control import WallParser, PrecedenceGraph
from mbzirc_mission_control.srv import NextTask, NextTaskResponse, CompletedTask

class WallCoordinatorUAV(object):
    def __init__(self):
        # Create predefined class variables.
        self.brick_colors = {'R': 'red', 'G': 'green', 'B': 'blue', 'O': 'orange'}
        self.latest_task = None
        self.failed_tasks = []
        self.completed_tasks = []
        self.color_filter = set()

        # Get all necessary parameters for parsing the wall.
        self.wall_pattern_file = rospy.get_param('CH2/wall_pattern')
        self.pattern_file_options = rospy.get_param('CH2/config/pattern_file_options')
        self.construction_options = rospy.get_param('CH2/config/construction_options')

        # Create a WallParser object. Wall info is immediately parsed from file.
        self.wp = WallParser(self.wall_pattern_file, self.pattern_file_options, self.construction_options)
        self.wp.parse_wall()
        rospy.loginfo('Done parsing wall pattern.')

        # Make a schedule, i.e. determine the possible order of the tasks.
        self.make_schedule()

        rospy.Service('request_next_task', NextTask, self.send_next_task_srv)
        rospy.Service('register_completed_task', CompletedTask, self.register_completed_srv)

        rospy.spin()

    def make_schedule(self):
        # Combine relations of each segment in one list
        self.new_task_relations = []
        for task_relations_segment in self.wp.report_relations.values():
            self.new_task_relations.extend(task_relations_segment)

        # Combine tasks of each segment in one list.
        self.new_tasks = list(itertools.chain.from_iterable(self.wp.report_tasks.values()))

        # Create a precedence graph out of combined list of task relations.
        self.precedence_graph = PrecedenceGraph(self.new_task_relations, self.new_tasks)
        self.precedence_graph.update_layers()

    def reschedule(self, layer, color, fixed_tasks):
        self.wp.modify_pattern(self.wp.report_tasks.keys(), layer, color, fixed_tasks)
        self.make_schedule()
        self.precedence_graph.remove_nodes_from(self.completed_tasks)
        self.precedence_graph.update_layers()

    def calc_task_priority(self, task):
        segment_priority = self.wp.report_score[int(task.split('_')[1][0])] / 1000.0
        layer_priority = (2 - int(task.split('.')[1])) * 100
        return layer_priority + segment_priority

    def filter_out_task(self, task):
        if task in self.failed_tasks:
            return True

        if task[0] in self.color_filter:
            return True

        return False

    def send_next_task_srv(self, req):
        # Package the next task into a service response.
        resp = NextTaskResponse()

        # There are no more tasks in the tree. Return empty response.
        if len(self.precedence_graph.tasks) == 0:
            return resp

        while True:
            # Get the enabled tasks (bricks that can be placed at this moment) and sort to get the best one.
            enabled = self.precedence_graph.Tf
            enabled.sort(key=self.calc_task_priority, reverse=True)
            for next_task in enabled:
                # If the brick can't be picked up, skip it.
                if self.filter_out_task(next_task):
                    continue
                resp.task_id = next_task
                resp.color = self.brick_colors[next_task[0]]
                resp.brick_pose = self.wp.brick_positions[next_task][0]
                self.latest_task = next_task
                return resp
            else:
                # If one or two colors failed, re-plan by moving them to the end of wall segments.
                if len(self.color_filter) < 3:
                    for failed_color in self.color_filter:
                        self.reschedule(1, failed_color, self.completed_tasks)
                # Otherwise, just keep trying.
                self.color_filter.clear()
                self.failed_tasks = []


    def register_completed_srv(self, req):
        task_id = req.task_id
        if req.task_id == '':
            task_id = self.latest_task

        if task_id in self.precedence_graph.tasks:
            if req.success:
                rospy.loginfo('Registering successfuly completed task: %s', task_id)
                self.precedence_graph.remove_node(task_id)
                self.precedence_graph.update_layers()
                self.completed_tasks.append(task_id)
                return True
            else:
                rospy.logwarn('Registering failed task: %s', task_id)
                self.failed_tasks.append(task_id)
                self.color_filter.add(task_id[0])
                return False
        else:
            rospy.logerr('Task %s unknown or removed from tree!', task_id)
            return False


if __name__ == "__main__":
    rospy.init_node("wall_coordinator")

    try:
        node = WallCoordinatorUAV()
    except rospy.ROSInterruptException:
        pass

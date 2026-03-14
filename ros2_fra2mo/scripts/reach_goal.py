#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from ros2_kdl_package.action import KdlFeedback

"""
Basic navigation demo to go to pose.
"""
def send_inspection_goal(node, action_client):
    goal = KdlFeedback.Goal()
    goal.start = True

    print("Sending inspection action goal...")
    future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)

    goal_handle = future.result()
    if not goal_handle.accepted:
        print("Inspection goal rejected")
        return

    print("Inspection goal accepted, waiting for result...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result().result
    print("Inspection completed")
    if result.completed:
        print("Part is conformant")
    else:
        print("Part is not conformant")
    return result.completed
    


def main():
    rclpy.init()
    node = rclpy.create_node('reach_goal')
    navigator = BasicNavigator()
    GREEN_MAT_POSE = (2.75, 2.0, 0.0)
    RED_MAT_POSE   = (2.75, -2.0, 0.0)
    inspection_client = ActionClient(
    node,
    KdlFeedback,
    '/kdl_feedback'
    )

    print("Waiting for iiwa action server...")
    inspection_client.wait_for_server()
    print("iiwa action server available")

    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 3.45
    # initial_pose.pose.position.y = 2.15
    # initial_pose.pose.orientation.z = 1.0
    # initial_pose.pose.orientation.w = 0.0
    # navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    global_costmap = navigator.getGlobalCostmap()
    local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose

    i=0
    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    conformity=send_inspection_goal(node, inspection_client)
    if conformity:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = GREEN_MAT_POSE[0]
        goal_pose.pose.position.y = GREEN_MAT_POSE[1]
        goal_pose.pose.orientation.w = GREEN_MAT_POSE[2]
        navigator.goToPose(goal_pose)
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')
                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600):
                    navigator.cancelTask()
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Reached deployment point')
    else:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = RED_MAT_POSE[0]
        goal_pose.pose.position.y = RED_MAT_POSE[1]
        goal_pose.pose.orientation.w = RED_MAT_POSE[2]
        navigator.goToPose(goal_pose)
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')
                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600):
                    navigator.cancelTask()
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Reached deployment point')

            

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
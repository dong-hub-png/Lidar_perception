# Copyright (c) 2022 Homalozoa
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

import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml
from ament_index_python.packages import get_package_share_directory

class GroundSegmentationPipeline:
    def __init__(self, context):
        self.context = context
        ground_segmentation_param_path = os.path.join(
            get_package_share_directory("ground_segmentation"),LaunchConfiguration("obstacle_segmentation_ground_segmentation_param_path").perform(
                context
            ),
        )
        with open(ground_segmentation_param_path, "r") as f:
            self.ground_segmentation_param = yaml.safe_load(f)["/**"]["ros__parameters"]


    def create_common_pipeline(self):
        components = []
        components.append(
            ComposableNode(
                package="ground_segmentation",
                plugin=self.ground_segmentation_param["common_ground_filter"]["plugin"],
                name="common_ground_filter",
                parameters=[
                    self.ground_segmentation_param["common_ground_filter"]["parameters"],
                ],
            )
        )
        return components
def launch_setup(context, *args, **kwargs):
    pipeline = GroundSegmentationPipeline(context)

    components = []
    components.extend(
        pipeline.create_common_pipeline()
    )

    individual_container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=components,
        output="screen",
    )
    return [individual_container]

def generate_launch_description():
    """Generate launch description with multiple components."""
    #添加一些启动参数
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))
    #add_launch_arg("base_frame", LaunchConfiguration("base_frame"))
    add_launch_arg("base_frame", "base_link")
    add_launch_arg("container_name", "perception_pipeline_container")
    add_launch_arg("obstacle_segmentation_ground_segmentation_param_path", "config/ground_segmentation.param.yaml")
    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
    )
    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable]
        + [OpaqueFunction(function=launch_setup)])

"""
Perception for the maneuvers: filtering and clustering of the lidar cloud.

The boat's lidar is a 16-beam VLP-16 and the ros_gz bridge publishes its point
cloud straight onto /lidar/points, which is what the pcd pipeline clusters on.
On the real boat the driver publishes the same message on the same topic, so
the only simulator-specific thing left here is the blind-spot mask.

HISTORY, because this file got it wrong for a week. It used to run
scan_to_cloud, converting sensor_msgs/LaserScan into a cloud, on the
understanding that "the simulated boat carries a flat single-ring scanner".
That was true of the hand-written models/prop/model.sdf, whose lidar had no
<vertical> block at all. Since Carlos's #577 the boat is built from
xacro/prop.sdf.xacro with sixteen beams over +/-15 degrees, and prop_bridge.yaml
already bridges the cloud. Leaving scan_to_cloud running put TWO publishers on
/lidar/points -- the bridge's real cloud and scan_to_cloud's conversion -- and
pcl_filter saw them interleaved. Worse, the converted ones were always empty:
the bridged LaserScan is the MIDDLE of the sixteen rings, about a degree above
horizontal, which passes clean over the 0.5 m buoys. Measured 2026-09-13: a
median of 0 finite returns out of 1875, which is where the endless
"[pcl::fromPCLPointCloud2] No data to copy." came from, and it halved the rate
at which clustering actually saw anything.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# Where the mask republishes the cloud once it has cut the blind wedges out.
MASKED_CLOUD_TOPIC = "/lidar/points_masked"


def generate_launch_description():
    """Launch the blind-spot mask (optional), pcl_filter and pcl_clustering, wired together."""
    maneuvers_share = get_package_share_directory("prop_maneuvers")
    pcd_params = os.path.join(
        get_package_share_directory("pcd"),
        "config",
        "pcd_params.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the /clock topic. True in simulation, false on the real boat.",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/lidar/points",
                description="PointCloud2 the chain reads, straight from the lidar.",
            ),
            DeclareLaunchArgument(
                "cluster_min_points",
                default_value="3",
                description=(
                    "Simulator override, and a low one. A 0.5 m buoy at circling "
                    "distance is a couple of dozen points after the 0.1 m voxel "
                    "grid, so pcd's own 20 is close to the edge. Worth revisiting "
                    "now that the chain reads the real sixteen-beam cloud rather "
                    "than a single converted ring: 3 also lets noise become a cluster."
                ),
            ),
            DeclareLaunchArgument(
                "water_z_min",
                default_value="-0.45",
                description=(
                    "Simulator override of pcd's own -0.50. Levelling the z cut by "
                    "roll/pitch (pcl_filter.hpp) is the structural fix for the water "
                    "returns that used to shatter into 18-22 clusters a frame; this "
                    "tightens the threshold to the exact value an offline replay of "
                    "12 captured mid-circle frames measured as clean (median 2 "
                    "clusters, every frame) once levelling is in effect. Left at "
                    "pcd's default -0.50, the same replay still gave a median of 14. "
                    "NOT copied into pcd_params.yaml: the real boat's IMU noise and "
                    "actual chop may call for a different number than this sim run did."
                ),
            ),
            DeclareLaunchArgument(
                "blind_spots",
                default_value="false",
                description="Fake the real boat's blind spots by masking the cloud.",
            ),
            # Cuts the antenna wedges out of the cloud before anything clusters
            # it. Only runs when asked for; without it the simulated lidar sees
            # a full circle, which the real one does not.
            Node(
                package="prop_maneuvers",
                executable="blind_spot_mask",
                name="blind_spot_mask",
                output="screen",
                parameters=[
                    os.path.join(maneuvers_share, "config", "maneuvers.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                remappings=[
                    ("cloud", LaunchConfiguration("cloud_topic")),
                    ("masked_cloud", MASKED_CLOUD_TOPIC),
                ],
                condition=IfCondition(LaunchConfiguration("blind_spots")),
            ),
            Node(
                package="pcd",
                executable="pcl_filter_node",
                name="pcl_filter",
                output="screen",
                parameters=[
                    pcd_params,
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"water_z_min": LaunchConfiguration("water_z_min")},
                    # Read the mask's output when there is one, the raw cloud
                    # otherwise. Pointing this at the raw cloud while the mask
                    # runs is exactly the bug this file used to have.
                    {
                        "input_topic": PythonExpression(
                            [
                                f'"{MASKED_CLOUD_TOPIC}" if "',
                                LaunchConfiguration("blind_spots"),
                                '" == "true" else "',
                                LaunchConfiguration("cloud_topic"),
                                '"',
                            ],
                        ),
                    },
                ],
            ),
            Node(
                package="pcd",
                executable="pcl_clustering_node",
                name="pcl_clustering",
                output="screen",
                parameters=[
                    pcd_params,
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"cluster_min_points": LaunchConfiguration("cluster_min_points")},
                ],
            ),
        ],
    )

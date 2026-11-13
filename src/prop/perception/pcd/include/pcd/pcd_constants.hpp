/**
 * @file pcd_constants.hpp
 * @brief Base class that declares and loads all PCD perception parameters
 *        from the ROS 2 parameter server.
 *
 * Both PclFilter and PclClustering inherit from this class so that every
 * tunable value lives in exactly one place (config/pcd_params.yaml) and is
 * never duplicated in code.
 */

#pragma once

#include <string>

#include <rclcpp/node.hpp>

namespace pcd
{

/**
 * @class PcdConstants
 * @brief Mixin base that declares and caches all perception parameters.
 *
 * Inherit from this class alongside rclcpp::Node (via CRTP or multiple
 * inheritance) and call load_constants() from your constructor *after*
 * declare_parameters() has been invoked.
 *
 * Example:
 * @code
 *   class PclFilter : public rclcpp::Node, public pcd::PcdConstants
 *   {
 *   public:
 *     PclFilter() : rclcpp::Node("pcl_filter"), PcdConstants(this) {}
 *   };
 * @endcode
 */
class PcdConstants
{
  public:
    // ── Default parameter values (named constants to avoid magic numbers) ────
    static constexpr double kDefaultMinDistance = 0.5;
    static constexpr double kDefaultMaxDistance = 30.0;
    static constexpr double kDefaultWaterZMin = -0.5;
    static constexpr double kDefaultWaterZMax = 10.0;
    static constexpr double kDefaultVoxelLeafSize = 0.1;
    static constexpr double kDefaultClusterTolerance = 0.5;
    static constexpr int kDefaultClusterMinPoints = 20;
    static constexpr int kDefaultClusterMaxPoints = 25000;
    static constexpr double kDefaultClusterFlatnessThreshold = 3.0;
    static constexpr double kDefaultMaxAssociationDist = 3.0;
    static constexpr int kDefaultMaxMissedFrames = 10;
    static constexpr int kDefaultMaxMissedTentative = 2;
    static constexpr int kDefaultMinHits = 2;
    static constexpr int kDefaultMaxTracks = 20;
    static constexpr int kDefaultMaxDetections = 50;
    static constexpr double kDefaultMergeGap = 0.75;
    static constexpr double kDefaultMaxTrackSpeed = 5.0;

    /**
     * @brief Construct the constants mixin.
     * @param node  Pointer to the owning rclcpp::Node.  Parameters are
     *              declared and read from this node's parameter server.
     */
    explicit PcdConstants(rclcpp::Node *node)
    {
        // ── Distance pass-through filter ────────────────────────────────────
        node->declare_parameter("min_distance", kDefaultMinDistance);
        node->declare_parameter("max_distance", kDefaultMaxDistance);

        // ── Water-surface / height rejection ────────────────────────────────
        node->declare_parameter("water_z_min", kDefaultWaterZMin);
        node->declare_parameter("water_z_max", kDefaultWaterZMax);

        // ── Voxel-grid down-sampling ─────────────────────────────────────────
        node->declare_parameter("voxel_leaf_size", kDefaultVoxelLeafSize);

        // ── Euclidean clustering ─────────────────────────────────────────────
        node->declare_parameter("cluster_tolerance", kDefaultClusterTolerance);
        node->declare_parameter("cluster_min_points", kDefaultClusterMinPoints);
        node->declare_parameter("cluster_max_points", kDefaultClusterMaxPoints);
        node->declare_parameter("cluster_flatness_threshold", kDefaultClusterFlatnessThreshold);

        // ── EKF tracker ──────────────────────────────────────────────────────
        /// Max centroid-to-centroid distance [m] for a detection to be
        /// associated with an existing track. Detections farther away start
        /// a new track.
        node->declare_parameter("max_association_dist", kDefaultMaxAssociationDist);
        /// Number of consecutive missed frames before a track is deleted.
        node->declare_parameter("max_missed_frames", kDefaultMaxMissedFrames);
        /// Consecutive misses before a tentative (unconfirmed) track is deleted.
        node->declare_parameter("max_missed_tentative", kDefaultMaxMissedTentative);
        /// Minimum number of consecutive hits before a track is published.
        /// Suppresses single-frame phantom detections.
        node->declare_parameter("min_hits", kDefaultMinHits);
        /// Target frame for tracking (e.g. "odom" or "map").
        node->declare_parameter<std::string>("target_frame", "odom");

        node->declare_parameter("max_tracks", kDefaultMaxTracks);
        /// Hard cap on detections consumed per frame (applied after merge).
        node->declare_parameter("max_detections", kDefaultMaxDetections);
        /// Box-to-box gap [m] below which detections are merged before association.
        node->declare_parameter("merge_gap", kDefaultMergeGap);
        /// Maximum EKF speed [m/s]; velocity is scaled down if it exceeds this.
        node->declare_parameter("max_track_speed", kDefaultMaxTrackSpeed);

        // ── I/O ──────────────────────────────────────────────────────────────
        node->declare_parameter<std::string>("input_topic", "/velodyne_points");

        // Read them all back into member variables.
        min_distance_ = node->get_parameter("min_distance").as_double();
        max_distance_ = node->get_parameter("max_distance").as_double();
        water_z_min_ = node->get_parameter("water_z_min").as_double();
        water_z_max_ = node->get_parameter("water_z_max").as_double();
        voxel_leaf_size_ = node->get_parameter("voxel_leaf_size").as_double();
        cluster_tolerance_ = node->get_parameter("cluster_tolerance").as_double();
        cluster_min_points_ = static_cast<int>(node->get_parameter("cluster_min_points").as_int());
        cluster_max_points_ = static_cast<int>(node->get_parameter("cluster_max_points").as_int());
        cluster_flatness_threshold_ = node->get_parameter("cluster_flatness_threshold").as_double();

        max_association_dist_ = node->get_parameter("max_association_dist").as_double();
        max_missed_frames_ = static_cast<int>(node->get_parameter("max_missed_frames").as_int());
        max_missed_tentative_ = static_cast<int>(node->get_parameter("max_missed_tentative").as_int());
        min_hits_ = static_cast<int>(node->get_parameter("min_hits").as_int());
        target_frame_ = node->get_parameter("target_frame").as_string();
        input_topic_ = node->get_parameter("input_topic").as_string();
        max_tracks_ = static_cast<int>(node->get_parameter("max_tracks").as_int());
        max_detections_ = static_cast<int>(node->get_parameter("max_detections").as_int());
        merge_gap_ = node->get_parameter("merge_gap").as_double();
        max_track_speed_ = node->get_parameter("max_track_speed").as_double();
    }

  protected:
    // ── Distance filter ──────────────────────────────────────────────────────
    /// Minimum radial distance [m]. Points closer than this are discarded.
    double min_distance_{
        kDefaultMinDistance
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
    /// Maximum radial distance [m]. Points farther than this are discarded.
    double max_distance_{
        kDefaultMaxDistance
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)

    // ── Water-surface rejection ──────────────────────────────────────────────
    /// Lower Z bound [m]. Points below this are treated as water returns.
    double water_z_min_{
        kDefaultWaterZMin
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
    /// Upper Z bound [m]. Points above this are treated as sky / mast noise.
    double water_z_max_{
        kDefaultWaterZMax
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)

    // ── Voxel-grid down-sampling ─────────────────────────────────────────────
    /// Voxel leaf size [m]. Set to 0.0 to disable down-sampling.
    double voxel_leaf_size_{
        kDefaultVoxelLeafSize
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)

    // ── Euclidean cluster extraction ─────────────────────────────────────────
    /// Max distance [m] between two points to be considered neighbours.
    double cluster_tolerance_{
        kDefaultClusterTolerance
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
    /// Minimum number of points for a cluster to be kept.
    int cluster_min_points_{
        kDefaultClusterMinPoints
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
    /// Maximum number of points allowed in a single cluster.
    int cluster_max_points_{
        kDefaultClusterMaxPoints
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)

    double cluster_flatness_threshold_{
        kDefaultClusterFlatnessThreshold
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
        // ///< Max Z extent / XY extent ratio to
        // consider a cluster "flat"

    // ── EKF tracker ──────────────────────────────────────────────────────────
    /// Max centroid-to-centroid distance [m] for nearest-neighbour association.
    double max_association_dist_{
        kDefaultMaxAssociationDist
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
    /// Frames of consecutive misses before a confirmed track is removed.
    int max_missed_frames_{
        kDefaultMaxMissedFrames
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
    /// Frames of consecutive misses before a tentative track is removed.
    int max_missed_tentative_{
        kDefaultMaxMissedTentative
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
    /// Consecutive hits before a track is published (anti-spurious filter).
    int min_hits_{
        kDefaultMinHits
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
    /// Target frame to transform cluster centroids into for tracking (e.g. "odom" or "map").
    std::string target_frame_{
        "odom"
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)

    int max_tracks_{
        kDefaultMaxTracks
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
        // ///< Hard cap on number of tracks maintained
    int max_detections_{
        kDefaultMaxDetections
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
        // ///< Hard cap on number of detections consumed per frame
    /// Box-to-box gap [m] for merging fragment detections before Hungarian.
    double merge_gap_{
        kDefaultMergeGap
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
    /// Speed clamp [m/s] applied to EKF velocity.
    double max_track_speed_{
        kDefaultMaxTrackSpeed
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)

    // ── Topics ───────────────────────────────────────────────────────────────
    /// Input PointCloud2 topic name.
    std::string input_topic_{
        "/velodyne_points"
    };  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
};

}  // namespace pcd

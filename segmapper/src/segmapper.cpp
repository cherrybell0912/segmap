#include "segmapper/segmapper.hpp"

#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <laser_slam/benchmarker.hpp>
#include <laser_slam/common.hpp>
#include <laser_slam_ros/common.hpp>
#include <ros/ros.h>
#include <segmatch/utilities.hpp>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using namespace laser_slam;
using namespace laser_slam_ros;
using namespace segmatch;
using namespace segmatch_ros;
using namespace std;
// std::ofstream record("/home/xyz/Documents/segmap_ws/data/loop.txt", std::ios::trunc);
// std::ofstream tract("/home/xyz/Documents/segmap_ws/data/aftlooptracj.txt", std::ios::trunc);
// std::ofstream record0("/home/xyz/Documents/segmap_ws/data/path0.txt", std::ios::trunc);
// std::ofstream record1("/home/xyz/Documents/segmap_ws/data/path1.txt", std::ios::trunc);
// int pathNum = 0;
SegMapper::SegMapper(ros::NodeHandle& n) : nh_(n) {
  // Load ROS parameters from server.
  getParameters();

  // TODO: it would be great to have a cleaner check here, e.g. by having the segmenter interface
  // telling us if normals are needed or not. Unfortunately, at the moment the segmenters are
  // created much later ...
  const std::string& segmenter_type =
      segmatch_worker_params_.segmatch_params.segmenter_params.segmenter_type;
  const bool needs_normal_estimation =
      (segmenter_type == "SimpleSmoothnessConstraints") ||
      (segmenter_type == "IncrementalSmoothnessConstraints");

  // Configure benchmarker
  Benchmarker::setParameters(benchmarker_params_);

  // Create an incremental estimator.
  std::shared_ptr<IncrementalEstimator> incremental_estimator(
      new IncrementalEstimator(params_.online_estimator_params, params_.number_of_robots));

  incremental_estimator_ = incremental_estimator;

  // Create local map publisher
  local_maps_mutexes_ = std::vector<std::mutex>(params_.number_of_robots);
  // if (laser_slam_worker_params_.publish_local_map) {
  //   local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
  //       laser_slam_worker_params_.local_map_pub_topic,
  //       kPublisherQueueSize);
  // }

  // Setup the laser_slam workers.
  ROS_INFO_STREAM("Number of laser_slam workers: " << params_.number_of_robots);
  for (unsigned int i = 0u; i < params_.number_of_robots; ++i) {
    // Adjust the topics and frames for that laser_slam worker.
    LaserSlamWorkerParams params = laser_slam_worker_params_;

    // Create a local map for each robot.
    std::unique_ptr<NormalEstimator> normal_estimator = nullptr;
    if (needs_normal_estimation) {
      normal_estimator = NormalEstimator::create(
          segmatch_worker_params_.segmatch_params.normal_estimator_type,
          segmatch_worker_params_.segmatch_params.radius_for_normal_estimation_m);
    }
    local_maps_.emplace_back(
        segmatch_worker_params_.segmatch_params.local_map_params, std::move(normal_estimator));

    // TODO rm offset when updating mr_foundry.
    const unsigned int offset = 0;
    if (params_.number_of_robots > 1) {
      // Subscribers.

      params.assembled_cloud_sub_topic = "/" + params_.robot_prefix + std::to_string(i + offset) +
          "/" + laser_slam_worker_params_.assembled_cloud_sub_topic;

      // TF frames.
      params.odom_frame =  params_.robot_prefix + std::to_string(i + offset) +
          "/" + laser_slam_worker_params_.odom_frame;
      params.sensor_frame =  params_.robot_prefix + std::to_string(i + offset) +
          "/" + laser_slam_worker_params_.sensor_frame;

      // Publishers.
     
      params.local_map_pub_topic = params_.robot_prefix + std::to_string(i + offset) + "/" +
          laser_slam_worker_params_.local_map_pub_topic;
    }

    LOG(INFO) << "Robot " << i << " subscribes to " << params.assembled_cloud_sub_topic << " "
        << params.odom_frame << " and " << params.sensor_frame;

    LOG(INFO) << "Robot " << i << " publishes to " << params.trajectory_pub_topic << " and "
        << params.local_map_pub_topic;

    std::unique_ptr<LaserSlamWorker> laser_slam_worker(new LaserSlamWorker());
    laser_slam_worker->init(nh_, params, incremental_estimator_, i);
    laser_slam_workers_.push_back(std::move(laser_slam_worker));
  }


  // Initialize the SegMatchWorker.
  if (segmatch_worker_params_.localize || segmatch_worker_params_.close_loops) {//yaml文件
    segmatch_worker_.init(n, segmatch_worker_params_, params_.number_of_robots);
  }
  
  for (size_t i = 0u; i < laser_slam_workers_.size(); ++i) {
      skip_counters_.push_back(0u);
      first_points_received_.push_back(false);
  }
}

SegMapper::~SegMapper() {}

void SegMapper::publishMapThread() {
  // Check if map publication is required.
  if (!laser_slam_worker_params_.publish_local_map)
    return;
  ros::Rate thread_rate(laser_slam_worker_params_.map_publication_rate_hz);
  while (ros::ok()) {
    LOG(INFO) << "publishing local maps";
    MapCloud local_maps;

    for (size_t i = 0u; i < local_maps_.size(); ++i) { 
      std::unique_lock<std::mutex> map_lock(local_maps_mutexes_[i]);
      if(local_maps_[i].getFilteredPoints().size()==0)
      continue;
      local_maps += local_maps_[i].getFilteredPoints();  
      map_lock.unlock();
    }
    sensor_msgs::PointCloud2 msg;
    laser_slam_ros::convert_to_point_cloud_2_msg(
        local_maps,
        params_.world_frame, &msg);
   // msg.header.stamp.nsec = current_pose.time_ns;
    msg.header.stamp = ros::Time::now();
   // local_map_pub_.publish(msg);
    thread_rate.sleep();
  }
}

void SegMapper::publishTfThread() {
  if (params_.publish_world_to_odom) {
    ros::Rate thread_rate(params_.tf_publication_rate_hz);
    while (ros::ok()) {
      for (size_t i = 0u; i < laser_slam_workers_.size(); ++i) {
        tf::StampedTransform world_to_odom = laser_slam_workers_[i]->getWorldToOdom();
        world_to_odom.stamp_ = ros::Time::now();
        tf_broadcaster_.sendTransform(world_to_odom);
      }
      thread_rate.sleep();
    }
  }
}

void SegMapper::print4x4Matrix(const Eigen::Matrix4f & matrix) {
  std::cout << setw(15) << matrix(0,0) << setw(15) << matrix(0,1) << setw(15) << matrix(0,2) << setw(15) << matrix(0,3) << std::endl;
  std::cout << setw(15) << matrix(1,0) << setw(15) << matrix(1,1) << setw(15) << matrix(1,2) << setw(15) << matrix(1,3) << std::endl;
  std::cout << setw(15) << matrix(2,0) << setw(15) << matrix(2,1) << setw(15) << matrix(2,2) << setw(15) << matrix(2,3) << std::endl;
  std::cout << setw(15) << matrix(3,0) << setw(15) << matrix(3,1) << setw(15) << matrix(3,2) << setw(15) << matrix(3,3) << std::endl;
}

void SegMapper::setLockScanCallback(bool new_state) {
  std::lock_guard<std::recursive_mutex> lock(scan_callback_mutex_);
  lock_scan_callback_ = new_state;
}

void SegMapper::cloud_cb (const sensor_msgs::PointCloud2& cloud_msg_in){
  std::lock_guard<std::recursive_mutex> lock_scan_callback(scan_callback_mutex_);
  if(!lock_scan_callback_){
  laser_slam_ros::LaserSlamWorker laser_worker_transform;

  if (tf_listener1.waitForTransform(laser_slam_worker_params_.odom_frame, laser_slam_worker_params_.sensor_frame,
                                      cloud_msg_in.header.stamp, ros::Duration(kTimeout_s))) {
      tf::StampedTransform tf_transform;
      tf_listener1.lookupTransform(laser_slam_worker_params_.odom_frame, laser_slam_worker_params_.sensor_frame,
                                   cloud_msg_in.header.stamp, tf_transform);
      //record0<<pathNum<<" "<<tf_transform.getOrigin().x()<<" "<<tf_transform.getOrigin().y()<<" "<<tf_transform.getOrigin().z()<<std::endl;
      
      current_pose.T_w=laser_slam_workers_[0u]->tfTransformToPose(tf_transform).T_w;
      current_pose.time_ns=laser_slam_workers_[0u]->tfTransformToPose(tf_transform).time_ns;
      SE3 pose = current_pose.T_w;

      //record1<<pathNum<<" "<<pose.getPosition()[0]<<" "<<pose.getPosition()[1]<<" "<<pose.getPosition()[2]<<std::endl;
      //pathNum++;
      gtsam::NonlinearFactorGraph new_factors0;
      gtsam::Values new_values0;
      bool is_prior0;
      LaserScan thisscan;  
      thisscan.time_ns = laser_slam_workers_[0u]->rosTimeToCurveTime(cloud_msg_in.header.stamp.toNSec());
      laser_slam_workers_[0u]->laser_track_->processPoseAndLaserScan(current_pose, thisscan,
                                        &new_factors0, &new_values0, &is_prior0);
      gtsam::Values result0;
      if (is_prior0) {
        result0 = incremental_estimator_->registerPrior(new_factors0, new_values0, 0u);
      } 
      else {
        result0 = incremental_estimator_->estimate(new_factors0, new_values0, current_pose.time_ns);
      }
   
      laser_slam_workers_[0u]->laser_track_->updateFromGTSAMValues(result0);
     // std::cout<<"pose: "<<tf_transform<<" after: "<<current_pose.T_w<<std::endl;
     // std::cout<<"pose: "<<current_pose.T_w<<std::endl;
     // std::cout<<"time: "<<cloud_msg_in.header.stamp<<" before: "<<tf_transform.stamp_.toNSec()<<" after: "<<current_pose.time_ns<<std::endl;

   }
   input.clear();
   pcl::fromROSMsg(cloud_msg_in, input);
   new_points_queue.push_back(input);
   if(!input.empty())
   {
    getinputcloud = true;
   }
 }
}


void SegMapper::segMatchThread() {
  // Terminate the thread if localization and loop closure are not needed.
  if ((!segmatch_worker_params_.localize &&
      !segmatch_worker_params_.close_loops) ||
      laser_slam_workers_.empty())
    return;

  ros::Subscriber scan_sub = nh_.subscribe(laser_slam_worker_params_.assembled_cloud_sub_topic, kScanSubscriberMessageQueueSize,
                           &SegMapper::cloud_cb, this);

  unsigned int track_id = laser_slam_workers_.size() - 1u;
  // Number of tracks skipped because waiting for new voxels to activate.
  unsigned int skipped_tracks_count = 0u;
  ros::Duration sleep_duration(kSegMatchSleepTime_s);

  unsigned int n_loops = 0u;

  while (ros::ok()) {
  
    if(getinputcloud == true){
    // std::cout<<"start"<<std::endl;
    // If all the tracks have been skipped consecutively连续的, sleep for a bit to
    // free some CPU time.
    if (skipped_tracks_count == laser_slam_workers_.size()) {
      skipped_tracks_count = 0u;
      sleep_duration.sleep();
    }
    // Make sure that all the measurements in this loop iteration will get the same timestamp. This
    // makes it easier to plot the data.
    // No, we don't include sleeping in the timing, as it is an intended delay.
     BENCHMARK_START_NEW_STEP();
     // No, we don't include sleeping in the timing, as it is an intended delay.
     BENCHMARK_START("SM");
    // Set the next source cloud to process.
    track_id = (track_id + 1u) % laser_slam_workers_.size();

    // Get the queued points.
    // auto new_points = laser_slam_workers_[track_id]->getQueuedPoints();
    std::vector<laser_slam_ros::PointCloud> new_points;
    if (new_points_queue.size() < 1) {
      BENCHMARK_STOP_AND_IGNORE("SM");
      ++skipped_tracks_count;
      // Keep asking for publishing to increase the publishing counter.
      segmatch_worker_.publish();
      skip_counters_[track_id]++;
      
      if (first_points_received_[track_id] && skip_counters_[track_id] == deactivate_track_when_skipped_x_) {
          bool is_one_still_active = false;
          for (const auto& counter : skip_counters_) {
              if (counter <= 1u) is_one_still_active = true;
          }
          if (is_one_still_active) {
              segmatch_worker_.stopPublishing(track_id);
          } else {
              skip_counters_[track_id] = 0u;
          }
      }
      continue;
    } else {
        new_points.swap(new_points_queue);
         // std::cout<<"queue "<<new_points.size()<<std::endl;
        if (!first_points_received_[track_id]) {
            first_points_received_[track_id] = true;
            skip_counters_[track_id] = 0u;
        }
    }

    // Update the local map with the new points and the new pose. 
    //std::cout<<"a"<<std::endl;
    local_maps_[track_id].updatePoseAndAddPoints(new_points, current_pose);
    //std::cout<<"b"<<std::endl;
    // Process the source cloud.
    RelativePose loop_closure;
    if (segmatch_worker_.processLocalMap(local_maps_[track_id], current_pose,
                                         track_id, &loop_closure)) {

        BENCHMARK_BLOCK("SM.ProcessLoopClosure");
        distance = 0;
         Eigen::Matrix4f transformation_ICP = loop_closure.T_a_b.getTransformationMatrix().cast<float>();
        LOG(INFO) << "Found loop closure! track_id_a: " << loop_closure.track_id_a <<
            " time_a_ns: " << loop_closure.time_a_ns <<
            " track_id_b: " << loop_closure.track_id_b <<
            " time_b_ns: " << loop_closure.time_b_ns;
        //record<<transformation_ICP<<std::endl;
        print4x4Matrix(transformation_ICP); 
        // Prevent the workers to process further scans (and add variables to the graph).
        BENCHMARK_START("SM.ProcessLoopClosure.WaitingForLockOnLaserSlamWorkers");
        setLockScanCallback(true); 
        BENCHMARK_STOP("SM.ProcessLoopClosure.WaitingForLockOnLaserSlamWorkers");
        // std::cout<<"c"<<std::endl;
        // Save last poses for updating the local maps.
        BENCHMARK_START("SM.ProcessLoopClosure.GettingLastPoseOfTrajectories");
        Trajectory trajectory;
        std::vector<SE3> last_poses_before_update;
        std::vector<laser_slam::Time> last_poses_timestamp_before_update_ns;
        if (!params_.clear_local_map_after_loop_closure) {
          for (const auto& worker: laser_slam_workers_) {
            worker->getTrajectory(&trajectory);   
            last_poses_before_update.push_back(trajectory.rbegin()->second);     
            last_poses_timestamp_before_update_ns.push_back(trajectory.rbegin()->first);         
          }
        }
        BENCHMARK_STOP("SM.ProcessLoopClosure.GettingLastPoseOfTrajectories");
        BENCHMARK_START("SM.ProcessLoopClosure.UpdateIncrementalEstimator");
        incremental_estimator_->processLoopClosure(loop_closure);
        BENCHMARK_STOP("SM.ProcessLoopClosure.UpdateIncrementalEstimator");
         std::cout<<"c"<<std::endl;
        BENCHMARK_START("SM.ProcessLoopClosure.ProcessLocalMap");
        for (size_t i = 0u; i < laser_slam_workers_.size(); ++i) {
          if (!params_.clear_local_map_after_loop_closure) {
            laser_slam::SE3 local_map_update_transform =
                laser_slam_workers_[i]->getTransformBetweenPoses(
                    last_poses_before_update[i], last_poses_timestamp_before_update_ns[i]);
            std::unique_lock<std::mutex> map_lock2(local_maps_mutexes_[i]);
            local_maps_[i].transform(local_map_update_transform.cast<float>());
            map_lock2.unlock();
          } else {
            std::unique_lock<std::mutex> map_lock2(local_maps_mutexes_[i]);
            local_maps_[i].clear();
            map_lock2.unlock();
          }
        }
        BENCHMARK_STOP("SM.ProcessLoopClosure.ProcessLocalMap");
    
        MapCloud local_maps;
        for (size_t i = 0u; i < local_maps_.size(); ++i) {
          std::unique_lock<std::mutex> map_lock(local_maps_mutexes_[i]);
          local_maps += local_maps_[i].getFilteredPoints();
          map_lock.unlock();
        }
        sensor_msgs::PointCloud2 msg;
        laser_slam_ros::convert_to_point_cloud_2_msg(
            local_maps,
            params_.world_frame, &msg);
        local_map_pub_.publish(msg);
       
        // Update the Segmatch object.
        std::vector<Trajectory> updated_trajectories;
        for (const auto& worker: laser_slam_workers_) {
          worker->getTrajectory(&trajectory);
          updated_trajectories.push_back(trajectory);
        }
        BENCHMARK_START("SM.ProcessLoopClosure.UpdateSegMatch");
        segmatch_worker_.update(updated_trajectories);
        BENCHMARK_STOP("SM.ProcessLoopClosure.UpdateSegMatch");

        //Publish the trajectories.
        for (const auto& worker : laser_slam_workers_) {
          worker->publishTrajectories();
        }
        // Unlock the workers.
        setLockScanCallback(false); 
        n_loops++;
        LOG(INFO) << "That was the loop number " << n_loops << ".";
    
    }//localmap
    for (const auto& worker : laser_slam_workers_) {
      worker->publishTrajectories();
    }
   //std::cout<<"d"<<std::endl;
    skipped_tracks_count = 0;
    skip_counters_[track_id] = 0u;
    BENCHMARK_STOP("SM");
  }//if
 }//rosok
  Trajectory trajectory1;
  laser_slam_workers_[0]->getTrajectory(&trajectory1);

  for (const auto& timePose : trajectory1) {
   tract<<timePose.second.getPosition().x()<<' '<<timePose.second.getPosition().y()<<' '<<timePose.second.getPosition().z()<<std::endl;

  }
  Benchmarker::logStatistics(LOG(INFO));
  Benchmarker::saveData();
}


void SegMapper::getParameters() {
  // SegMapper parameters.
  const std::string ns = "/SegMapper";
  nh_.getParam(ns + "/number_of_robots",
               params_.number_of_robots);
  nh_.getParam(ns + "/robot_prefix",
               params_.robot_prefix);

  CHECK_GE(params_.number_of_robots, 0u);

  nh_.getParam(ns + "/publish_world_to_odom",
               params_.publish_world_to_odom);
  nh_.getParam(ns + "/world_frame",
               params_.world_frame);
  nh_.getParam(ns + "/tf_publication_rate_hz",
               params_.tf_publication_rate_hz);

  nh_.getParam(ns + "/clear_local_map_after_loop_closure",
               params_.clear_local_map_after_loop_closure);

  // laser_slam worker parameters.
  laser_slam_worker_params_ = laser_slam_ros::getLaserSlamWorkerParams(nh_, ns);
  laser_slam_worker_params_.world_frame = params_.world_frame;

  // Online estimator parameters.
  params_.online_estimator_params = laser_slam_ros::getOnlineEstimatorParams(nh_, ns);

  // Benchmarker parameters.
  benchmarker_params_ = laser_slam_ros::getBenchmarkerParams(nh_, ns);

  // ICP configuration files.
  nh_.getParam("icp_configuration_file",
               params_.online_estimator_params.laser_track_params.icp_configuration_file);
  nh_.getParam("icp_input_filters_file",
               params_.online_estimator_params.laser_track_params.icp_input_filters_file);

  // SegMatchWorker parameters.
  segmatch_worker_params_ = segmatch_ros::getSegMatchWorkerParams(nh_, ns);
  segmatch_worker_params_.world_frame = params_.world_frame;
}

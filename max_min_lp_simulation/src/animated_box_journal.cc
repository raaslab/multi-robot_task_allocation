/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#define PI 3.141592

namespace gazebo
{
  class AnimatedBox : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Next position of target.
      bool flag_next = false;

      // Parameters
      int time_duration = 10; // time duration (seconds) for each step
      int tot_num_steps = 30; // Total number of steps
      int num_targets = 1; // Number of targets
      time_t t;
      srand((unsigned) time(&t));
      int env_size = 20; // Size of the bounded environment
      int num_next_pose = 6; // Number of possible next positions
      int degree_between_poses = 360 / num_next_pose;
      int moving_dist = 5; // Distance that target moves at each step
      std::vector<double> next_pose_x;
      std::vector<double> next_pose_y;
      int temp_pose_id; // Temporal next position IDs

      int each_degree = 0;
      for (int i = 0; i < num_next_pose; i++) {
        next_pose_x.push_back(moving_dist * cos(each_degree*PI/180));
        next_pose_y.push_back(moving_dist * sin(each_degree*PI/180));
        // printf("\n%d pose\n", i+1);
        // printf("each_degree = %d\n", each_degree);
        // printf("next_pose_x = %.2f, next_pose_y = %.2f\n", cos(each_degree*PI/180), sin(each_degree*PI/180));
        each_degree += degree_between_poses;
      }

      // Variables for targets
      std::vector<double> target_pos_x; std::vector<double> target_pos_y;
      std::vector<double> prev_target_pos_x; std::vector<double> prev_target_pos_y;
      std::vector<gazebo::common::PoseAnimationPtr> anim;
      std::vector<gazebo::common::PoseAnimationPtr> anim_next;

      // Targets
      for (int i = 0; i < num_targets; i++) {
        target_pos_x.push_back(rand()%env_size-env_size/2);
        target_pos_y.push_back(rand()%env_size-env_size/2);
        std::string temp_name = "target_"+boost::lexical_cast<std::string>(i);
        gazebo::common::PoseAnimationPtr temp_anim(
            new gazebo::common::PoseAnimation(temp_name, 1000.0, false));
        anim.push_back(temp_anim);
        if (flag_next) {
          std::string temp_name_next = "target_"+boost::lexical_cast<std::string>(i)+"_next_pose";
          gazebo::common::PoseAnimationPtr temp_anim_next(
              new gazebo::common::PoseAnimation(temp_name_next, 1000.0, false));
          anim_next.push_back(temp_anim_next);
        }

        gazebo::common::PoseKeyFrame *temp_key = anim[i]->CreateKeyFrame(0);
        temp_key->Translation(ignition::math::Vector3d(target_pos_x[i], target_pos_y[i], 0));
        temp_key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        for (int j = 0; j < tot_num_steps; j++) {
          prev_target_pos_x.push_back(target_pos_x[i]);
          prev_target_pos_y.push_back(target_pos_y[i]);

          while (1) {
            double temp_target_pos_x = target_pos_x[i];
            double temp_target_pos_y = target_pos_y[i];
            temp_pose_id = rand()%num_next_pose;
            temp_target_pos_x += next_pose_x[temp_pose_id];
            temp_target_pos_y += next_pose_y[temp_pose_id];

            if ((temp_target_pos_x <= env_size/2)&&(temp_target_pos_x >= (-1)*env_size/2)&&
              (temp_target_pos_y <= env_size/2)&&(temp_target_pos_y >= (-1)*env_size/2)) {
              break;
            }
          }

          target_pos_x[i] += next_pose_x[temp_pose_id];
          target_pos_y[i] += next_pose_y[temp_pose_id];
          // printf("Target_%d pose: (%.2f, %.2f)\n", i+1, target_pos_x[i], target_pos_y[i]);
          // printf("Distance traveled: %.2f\n", sqrt((prev_target_pos_x[i]-target_pos_x[i])*(prev_target_pos_x[i]-target_pos_x[i])
          //   +(prev_target_pos_y[i]-target_pos_y[i])*(prev_target_pos_y[i]-target_pos_y[i])));
          if (flag_next) {
            gazebo::common::PoseKeyFrame *temp_key_next = anim_next[i]->CreateKeyFrame(time_duration*j);
            temp_key_next->Translation(ignition::math::Vector3d(target_pos_x[i], target_pos_y[i], 0));
            temp_key_next->Rotation(ignition::math::Quaterniond(0, 0, 0));
          }
          temp_key = anim[i]->CreateKeyFrame(time_duration*(j+1));
          temp_key->Translation(ignition::math::Vector3d(target_pos_x[i], target_pos_y[i], 0));
          temp_key->Rotation(ignition::math::Quaterniond(0, 0, 0));
          if (flag_next) {
            gazebo::common::PoseKeyFrame *temp_key_next = anim_next[i]->CreateKeyFrame(time_duration*j);
            temp_key_next = anim_next[i]->CreateKeyFrame(time_duration*(j+1)-1);
            temp_key_next->Translation(ignition::math::Vector3d(target_pos_x[i], target_pos_y[i], 0));
            temp_key_next->Rotation(ignition::math::Quaterniond(0, 0, 0));
          }
        }

        if (std::strcmp(_parent->GetName().c_str(), temp_name.c_str()) == 0) {
          _parent->SetAnimation(anim[i]);
        }
        if (flag_next) {
          std::string temp_name_next = "target_"+boost::lexical_cast<std::string>(i)+"_next_pose";
          if (std::strcmp(_parent->GetName().c_str(), temp_name_next.c_str()) == 0) {
            _parent->SetAnimation(anim_next[i]);
          }
        }
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
}

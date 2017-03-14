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

namespace gazebo
{
  class AnimatedBox : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

        // create the animation
        gazebo::common::PoseAnimationPtr anim(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test", 10.0, true));

        gazebo::common::PoseKeyFrame *key;

        // set starting location of the box
        key = anim->CreateKeyFrame(0);
        key->Translation(ignition::math::Vector3d(0, 3, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // // set waypoint location after 2 seconds
        // key = anim->CreateKeyFrame(2.0);
        // key->Translation(ignition::math::Vector3d(-50, -50, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(4.0);
        // key->Translation(ignition::math::Vector3d(10, 20, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(6.0);
        // key->Translation(ignition::math::Vector3d(-10, 20, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(8.0);
        // key->Translation(ignition::math::Vector3d(10, -20, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));

        // // set final location equal to starting location
        // key = anim->CreateKeyFrame(10);
        // key->Translation(ignition::math::Vector3d(0, 0, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_1") == 0) {
          // set the animation
          _parent->SetAnimation(anim);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_2(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_2", 10.0, true));

        gazebo::common::PoseKeyFrame *key_2;

        // set starting location of the box
        key_2 = anim_2->CreateKeyFrame(0);
        key_2->Translation(ignition::math::Vector3d(-3, 6, 0));
        key_2->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_2") == 0) {
          // set the animation
          _parent->SetAnimation(anim_2);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_3(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_3", 10.0, true));

        gazebo::common::PoseKeyFrame *key_3;

        // set starting location of the box
        key_3 = anim_3->CreateKeyFrame(0);
        key_3->Translation(ignition::math::Vector3d(4, 8, 0));
        key_3->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_3") == 0) {
          // set the animation
          _parent->SetAnimation(anim_3);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_4(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_4", 10.0, true));

        gazebo::common::PoseKeyFrame *key_4;

        // set starting location of the box
        key_4 = anim_4->CreateKeyFrame(0);
        key_4->Translation(ignition::math::Vector3d(5, 5, 0));
        key_4->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_4") == 0) {
          // set the animation
          _parent->SetAnimation(anim_4);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_5(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_5", 10.0, true));

        gazebo::common::PoseKeyFrame *key_5;

        // set starting location of the box
        key_5 = anim_5->CreateKeyFrame(0);
        key_5->Translation(ignition::math::Vector3d(-4, 6, 0));
        key_5->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_5") == 0) {
          // set the animation
          _parent->SetAnimation(anim_5);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_6(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_6", 10.0, true));

        gazebo::common::PoseKeyFrame *key_6;

        // set starting location of the box
        key_6 = anim_6->CreateKeyFrame(0);
        key_6->Translation(ignition::math::Vector3d(6, 4, 0));
        key_6->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_6") == 0) {
          // set the animation
          _parent->SetAnimation(anim_6);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_7(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_7", 10.0, true));

        gazebo::common::PoseKeyFrame *key_7;

        // set starting location of the box
        key_7 = anim_7->CreateKeyFrame(0);
        key_7->Translation(ignition::math::Vector3d(6, 1, 0));
        key_7->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_7") == 0) {
          // set the animation
          _parent->SetAnimation(anim_7);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_8(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_8", 10.0, true));

        gazebo::common::PoseKeyFrame *key_8;

        // set starting location of the box
        key_8 = anim_8->CreateKeyFrame(0);
        key_8->Translation(ignition::math::Vector3d(2, 3, 0));
        key_8->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_8") == 0) {
          // set the animation
          _parent->SetAnimation(anim_8);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_9(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_9", 10.0, true));

        gazebo::common::PoseKeyFrame *key_9;

        // set starting location of the box
        key_9 = anim_9->CreateKeyFrame(0);
        key_9->Translation(ignition::math::Vector3d(-5, 3, 0));
        key_9->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_9") == 0) {
          // set the animation
          _parent->SetAnimation(anim_9);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_10(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_10", 10.0, true));

        gazebo::common::PoseKeyFrame *key_10;

        // set starting location of the box
        key_10 = anim_10->CreateKeyFrame(0);
        key_10->Translation(ignition::math::Vector3d(0, -3, 0));
        key_10->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_10") == 0) {
          // set the animation
          _parent->SetAnimation(anim_10);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_11(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_11", 10.0, true));

        gazebo::common::PoseKeyFrame *key_11;

        // set starting location of the box
        key_11 = anim_11->CreateKeyFrame(0);
        key_11->Translation(ignition::math::Vector3d(0, 8, 0));
        key_11->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_11") == 0) {
          // set the animation
          _parent->SetAnimation(anim_11);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_12(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_12", 10.0, true));

        gazebo::common::PoseKeyFrame *key_12;

        // set starting location of the box
        key_12 = anim_12->CreateKeyFrame(0);
        key_12->Translation(ignition::math::Vector3d(-1, -4, 0));
        key_12->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_12") == 0) {
          // set the animation
          _parent->SetAnimation(anim_12);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_13(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_13", 10.0, true));

        gazebo::common::PoseKeyFrame *key_13;

        // set starting location of the box
        key_13 = anim_13->CreateKeyFrame(0);
        key_13->Translation(ignition::math::Vector3d(2, -3, 0));
        key_13->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_13") == 0) {
          // set the animation
          _parent->SetAnimation(anim_13);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_14(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_14", 10.0, true));

        gazebo::common::PoseKeyFrame *key_14;

        // set starting location of the box
        key_14 = anim_14->CreateKeyFrame(0);
        key_14->Translation(ignition::math::Vector3d(4, -3, 0));
        key_14->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_14") == 0) {
          // set the animation
          _parent->SetAnimation(anim_14);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_15(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_15", 10.0, true));

        gazebo::common::PoseKeyFrame *key_15;

        // set starting location of the box
        key_15 = anim_15->CreateKeyFrame(0);
        key_15->Translation(ignition::math::Vector3d(-6, 0, 0));
        key_15->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_15") == 0) {
          // set the animation
          _parent->SetAnimation(anim_15);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_16(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_16", 10.0, true));

        gazebo::common::PoseKeyFrame *key_16;

        // set starting location of the box
        key_16 = anim_16->CreateKeyFrame(0);
        key_16->Translation(ignition::math::Vector3d(-4, -4, 0));
        key_16->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_16") == 0) {
          // set the animation
          _parent->SetAnimation(anim_16);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_17(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_17", 10.0, true));

        gazebo::common::PoseKeyFrame *key_17;

        // set starting location of the box
        key_17 = anim_17->CreateKeyFrame(0);
        key_17->Translation(ignition::math::Vector3d(8, -2, 0));
        key_17->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_17") == 0) {
          // set the animation
          _parent->SetAnimation(anim_17);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_18(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_18", 10.0, true));

        gazebo::common::PoseKeyFrame *key_18;

        // set starting location of the box
        key_18 = anim_18->CreateKeyFrame(0);
        key_18->Translation(ignition::math::Vector3d(1, -2, 0));
        key_18->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_18") == 0) {
          // set the animation
          _parent->SetAnimation(anim_18);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_19(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_19", 10.0, true));

        gazebo::common::PoseKeyFrame *key_19;

        // set starting location of the box
        key_19 = anim_19->CreateKeyFrame(0);
        key_19->Translation(ignition::math::Vector3d(-8, -3, 0));
        key_19->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_19") == 0) {
          // set the animation
          _parent->SetAnimation(anim_19);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_20(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_20", 10.0, true));

        gazebo::common::PoseKeyFrame *key_20;

        // set starting location of the box
        key_20 = anim_20->CreateKeyFrame(0);
        key_20->Translation(ignition::math::Vector3d(-1, -3, 0));
        key_20->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_20") == 0) {
          // set the animation
          _parent->SetAnimation(anim_20);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_21(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_21", 10.0, true));

        gazebo::common::PoseKeyFrame *key_21;

        // set starting location of the box
        key_21 = anim_21->CreateKeyFrame(0);
        key_21->Translation(ignition::math::Vector3d(5, 9, 0));
        key_21->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_21") == 0) {
          // set the animation
          _parent->SetAnimation(anim_21);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_22(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_22", 10.0, true));

        gazebo::common::PoseKeyFrame *key_22;

        // set starting location of the box
        key_22 = anim_22->CreateKeyFrame(0);
        key_22->Translation(ignition::math::Vector3d(-7, -1, 0));
        key_22->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_22") == 0) {
          // set the animation
          _parent->SetAnimation(anim_22);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_23(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_23", 10.0, true));

        gazebo::common::PoseKeyFrame *key_23;

        // set starting location of the box
        key_23 = anim_23->CreateKeyFrame(0);
        key_23->Translation(ignition::math::Vector3d(7, 0, 0));
        key_23->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_23") == 0) {
          // set the animation
          _parent->SetAnimation(anim_23);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_24(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_24", 10.0, true));

        gazebo::common::PoseKeyFrame *key_24;

        // set starting location of the box
        key_24 = anim_24->CreateKeyFrame(0);
        key_24->Translation(ignition::math::Vector3d(-1, 2, 0));
        key_24->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_24") == 0) {
          // set the animation
          _parent->SetAnimation(anim_24);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_25(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_25", 10.0, true));

        gazebo::common::PoseKeyFrame *key_25;

        // set starting location of the box
        key_25 = anim_25->CreateKeyFrame(0);
        key_25->Translation(ignition::math::Vector3d(4, 9, 0));
        key_25->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_25") == 0) {
          // set the animation
          _parent->SetAnimation(anim_25);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_26(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_26", 10.0, true));

        gazebo::common::PoseKeyFrame *key_26;

        // set starting location of the box
        key_26 = anim_26->CreateKeyFrame(0);
        key_26->Translation(ignition::math::Vector3d(6, 0, 0));
        key_26->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_26") == 0) {
          // set the animation
          _parent->SetAnimation(anim_26);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_27(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_27", 10.0, true));

        gazebo::common::PoseKeyFrame *key_27;

        // set starting location of the box
        key_27 = anim_27->CreateKeyFrame(0);
        key_27->Translation(ignition::math::Vector3d(-5, 1, 0));
        key_27->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_27") == 0) {
          // set the animation
          _parent->SetAnimation(anim_27);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_28(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_28", 10.0, true));

        gazebo::common::PoseKeyFrame *key_28;

        // set starting location of the box
        key_28 = anim_28->CreateKeyFrame(0);
        key_28->Translation(ignition::math::Vector3d(-1, 6, 0));
        key_28->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_28") == 0) {
          // set the animation
          _parent->SetAnimation(anim_28);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_29(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_29", 10.0, true));

        gazebo::common::PoseKeyFrame *key_29;

        // set starting location of the box
        key_29 = anim_29->CreateKeyFrame(0);
        key_29->Translation(ignition::math::Vector3d(1, -3, 0));
        key_29->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_29") == 0) {
          // set the animation
          _parent->SetAnimation(anim_29);
        }

        // create the animation
        gazebo::common::PoseAnimationPtr anim_30(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test_30", 10.0, true));

        gazebo::common::PoseKeyFrame *key_30;

        // set starting location of the box
        key_30 = anim_30->CreateKeyFrame(0);
        key_30->Translation(ignition::math::Vector3d(7, -2, 0));
        key_30->Rotation(ignition::math::Quaterniond(0, 0, 0));

        if (std::strcmp(_parent->GetName().c_str(), "target_30") == 0) {
          // set the animation
          _parent->SetAnimation(anim_30);
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

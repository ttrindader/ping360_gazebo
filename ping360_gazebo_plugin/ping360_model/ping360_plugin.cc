#ifndef _PING_PLUGIN_HH_
#define _PING_PLUGIN_HH_

#include <thread>
#include <cmath>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <ping360_sonar/SonarEcho.h>
#include <sensor_msgs/LaserScan.h>

namespace gazebo
{
  /// \brief A plugin to control the sensor.
  class PingPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: PingPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      for (auto joint: _model->GetJoints())
        if (joint->GetName() == "inner_disk_joint")
          this->joint = joint;

      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(0.1, 0, 0);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint->GetScopedName(), this->pid);

      // Check that the velocity element exists, then read the value
      double velocity = 0;
      if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");

      // Set the joint's target velocity. This target velocity is just
      // for demonstration purposes.
      this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), velocity);

      // Start ROS related
      this->rosNode.reset(new ros::NodeHandle(""));
      this->sonarMsgPub = this->rosNode->advertise<ping360_sonar::SonarEcho>("ping360_node/sonar/data", 10);
      this->sonarMsgInfoPub = this->rosNode->advertise<sensor_msgs::LaserScan>("ping360_node/sonar/scan", 10);

      // Cast the sensor to a RaySensor and store it in a member variable
      sensors::SensorManager* sensorManager = sensors::SensorManager::Instance();
      this->sensor = sensorManager->GetSensor("sonar_sensor");
      this->raySensor = std::dynamic_pointer_cast<sensors::RaySensor>(this->sensor);
      this->updateConnection = raySensor->ConnectUpdated(std::bind(&PingPlugin::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      auto rangeSensor = std::dynamic_pointer_cast<sensors::RaySensor>(this->sensor);

      // Get the latest range measurements from the sensor
      double range = rangeSensor->Range(0);
      // Get the joint angle in degrees
      double angle = this->joint->Position(0) * 180/M_PI;
      int angle_ranged = angle - static_cast<int>(angle/360)*360;

      // // Print the range to the console
      // std::cout << "\nRange: " << range;
      // std::cout << "\nJoint angle: " << angle_ranged << std::endl;

      // If resolution is past 10 degrees, publish it
      if (abs(angle_ranged % 10) <= 3)
      {
        // Publish to ROS
        ping360_sonar::SonarEcho msg;
        msg.header.frame_id = "sonar_frame";
        msg.header.stamp = ros::Time::now();
        msg.number_of_samples = 1000;
        msg.speed_of_sound = 1550;
        msg.transmit_frequency = 1000;
        msg.angle = static_cast<float>(angle_ranged);
        msg.range = range;
        this->sonarMsgPub.publish(msg);
        sensor_msgs::LaserScan msg_info;
        msg_info.header = msg.header;
        msg_info.angle_max = 2*M_PI;
        msg_info.angle_increment = M_PI/10;
        msg_info.range_min = 0.75;
        msg_info.range_max = 30;
        msg_info.ranges = std::vector<float>{range};
        msg_info.intensities = std::vector<float>{0};
        this->sonarMsgInfoPub.publish(msg_info);
      }
    }

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    /// \brief range sonar to get the measures
    private: sensors::SensorPtr sensor;
    private: sensors::RaySensorPtr raySensor;

    /// \brief connection bind for each update from the world
    private: event::ConnectionPtr updateConnection;

    /// \brief ROS node handle
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief ROS sonar message publisher
    private: ros::Publisher sonarMsgPub;
    private: ros::Publisher sonarMsgInfoPub;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(PingPlugin)
}
#endif
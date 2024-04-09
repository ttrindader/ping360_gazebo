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

  class PingPlugin : public ModelPlugin
  {

    public: PingPlugin() {}


    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }


      this->model = _model;


      for (auto joint: _model->GetJoints())
        if (joint->GetName() == "ping360__inner_disk_joint")
          this->joint = joint;


      this->pid = common::PID(0.1, 0, 0);


      this->model->GetJointController()->SetVelocityPID(
          this->joint->GetScopedName(), this->pid);


      double velocity = 0;
      if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");
        

      this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), velocity);


      this->rosNode.reset(new ros::NodeHandle(""));
      this->sonarMsgPub = this->rosNode->advertise<ping360_sonar::SonarEcho>("ping360_node/sonar/data", 10);


      sensors::SensorManager* sensorManager = sensors::SensorManager::Instance();
      this->sensor = sensorManager->GetSensor("sonar_sensor");
      this->raySensor = std::dynamic_pointer_cast<sensors::RaySensor>(this->sensor);
      this->updateConnection = raySensor->ConnectUpdated(std::bind(&PingPlugin::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      auto rangeSensor = std::dynamic_pointer_cast<sensors::RaySensor>(this->sensor);


      std::vector<double> ranges;
      rangeSensor->Ranges(ranges);

      std::vector<float> ranges_float(ranges.size());
      for (size_t i = 0; i < ranges.size(); i++){
        ranges_float[i] = std::isinf(ranges[i]) ? 255 : 12.75*static_cast<float>(ranges[i]); //12.75 depends - sonarranges max
        //std::cerr << "ranges_float de " << i << " = " << ranges_float[i] << " \n ";
    }
      double range = *min_element(ranges_float.begin(), ranges_float.end()); // [cm]

      // Get the joint angle in degrees
      float angle = this->joint->Position(0) * 180/M_PI;
      int angle_ranged = angle - static_cast<int>(angle/360)*360;

      // Calculate intensity based on range
      std::vector<float> intensities(ranges_float.size());
      for (size_t i = 0; i < ranges_float.size(); i++)
      {
      /*-   if (std::isinf(ranges_float[i]))
        {
          intensities[i] = 0.0;
        }
        else
        {
          // Example calculation for intensity (you can adjust this based on your requirements)
          intensities[i] = 255.0 * (ranges_float[i]);
        }*/
        intensities[i] = 255 - (unsigned char)ranges_float[i];
      }


      std::vector<unsigned char> intensities_char;
      intensities_char.reserve(intensities.size());
      for (size_t i = 0; i < intensities.size(); ++i) {
          intensities_char.push_back(static_cast<unsigned char>(intensities[i]));
      }


      if (abs(angle_ranged % 5) <= 0)
      {
        // Publish to ROS
        ping360_sonar::SonarEcho msg;
        msg.header.frame_id = "sonar_frame";
        msg.header.stamp = ros::Time::now();
        msg.number_of_samples = ranges_float.size();
        msg.speed_of_sound = 1499;
        msg.transmit_frequency = 740;
        msg.angle = static_cast<float>(angle_ranged);
        msg.gain = 1; 
        msg.range = range; 
        msg.intensities = intensities_char;
        this->sonarMsgPub.publish(msg);
        
      }
    }

    private: physics::ModelPtr model;

    private: physics::JointPtr joint;

    private: common::PID pid;

    private: sensors::SensorPtr sensor;
    private: sensors::RaySensorPtr raySensor;

    private: event::ConnectionPtr updateConnection;

    private: std::unique_ptr<ros::NodeHandle> rosNode;

    private: ros::Publisher sonarMsgPub;
    private: ros::Publisher sonarMsgInfoPub;
  };


  GZ_REGISTER_MODEL_PLUGIN(PingPlugin)
}
#endif

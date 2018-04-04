#include "../include/joystick.h"
#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "p2os_msgs/PTZState.h"

#include <map>
#include <string>


namespace joystick
{

    struct Joystick::Impl
    {
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
        void ptzStateCallback(const p2os_msgs::PTZState::ConstPtr &ptz_state_msg);

        void processRobotMove();
        void processCameraMove();

        ros::Subscriber joy_sub;
        ros::Subscriber ptz_state_sub;
        ros::Publisher cmd_vel_pub;
        ros::Publisher ptz_control_pub;

        double axes[6];
        int buttons[12];
        int pan, tilt, zoom;

        int enable_button;
        int enable_turbo_button;

        std::map<std::string, int> axis_linear_map;
        std::map<std::string, double> scale_linear_map;
        std::map<std::string, double> scale_linear_turbo_map;

        std::map<std::string, int> axis_angular_map;
        std::map<std::string, double> scale_angular_map;
        std::map<std::string, double> scale_angular_turbo_map;

        bool sent_disable_msg;

        int step_pan, step_tilt, step_zoom;

        int pan_positive_button, pan_negative_button;
        int tilt_positive_button, tilt_negative_button;
        int zoom_positive_button, zoom_negative_button;
    };

    // ctor
    Joystick::Joystick(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
    {
        pimpl_ = new Impl;

        pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &Joystick::Impl::joyCallback, pimpl_);
        pimpl_->ptz_state_sub = nh->subscribe<p2os_msgs::PTZState>("ptz_state", 1, &Joystick::Impl::ptzStateCallback, pimpl_);

        pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, false);
        pimpl_->ptz_control_pub = nh->advertise<p2os_msgs::PTZState>("ptz_control", 1, false);


        nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
        nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

        nh_param->param<int>("step_pan", pimpl_->step_pan, 10);
        nh_param->param<int>("step_tilt", pimpl_->step_tilt, 10);
        nh_param->param<int>("step_zoom", pimpl_->step_zoom, 100);

        nh_param->param<int>("tilt_positive_button", pimpl_->tilt_positive_button, 0);
        nh_param->param<int>("pan_positive_button", pimpl_->pan_positive_button, 1);
        nh_param->param<int>("tilt_negative_button", pimpl_->tilt_negative_button, 2);
        nh_param->param<int>("pan_negative_button", pimpl_->pan_negative_button, 3);
        nh_param->param<int>("zoom_negative_button", pimpl_->zoom_negative_button, 4);
        nh_param->param<int>("zoom_positive_button", pimpl_->zoom_positive_button, 5);

        // get linear scales
        if (nh_param->getParam("axis_linear", pimpl_->axis_linear_map))
        {
        nh_param->getParam("axis_linear", pimpl_->axis_linear_map);
        nh_param->getParam("scale_linear", pimpl_->scale_linear_map);
        nh_param->getParam("scale_linear_turbo", pimpl_->scale_linear_turbo_map);
        }
        // default linear scales
        else
        {
        nh_param->param<int>("axis_linear", pimpl_->axis_linear_map["x"], 1);
        nh_param->param<double>("scale_linear", pimpl_->scale_linear_map["x"], 0.5);
        nh_param->param<double>("scale_linear_turbo", pimpl_->scale_linear_turbo_map["x"], 1.0);
        }

        // get angular scales
        if (nh_param->getParam("axis_angular", pimpl_->axis_angular_map))
        {
        nh_param->getParam("axis_angular", pimpl_->axis_angular_map);
        nh_param->getParam("scale_angular", pimpl_->scale_angular_map);
        nh_param->getParam("scale_angular_turbo", pimpl_->scale_angular_turbo_map);
        }
        // default angular scales
        else
        {
        nh_param->param<int>("axis_angular", pimpl_->axis_angular_map["yaw"], 0);
        nh_param->param<double>("scale_angular", pimpl_->scale_angular_map["yaw"], 0.5);
        nh_param->param<double>("scale_angular_turbo",
            pimpl_->scale_angular_turbo_map["yaw"], pimpl_->scale_angular_map["yaw"]);
        }

        ROS_INFO("Teleop enable button %i.", pimpl_->enable_button);
        ROS_INFO_COND(pimpl_->enable_turbo_button >= 0,
          "Turbo on button %i.", pimpl_->enable_turbo_button);

        for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_map.begin();
          it != pimpl_->axis_linear_map.end(); ++it)
        {
        ROS_INFO("Linear axis %s on %i at scale %f.",
        it->first.c_str(), it->second, pimpl_->scale_linear_map[it->first]);
        ROS_INFO_COND(pimpl_->enable_turbo_button >= 0,
            "Turbo for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_turbo_map[it->first]);
        }

        for (std::map<std::string, int>::iterator it = pimpl_->axis_angular_map.begin();
          it != pimpl_->axis_angular_map.end(); ++it)
        {
        ROS_INFO("Angular axis %s on %i at scale %f.",
        it->first.c_str(), it->second, pimpl_->scale_angular_map[it->first]);
        ROS_INFO_COND(pimpl_->enable_turbo_button >= 0,
            "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_turbo_map[it->first]);
        }

        pimpl_->sent_disable_msg = false;

        // camera
        ROS_INFO("Step pan %i.", pimpl_->step_pan);
        ROS_INFO("Step tilt %i.", pimpl_->step_tilt);
        ROS_INFO("Pan positive button %i.", pimpl_->pan_positive_button);
        ROS_INFO("Pan negative button %i.", pimpl_->pan_negative_button);
        ROS_INFO("Tilt positive button %i.", pimpl_->tilt_positive_button);
        ROS_INFO("Tilt negative button %i.", pimpl_->tilt_negative_button);
        ROS_INFO("Zoom positive button %i.", pimpl_->zoom_positive_button);
        ROS_INFO("Zoom negative button %i.", pimpl_->zoom_negative_button);

        spin();
    }

    void Joystick::spin()
    {
        ros::Rate loop_rate(50);
        while(ros::ok())
        {
            // atualiza callbacks
            ros::spinOnce();

            pimpl_->processRobotMove();
            pimpl_->processCameraMove();
            loop_rate.sleep();
        }

    }

    // callback
    void Joystick::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        for(int i = 0; i < 6; i++)
                axes[i] = joy_msg->axes[i];

        for(int i = 0; i < 12; i++)
                buttons[i] = joy_msg->buttons[i];
    }

    void Joystick::Impl::ptzStateCallback(const p2os_msgs::PTZState::ConstPtr& ptz_state_msg)
    {
        pan = ptz_state_msg->pan;
        tilt = ptz_state_msg->tilt;
        zoom = ptz_state_msg->zoom;
    }

    void Joystick::Impl::processRobotMove()
    {
        // Initializes with zeros by default.
        geometry_msgs::Twist cmd_vel_msg;

        // When turbo button
        if (enable_turbo_button >= 0 && buttons[enable_turbo_button])
        {
          if (axis_linear_map.find("x") != axis_linear_map.end())
          {
            cmd_vel_msg.linear.x = axes[axis_linear_map["x"]] * scale_linear_turbo_map["x"];
          }
          if (axis_linear_map.find("y") != axis_linear_map.end())
          {
            cmd_vel_msg.linear.y = axes[axis_linear_map["y"]] * scale_linear_turbo_map["y"];
          }
          if  (axis_linear_map.find("z") != axis_linear_map.end())
          {
            cmd_vel_msg.linear.z = axes[axis_linear_map["z"]] * scale_linear_turbo_map["z"];
          }
          if  (axis_angular_map.find("yaw") != axis_angular_map.end())
          {
            cmd_vel_msg.angular.z = axes[axis_angular_map["yaw"]] * scale_angular_turbo_map["yaw"];
          }
          if  (axis_angular_map.find("pitch") != axis_angular_map.end())
          {
            cmd_vel_msg.angular.y = axes[axis_angular_map["pitch"]] * scale_angular_turbo_map["pitch"];
          }
          if  (axis_angular_map.find("roll") != axis_angular_map.end())
          {
            cmd_vel_msg.angular.x = axes[axis_angular_map["roll"]] * scale_angular_turbo_map["roll"];
          }

          cmd_vel_pub.publish(cmd_vel_msg);
          sent_disable_msg = false;
        }
        else if (buttons[enable_button])
        {
          if  (axis_linear_map.find("x") != axis_linear_map.end())
          {
            cmd_vel_msg.linear.x = axes[axis_linear_map["x"]] * scale_linear_map["x"];
          }
          if  (axis_linear_map.find("y") != axis_linear_map.end())
          {
            cmd_vel_msg.linear.y = axes[axis_linear_map["y"]] * scale_linear_map["y"];
          }
          if  (axis_linear_map.find("z") != axis_linear_map.end())
          {
            cmd_vel_msg.linear.z = axes[axis_linear_map["z"]] * scale_linear_map["z"];
          }
          if  (axis_angular_map.find("yaw") != axis_angular_map.end())
          {
            cmd_vel_msg.angular.z = axes[axis_angular_map["yaw"]] * scale_angular_map["yaw"];
          }
          if  (axis_angular_map.find("pitch") != axis_angular_map.end())
          {
            cmd_vel_msg.angular.y = axes[axis_angular_map["pitch"]] * scale_angular_map["pitch"];
          }
          if  (axis_angular_map.find("roll") != axis_angular_map.end())
          {
            cmd_vel_msg.angular.x = axes[axis_angular_map["roll"]] * scale_angular_map["roll"];
          }

          cmd_vel_pub.publish(cmd_vel_msg);
          sent_disable_msg = false;
        }
        else
        {
          // When enable button is released, immediately send a single no-motion command
          // in order to stop the robot.
          if (!sent_disable_msg)
          {
            cmd_vel_pub.publish(cmd_vel_msg);
            sent_disable_msg = true;
          }
        }

    }

    void Joystick::Impl::processCameraMove()
    {
        static p2os_msgs::PTZState ptz_state;

        if(buttons[pan_positive_button])
        {
          ptz_state.pan = pan + step_pan;
          if(ptz_state.pan > MAX_PAN) ptz_state.pan = MAX_PAN;
        }

        if(buttons[pan_negative_button])
        {
          ptz_state.pan = pan - step_pan;
          if(ptz_state.pan < MIN_PAN) ptz_state.pan = MIN_PAN;
        }

        if(buttons[tilt_positive_button])
        {
          ptz_state.tilt = tilt + step_tilt;
          if(ptz_state.tilt > MAX_TILT) ptz_state.tilt = MAX_TILT;
        }

        if(buttons[tilt_negative_button])
        {
          ptz_state.tilt = tilt - step_tilt;
          if(ptz_state.tilt < MIN_TILT) ptz_state.tilt = MIN_TILT;
        }

        if(buttons[zoom_positive_button])
        {
          ptz_state.zoom = zoom + step_zoom;
          if(ptz_state.zoom > MAX_ZOOM) ptz_state.zoom = MAX_ZOOM;
        }

        if(buttons[zoom_negative_button])
        {
          ptz_state.zoom = zoom - step_zoom;
          if(ptz_state.zoom < MIN_ZOOM) ptz_state.zoom = MIN_ZOOM;
        }

        ptz_state.relative = false;
        ptz_control_pub.publish(ptz_state);
    }

}  // namespace joystick

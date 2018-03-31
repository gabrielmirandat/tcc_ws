#ifndef JOYSTICK_H
#define JOYSTICK_H

namespace ros { class NodeHandle; }

#define MAX_PAN 97
#define MAX_TILT 87
#define MIN_PAN -97
#define MIN_TILT -29

namespace joystick
{
    class Joystick
    {
    public:
        Joystick(ros::NodeHandle* nh, ros::NodeHandle* nh_param);
        void spin();

    private:
        struct Impl;
        Impl* pimpl_;
    };
}  // namespace joystick

#endif  // JOYSTICK_H

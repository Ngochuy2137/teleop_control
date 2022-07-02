#include <ros/ros.h>
#include <ros/param.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#else
#include <windows.h>
#endif

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76

#define LINEAR_VEL 0.1
#define ANGULAR_VEL 0.1

class KeyboardReader
{
public:
    KeyboardReader()
#ifndef _WIN32
        : kfd(0)
#endif
    {
#ifndef _WIN32
        // get the console in raw mode
        tcgetattr(kfd, &cooked);
        struct termios raw;
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        // Setting a new line, then end of file
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
#endif
    }
    void readOne(char *c)
    {
#ifndef _WIN32
        int rc = read(kfd, c, 1);
        if (rc < 0)
        {
            throw std::runtime_error("read failed");
        }
#else
        for (;;)
        {
            HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
            INPUT_RECORD buffer;
            DWORD events;
            PeekConsoleInput(handle, &buffer, 1, &events);
            if (events > 0)
            {
                ReadConsoleInput(handle, &buffer, 1, &events);
                if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT)
                {
                    *c = KEYCODE_LEFT;
                    return;
                }
                else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP)
                {
                    *c = KEYCODE_UP;
                    return;
                }
                else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
                {
                    *c = KEYCODE_RIGHT;
                    return;
                }
                else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
                {
                    *c = KEYCODE_DOWN;
                    return;
                }
                else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x42)
                {
                    *c = KEYCODE_B;
                    return;
                }
                else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x43)
                {
                    *c = KEYCODE_C;
                    return;
                }
                else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x44)
                {
                    *c = KEYCODE_D;
                    return;
                }
                else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x45)
                {
                    *c = KEYCODE_E;
                    return;
                }
                else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x46)
                {
                    *c = KEYCODE_F;
                    return;
                }
                else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x47)
                {
                    *c = KEYCODE_G;
                    return;
                }
                else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x51)
                {
                    *c = KEYCODE_Q;
                    return;
                }
                else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x52)
                {
                    *c = KEYCODE_R;
                    return;
                }
                else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x54)
                {
                    *c = KEYCODE_T;
                    return;
                }
                else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x56)
                {
                    *c = KEYCODE_V;
                    return;
                }
            }
        }
#endif
    }
    void shutdown()
    {
#ifndef _WIN32
        tcsetattr(kfd, TCSANOW, &cooked);
#endif
    }

private:
#ifndef _WIN32
    int kfd;
    struct termios cooked;
#endif
};

KeyboardReader input;

class TeleopControl
{
public:
    TeleopControl(double linear_vel_, double angular_vel_);
    void keyLoop();

private:
    ros::NodeHandle nh_;
    double linear_vel_max, angular_vel_max, linear_vel_set, angular_vel_set;
    ros::Publisher twist_pub_;
};

TeleopControl::TeleopControl(double linear_vel_, double angular_vel_) : linear_vel_set(0),
                                                                        angular_vel_set(0),
                                                                        linear_vel_max(linear_vel_),
                                                                        angular_vel_max(angular_vel_)
{
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void quit(int sig)
{
    (void)sig;
    input.shutdown();
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_control");
    double linear_vel_;
    double angular_vel_;
    if(argc == 3)
    {
        linear_vel_  = atof(argv[1]);
        angular_vel_ = atof(argv[2]);

        std::cout << "Setup:\n  linear_vel_ = " << linear_vel_ << " m/s \n  angular_vel_ = " << angular_vel_ << " rad/s " << std::endl;
    }
    else
    {
        ROS_INFO("Using default values: \n  linear_vel_ = 1.0 m/s\n  angular_vel_ = 0.5 rad/s");
        linear_vel_  = 1.0;
        angular_vel_ = 0.5;
    }


    TeleopControl teleop_control(linear_vel_, angular_vel_);

    signal(SIGINT, quit);

    teleop_control.keyLoop();
    quit(0);

    return (0);
}

void TeleopControl::keyLoop()
{
    char c;
    bool dirty = false;

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robot. 'q' to quit.");

    for (;;)
    {
        // get the next event from the keyboard
        try
        {
            input.readOne(&c);
        }
        catch (const std::runtime_error &)
        {
            perror("read():");
            return;
        }

        ROS_DEBUG("value: 0x%02X\n", c);
        linear_vel_set = angular_vel_set = 0.0;
        switch (c)
        {
        case KEYCODE_LEFT:
            ROS_DEBUG("LEFT");
            angular_vel_set = angular_vel_max;
            dirty = true;
            break;
        case KEYCODE_RIGHT:
            ROS_DEBUG("RIGHT");
            angular_vel_set = -angular_vel_max;
            dirty = true;
            break;
        case KEYCODE_UP:
            ROS_DEBUG("UP");
            linear_vel_set = linear_vel_max;
            dirty = true;
            break;
        case KEYCODE_DOWN:
            ROS_DEBUG("DOWN");
            linear_vel_set = -linear_vel_max;
            dirty = true;
            break;
        case KEYCODE_Q:
            ROS_DEBUG("quit");
            return;
        }

        geometry_msgs::Twist twist;
        twist.angular.z = angular_vel_set;
        twist.linear.x  = linear_vel_set;
        if (dirty == true)
        {
            twist_pub_.publish(twist);
            dirty = false;
        }
    }
    return;
}

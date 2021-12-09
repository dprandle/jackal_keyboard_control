#include <ros/ros.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <unordered_set>

const int MAX_GEAR = 10;
geometry_msgs::Twist msg;
std::unordered_set<int> keys_pressed;
int gear = 0;
double angular_gear_mult = 0.14;
double linear_gear_mult = 0.1;
ros::Publisher cmd_vel_pub;

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS)
        keys_pressed.insert(key);
    else if (action == GLFW_RELEASE)
        keys_pressed.erase(key);

    if (key == GLFW_KEY_UP && action == GLFW_PRESS && gear < MAX_GEAR)
        ++gear;
    if (key == GLFW_KEY_DOWN && action == GLFW_PRESS && gear > 0)
        --gear;
}

void window_close_callback(GLFWwindow *window)
{
    ros::shutdown();
    glfwDestroyWindow(window);
}

void update_msg(geometry_msgs::Twist &msg)
{
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;

    if (keys_pressed.find(GLFW_KEY_W) != keys_pressed.end())
        msg.linear.x += linear_gear_mult * gear;
    if (keys_pressed.find(GLFW_KEY_S) != keys_pressed.end())
        msg.linear.x -= linear_gear_mult * gear;
    if (keys_pressed.find(GLFW_KEY_A) != keys_pressed.end())
        msg.angular.z += angular_gear_mult * gear;
    if (keys_pressed.find(GLFW_KEY_D) != keys_pressed.end())
        msg.angular.z -= angular_gear_mult * gear;

    if (keys_pressed.find(GLFW_KEY_SPACE) != keys_pressed.end())
    {
        gear = 0;
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
    }
    cmd_vel_pub.publish(msg);
}

int main(int argc, char **argv)
{
    if (!glfwInit())
        return 0;

    ros::init(argc, argv, "jackal_keyboard_control");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    GLFWwindow *window = glfwCreateWindow(640, 480, "Input Detection", nullptr, nullptr);
    glfwSetKeyCallback(window, key_callback);
    glfwSetWindowCloseCallback(window, window_close_callback);

    while (ros::ok())
    {
        update_msg(msg);
        std::cout << "Gear: " << gear << " and Twist: " << msg;
        glfwPollEvents();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
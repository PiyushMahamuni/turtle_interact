#include "ros/ros.h"
#include "turtle_interact/Order.h"
#include <sstream>

// Constants
const char *node_name{"order_turtle_draw"};
const char *order_pub_topic{"turtle_draw/order_queue"};

// Globals
ros::Publisher order_pub; // publishes msgs to turtle_draw/order_queue topic

// main driver
int main(int argc, char **argv)
{
    // setting up node
    ros::init(argc, argv, node_name);
    ros::NodeHandle node;
    order_pub = node.advertise<turtle_interact::Order>(order_pub_topic, 100);

    // setting up variables for menu
    turtle_interact::Order msg;
    std::string cmd;

    // MENU
    std::cout << "--- MENU ---\n"
              << "quit: quit this program\n"
              << "turnr <angle in radians> <0 -clk/1 -cclk>: turn robot through given radians\n"
              << "turnd <angle in degrees> <0 -clk/1 -cclk>: turn robot through given degress\n"
              << "turntor <angle in radians> <0 -clk/1 -cclk>: turn robot to specific tehta value in radians\n"
              << "turntod <angle in degrees> <0 -clk/1 -cclk>: turn robot to specific theta value in degrees\n"
              << "forward <lenght>: to move robot forward through given amount\n"
              << "backward <length>: to move robot backwards through given amount\n"
              << "square <side> <0 - clk/1 -cclk>\n"
              << "circle <radius> <0 -clk/1 -cclk>\n"
              << "moveto <x> <y>: move to given x and y cordinates\n"
              << std::endl;

    while (ros::ok())
    {
        std::stringstream ss;
        std::getline(std::cin, cmd);
        ss << cmd;
        if (!(ss >> cmd))
        {
            std::cout << "Invalid command, try again!" << std::endl;
        }
        else if (cmd == "quit")
        {
            goto ending;
        }
        else if (cmd == "turnr")
        {
            msg.command = cmd;
            if (!(ss >> msg.val1))
                std::cout << "Invalid argument with turnr, try again!" << std::endl;
            else
            {
                order_pub.publish(msg);
            }
        }
        else if (cmd == "turnd")
        {
            msg.command = cmd;
            if (!(ss >> msg.val1))
                std::cout << "Invalid argument with turnd, try again!" << std::endl;
            else
            {
                order_pub.publish(msg);
            }
        }
        else if (cmd == "turntor")
        {
            msg.command = cmd;
            if (!(ss >> msg.val1))
                std::cout << "Invalid argument with turntor, try again!" << std::endl;
            else
            {
                order_pub.publish(msg);
            }
        }
        else if (cmd == "turntod")
        {
            msg.command = cmd;
            if (!(ss >> msg.val1))
                std::cout << "Invalid argument with turntod, try again!" << std::endl;
            else
            {
                order_pub.publish(msg);
            }
        }
        else if (cmd == "forward")
        {
            msg.command = cmd;
            if (!(ss >> msg.val1))
                std::cout << "Invalid argument with forward, try again!" << std::endl;
            else
                order_pub.publish(msg);
        }
        else if (cmd == "square")
        {
            msg.command = cmd;
            if (!(ss >> msg.val1 >> msg.cclk))
                std::cout << "Invalid arguments with turnd, try again!" << std::endl;
            else
            {
                order_pub.publish(msg);
            }
        }
        else if (cmd == "circle")
        {
            msg.command = cmd;
            if (!(ss >> msg.val1 >> msg.cclk))
                std::cout << "Invalid arguments with turnd, try again!" << std::endl;
            else
            {
                order_pub.publish(msg);
            }
        }
        else if (cmd == "backward")
        {
            msg.command = cmd;
            if (!(ss >> msg.val1))
                std::cout << "Invalid argument with backward, try again!" << std::endl;
            else
                order_pub.publish(msg);
        }
        else if (cmd == "moveto")
        {
            msg.command = cmd;
            if (!(ss >> msg.val1 >> msg.val2))
                std::cout << "Invalid argument(s) with moveto, try again!" << std::endl;
            else
                order_pub.publish(msg);
        }
        else
        {
            std::cout << "Invalid command, try again!" << std::endl;
        }
    }
ending:
    return 0;
}
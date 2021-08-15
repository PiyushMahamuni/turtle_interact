// Including req libraries
#include "ros/ros.h"
#include "turtlesim/Pose.h"        // msg type
#include "geometry_msgs/Twist.h"   // msg type
#include "turtle_interact/Order.h" // msg type
#include <sstream>
#include <queue>

// Constants -
const char *node_name{"turtle_draw"};
const char *vel_pub_topic{"turtle1/cmd_vel"};
const char *pose_sub_topic{"turtle1/pose"};
const char *order_sub_topic{"turtle_draw/order_queue"};
const _Float32 pi{3.14159265358979};
const _Float32 pi_2{2 * pi};
const _Float32 pi_by_2{pi / 2};
const _Float32 pi_by_4{pi_by_2 / 2};
const _Float32 linear_sp{0.6};
const _Float32 angular_sp{pi_2 / 10};
const _Float32 xylim1{0.035}, xylim2{10 - xylim1}; // limits
const _Float32 ltd{0.05};                          // linear threshold distance
const ros::Duration *blink;                        // blinking time, to keep cpu from overclocking, need to be setup after starting node in main
#define BLINKDUR 1.0 / 2000.0

// Globals -
ros::Subscriber pose_sub;                       // subscriber of turtle1/pose
ros::Subscriber order_sub;                      // subscriber of turtle_draw/order
ros::Publisher vel_pub;                         // publisher of turtle1/cmd_vel
geometry_msgs::Twist msg, stop_msg;             // initialise to correct values in main
turtlesim::Pose tpos, cpos;                     // target position and current position
ros::Timer vel_pub_timer;                       // timer to periodically call vel_pub.publish();
std::queue<turtle_interact::Order> order_queue; // queue of received orders
ros::Timer check_orders_timer;                  // timer to periodically check pending orders

// callback functions for turtlesim/pose topic
// takes theta as is. i.e. in range -pi to pi
void use_theta_range1(const turtlesim::Pose::ConstPtr &msg)
{
    cpos.x = msg->x;
    cpos.y = msg->y;
    cpos.theta = msg->theta;
    return;
}
// converts theta in range from 0 to 2 pi
void use_theta_range2(const turtlesim::Pose::ConstPtr &msg)
{
    cpos.x = msg->x;
    cpos.y = msg->y;
    cpos.theta = (msg->theta < 0) ? pi_2 + msg->theta : msg->theta;
    return;
}
// global pointer to select callback funtion for tutlesim/pose topic. set to use_theta_range1
void (*select_range)(const turtlesim::Pose::ConstPtr &){use_theta_range1};
// callback driver function for "turtle1/pose" topic
void update_pose(const turtlesim::Pose::ConstPtr &msg)
{
    select_range(msg); // calling selected callback function
    return;
}

// checker function to avoid running into wall
bool is_crashing()
{
    return tpos.x > xylim2 || tpos.x < xylim1 || tpos.y > xylim2 || tpos.y < xylim1;
}

// function attached with pub_vel_timer which periodically publishes velocity on
// tutle1/cmd_vel topic
void publish_vel(const ros::TimerEvent &event)
{
    vel_pub.publish(msg);
    return;
}

// ------------------ FUNCTIONS CONTROLLING MOTIONS ----------------------

// **** NOTE: *****
// MAKE SURE THAT ALL THE ACTUAL TURNING OPERATION IS DELEGATED TO turnr() function
// DO NOT ATTEMPT TO WRITE ANOTHER FUNCTION TO TURN ROBOT ON ITS OWN, FIND RADIAN ANGLES
// THROUGH WHICH YOU WANT TO TURN ROBOT AND THEN DELEGATE THE TASK TO turnr()
// ****************

// 1. STOP THE ROBOT AND ALSO UPDATE cpos
inline void stop_robot()
{
    msg = stop_msg;
    for (int i{}; i < 4; i++)
    {
        vel_pub.publish(msg);
        ros::spinOnce();
    }
    return;
}
// 2. TURNS BOT THROUGH GIVEN FUNCTION
// *** NOTE ***
// DON'T MAKE CALL TO THIS FUNCTION, CALL turnr
// CAN'T WORK WITH VALUES GREATER THAN 2PI OR LESS THAN -2PI
// The default paramter slwd tells whether to or not to slow down in the last moments
// of appraoching target angle value
inline void turn(const _Float32 &angle, bool slwd = true)
{
    static _Float32 ltt{cpos.theta}; // last theta target, used in turnr()
                                     //to avoid accumulation of error

    // to slow down before 4.7 degrees from reaching target theta value
    static const _Float32 diff{4.7 * pi_by_4 / 45};

    // calculate target theta
    tpos.theta = ltt + angle; // use ltt instead of cpos.theta to avoid error accumulation
    while (tpos.theta > pi)
        tpos.theta -= pi_2;
    while (tpos.theta < -pi)
        tpos.theta += pi_2;
    ltt = tpos.theta;
    // Now tpos.theta is in the range from -pi to pi

    // if tpos.theta is near pi or -pi, it will cause lot of problems since you suddenly jump to
    // one end of range from another, it's much more easier to change the range to 0 to 2 pi while
    // turning to target angle near pi radians and move the troublesome jumping point to 0 radians
    if (tpos.theta > 3 * pi_by_4 || tpos.theta < -3 * pi_by_4)
    {
        // we need to change range to 0 to 2 pi range
        select_range = use_theta_range2;
        // change tpos.theta and cpos.theta to this new range
        if (tpos.theta < 0)
            tpos.theta += pi_2;
        if (cpos.theta < 0)
            cpos.theta += pi_2;
    }

    // start moving robot
    msg.angular.z = (angle > 0) ? angular_sp : -angular_sp;
    if (angle > 0)
    {
        // robot is turning counter clockwise
        // robot should stop when it has JUST achieved angle greater than tpos.theta
        // it could be already greater in that case, keep rotating until cpos.theta reaches to
        // smaller end of range of angles

        // setting intermediate target 5.5 degrees less than atcual tpos.theta
        if (abs(tpos.theta - cpos.theta) < diff)
            goto last_part1;
        else
            tpos.theta -= diff;
        while (cpos.theta > tpos.theta)
        {
            ros::spinOnce();
            blink->sleep();
        }
        while (cpos.theta < tpos.theta)
        {
            ros::spinOnce();
            blink->sleep();
        }
        tpos.theta += diff;
    // slow down robot for more precision
    last_part1:
        if (slwd)
        {
            msg.angular.z /= 15;
            vel_pub.publish(msg);
        }
        while (cpos.theta < tpos.theta)
        {
            ros::spinOnce();
            // without sleep for accuraccy
        }
    }
    else
    {
        // turning clockwise
        // robot will have to stop when it's angle has JUST been made smaller than target theta val
        // it could already smaller than target to begin with

        // setting intermediate target before 5.5 degrees of achieiving target
        if (abs(tpos.theta - cpos.theta) < diff)
            goto last_part2;
        else
            tpos.theta += diff;
        while (cpos.theta < tpos.theta)
        {
            ros::spinOnce();
            blink->sleep();
        }
        while (cpos.theta > tpos.theta)
        {
            ros::spinOnce();
            blink->sleep();
        }
        tpos.theta -= diff;
    // slow down
    last_part2:
        if (slwd)
        {
            msg.angular.z /= 15;
            vel_pub.publish(msg);
        }
        while (cpos.theta > tpos.theta)
        {
            ros::spinOnce();
        }
    }
    // change back callback function of pose topic if ever changed
    select_range = use_theta_range1;
    if (slwd)
        stop_robot();
    return;
}
// 3. TURNING THROUGH GIVEN RADIANS - YOU MAKE CALL TO THIS FUNCTINO
inline void turnr(_Float32 angle, bool log = false)
{
    stop_robot();

    // calculate target
    tpos.theta = cpos.theta + angle;
    while (tpos.theta > pi)
        tpos.theta -= pi_2;
    while (tpos.theta < -pi)
        tpos.theta += pi_2;

    if (log)
    {
        ROS_INFO("[%s] Command recieved to make the robot move through %f radians",
                 node_name, angle);
        std::cout << "Current Theta: " << cpos.theta << " Target Theta: " << tpos.theta << std::endl;
        std::cout << "Enter any key to continue Enter `abort` to abort current operation: ";
        std::string cmd;
        std::getline(std::cin, cmd);
        if (cmd == "abort")
        {
            ROS_INFO("[%s] Order aborted by user.\n", node_name);
            return;
        }
    }
    while (angle > pi_2)
    {
        angle -= pi_2;
        for (int i{}; i < 4; i++)
        {
            turn(pi_by_2, false);
        }
    }
    while (angle < -pi_2)
    {
        angle += pi_2;
        for (int i{}; i < 4; i++)
        {
            turn(-pi_by_2, false);
        }
    }
    turn(angle);
    if (log)
        ROS_INFO("[%s] Targeth theta of %f achieved!\n", node_name, cpos.theta);
    return;
}
// 4. TURNING ROBOT THROUGH GIVEN DEGREES
inline void turnd(const _Float32 &angle, bool log = false)
{
    turnr(angle * pi_by_4 / 45, log);
    return;
}
// 5. TURN ROBOT TO HAVE SPECIFIC THETA VALUE IN RADIANS
inline void turntor(const _Float32 &angle, bool log = false)
{
    stop_robot();
    // calculate target theta
    tpos.theta = angle;
    while (tpos.theta > pi)
        tpos.theta -= pi_2;
    while (tpos.theta < -pi)
        tpos.theta += pi_2;
    _Float32 temp{tpos.theta - cpos.theta};
    while (temp > pi)
        temp -= pi_2;
    while (temp < -pi)
        temp += pi_2;

    // print log messages if required
    if (log)
    {
        ROS_INFO("[%s] command recieved to turn robot to %f radians", node_name, tpos.theta);
        std::cout << "Current theta: " << cpos.theta << " Target theta: " << tpos.theta << std::endl;
        std::cout << "Enter any key to continue Enter `abort` to abort current operation: ";
        std::string cmd;
        std::getline(std::cin, cmd);
        if (cmd == "abort")
        {
            ROS_INFO("[%s] Order aborted by user.\n", node_name);
            return;
        }
    }
    turn(temp);
    if (log)
        ROS_INFO("[%s] Target theta of %f achieved!\n", node_name, cpos.theta);
    return;
}
// 6. TURN ROBOT TO HAVE SPECIFIC THETA VALUE IN DEGREES
inline void turntod(const _Float32 &angle, bool log = false)
{
    turntor(angle * pi_by_4 / 45, log);
    return;
}
// 7. MOVE THE ROBOT FORWARD BY GIVEN UNITS
inline bool forward(const _Float32 &len, bool log = false)
{
    stop_robot();
    static const _Float32 diff{0.25};
    // to slow down before 0.2 units of reaching target for precision

    // Calculate target values
    tpos.x = cpos.x + cos(cpos.theta) * len;
    tpos.y = cpos.y + sin(cpos.theta) * len;

    if (is_crashing())
    {
        ROS_INFO("[%s] ABORTING TRAVERSAL SINCE ROBOT WILL CRASH!\n", node_name);
        return false;
    }

    if (log)
    {
        ROS_INFO("[%s] Recieved command to make the robot move forward by %f units", node_name, len);
        std::cout << "Currnet x = " << cpos.x << ", y = " << cpos.y
                  << "\nTarget x = " << tpos.x << ", y = " << tpos.y;
        std::cout << "Enter any key to continue Enter `abort` to abort current operation: ";
        std::string cmd;
        std::getline(std::cin, cmd);
        if (cmd == "abort")
        {
            ROS_INFO("[%s] Order aborted by user.\n", node_name);
            return false;
        }
    }

    // start moving forward
    msg.linear.x = len > 0 ? linear_sp : -linear_sp;

    // Decide whether to use x or y cordinate to stop the robot
    if (abs(tpos.x - cpos.x) > abs(tpos.y - cpos.y))
    {
        // use x cord
        if (tpos.x > cpos.x)
        {
            // intermediate target
            if (tpos.x - cpos.x > diff)
                tpos.x -= diff;
            else
                goto last_part1;
            while (tpos.x > cpos.x)
            {
                ros::spinOnce();
                blink->sleep();
            }
        last_part1:
            // slow down
            msg.linear.x /= 4;
            vel_pub.publish(msg);
            tpos.x += diff;
            while (tpos.x > cpos.x)
            {
                ros::spinOnce();
            }
        }
        else
        {
            // intermediate target
            if (cpos.x - tpos.x > diff)
                tpos.x += diff;
            else
                goto last_part2;
            while (tpos.x < cpos.x)
            {
                ros::spinOnce();
                blink->sleep();
            }
        last_part2:
            // slow down
            msg.linear.x /= 4;
            vel_pub.publish(msg);
            tpos.x -= diff;
            while (tpos.x < cpos.x)
            {
                ros::spinOnce();
            }
        }
    }
    else
    {
        // use y cord
        if (tpos.y > cpos.y)
        {
            // intermediate target
            if (tpos.y - cpos.y > diff)
                tpos.y -= diff;
            else
                goto last_part3;
            while (tpos.y > cpos.y)
            {
                ros::spinOnce();
                blink->sleep();
            }
        last_part3:
            // slow down
            msg.linear.x /= 4;
            vel_pub.publish(msg);
            tpos.y += diff;
            while (tpos.y > cpos.y)
            {
                ros::spinOnce();
            }
        }
        else
        {
            // intermediate target
            if (cpos.y - tpos.y > diff)
                tpos.y += diff;
            else
                goto last_part4;
            while (tpos.y < cpos.y)
            {
                ros::spinOnce();
                blink->sleep();
            }
        last_part4:
            // slow down
            msg.linear.x /= 4;
            vel_pub.publish(msg);
            tpos.y -= diff;
            while (tpos.y < cpos.y)
            {
                ros::spinOnce();
            }
        }
    }

    stop_robot();

    if (log)
        ROS_INFO("[%s] Target location achieved! x = %f, y = %f\n", node_name, cpos.x, cpos.y);
    return true;
}
// 8. DRAW SQUARE WITH ROBOT
inline bool square(const _Float32 &side, bool cclk = true, bool log = false)
{
    stop_robot();

    // log messages if req
    if (log)
    {
        ROS_INFO("[%s] Command received to make the robot trace square with %f side in %s dir",
                 node_name, side, (cclk) ? "cclk" : "clk");
        std::cout << "Enter any key to continue Enter `abort` to abort current operation: ";
        std::string cmd;
        std::getline(std::cin, cmd);
        if (cmd == "abort")
        {
            ROS_INFO("[%s] Order aborted by user.\n", node_name);
            return false;
        }
    }
    _Float32 angle{(cclk) ? pi_by_2 : -pi_by_2};
    for (int i{}; i < 3; i++)
    {
        if (!forward(side))
            return false;
        turnr(angle);
    }
    if (!forward(side))
        return false;
    if (log)
        ROS_INFO("[%s] Tracing squre compelted!\n", node_name);
    return true;
}
// 9. DRAW CIRCLE WITH ROBOT
inline bool circle(const _Float32 &radius, bool cclk = true, bool log = false)
{
    stop_robot();

    // check if robot will crash
    // find centre of circle and then add/subtract radius to find upper and lower bounds of circle
    tpos.theta = cpos.theta + cclk ? pi_by_2 : -pi_by_2;
    tpos.x = cpos.x + cos(tpos.theta) * radius + radius;
    tpos.y = cpos.y + sin(tpos.theta) * radius + radius;
    bool abrt{is_crashing()};
    // lower bounds
    tpos.x -= 2 * radius;
    tpos.y -= 2 * radius;
    abrt = abrt || is_crashing();
    if (abrt)
    {
        ROS_INFO("[%s] Can't perform requested operation, robot will run into wall!", node_name);
        return false;
    }

    // calculate intermediate target
    if (cpos.theta > 3 * pi_by_4 || cpos.theta < -3 * pi_by_4)
    {
        select_range = use_theta_range2;
        if (cpos.theta < 0)
            cpos.theta += pi_2;
        ros::spinOnce();
    }
    _Float32 itt{cpos.theta + ((cclk) ? pi_2 - static_cast<_Float32>(5.5) * pi_by_4 / static_cast<_Float32>(45) : static_cast<_Float32>(5.5) * pi_by_4 / static_cast<_Float32>(45) - pi_2)};
    if (select_range == use_theta_range1)
    {
        while (itt > pi)
            itt -= pi_2;
        while (itt < -pi)
            itt += pi_2;
    }
    else
    {
        while (itt > pi_2)
            itt -= pi_2;
        while (itt < 0)
            itt += pi_2;
    }

    tpos = cpos; // target Pose is same and current Pose

    if (log)
    {
        ROS_INFO("[%s] Command recieved to make the robot trace circle with %f radius in %s dir",
                 node_name, radius, cclk ? "cclk" : "clk");
        std::cout << "Enter any key to continue Enter `abort` to abort current operation: ";
        std::string cmd;
        std::getline(std::cin, cmd);
        if (cmd == "abort")
        {
            ROS_INFO("[%s] Order aborted by user.\n", node_name);
            return false;
        }
    }
    // start the circular motion, v = r * w
    if (radius < 1) // maintain upper bound on angular velocity
        msg.linear.x = abs((msg.angular.z = cclk ? angular_sp : -angular_sp)) * radius;
    else // maintain upper bound on linear velocity
        msg.angular.z = (msg.linear.x = linear_sp) / ((cclk) ? radius : -radius);

    if (cclk)
    {
        // itt is greater
        while (cpos.theta > itt)
        {
            ros::spinOnce();
            blink->sleep();
        }
        while (cpos.theta < itt)
        {
            ros::spinOnce();
            blink->sleep();
        }
        msg.angular.z /= 3;
        msg.linear.x /= 3;
        vel_pub.publish(msg);
        while (cpos.theta < tpos.theta)
        {
            ros::spinOnce();
        }
    }
    else
    {
        // itt is smaller
        while (cpos.theta < itt)
        {
            ros::spinOnce();
            blink->sleep();
        }
        while (cpos.theta > itt)
        {
            ros::spinOnce();
            blink->sleep();
        }
        msg.angular.z /= 3;
        msg.linear.x /= 3;
        vel_pub.publish(msg);
        while (cpos.theta > tpos.theta)
        {
            ros::spinOnce();
        }
    }
    // for higher precision, check x and y cordinates too
    if (abs(tpos.x - cpos.x) > ltd)
    {
        if (tpos.x > cpos.x)
        {
            while (tpos.x > cpos.x)
            {
                ros::spinOnce();
            }
        }
        else
        {
            while (tpos.x < cpos.x)
            {
                ros::spinOnce();
            }
        }
    }
    if (abs(tpos.y - cpos.y) > ltd)
    {
        if (tpos.y > cpos.y)
        {
            while (tpos.y > cpos.y)
            {
                ros::spinOnce();
            }
        }
        else
        {
            while (tpos.y < cpos.y)
            {
                ros::spinOnce();
            }
        }
    }

    stop_robot();

    if (log)
        ROS_INFO("[%s] Tracing circle completed!\n", node_name);

    return true;
}
// 10. MOVES ROBOT BACKWARDS BY GIVEN AMOUNT
inline bool backward(const _Float32 &len, bool log = false)
{
    return forward(-len, log);
}
// 11. MOVE ROBOT TO TARGET LOCATION IN STRAIGHT LINE
inline bool moveto(const _Float32 &x, const _Float32 &y, bool log = false)
{
    ros::spinOnce(); // update cpos
    // check if robot will run into wall
    tpos.x = x;
    tpos.y = y;
    if (log)
        ROS_INFO("[%s] Command received to move to target x = %f, y = %f", node_name, x, y);
    if (is_crashing())
    {
        ROS_INFO("[%s] ABORTING, robot will crash!\n", node_name);
        return false;
    }
    // calculate required theta and turn to it
    tpos.x -= cpos.x;
    tpos.y -= cpos.y;
    _Float32 len{sqrt(tpos.x * tpos.x + tpos.y * tpos.y)};
    _Float32 angle{static_cast<_Float32>(asinf32(tpos.y / len))};
    _Float32 diff{static_cast<_Float32>(5.5) * pi_by_4 / static_cast<_Float32>(45)};
    if (log)
    {
        std::cout << "Differences:\n"
                  << "cpos.x = " << cpos.x << " tpos.x = " << x << " dx = " << tpos.x << "\n"
                  << "cpos.y = " << cpos.y << " tpos.y = " << y << " dy = " << tpos.y << std::endl
                  << "Linear dist to target: " << len << std::endl
                  << "Enter any key to continue or abort to abort current order: ";
        std::string cmd;
        std::getline(std::cin, cmd);
        if (cmd == "abort")
        {
            ROS_INFO("[%s] Order aborted by user!\n", node_name);
            return false;
        }
    }

    // same value of sin for two angles, asinf returns angle from 1st and 4th quad
    if (x < cpos.x)
    {
        if (angle > 0)
            angle = pi - angle;
        else
            angle = -pi - angle;
    }
    std::cout << "Approach angle: " << angle << std::endl;
    // turn
    // slow down for precision
    // if(abs(cpos.theta - angle) > diff){
    //     // traverse the first part at normal speed
    //     if(angle > cpos.theta)
    //     turn(angle - cpos.theta - diff, false);
    //     else
    //     turn(angle - cpos.theta + diff, false);
    // }
    // angular_sp /= 3;
    // turntor(angle);
    // angular_sp *= 3;

    turntor(angle);
    // move
    forward(len);

    if (log)
        ROS_INFO("[%s] Moved to location, x = %f, y = %f\n", node_name, cpos.x, cpos.y);

    // if it is first call, i.e not a recursive call

    return true;
}
// -----------------------------------------------------------------------

//  callback function for turtle_draw/order_queue topic
void order_callback(const turtle_interact::Order::ConstPtr &msg)
{
    turtle_interact::Order new_order;
    new_order.command = msg->command;
    new_order.val1 = msg->val1;
    new_order.val2 = msg->val2;
    new_order.cclk = msg->cclk;
    order_queue.push(new_order);
    ROS_INFO("[%s] Command received on turtle_draw/order_queue topic. Pushing onto order queue", node_name);
}

// to check pending orders periodically
void check_orders(const ros::TimerEvent &event)
{
    static turtle_interact::Order new_order;
    if (order_queue.size())
    {
        new_order = order_queue.front();
        if (new_order.command == "turnr")
        {
            turnr(new_order.val1);
        }
        else if (new_order.command == "turnd")
        {
            turnd(new_order.val1);
        }
        else if (new_order.command == "turntor")
        {
            turntor(new_order.val1);
        }
        else if (new_order.command == "turntod")
        {
            turntod(new_order.val1);
        }
        else if (new_order.command == "forward")
        {
            forward(new_order.val1);
        }
        else if (new_order.command == "backward")
        {
            backward(new_order.val1);
        }
        else if (new_order.command == "square")
        {
            square(new_order.val1, new_order.cclk);
        }
        else if (new_order.command == "circle")
        {
            circle(new_order.val1, new_order.cclk);
        }
        else if (new_order.command == "moveto")
        {
            moveto(new_order.val1, new_order.val2);
        }
        else
        {
            ROS_INFO("[%s] Invalid command in order queue, moving to next one!", node_name);
        }
        order_queue.pop();
    }
}

// main program driver
int main(int argc, char **argv)
{
    // initialize ros and setting up globals
    ros::init(argc, argv, node_name);
    ros::NodeHandle node;
    msg.linear.x = msg.linear.y = msg.linear.z = msg.angular.x = msg.angular.y = msg.angular.z = stop_msg.linear.x = stop_msg.linear.y = stop_msg.linear.z = stop_msg.angular.x = stop_msg.angular.y = stop_msg.angular.z = 0;
    blink = new ros::Duration(BLINKDUR);

    // This node is publisher of /turtle1/cmd_vel topic which uses Twist msg type
    vel_pub = node.advertise<geometry_msgs::Twist>(vel_pub_topic, 10);

    // This node is subscriber of /turtle1/pose topic which uses turltesim/pose msg type
    pose_sub = node.subscribe(pose_sub_topic, 1, update_pose);

    // This node is subscriber of /turtle_draw/order_queue topic which uses turtle_interact/Order type
    order_sub = node.subscribe(order_sub_topic, 100, order_callback);

    // setting up timer for vel_pub
    vel_pub_timer = node.createTimer(ros::Duration(0.3), publish_vel);

    // setting up timer for periodically checking order queue
    check_orders_timer = node.createTimer(ros::Duration(0.02), check_orders);
    // Declaring variables used in menu

    std::string choice, cmd;
    _Float32 val;

    // Wait for a publisher of turtle1/pose, without which we can't operate
    std::cout << "Waiting for a publisher of turtle1/pose..." << std::endl;
    while (!pose_sub.getNumPublishers())
    {
        if (!ros::ok())
        {
            goto ending;
        }
    }

    std::cout << "Will you use in manual mode ? ";
    std::cin >> choice;

    if (choice != "no" || choice != "n")
    {
        ros::spin();
        goto ending;
    }

    // MENU ------
    std::cout << "--- MENU ---\n"
              << "quit: quit this program\n"
              << "turnr <angle in radians>: turn robot through given radians\n"
              << "turnd <angle in degrees>: turn robot through given degress\n"
              << "turntor <angle in radians>: turn robot to specific tehta value in radians\n"
              << "turntod <angle in degrees>: turn robot to specific theta value in degrees\n"
              << "forward <lenght>: to move robot forward through given amount\n"
              << "backward <length>: to move robot backwards through given amount\n"
              << "square <side> <clk/cclk>\n"
              << "circle <radius> <clk/cclk>\n"
              << "moveto <x> <y>: move to given x and y cordinates\n"
              << std::endl;
    while (choice != "quit" && ros::ok())
    {
        std::stringstream ss;
        std::getline(std::cin, choice);
        ss << choice;
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
            if (!(ss >> val))
                std::cout << "Invalid argument with turnr, try again!" << std::endl;
            else
                turnr(val, true);
        }
        else if (cmd == "turnd")
        {
            if (!(ss >> val))
                std::cout << "Invalid argument with turnd, try again!" << std::endl;
            else
                turnd(val, true);
        }
        else if (cmd == "turntor")
        {
            if (!(ss >> val))
                std::cout << "Invalid argument with turntor, try again!" << std::endl;
            else
                turntor(val, true);
        }
        else if (cmd == "turntod")
        {
            if (!(ss >> val))
                std::cout << "Invalid argument with turntod, try again!" << std::endl;
            else
                turntod(val, true);
        }
        else if (cmd == "forward")
        {
            if (!(ss >> val))
                std::cout << "Invalid argument with forward, try again!" << std::endl;
            else
                forward(val, true);
        }
        else if (cmd == "square")
        {
            if (!(ss >> val))
                std::cout << "Invalid 1st argument with square, try again" << std::endl;
            else if (!(ss >> cmd))
                std::cout << "Invalid 2nd argument with square, try again" << std::endl;
            else if (cmd != "clk" && cmd != "cclk")
                std::cout << "Invalid 2nd argument with square, try again" << std::endl;
            else
                square(val, cmd == "cclk", true);
        }
        else if (cmd == "circle")
        {
            if (!(ss >> val))
                std::cout << "Invalid 1st argument with circle, try again" << std::endl;
            else if (!(ss >> cmd))
                std::cout << "Invalid 2nd argument with circle, try again" << std::endl;
            else if (cmd != "clk" && cmd != "cclk")
                std::cout << "Invalid 2nd argument with circle, try again" << std::endl;
            else
                circle(val, cmd == "cclk", true);
        }
        else if (cmd == "backward")
        {
            if (!(ss >> val))
                std::cout << "Invalid argument with backward, try again!" << std::endl;
            else
                backward(val);
        }
        else if (cmd == "moveto")
        {
            _Float32 val2;
            if (!(ss >> val >> val2))
                std::cout << "Invalid argument(s) with moveto, try again!" << std::endl;
            else
                moveto(val, val2, true);
        }
        else
        {
            std::cout << "Invalid command, try again!" << std::endl;
        }
    }
ending:
    std::cout << "Terminating..." << std::endl;
    delete blink;
    vel_pub_timer.stop();
    check_orders_timer.stop();
    return 0;
}

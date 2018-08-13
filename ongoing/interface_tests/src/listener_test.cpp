#include "ros/ros.h"

#include <interface_tests/StateSpaceT.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void objectiveCallback(const interface_tests::StateSpaceT& msg)
{
  std::ostringstream s;
  s << "Current Objective : " << std::endl << " Name : "<< msg.name << std::endl;
  //s << "  Dimension :" << (int) msg.dimension << std::endl;

  std::vector<uint8_t> discrete_data = msg.discrete_data;
  std::vector<float> real_data = msg.real_data;

  switch(msg.state_type)
  {
    case(interface_tests::StateSpaceT::STATE_BOOL):
      if(discrete_data.size() !=1)
         ROS_WARN("Receiving STATE_BOOL objective with invalid dimension");
      else
      {
        s << "  Switch State !" << std::endl;
      }
      break;

    case(interface_tests::StateSpaceT::STATE_2D):
      if(real_data.size() != 3)
        ROS_WARN("Receiving STATE_2D objective with invalid dimension");
      else
      {
        s << "  Position : " << std::endl;
        s << "    x : "<< real_data[0] << std::endl;
        s << "    y : "<< real_data[1] << std::endl;
        s << "  Orientation : " << std::endl;
        s << "    theta : "<< real_data[2] << std::endl;
      }
      break;

    case(interface_tests::StateSpaceT::STATE_3D):
      if(real_data.size() != 7)
        ROS_WARN("Receiving STATE_3D objective with invalid dimension");
      else
      {
        s << "  Position : " << std::endl;
        s << "    x : "<< real_data[0] << std::endl;
        s << "    y : "<< real_data[1] << std::endl;
        s << "    z : "<< real_data[2] << std::endl;
        s << "  Orientation : " << std::endl;
        s << "    w : "<< real_data[3] << std::endl;
        s << "    x : "<< real_data[4] << std::endl;
        s << "    y : "<< real_data[5] << std::endl;
        s << "    z : "<< real_data[6] << std::endl;
      }
      break;

    case(interface_tests::StateSpaceT::ORDINARY):
      s << "  Discrete data : "<< discrete_data.size() << std::endl;
      for(unsigned int i =0; i<discrete_data.size();i++)
      {
         s << "    "<< discrete_data[i]<<std::endl;
      }
      s << "  Real data : "<< real_data.size() << std::endl;
      for(unsigned int i =0; i<real_data.size();i++)
      {
         s << "    "<< real_data[i]<<std::endl;
      }
      break;
  }

  ROS_INFO_STREAM(s.str());
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("state_objective", 1000, objectiveCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
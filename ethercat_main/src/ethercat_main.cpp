#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt_rosclock/rtt_rosclock.h>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>

extern "C" {
  #include "ethercat_igh/ethercat_igh.h"
}

using namespace RTT;

class EthercatIGH : public RTT::TaskContext{

  private:
    const static int enc_count = 5;
    int curr_pos = 0;

    // Necessary components to run thread for serving ROS callbacks
    boost::thread non_rt_ros_queue_thread_;
    boost::shared_ptr<ros::NodeHandle> non_rt_ros_nh_;
    ros::CallbackQueue non_rt_ros_queue_;

  public:
    EthercatIGH(const std::string& name):
      TaskContext(name)
    {}

    ~EthercatIGH()
    {}

  private:
    bool configureHook()
    {
      return igh_configure(); 
    }

    bool startHook()
    {
      return igh_start(); 
    }

    void updateHook()
    {
      curr_pos = igh_update(enc_count);
      log(Info) << "EthercatIGH Update ! curr_pos = " << curr_pos << endlog();
    }

    void stopHook()
    {
      igh_stop();
    }

    void cleanupHook()
    {
      igh_cleanup();
      //non_rt_ros_nh_->shutdown();
      //non_rt_ros_queue_thread_.join();
    }

    void serviceNonRtRosQueue()
    {
      static const double timeout = 0.001;

      while (this->non_rt_ros_nh_->ok()){
        this->non_rt_ros_queue_.callAvailable(ros::WallDuration(timeout));
      }
    }
};
ORO_CREATE_COMPONENT(EthercatIGH)

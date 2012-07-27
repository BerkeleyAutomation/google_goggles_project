#include "tracker_ros.h"
#include "misc_srvs/SetSwitch.h"
#include <ros/ros.h>


struct TrackerServer {
  TabletopTrackerROS tracker;
  bool state;

  bool callback(misc_srvs::SetSwitch::Request& request, misc_srvs::SetSwitch::Response& response) {
    if (!request.state && state) tracker.reset();
    state = request.state;
    response.success = true;
    return true;
  }
  

  TrackerServer(ros::NodeHandle nh) : 
    tracker(nh),
    state(false)
  {
  }

  void run() {
    while (ros::ok()) {
      ros::spinOnce();
      if (!state) {
	ROS_DEBUG("tracker server is sleeping");
	ros::Duration(.1).sleep();
	continue;
      }

      for (int i=0; i < 10; i++) ros::spinOnce();
      if (tracker.hasPendingMessage) {
	ROS_INFO("tracker has pending message. updating");
	try {
	  tracker.updateAll();
	  tracker.publish();
	  tracker.hasPendingMessage = false;
	}
	catch (std::runtime_error err) {
	  ROS_ERROR_STREAM("error while updating tracker: " <<err.what());
	  ros::Duration(.1).sleep();
	}
      }
      else {
	ros::Duration(0.01).sleep();
      }
    
    }

  }

};



int main(int argc, char* argv[]) {
  ros::init(argc, argv, "spinning_table_tracker_server");
  ros::NodeHandle nh;

  TrackerServer server(nh);
  ros::ServiceServer service = nh.advertiseService("spinning_tabletop/set_state", &TrackerServer::callback, &server);

  server.run();

}

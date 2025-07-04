#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/String.h>     // 红绿灯
#include <std_msgs/Bool.h>       // 急刹危险 & emergency_stop

class DecisionNode
{
public:
  DecisionNode()
  {
    /* 订阅感知层话题 */
    tl_sub_ = nh_.subscribe("/perception/traffic_light_status", 1,
                            &DecisionNode::tlCb, this);
    fhz_sub_ = nh_.subscribe("/perception/front_hazard", 1,
                             &DecisionNode::fhzCb, this);

    /* 发布给控制层 */
    estop_pub_ = nh_.advertise<std_msgs::Bool>("/decision/emergency_stop", 1);
  }

  void spin()
  {
    ros::Rate r(20);                             // 20 Hz
    while (ros::ok())
    {
      std_msgs::Bool msg;
      msg.data = (light_red_ || front_hazard_);  // true → 控制层停车
      estop_pub_.publish(msg);

      ros::spinOnce();
      r.sleep();
    }
  }

private:
  /* 红绿灯回调 */
  void tlCb(const std_msgs::String::ConstPtr& s)
  {
    std::string v = s->data;
    std::transform(v.begin(), v.end(), v.begin(), ::tolower);
    light_red_ = (v == "red");
  }
  /* 急刹危险回调 */
  void fhzCb(const std_msgs::Bool::ConstPtr& b)
  { front_hazard_ = b->data; }

  ros::NodeHandle nh_;
  ros::Subscriber tl_sub_, fhz_sub_;
  ros::Publisher  estop_pub_;

  bool light_red_{false};
  bool front_hazard_{false};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "decision_node");
  DecisionNode node;
  node.spin();
  return 0;
}

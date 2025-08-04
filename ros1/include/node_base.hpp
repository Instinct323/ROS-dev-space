#include <ros/ros.h>

namespace ros {

/*
 * @brief Emulates the functionality of rclcpp::Node in ROS 2.
 *
 * Example usage:
 * @code
 * int main(int argc, char **argv) {
 *    ros::init(argc, argv, "demo");
 *    ros::Node node;
 *    ros::spin();
 *    return 0;
 * }
 * @endcode
 */
class NodeBase : public ros::NodeHandle {

public:
    NodeBase() : ros::NodeHandle() {}

    template<typename MsgT>
    ros::Publisher create_publisher(const std::string &topic, uint32_t queue_size) {
      return advertise<MsgT>(topic, queue_size);
    }

    template<typename MsgT, typename CallbackT>
    ros::Subscriber create_subscribtion(const std::string &topic, uint32_t queue_size, CallbackT callback) {
      return subscribe(topic, queue_size, callback);
    }

    template<typename SrvT>
    ros::ServiceClient create_client(const std::string &service) {
      return serviceClient<SrvT>(service);
    }

    template<typename SrvT, typename CallbackT>
    ros::ServiceServer create_server(const std::string &service, CallbackT callback) {
      return advertiseService(service, callback);
    }
};

}

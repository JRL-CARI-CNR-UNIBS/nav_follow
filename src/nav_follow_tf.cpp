#include <nav_follow/nav_follow.h>


using namespace boost::assign;
using namespace boost::adaptors;
using namespace pcl::registration;  
using namespace std::chrono_literals;


inline bool exists_file (const std::string& name) {
    return ( access( name.c_str(), F_OK ) != -1 );
}


void nav_follow_class::current_tf_thread()
{
    while(rclcpp::ok())
    {
        try {
            std::scoped_lock<std::mutex> guard_tf(_tf_mutex);
            _tf_master_slave = _tf_buffer->lookupTransform(
            _params.frame_name_base_master, _params.frame_name_base_slave,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            _params.frame_name_base_master.c_str(), _params.frame_name_base_slave.c_str(), ex.what());
        }
        // std::this_thread::sleep_for(50ms);
        _rate_tf->sleep();
    }
}

void nav_follow_class::get_tf_goal()
{
    //TODO spin_once ??
    tf2::Stamped<tf2::Transform> stamped_transform;
    while(!_tf_buffer->canTransform(_params.frame_name_base_master, _params.frame_name_base_slave, tf2::TimePointZero, 2000ms) && rclcpp::ok()) {
      RCLCPP_INFO(
                    get_logger(), "waiting 2000 ms for %s->%s transform to become available",
                    _params.frame_name_base_master.c_str(), _params.frame_name_base_slave.c_str());
      std::this_thread::sleep_for(200ms);
    }
    _tf_goal_msg = _tf_buffer->lookupTransform(
            _params.frame_name_base_master, _params.frame_name_base_slave,
            tf2::TimePointZero);

    tf2::fromMsg(_tf_goal_msg, _tf_goal_transform);

    RCLCPP_INFO(get_logger(), "Initial transformation stored");
}

void nav_follow_class::tf_follow_thread()
{

    tf2::Stamped<tf2::Transform> stamped_transform_now;
    geometry_msgs::msg::Twist vel_cmd;
    double x_err,y_err,w_err;
    double x_cmd,y_cmd,w_cmd;

    while(rclcpp::ok() && this->_tf_controlling)
    {
        {
            std::scoped_lock<std::mutex> guard_tf(_tf_mutex);
            tf2::fromMsg(_tf_master_slave, stamped_transform_now);
        }

        tf2::Transform err =  stamped_transform_now.inverse() * _tf_goal_transform;

        x_err = err.getOrigin().getX();
        y_err = err.getOrigin().getY();

        double r{}, p{}, y{};
        tf2::Matrix3x3 m(err.getRotation());
        m.getRPY(r, p, y);
        w_err = y;


        // RCLCPP_INFO_STREAM(get_logger(),"err: " << x_err << " " << y_err << " " << w_err << " " << "\n");
        //TODO rate to Duration ?
        auto dt = rclcpp::Duration(50ms);
        x_cmd = _x_pid->computeCommand(x_err,dt);
        y_cmd = _y_pid->computeCommand(y_err,dt);
        w_cmd = _w_pid->computeCommand(w_err,dt);
        
        // RCLCPP_INFO_STREAM(get_logger(),"cmd: " << x_cmd << " " << y_cmd << " " << w_cmd << " " << "\n");
        {
            std::scoped_lock<std::mutex> guard_tf(_cmd_vel_mutex_tf);
            _cmd_vel_tf_msg.linear.x  = x_cmd;
            _cmd_vel_tf_msg.linear.y  = y_cmd;
            _cmd_vel_tf_msg.angular.z = w_cmd;
        }  
        _rate_follow->sleep();
    }
}

inline Eigen::Matrix3d Skew(Eigen::Vector3d vec) {
return (Eigen::Matrix3d() << 
    0.0, -vec[2], vec[1],
    vec[2], 0.0, -vec[0],
    -vec[1], vec[0], 0.0).finished();
}

void Adjoint_util(Eigen::Matrix<double,6,6> & mat, Eigen::Affine3d frame)
{
    mat.topLeftCorner(3,3)     = frame.linear();
    mat.bottomRightCorner(3,3) = frame.linear();
    mat.topRightCorner(3,3)    = Skew(frame.translation()) * frame.linear();
}

void nav_follow_class::cmd_vel_feedforward(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
{
    tf2::Stamped<tf2::Transform> stamped_transform_now;
    geometry_msgs::msg::Twist vel_cmd;
    Eigen::Affine3d tmp_tf;

    {
        std::scoped_lock<std::mutex> guard_tf(_tf_mutex);
        tmp_tf = tf2::transformToEigen(_tf_master_slave);
    }  

    
    Eigen::Matrix<double,6,6> mat = Eigen::Matrix<double,6,6>::Zero();
    Adjoint_util(mat, tmp_tf.inverse());
    Eigen::Matrix<double,6,1> tw_in;
    tw_in << msg->linear.x,msg->linear.y,msg->linear.z,msg->angular.x,msg->angular.y,msg->angular.z;

    auto tx_out = mat * tw_in;

    {
        std::scoped_lock<std::mutex> guard_tf(_cmd_vel_mutex_feed);
        _cmd_vel_feed_msg.linear.x  = tx_out[0];
        _cmd_vel_feed_msg.linear.y  = tx_out[1];
        _cmd_vel_feed_msg.angular.z = tx_out[5];
    }  

}
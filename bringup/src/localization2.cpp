#include <Localization.h>

Localization::Localization(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    subscribe_name_ = nh_.subscribe("/model_name", 1, &Localization::NameCallBack,this);
}

void Localization::publish_lidar_link(){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0,0,0));
  tf::Quaternion q;
  q.setRPY(0, 3.142, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link" , robot_name_+"/Lidar"));
}

void Localization::getGPS(){
    subscribe_gps_ = nh_.subscribe(robot_name_+"/global/values", 1, &Localization::GPSCallBack,this);
}

void Localization::getIMU(){
    subscribe_imu_ = nh_.subscribe(robot_name_+"/IMU/quaternion", 1, &Localization::IMUCallBack,this);
}

void Localization::getOdom(){
    subscribe_odom_ = nh_.subscribe(robot_name_+"/odom", 1, &Localization::OdomCallBack,this);
}

void Localization::NameCallBack(const std_msgs::String& msg){
    Localization::robot_name_ = msg.data;
    getGPS();
    getIMU();
    getOdom();
}

void Localization::GPSCallBack(const geometry_msgs::PointStamped& msg){
    current_x = msg.point.x;
    current_y = msg.point.y;
    current_z = msg.point.z;
}

void Localization::IMUCallBack(const sensor_msgs::Imu& msg){
    current_rot_x = msg.orientation.x;
    current_rot_y = msg.orientation.y;
    current_rot_z = msg.orientation.z;
    current_rot_w = msg.orientation.w;
    publish_base_link();
    publish_lidar_link();
}

void Localization::OdomCallBack(const nav_msgs::Odometry& msg){
    float current_x = msg.pose.pose.position.x;
    float current_y = msg.pose.pose.position.y;
    float current_z = msg.pose.pose.position.z;
    /*
    float current_rot_x = msg.pose.pose.orientation.x;
    float current_rot_y = msg.pose.pose.orientation.y;
    float current_rot_z = msg.pose.pose.orientation.z;
    float current_rot_w = msg.pose.pose.orientation.w;
    */
    // ToEulerAngles(current_rot_x, current_rot_y, current_rot_z, current_rot_w);
    
}

void Localization::ToEulerAngles(float x, float y , float z , float w ) {

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll_ = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch_ = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch_ = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw_ = std::atan2(siny_cosp, cosy_cosp);
    ROS_INFO("Euler Angles are: [%f] [%f] [%f]", roll_, pitch_, yaw_);
}

void Localization::publish_base_link(){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(-current_x, current_z, current_y) );
  ToEulerAngles(current_rot_x,current_rot_y,current_rot_z,current_rot_w);
  tf::Quaternion q;
  q.setRPY(0, 0, roll_);
  // transform.setRotation(tf::Quaternion(current_rot_x,current_rot_y,current_rot_z,current_rot_w));
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}


int main(int argc, char **argv){
    ros::init(argc, argv, "localization");
    ros::NodeHandle nh;
    Localization member(&nh);
    ros::spin();

    return 0;
}

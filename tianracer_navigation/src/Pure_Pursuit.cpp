#include "tianracer_navigation/Pure_Pursuit.h"

using namespace std;

PurePursuit::PurePursuit() {
    // 创建私有的节点句柄
    ros::NodeHandle pravite_nh("~");
    //Car parameter
    pravite_nh.param("base_shape_L", base_shape_L, 0.33); // 机器人轴距
    pravite_nh.param("reference_v", reference_v, 2.0);// 目标速度
    pravite_nh.param("Lfw", Lfw, 1.5); // 前视距离

    //Controller parameter
    pravite_nh.param("controller_freq", controller_freq_, 50);   // 控制频率
    pravite_nh.param("goal_radius", goal_radius, 0.2); // 目标容忍度
    pravite_nh.param("debug_mode", debug_mode, false); // debug mode

    //Publishers and Subscribers
    odom_sub_ = nh_.subscribe("/odom", 1, &PurePursuit::odomCallback, this);
//    path_sub_ = nh_.subscribe("/move_base/TebLocalPlannerROS/global_plan", 1, &PurePursuit::pathCallback, this);
    path_sub_ = nh_.subscribe("/global_plan", 1, &PurePursuit::pathCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &PurePursuit::goalCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/path_marker", 10);
    ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);    //Timer
    timer1 = nh_.createTimer(ros::Duration((1.0) / controller_freq_), &PurePursuit::controlLoopCallback, this); // 发布控制

    //Init variables
    foundForwardPt_ = false;
    goal_received_ = false;
    goal_reached_ = false;
    velocity = 0.0;
    steering_ = 0.0;
    ackermann_cmd_ = ackermann_msgs::AckermannDriveStamped();
    initMarker();
}
PurePursuit::~PurePursuit(){};
void PurePursuit::initMarker() {
    points_.header.frame_id = line_strip_.header.frame_id = goal_circle_.header.frame_id = "odom";
    points_.ns = line_strip_.ns = goal_circle_.ns = "Markers";
    points_.action = line_strip_.action = goal_circle_.action = visualization_msgs::Marker::ADD;
    points_.pose.orientation.w = line_strip_.pose.orientation.w = goal_circle_.pose.orientation.w = 1.0;
    points_.id = 0;
    line_strip_.id = 1;
    goal_circle_.id = 2;

    points_.type = visualization_msgs::Marker::POINTS;
    line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle_.type = visualization_msgs::Marker::CYLINDER;
    // 点大小设置
    points_.scale.x = 0.2;
    points_.scale.y = 0.2;

    // 线宽度设置
    line_strip_.scale.x = 0.1;

    goal_circle_.scale.x = goal_radius;
    goal_circle_.scale.y = goal_radius;
    goal_circle_.scale.z = 0.1;

    // 绿色
    points_.color.g = 1.0f;
    points_.color.a = 1.0;

    // 蓝色
    line_strip_.color.b = 1.0;
    line_strip_.color.a = 1.0;

    // 黄色
    goal_circle_.color.r = 1.0;
    goal_circle_.color.g = 1.0;
    goal_circle_.color.b = 0.0;
    goal_circle_.color.a = 0.5;
}
/*!
 *
 * @param wayPt  路径点
 * @param car_pos   位姿
 * @return 距离
 */
double PurePursuit::PointDistanceSquare(geometry_msgs::PoseStamped& wayPt,const geometry_msgs::Point& car_pos){
    double dx = wayPt.pose.position.x - car_pos.x;
    double dy = wayPt.pose.position.y - car_pos.y;
    return dx * dx + dy * dy;
}
/*!
 *
 * @param  odomMsg odom传感器数据
 * @author 测试
 * @author odom数据会出现漂移导致不准，可尝试使用tf变换 计算出base_link在map下的坐标
 */
void PurePursuit::odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg) {
    this->odom_ = *odomMsg;
    if(this->goal_received_)
    {
        double car2goal_x = this->goal_pos_.x - odomMsg->pose.pose.position.x;
        double car2goal_y = this->goal_pos_.y - odomMsg->pose.pose.position.y;
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
        /// 和move_base一起使用时这段注释解除
//        if(dist2goal < this->goal_radius)
//        {
//            this->goal_reached_ = true;
//            this->goal_received_ = false;
//            ROS_INFO("Goal Reached !");
//        }
    }
}

/*!
 *
 * @param pathMsg 全局路径
 * TODO importion 后期会使用局部路径
 */
void PurePursuit::pathCallback(const nav_msgs::Path::ConstPtr &pathMsg) {
    this->map_path_ = *pathMsg;
    /// 和move_base一起使用时下面两行注释掉
    goal_received_ = true;
    goal_reached_ = false;
}
/*!
 *
 * @param goalMsg
 */
void PurePursuit::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    this->goal_pos_ = goalMsg->pose.position;
    try
    {
        geometry_msgs::PoseStamped odom_goal;
        tf_listener_.transformPose("odom", ros::Time(0) , *goalMsg, "map" ,odom_goal);
        odom_goal_pos_ = odom_goal.pose.position;
        goal_received_ = true;
        goal_reached_ = false;
        /*Draw Goal on RVIZ*/
        goal_circle_.pose = odom_goal.pose;
        marker_pub_.publish(goal_circle_);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}
/*!
 *
 * @param carPose 当前位姿
 * @return 获取航向角误差
 */
double PurePursuit::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);
    return atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
}
/*!
 *
 * @param wayPt 寻找到的预选点
 * @param car_pos 位姿
 * @return 是否为前瞻点
 */
bool PurePursuit::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < Lfw)
        return false;
    else if(dist >= Lfw)
        return true;
}
/*!
 *
 * @param wayPt 路径点
 * @param carPose 机器人位姿
 * @return 判断路径点是否在机器人的前方
 */
int PurePursuit::minIndex(const geometry_msgs::Point& carPose){
    int index_min = 0;
    int d_min = INT16_MAX;
    for(int i =0; i< map_path_.poses.size(); i++)
    {
        geometry_msgs::PoseStamped map_path_pose = map_path_.poses[i];
        double d_temp = PointDistanceSquare(map_path_pose,carPose);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    return index_min;


}
bool PurePursuit::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta =  tf::getYaw(carPose.orientation);

    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;

    if(car_car2wayPt_x >0) /*is Forward WayPt*/
        return true;
    else
        return false;
}
/*!
 *
 * @param carPose 机器人位姿
 * @return 预瞄点对于机器人的距离
 */
geometry_msgs::Point PurePursuit::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = tf::getYaw(carPose.orientation);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt_ = false;

    if(!goal_reached_){
        //寻找离机器人最近的路径点
        int index = minIndex(carPose_pos);
        for(int i =index; i< map_path_.poses.size(); i++)
        {
            geometry_msgs::PoseStamped map_path_pose = map_path_.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;

            try
            {
                tf_listener_.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;

                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);

                if(_isForwardWayPt)
                {
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
                    if(_isWayPtAwayFromLfwDist)
                    {
                        forwardPt = odom_path_wayPt;
                        foundForwardPt_ = true;
                        break;
                    }
                }
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }

    }
    else if(goal_reached_)
    {
        forwardPt = odom_goal_pos_;
        foundForwardPt_ = false;
        //ROS_INFO("goal REACHED!");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points_.points.clear();
    line_strip_.points.clear();

    if(foundForwardPt_ && !goal_reached_)
    {
        points_.points.push_back(carPose_pos);
        points_.points.push_back(forwardPt);
        line_strip_.points.push_back(carPose_pos);
        line_strip_.points.push_back(forwardPt);
    }

    marker_pub_.publish(points_);
    marker_pub_.publish(line_strip_);

    // 转到小车坐标系
    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    return odom_car2WayPtVec;
}
/*!
 *
 * @param eta 获取航向角误差
 * @return 机器人舵机转角
 */
double PurePursuit::getSteering(double eta)
{
    return atan2(2*(this->base_shape_L*sin(eta)),(this->Lfw));
}


void PurePursuit::controlLoopCallback(const ros::TimerEvent&)
{

    geometry_msgs::Pose carPose = this->odom_.pose.pose;
    geometry_msgs::Twist carVel = this->odom_.twist.twist;

    if(this->goal_received_)
    {

        double eta = getEta(carPose);
//        cout<<eta<<endl;
        if(foundForwardPt_)
        {
            this->steering_ = this->base_angle_ + getSteering(eta);
            /*Estimate Gas Input*/
            if(!this->goal_reached_)
            {
                this->velocity = this->reference_v;
            }
        }
    }

    if(goal_reached_)
    {
        velocity = 0.0;
        steering_ = 0.0;
    }

    this->ackermann_cmd_.drive.steering_angle = this->steering_;
    this->ackermann_cmd_.drive.speed = this->velocity;
    this->ackermann_pub_.publish(this->ackermann_cmd_);
}


int main(int argc, char **argv) {
    // 创建ROS节点
    ros::init(argc, argv, "PurePursuit");
    // 调用类
    PurePursuit ppc;
    // 多线程工作
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

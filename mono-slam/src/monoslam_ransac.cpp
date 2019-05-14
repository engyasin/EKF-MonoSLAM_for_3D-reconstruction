#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include "RosVSLAMRansac.hpp"
#include "utils.hpp"

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>


using namespace cv;
using namespace std;
using namespace geometry_msgs;


namespace enc = sensor_msgs::image_encodings;

//static const char WINDOW[] = "Image window";

Vector3f quat2vec(Vector4f quat) {
  float n = acos(quat(0))*2;
  Vector3f vec;
  if (n>0.0001) {
    float n1 = n/sin(n/2);
    vec << quat(1)*n1,quat(2)*n1,quat(3)*n1;
  }else{
    vec << 0.0,0.0,0.0;
  }
  return vec;
}

float poses_diff(VectorXf state_old , VectorXf state_new, Vector3f last_rot){

  float a = (state_old.segment<3>(0)-state_new.segment<3>(0)).norm()*3.33;//is sqrt or not
  Vector3f b = (last_rot - quat2vec(state_new.segment<4>(3)))*57.29577951308232;
  //std::cout << " a, and b for poses_diff" << std:: endl;
  //std::cout << a << std::endl;
  //std::cout << b << std::endl;
  return a+abs(b(0))+abs(b(1))+abs(b(2));
}

Matrix4f YupsilonMatric1(Vector4f q1) {
	Matrix4f res;

    const float r1=q1(0);
    const float x1=q1(1);
    const float y1=q1(2);
   	const float z1=q1(3);

	res <<  r1, -x1, -y1, -z1,
		    x1,  r1, -z1,  y1,
		    y1,  z1,  r1, -x1,
		    z1, -y1,  x1,  r1;

	return res;

}

Vector4f quatCrossProduct(Vector4f q1, Vector4f q2) {
    return YupsilonMatric1(q1)*q2;
}

Vector4f quatinvCrossProduct(Vector4f q1, Vector4f q2) {
    return YupsilonMatric1(q1).inverse()*q2;
}

Vector4f vec2quat2(Vector3f vec) {
    float alpha = vec.norm();

    if (alpha != 0) {
        Vector4f quat;
        quat(0) = cos(alpha/2);
        quat.segment<3>(1) = vec*sin(alpha/2)/alpha;
        return quat;
    }
}

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Publisher camera_poses_pub;
  ros::Publisher camera_pose_pub;

  ofstream node_proj,node_proj2;
  ofstream f_points;
  ofstream cov_cams,cov_cams2;
  //ofstream traj;
  //ofstream traj_rts;

  ros::Publisher features_pub;
  ros::Publisher camera_pub;

  ros::Publisher odometry_pub;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  //ros::Publisher quality_index_pub;

private:
	RosVSLAM slam;
	int frameId;
	PoseArray poses;

	ros::Subscriber odom_subscriber,imu_subscriber,odom_subscriber_bag,tilt_subscriber_bag;

  Eigen::Vector3f V,Omiga,acc;
  Eigen::VectorXf P_O;
  float tilt;
  float cur_tilt;
  Eigen::Vector3f V_old,Omiga_old;

	//fstream fs;

public:
  int Num_of_points_thershold;
  float min_cov_for_pose,frame_ratio;
  int take_image_every_x_frame , selectedFrames ;
  cv::Mat Selected_Pose;
  MatrixX3i min_projs;
  int Pose_id;
  VectorXf min_stat;
  MatrixXf min_camscov, min_camscov2;
  bool control;
  Vector3f traj_xyz;
  Vector4f traj_quan;

  float MoveThresh;
  Vector3f last_vrot , last_vrot2;
  VectorXf last_image_pose, last_image_pose2;
  
  //Big Orb loop close
  vector<KeyPoint> Forb;
  vector<DMatch> matches;
  vector<int> firstImgPatches;
  vector<cv::Point2f> firstImgCorners;
  Ptr<DescriptorMatcher> descriptorMatcher;
  Mat FImgd, PLImgd;
  MatND FHist;
  Ptr<Feature2D> b;
  int distanceThresh;
  Mat FImg,FImg_origin;

  //loop closing member
  //haloc::LoopClosure detectLoop;
  //haloc::LoopClosure::Params paramsloop;

  //bool endthis = false;

  // vars for smoothing
  vector<VectorXf> MUs;
  vector<MatrixXf> Sigmas;
  vector<Vector3f> Us;//size n-1
  vector<double> dTs;
  vector<double> Timestamps;
  //for writing
  vector<int> good_poses;
  vector<MatrixX3i> PntsPrj;

  //end

  ImageConverter(char *file = NULL)
    : it_(nh_), frameId(0), slam(file)
  {
    Num_of_points_thershold = 10;
    min_cov_for_pose = 10000000;
    take_image_every_x_frame = 600;
    frame_ratio = 1.0;
    selectedFrames = 0;
    control = false;
    traj_quan << 0.46231064, 0.01957413, 0.87940359, 0.11179426;
    traj_xyz << -3.1739 , 1.35  ,  0.5686;

    MoveThresh = 18;
    last_vrot << 0.0,0.0,0.0;
    last_vrot2 << 0.0,0.0,0.0;
    last_image_pose = VectorXf::Zero(7,1);
    last_image_pose2 = VectorXf::Zero(7,1);

    min_projs = MatrixX3i::Zero(1,3);
    Pose_id = 0;
    min_stat = VectorXf::Zero(7,1);
    min_camscov = MatrixXf::Zero(14,14);
    min_camscov2 = MatrixXf::Zero(14,14);
    P_O = VectorXf::Zero(13);

    V = Eigen::Vector3f::Zero(1,3);
    Omiga = Eigen::Vector3f::Zero(1,3);

    V_old = Eigen::Vector3f::Zero(1,3);
    Omiga_old = Eigen::Vector3f::Zero(1,3);

    cur_tilt = -0.1570795 ;//-9.0;

    //loop clousre params
    b = ORB::create();
    descriptorMatcher = DescriptorMatcher::create("BruteForce-Hamming");
    distanceThresh = 10000;

    //some parameters:

  	poses.header.frame_id = "/world";
    image_pub_ = it_.advertise("/monoslam/imgproc", 1);
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);

    //odom_subscriber = nh_.subscribe("/cmd_vel_mux/input/teleop", 1, &ImageConverter::odom_callback, this);
    //imu_subscriber = nh_.subscribe("/imu", 1, &ImageConverter::imu_callback, this);
    odom_subscriber_bag = nh_.subscribe("/pose", 1, &ImageConverter::odom_add, this);
    tilt_subscriber_bag = nh_.subscribe("/cur_tilt_angle", 1, &ImageConverter::tilt_add, this);

  	node_proj.open("nodes_and_prjcts.txt");
  	node_proj2.open("nodes_and_prjcts2.txt");
    cov_cams.open("cams_cov.txt");
    cov_cams2.open("cams_cov2.txt");
  	f_points.open("points.txt");
  	/*traj.open("trajs.txt");

    traj << fixed << showpoint;
    traj << setprecision(5); 

  	traj_rts.open("trajs_rts.txt");

    traj_rts << fixed << showpoint;
    traj_rts << setprecision(5); 
    */
    //cv::namedWindow(WINDOW);
  	camera_poses_pub  = nh_.advertise<nav_msgs::Path>("/monoslam/camera_poses", 1000);
  	camera_pose_pub  = nh_.advertise<geometry_msgs::Pose>("/monoslam/camera_pose", 1000);

  	odometry_pub = nh_.advertise<nav_msgs::Odometry>("/monoslam/visual_odometry", 1000);


  	camera_pub  = nh_.advertise<visualization_msgs::MarkerArray>("/monoslam/camera", 1000);

  	features_pub  = nh_.advertise<visualization_msgs::MarkerArray>("/monoslam/features", 1000);

  	//quality_index_pub = nh_.advertise<std_msgs::Float32>("/monoslam/quality_index",1000);

  }


  ~ImageConverter()
  {
    //cv::destroyWindow(WINDOW);
  	//std::cout << "Imageclass destroyed1"<<std::endl;
    //RTSSmoothing();
  	node_proj.close();
  	node_proj2.close();
    cov_cams.close();
    cov_cams2.close();

  	MatrixXf pointsl = slam.getPointsFeatures();
  	f_points << pointsl;
    f_points.close();
    //traj.close();
    //traj_rts.close();
    //fs.close();	f_points << slam.getPointsFeatures();
  	//std::cout << slam.getPointsFeatures() <<std::endl;
    std::cout << "Imageclass destroyed"<<std::endl;
  	//node_proj.close();
    //f_points.close();
  }

  void RTSSmoothing(){

    for(int i=MUs.size()-2;i>=0;i--){
      slam.rts_epoch(MUs.at(i),Sigmas.at(i),MUs.at(i+1),Sigmas.at(i+1),-Us.at(2*i),-Us.at(2*i+1),dTs.at(i));
    }
    
    for(int i = 0; i < Timestamps.size(); i++)
    {

      RowVectorXd traj1 = RowVectorXd::Zero(8);

      traj1(0) = Timestamps.at(i);
      VectorXf stat14 = MUs.at(i);
      traj1.segment<3>(1) =  (stat14.segment<3>(0) + traj_xyz).transpose().cast<double>();
      Vector4f q = quatCrossProduct(stat14.segment<4>(3), traj_quan);
      traj1.segment<3>(4) =  q.segment<3>(1).transpose().cast<double>();
      traj1(7) =  q(0);
      //traj_rts << traj1 << endl;
    }

    /*
    for(int i = 0; i < good_poses.size(); i++)
    {
      int k = good_poses.at(i)+1;
      node_proj << "P" << k << endl;
      VectorXf min_stat = MUs.at(k-1);
      node_proj << min_stat.block<7,1>(0,0)<< endl;
      node_proj << PntsPrj.at(i) << endl;
    }
    */
  }

  string ToString(int val)
  {
      stringstream stream;
      stream << val;
      return stream.str();
  }

  void tilt_add(const std_msgs::Float64::ConstPtr& msg)
  {
    tilt = msg->data*3.14159/180;
  }

  void odom_add(const nav_msgs::Odometry::ConstPtr& msg)
  {
    //VectorXf P_O(7);
    //const std_msgs::Float64::ConstPtr&
    P_O(0) = msg->pose.pose.position.x;
    P_O(1) = msg->pose.pose.position.y;
    P_O(2) = msg->pose.pose.position.z;
//
    P_O(3) = msg->pose.pose.orientation.w;
    P_O(4) = msg->pose.pose.orientation.x;
    P_O(5) = msg->pose.pose.orientation.y;
    P_O(6) = msg->pose.pose.orientation.z;

    P_O(7) = msg->twist.twist.linear.x;
    P_O(8) = msg->twist.twist.linear.y;
    P_O(9) = msg->twist.twist.linear.z;

    P_O(10) = msg->twist.twist.angular.x;
    P_O(11) = msg->twist.twist.angular.y;
    P_O(12) = msg->twist.twist.angular.z;
    control  = true;
  }

  void odom_callback(const geometry_msgs::Twist::ConstPtr& odom_msg)
  {
		VectorXf stat14 = slam.getState();
    Vector4f q = stat14.segment<4>(3);
    Quaternionf qan;
    qan.w() = q(0);
    qan.x() = q(1);
    qan.y() = q(2);
    qan.z() = q(3);
    Eigen::Matrix3f R = qan.normalized().toRotationMatrix();
    float theta = atan2(R(1,0),R(0,0));

	  V(2) = odom_msg->linear.x * cos(theta);
	  V(0) = odom_msg->linear.x * sin(theta);
	  V(1) = odom_msg->linear.z;

	  Omiga(0) = odom_msg->angular.x;
	  Omiga(2) = odom_msg->angular.y;
    Omiga(1) = odom_msg->angular.z;
    control  = true;
  }

  void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
  {
	  acc(0) = imu_msg->linear_acceleration.x;
	  acc(1) = imu_msg->linear_acceleration.y;
	  acc(2) = imu_msg->linear_acceleration.z;
    //control  = false;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	  //std::cout << "New Frame Caputerd" << endl;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


	//int scale = 1;
	//cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(cv_ptr->image.size().width/scale, cv_ptr->image.size().height/scale));

	if (frameId==0) {

		frameId += 1;

	  slam.captureNewFrame(cv_ptr->image , msg->header.stamp.toSec());

  	slam.findNewFeatures(10);


    // back first image
    cvtColor(cv_ptr->image, FImg_origin , CV_BGR2GRAY );

    // back 1st img
    FImg = slam.returnGrayImg();

    for(int i = 1; i < slam.patchnumbre; i++)
    {
      firstImgPatches.push_back(i);
      firstImgCorners.push_back(slam.returnCentrPatchIndx(i));
    }

    int w = FImg.size().width ,h = FImg.size().height ;

    Forb.push_back(KeyPoint(w/4,h/4,h/2));
    Forb.push_back(KeyPoint(w/4,3*h/4,h/2));    
    Forb.push_back(KeyPoint(3*w/4,h/4,h/2));    
    Forb.push_back(KeyPoint(3*w/4,3*h/4,h/2));

    b->compute(FImg, Forb, FImgd);
    //compute histogram:
    ////////////////////////////////////////////////////
    /*
  	int hbins = 256; //histogram x axis size, that is histSize, 
  	int channels[] = { 0 }; //index of channel
  	int histSize[] = { hbins };
  	float hranges[] = { 0, 255 };
  	const float* ranges[] = { hranges };



  	calcHist(&FImg, 1, channels, Mat(), //MaskForHisto, // // do use mask
  		FHist, 1, histSize, ranges,
  		true, // the histogram is uniform
      false);
	  normalize(FHist, FHist, 1.0, 0.0, CV_MINMAX); 
    */	
    /////////////////////////////////////////////////////
    //not accurate
    dTs.push_back(0.03);
    Us.push_back(Eigen::Vector3f::Zero(1,3));
    Us.push_back(Eigen::Vector3f::Zero(1,3));
    //

		VectorXf stat14 = slam.getState();

    //cout <<"Scale map is: " << stat14(13) << endl;

    MUs.push_back(stat14);
    Sigmas.push_back(slam.getSigma());

//

//  quatinvCrossProduct() 
 /*   RowVectorXd traj1 = RowVectorXd::Zero(8);

    traj1(0) = msg->header.stamp.toSec();
    Timestamps.push_back(msg->header.stamp.toSec());
    traj1.segment<3>(1) =  (stat14.segment<3>(0) + traj_xyz).transpose().cast<double>();
    Vector4f q = quatCrossProduct(stat14.segment<4>(3), traj_quan);
    traj1.segment<3>(4) =  q.segment<3>(1).transpose().cast<double>();
    traj1(7) =  q(0);
    traj << traj1 << endl;
*/
	} else {

		slam.captureNewFrame(cv_ptr->image, msg->header.stamp.toSec());


		frameId += 1;
		// add your control input here if there's any
    // need to read delta V and Omiga here, to predict

		VectorXf stat14 = slam.getState();

    //cout <<"Scale map is: " << stat14(13) << endl;
    /*Eigen::Vector3f D_V = V - V_old;
    Eigen::Vector3f D_Omiga = Omiga - Omiga_old;*/

    Vector3f D_V = Eigen::Vector3f::Zero(1,3);
    Vector3f D_Omiga = Eigen::Vector3f::Zero(1,3);

//correct pose only
      /*D_V = V -  stat14.segment<3>(7);
      D_Omiga = Omiga - stat14.segment<3>(10);

      D_V = acc * slam.getDt();
      D_V = ((P_O.segment<3>(0) - stat14.segment<3>(0))/slam.getDt())-stat14.segment<3>(7);
      //Vector3f T_P_O = quat2vec(P_O.segment<4>(3));
      Vector4f q90y,qn90z;
      Vector3f qtil;
      qtil << tilt,0.0,0.0;
      q90y << 0.7071,0,0.7071,0;
      qn90z << 0.7071,0,0,-0.7071;
      Vector4f qres = quatCrossProduct(quatCrossProduct(
        quatCrossProduct(P_O.segment<4>(3),q90y),qn90z),vec2quat2(qtil));
      qres.normalized();
      std::cout << qres << std::endl;
      Vector4f qresx = quatinvCrossProduct(stat14.segment<4>(3),qres);
      qresx.normalized();
      std::cout << qresx << std::endl;
      D_Omiga = (quat2vec(qresx)/slam.getDt())-stat14.segment<3>(10);
      Vector3f T_P_O = quat2vec(quatCrossProduct(quatCrossProduct(P_O.segment<4>(3),q90y),qn90z));
      T_P_O(0) = T_P_O(0)+tilt;
      std::cout<< stat14.segment<4>(3)<< std::endl;
      std::cout<< qres << std::endl;
      std::cout<< T_P_O << std::endl;
      std::cout<< quat2vec(qres) << std::endl;
      //D_Omiga = ((quat2vec(qres) - quat2vec(stat14.segment<4>(3)))/slam.getDt())-stat14.segment<3>(10);
      control = false;
      //Vector3f D_Omiga = Eigen::Vector3f::Zero(1,3);
      std::cout<< D_Omiga<< std::endl;    */
      //
      //u really should take state from filter
      //Vector3f orientbase = quat2vec(P_O.segment<4>(3));//should be around -y
      //to make sure
      //Vector3f linearspeed(P_O(7)*cos(orientbase(2)),P_O(7)*sin(orientbase(2)),0.0);

      //D_V = (1*linearspeed) - stat14.segment<3>(7);

      Quaternionf  qcam(stat14(3),-stat14(4),-stat14(5),-stat14(6));
      // stat14.segment<4>(3));
      Vector3f zaxis(0,0,1);
      Vector3f xaxis(1,0,0);
      Vector3f angspeed = P_O(12)*(qcam.toRotationMatrix()*zaxis);
      Vector3f linearspeed = P_O(7) * (qcam.toRotationMatrix()*xaxis);
      Vector3f linearspd_m(linearspeed(0),-linearspeed(2),0.0);
      //std::cout << "linspeed" << endl;
      //std::cout << linearspeed << endl;
      // we will slow the tilt cause it's unsure to coverage(0.75)
      //angspeed(0) = angspeed(0) - ((tilt-cur_tilt)/(slam.getDt()*1.1));

      cur_tilt = tilt;
      D_Omiga = angspeed - stat14.segment<3>(10);
      D_V = (-1*linearspd_m) - stat14.segment<3>(7);
    //control = false;
    //modify some sigmas cause we're not sure 
    //here
    //vx,vy=0.03,0.03
    //wx,wy,wz=0.01,0.04,0.01
    //add some var (additional input to predict,use alter version of vmax)
     //D_V = Eigen::Vector3f::Zero(1,3);
     //D_Omiga = Eigen::Vector3f::Zero(1,3);
    slam.predict(D_V,D_Omiga, control);
    control = false;
    //V_old = V;
    //Omiga_old = Omiga;

		slam.update();

    //std::cout<< "pass1"<< std::endl;
    stat14 = slam.getState();
    /*RowVectorXd traj1 = RowVectorXd::Zero(8);

    traj1(0) = msg->header.stamp.toSec();
    Timestamps.push_back(msg->header.stamp.toSec());
    traj1.segment<3>(1) =  (stat14.segment<3>(0) + traj_xyz).transpose().cast<double>();
    Vector4f q = quatCrossProduct(stat14.segment<4>(3), traj_quan);
    traj1.segment<3>(4) =  q.segment<3>(1).transpose().cast<double>();
    traj1(7) =  q(0);
    traj << traj1 << endl;
    */

    /*MUs.push_back(stat14);
    Sigmas.push_back(slam.getSigma());
    dTs.push_back(slam.getDt());
    Us.push_back(D_V);
    Us.push_back(D_Omiga);
    */
		geometry_msgs::Pose camPose = slam.getCameraPose();

		poses.poses.push_back(camPose);
		camera_poses_pub.publish(slam.getCameraPath());

	// write the poses
  //only part of the path
  float DistWalked = poses_diff(last_image_pose,stat14.segment<7>(0),last_vrot);
/////////////////////////////////////////////////////////////////
// This part for comprasion with images chosen for movement only
/*
  if( DistWalked >= (MoveThresh) ){
    //save image and pose
    node_proj2 << "P" << frameId << endl;
    node_proj2 << stat14.segment<7>(0)<< endl;
    node_proj2 << "0  0  0" << endl;
    // will add here the image write
    min_camscov2 = slam.getSigma();
    cov_cams2 << min_camscov2.block<7,7>(0,0) << endl;
    string i_p = ToString(frameId);
    //std::string i_p = std::to_string(Pose_id);
    cv::imwrite(i_p +"0000.png",cv_ptr->image);

    //update last state
    //last_vrot2 = quat2vec(stat14.segment<4>(3));
    //last_image_pose2 = stat14.segment<7>(0);

  }
*/
// End of this part
/////////////////////////////////////////////////////////////////
  if ( DistWalked>(MoveThresh/2)   &&   DistWalked<MoveThresh ) {

    double some_var =  slam.Covariance_Parameter();
    //std::cout << "cov for Pose: " << some_var+ DistWalked << '\n';
    if(some_var <min_cov_for_pose){
      min_cov_for_pose = some_var;
      Pose_id = frameId;
      //replace traj1
      min_stat = stat14.segment<7>(0);
      min_camscov = slam.getSigma();
      //min_stat.segment<3>(0) = traj1.segment<3>(1).transpose().cast<float>();
      //min_stat(3) = traj1(7);//3
      //min_stat.segment<3>(4) = traj1.segment<3>(4).transpose().cast<float>();
      min_projs = slam.Point4sba;
      //will add here the image assinement
      Selected_Pose = cv_ptr->image.clone();
    }

  } else if ( DistWalked>=MoveThresh) {
    /* code */
    //1
    if(min_cov_for_pose<1000000){
      //std::cout << "Min cov for Pose: " << min_cov_for_pose << '\n';
      //good_poses.push_back(Pose_id-1);
      //PntsPrj.push_back(min_projs);
      ///////////////////////////////////************************new here
      //std::cout << "diff of cov:" << std::endl;
      //std::cout <<( slam.Covariance_Parameter()- min_cov_for_pose ) << std::endl;
      if(( slam.Covariance_Parameter()- min_cov_for_pose )< 0.000085){
        node_proj << "P" << frameId << endl;
        node_proj << stat14.segment<7>(0)<< endl;
        node_proj << "0  0  0" << endl;
        // will add here the image write
        min_camscov2 = slam.getSigma();
        cov_cams2 << min_camscov2.block<7,7>(0,0) << endl;
        cov_cams << min_camscov2.block<7,7>(0,0) << endl;
        string i_p = ToString(frameId);
        //std::string i_p = std::to_string(Pose_id);
        cv::imwrite(i_p +".png",cv_ptr->image);
      }else{
        node_proj << "P" << Pose_id << endl;
        node_proj << min_stat << endl;
        node_proj << min_projs << endl;

        cov_cams << min_camscov.block<7,7>(0,0) << endl;
        //return Sigma.block<STATE_DIM,STATE_DIM>(0,0);
        // will add here the image write
        string i_p = ToString(Pose_id);
        //std::string i_p = std::to_string(Pose_id);
        cv::imwrite(i_p +".png",Selected_Pose);
        //2
        //update last state
        //last_vrot = quat2vec(min_stat.segment<4>(3));
        //last_image_pose = min_stat;
      }
      last_vrot = quat2vec(stat14.segment<4>(3));
      last_image_pose = stat14.segment<7>(0);
    }else{
      // for first frame
      if(frameId < 5) {
      std::cout << "Taking current Pose" << '\n';
      node_proj << "P" << frameId << endl;
      node_proj << stat14.segment<7>(0)<< endl;
      node_proj << "0  0  0" << endl;

      min_camscov = slam.getSigma();
      cov_cams << min_camscov.block<7,7>(0,0) << endl;
      // will add here the image write
      string i_p = ToString(frameId);
      //std::string i_p = std::to_string(Pose_id);
      cv::imwrite(i_p +".png",cv_ptr->image);
      //update last state
      last_vrot = quat2vec(stat14.segment<4>(3));
      last_image_pose = stat14.segment<7>(0);  
      }
    }
    // Here we will take last image(no cov importance addin 2 to all names)
    min_cov_for_pose = 10000000;
  }
  
  /* 
  if (poses_diff(last_image_pose,stat14.segment<7>(0),last_vrot) > MoveThresh){
    //save image and pose
    node_proj << "P" << frameId << endl;
    node_proj << stat14.segment<7>(0)<< endl;
    node_proj << "0  0  0" << endl;
    // will add here the image write
    string i_p = ToString(frameId);
    //std::string i_p = std::to_string(Pose_id);
    cv::imwrite(i_p +".png",cv_ptr->image);

    //update last state
    last_vrot = quat2vec(stat14.segment<4>(3));
    last_image_pose = stat14.segment<7>(0);

  }
  //if(frameId>429 && frameId<620){
  //if(frameId<-1){
	if(slam.Point4sba.rows()>= Num_of_points_thershold){
      float some_var = slam.Covariance_Parameter();
      if(some_var <min_cov_for_pose){
        min_cov_for_pose = some_var;
        Pose_id = frameId;
        //replace traj1
        min_stat = stat14.segment<7>(0);
        //min_stat.segment<3>(0) = traj1.segment<3>(1).transpose().cast<float>();
        //min_stat(3) = traj1(7);//3
        //min_stat.segment<3>(4) = traj1.segment<3>(4).transpose().cast<float>();
        min_projs = slam.Point4sba;
        //will add here the image assinement
        Selected_Pose = cv_ptr->image.clone();
      }
    }

    //int term1 = slam.patchnumbre/(15*frame_ratio);
    //20 is for min feature
    //if(term1 >= selectedFrames){
    //  selectedFrames = term1+1;
    if((frameId%take_image_every_x_frame) == 0){
    /*  min_cov_for_pose = 100;
      Pose_id = frameId;
      min_stat.segment<3>(0) = traj1.segment<3>(1).transpose().cast<float>();
      min_stat(3) = traj1(7);//3
      min_stat.segment<3>(4) = traj1.segment<3>(4).transpose().cast<float>();
      min_projs = slam.Point4sba;
      //will add here the image assinement
      Selected_Pose = cv_ptr->image.clone();
      if(min_cov_for_pose<1000000){
        //std::cout << "Min cov for Pose: " << min_cov_for_pose << '\n';
        //good_poses.push_back(Pose_id-1);
        //PntsPrj.push_back(min_projs);
        node_proj << "P" << Pose_id << endl;
        node_proj << min_stat << endl;
        node_proj << min_projs << endl;
        // will add here the image write
        string i_p = ToString(Pose_id);
        //std::string i_p = std::to_string(Pose_id);
        //cv::imwrite(i_p +".png",Selected_Pose);
        }else{
        std::cout << "No Image has been taken" << '\n';
        }
      min_cov_for_pose = 1000000;
    }
  */

   // }
    /*
    Mat img = slam.returnGrayImg();
    b->compute(img, Forb, PLImgd);
    descriptorMatcher->match(FImgd, PLImgd, matches, Mat());

    int curr_dist = matches[0].distance +matches[1].distance +matches[2].distance +matches[3].distance;
    //alter curr dist
    int scale_dist = 100;
    curr_dist = curr_dist+(abs(stat14(0))+abs(stat14(1))+abs(stat14(2)))/(scale_dist*3);
    */
    //added to avoid errors
    int curr_dist = 100000000;
    if(curr_dist<distanceThresh){
      if(frameId>3 && frameId<24){
        distanceThresh = curr_dist+32;
      }else if(frameId <-100){
        //0) make sure of matching using SIFT? or histogram or many orbs

        cout << "Loop Closure detection here with distance: " << curr_dist<<endl;
        cout << "Image Number: " << frameId << endl;

        ///////////////check emd///////////////////////////////////
        /*
      	int hbins = 256; //histogram x axis size, that is histSize, 
      	int channels[] = { 0 }; //index of channel
      	int histSize[] = { hbins };
      	float hranges[] = { 0, 255 };
      	const float* ranges[] = { hranges };

        MatND CHist;
      	calcHist(&img, 1, channels, Mat(), //MaskForHisto, // // do use mask
      		CHist, 1, histSize, ranges,
      		true, // the histogram is uniform
          false);
    	  normalize(CHist, CHist, 1.0, 0.0, CV_MINMAX); 	

        //make signature
        Mat sig1(256, 2, CV_32FC1);
        Mat sig2(256, 2, CV_32FC1);

        //fill value into signature
        for(int h=0; h< hbins; h++)
          {
           float binval = FHist.at< float>(h);
           binval = (binval<0?0:binval);
           sig1.at< float>( h, 0) = binval;//some val are negtive
           sig1.at< float>( h, 1) = h;

           binval = CHist.at< float>(h);
           binval = (binval<0?0:binval);
           sig2.at< float>( h, 0) = binval;
           sig2.at< float>( h, 1) = h;
          }

        //compare similarity of 2images using emd.
        float emdT = 1 - cv::EMD(sig1, sig2, CV_DIST_L2); //emd 0 is best matching. 
        printf("similarity %5.5f %%\n", emdT*100 );
        */
        //emdT > 0.5
        if (slam.seedWithFirstImg(FImg,firstImgPatches,firstImgCorners)) {
          //1) add first patches to last img (patches, mu, sigma)
          //slam.seedWithFirstImg(FImg,firstImgPatches,firstImgCorners);
          //2) predict all
  		    slam.captureNewFrame(FImg_origin);//same dt
          slam.predict();
          //3) update to first image (positions) ,even if out of frame, to get Sigma and mu
          slam.loopUpdate();
          //4) perform rts_smoothing, only for poses
          //a) (FOR LOOP) pridect_rts (mu, sigma, u ,dTs) as vectors( u is 6 elements)
          //but mu and sigma as matrices
          cout << "Drift is (before rts ): "<< endl;
          cout << (MUs.back() - MUs.at(0)).transpose() << endl;
          //5) rts smoothing ==> when destroyed
          //6) endNode
          ros::shutdown();

        }else{
          distanceThresh = ((distanceThresh-curr_dist)>30? curr_dist+12:distanceThresh);
        }

      }
    }


		camera_pose_pub.publish(camPose);

		features_pub.publish(slam.getFeatures());
		camera_pub.publish(slam.ActualCameraRepr());

		odometry_pub.publish(slam.getVisualOdometry(msg->header.stamp));

	}

	cv_ptr->image = slam.returnImageDrawed();
  image_pub_.publish(cv_ptr->toImageMsg());

  }
};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "Mono_SLAM");
	char *file = NULL;
	if (argc > 1) file = argv[1];

	std::cout << "Starting monoslam_exec" << std::endl;

  ImageConverter ic(file);
  //ImageConverter* ip = new ImageConverter(file);
  ros::spin();
  //if (ros::ok()) {} else { delete ip;};
  //if(ic.testforESC()){return 0;};
  return 0;
}

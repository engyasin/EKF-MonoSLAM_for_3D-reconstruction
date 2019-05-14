#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <sstream>
// Messages
#include <visualization_msgs/Marker.h>
#include <sparse_bundle_adjustment/sba_file_io.h>

#include <sparse_bundle_adjustment/sba.h>
#include <sparse_bundle_adjustment/visualization.h>

#include <map>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>

#include <ros/callback_queue.h>


using namespace sba;
using namespace Eigen;

/*
  SBANode class, Methods:

  1. addFrame(): do many adding and sba and drawing ,each frame(node)
  2. addNode(): add state (r,q) and K matrix, (fixed unknown)
  3. addPoint(): add x,y,z,w=1; index same as order
  4. addProj(): add 2D points camera covariances and indexes
  5. doSBA(): do SBA according to rms>4 ,
  6. publishTopics(): draw on rviz;
  7. SBANode(): puplish topics (defs) , subscribe to (frames)
*/

Eigen::Matrix3f points_cov_2d(VectorXf cov_p, Quaternionf quatcam ,float z_m,float f_x=1131/2,float f_y=1144/2);
Eigen::Matrix3f prop_cov_2pnt(MatrixXf cov_p,VectorXf posecam,VectorXf pnt,float f_xy);

class SBANode
{
  public:
    SysSBA sba;
    ros::NodeHandle n;
    ros::Subscriber frame_sub;
    ros::Publisher cam_marker_pub;
    ros::Publisher point_marker_pub;

    ros::Timer timer_sba;
    ros::Timer timer_vis;

    // Mapping from external point index to internal (sba) point index
    std::map<unsigned int, unsigned int> point_indices;

    // Mapping from external node index to internal (sba) node index
    std::map<unsigned int, unsigned int> node_indices;
    // and the inverse
    std::map<unsigned int, unsigned int> node_indices_inv;
    Eigen::MatrixXf ponts;
    void addFrame();//what data types are these?
    void addNode(float xc, float yc,float zc,float wq,float xq,float  yq,float zq,int cam_msg_index);
    void addPoint(float msg_x,float msg_y,float msg_z,float msg_w,int msg_p_index);
    void addProj(int msg_u,int msg_v,int msg_pointindex, int msg_camindex,Matrix3f covariance );
    void doSBA(/*const ros::TimerEvent& event*/);
    void publishTopics(/*const ros::TimerEvent& event*/);
    void TestProjPnts();
    void WritePointsAndCameras(/* */);
    SBANode();
};

void SBANode::addFrame()
{

  ros::Time beginning = ros::Time::now();

  ifstream points,node_and_proj,cams_cov;
  std:vector<MatrixXf> Sigmas;
  Eigen::MatrixXf Sigma1 = MatrixXf::Zero(7,7);
  points.open("points.txt");
  node_and_proj.open("nodes_and_prjcts.txt");
  cams_cov.open("cams_cov.txt");
  std::cout << "Pass till here 0" << '\n';
  ponts = Eigen::MatrixXf::Zero(1,12);
  std::cout << "Pass till here 1 " << '\n';
  Eigen::RowVectorXf np(1,12);
  float x,y,z,xc,yc,zc,wq,xq,yq,zq;
  float c0,c1,c2,c3,c4,c5,c6;
  float sxx,syx,szx,sxy,syy,szy,sxz,syz,szz;//in that order
  points >> x >>y >>z;
  points >>sxx>>syx>>szx>>sxy>>syy>>szy>>sxz>>syz>>szz;
  ponts(0,0)=x;ponts(0,1)=y;ponts(0,2)=z;
  std::cout << "Pass till here 2" << '\n';
  ponts.block<1,9>(0,3) << sxx,syx,szx,sxy,syy,szy,sxz,syz,szz;
  cout <<"----" << endl;
  int i =1;

  while(!points.eof()){
      points >> x >>y >>z;
      np(0)=x;np(1)=y;np(2)=z;
      points >>sxx>>syx>>szx>>sxy>>syy>>szy>>sxz>>syz>>szz;
      np.block<1,9>(0,3) << sxx,syx,szx,sxy,syy,szy,sxz,syz,szz;
      Eigen::MatrixXf concatmat(ponts.rows()+1,12);
      concatmat << ponts,np;
      ponts = concatmat;
      if(x || y || z){
        addPoint(x,y,z,1.0,i);
      }
      i++;
      //z = 1000000;
  }
    cout << "Points added: " << ponts.rows()<< endl;
    //cout << ponts <<endl;
    char s;//P;
    string jx,jy,jz,t1;
    int cam_index = 1;
    int cam_order = 0;
    int u,v,point_index;

    node_and_proj >> s;
    node_and_proj >> cam_index;
    
    while(! cams_cov.eof()){
      /* code */

      for(int m = 0; m < 7; m++)
      {
        cams_cov>> c0>>c1>>c2>>c3>>c4>>c5>>c6;
        Sigma1.block<1,7>(m,0) << c0,c1,c2,c3,c4,c5,c6;
      }
      
      Sigmas.push_back(Sigma1);
    }

    Eigen::VectorXf cam_pose(7);
    cout << "Pass till here(1): " << endl;
    while(cam_index){
        cout <<"POSE(" <<cam_index<<")  is added"<< endl;
        node_and_proj >>xc>>yc>>zc>>wq>>xq>>yq>>zq;
        cam_pose << xc,yc,zc,wq,xq,yq,zq;
        addNode( xc,yc,zc,wq,xq,yq,zq,cam_index );
        Quaternionf q_cam(wq, xq, yq, zq);
        node_and_proj >>point_index>>u>>v;
        //point_index = point_index - 1 ;

        if(ponts(point_index,0) || ponts(point_index,1) || ponts(point_index,2)){
                //VectorXf cov_p_1 = ponts.block<1,9>(point_index,3).transpose();
                //Matrix3f cov_p_2 = points_cov_2d(cov_p_1, q_cam ,ponts(point_index,2));
                //cout << "Pass till here(100): " << endl;
                Matrix3f cov_p_2 = prop_cov_2pnt(Sigmas.at(cam_order),cam_pose,ponts.block<1,3>(point_index,0),2217.0187);
                cout << "Cov: "<<  endl;
                cout << cov_p_2 << endl;
                addProj(u,v,point_index,cam_index,cov_p_2);
        }
        node_and_proj >> t1;
        while(static_cast<int>(t1[0]) != 80){
            stringstream geek(t1);
            geek >>point_index;
            //point_index = point_index - 1 ;
            node_and_proj >>u>>v;

            if(ponts(point_index,0) || ponts(point_index,1) || ponts(point_index,2)){
                //VectorXf cov_p_1 = ponts.block<1,9>(point_index,3).transpose();
                //Matrix3f cov_p_2 = points_cov_2d(cov_p_1, q_cam ,ponts(point_index,2) );
                Matrix3f cov_p_2 = prop_cov_2pnt(Sigmas.at(cam_order),cam_pose,ponts.block<1,3>(point_index,0),2217.0187);
                addProj(u,v,point_index,cam_index,cov_p_2);
              }
            t1 = "P0";//for last of file;and cam index 0
            node_and_proj >> t1;
        }
        stringstream geek(t1.substr(1));
        geek >>cam_index;
        int times = 10;//ponts.rows()/10;
        if (sba.nodes.size() % times == 0)
          {
            publishTopics();
            TestProjPnts();
            doSBA();
          }
      cam_order++;
    }

  ros::Time end = ros::Time::now();

  printf("[SBA] Added frame with (%f s)\n", (end-beginning).toSec());
  doSBA();
  publishTopics();
  WritePointsAndCameras();
  node_and_proj.close();
  points.close();
  ros::shutdown();
}


void SBANode::addNode(float xc, float yc,float zc,float wq,float xq,float  yq,float zq,int cam_msg_index)
{
  Vector4d trans;
  MatrixXd camera_center(3,1);
  MatrixXd c(3,1);
  camera_center << xc,yc,zc;
  Quaterniond qrot(wq, xq, yq, zq);

  /*Matrix3d R = qrot.normalized().toRotationMatrix().transpose();
  c = -R*camera_center;
  trans << c(0),c(1),c(2),1.0;*/
  trans << xc,yc,zc,1.0;
  frame_common::CamParams cam_params;
  int scale = 1;
  cam_params.fx = 2217.0187/scale;//1272.13/scale;
  cam_params.fy = 2217.0187/scale;//1279.0/scale;
  cam_params.cx = 1280.5/scale;//231.73/scale;//232/scale;
  cam_params.cy = 960.5/scale;//220.55/scale;//220/scale;
  cam_params.tx = 0;//for mono

  bool fixed = false;//unknown last input to sba

  unsigned int newindex = sba.addNode(trans, qrot, cam_params,fixed);

  node_indices[cam_msg_index] = newindex;
  node_indices_inv[newindex] = cam_msg_index;
}

void SBANode::addPoint(float msg_x,float msg_y,float msg_z,float msg_w,int msg_p_index)
{
  Vector4d point(msg_x, msg_y, msg_z, msg_w);
  unsigned int newindex = sba.addPoint(point);

  point_indices[msg_p_index] = newindex;
}

void SBANode::addProj(int msg_u,int msg_v,int msg_pointindex, int msg_camindex,Matrix3f covariance)
{
        int camindex = node_indices[msg_camindex];
        int pointindex = point_indices[msg_pointindex];
        Vector2d keypoint(msg_u, msg_v);// msg.d ?
        bool stereo = false;
        bool usecovariance = false;
        Eigen::Matrix3d covariance1 = covariance.cast<double>() ;
        /*covariance << cov_p(0),cov_p(1),cov_p(2),
                      cov_p(3),cov_p(4),cov_p(5),
                      cov_p(6),cov_p(7),cov_p(8);*/

        // Make sure it's valid before adding it.
        if (pointindex < (int)sba.tracks.size() && camindex < (int)sba.nodes.size())
          {
          sba.addMonoProj(camindex, pointindex, keypoint);
          if (usecovariance){
            sba.setProjCovariance(camindex, pointindex, covariance1);
          }else{
              //std::cout <<" Projection_Cov : " <<covariance <<std::endl;
          }
          }
        else
          {
          ROS_INFO("Failed to add projection: C: %d, P: %d, Csize: %d, Psize: %d",
                  camindex, pointindex,(int)sba.nodes.size(),(int)sba.tracks.size());
          }
}

void SBANode::doSBA(/*const ros::TimerEvent& event*/)
{
  unsigned int projs = 0;
  // For debugging.
  for (int i = 0; i < (int)sba.tracks.size(); i++)
  {
    projs += sba.tracks[i].projections.size();
  }
  ROS_INFO("SBA Nodes: %d, Points: %d, Projections: %d", (int)sba.nodes.size(),
    (int)sba.tracks.size(), projs);

  if (sba.nodes.size() > 0)
  {
    // Copied from vslam.cpp: refine()
    sba.doSBA(10, 1.0e-4, SBA_SPARSE_CHOLESKY);

    double cost = sba.calcRMSCost();

    if (isnan(cost) || isinf(cost)) // is NaN?
    {
      ROS_INFO("NaN cost!");
    }
    else
    {
      if (sba.calcRMSCost() > 4.0)
        sba.doSBA(10, 1.0e-4, SBA_SPARSE_CHOLESKY);  // do more
      if (sba.calcRMSCost() > 4.0)
        sba.doSBA(15, 1.0e-4, SBA_SPARSE_CHOLESKY);  // do more
      std::cout<< "Final RMS error in pixels= "<< sba.calcRMSCost() <<std::endl;
    }
  }
}

void SBANode::publishTopics(/*const ros::TimerEvent& event*/)
{
  // Visualization
  if (cam_marker_pub.getNumSubscribers() > 0 || point_marker_pub.getNumSubscribers() > 0)
  {
     drawGraph(sba, cam_marker_pub, point_marker_pub);
     ros::Duration(1.0).sleep();
  }
}
void SBANode::TestProjPnts()
{
  int x = node_indices[0];
  Vector2d pn;
  Vector4d p3;
  p3 << ponts(3,0),ponts(3,1),ponts(3,2),1.0;
  const Node nd = sba.nodes[x];
  nd.project2im(pn,p3);
  cout << "The 3D Point (3 in cam 1) is :"<<endl;
  cout << p3 << endl;
  cout << "The Projected Point is: "<< endl;
  cout << pn << endl;
  cout << "Translaition matrix number " <<endl;
  cout << nd.trans << endl;
  cout << "Rotation  matrix number "<< endl;
  cout << nd.qrot.toRotationMatrix() << endl;
  cout << "w2i matrix  "<< endl;
  cout << nd.w2i << endl;
  cout << "w2n matrix number "<< endl;
  cout << nd.w2n << endl;
  cout << "Kcam matrix  "<< endl;
  cout << nd.Kcam << endl;
}
void SBANode::WritePointsAndCameras(/*const ros::TimerEvent& event*/)
{
  // make the txt files
  ofstream PointsOut,NodesOut;
  PointsOut.open("Points_Out.txt");
  NodesOut.open("Nodes_Out.txt");
  // read the points and write it
  MatrixX3d All_points((int)sba.tracks.size(),3);
  for (int i = 0; i < (int)sba.tracks.size(); i++)
  {
      Vector4d pt = sba.tracks[i].point;
      All_points.block(i,0,1,3) << pt(0),pt(1),pt(2);
  }
  PointsOut << All_points <<'\n';
  // close the file
  PointsOut.close();
  // read the cameras

  for (int i = 0; i < (int)sba.nodes.size(); i++){

      const Node nd = sba.nodes[i];
      MatrixXd t(3,1);
      MatrixXd c(3,1);
      t << nd.trans.x(),nd.trans.y(),nd.trans.z();
      //Matrix3d R = nd.qrot.normalized().toRotationMatrix();
      //TODO: check
      //c = -R.transpose()*t;
      NodesOut << "P" << node_indices_inv[i] << '\n';
      NodesOut << t << '\n';
      NodesOut << nd.qrot.normalized().w() << '\n';
      NodesOut << nd.qrot.normalized().x() << '\n';
      NodesOut << nd.qrot.normalized().y() << '\n';
      NodesOut << nd.qrot.normalized().z() << '\n';
  }
  // close the file
  NodesOut.close();
  std::cout << "Points and Nodes Written" << '\n';
}
SBANode::SBANode()
{
  // Advertise visualization topics.
  cam_marker_pub = n.advertise<visualization_msgs::Marker>("/sba/cameras", 1);
  point_marker_pub = n.advertise<visualization_msgs::Marker>("/sba/points", 1);

  // Subscribe to topics.
  sba.useCholmod(true);
  //addFrame();
  //sba.useCholmod(true);

  printf("[SBA] Initialization complete.\n");
}

Eigen::Matrix3f points_cov_2d( VectorXf cov_p,Quaternionf quatcam ,float z_m,float f_x, float f_y){

  Matrix3f cov3d;
  cov3d << cov_p(0),cov_p(1),cov_p(2),
           cov_p(3),cov_p(4),cov_p(5),
           cov_p(6),cov_p(7),cov_p(8);
  SelfAdjointEigenSolver<MatrixXf> eigenSolver(cov3d);
  Vector3f eigs = eigenSolver.eigenvalues();
  Matrix3f eigs_mat = Matrix3f::Zero(3,3);
  eigs_mat(0) = eigs(0); eigs_mat(4) = eigs(1); eigs_mat(8) = eigs(2);
  Matrix3f vecs = eigenSolver.eigenvectors();
  Matrix3f n_vecs = quatcam.toRotationMatrix()*vecs;
  Matrix3f cov2d = n_vecs*eigs_mat*(n_vecs.inverse());
  cov2d = cov2d *(1/z_m/z_m);
  cov2d(0)*=(f_x*f_x);
  cov2d(4)*=(f_y*f_y);
  cov2d(3)*=(f_x*f_y);
  cov2d(1)*=(f_x*f_y);
  cov2d.block<1,3>(2,0) << 0,0,0;
  cov2d.block<3,1>(0,2) << 0,0,0;
  return cov2d;
};

Eigen::Matrix3f prop_cov_2pnt(MatrixXf cov_p,VectorXf posecam,VectorXf pnt,float f_xy){

  MatrixXf Jacob_C = MatrixXf::Zero(2,7);
  Matrix2f cov_f;
  Matrix3f cov_r = Matrix3f::Zero(3,3);
  float x,y,z,fx,fy,cx,cy,cz;
  float qw,qx,qy,qz,v0,v1,v1_2,v3;
  x = pnt(0);
  y = pnt(1);
  z = pnt(2);
  fx = fy = f_xy;
  cx = posecam(0);cy = posecam(1);cy = posecam(1);
  qw = posecam(3);qx = posecam(4);qy = posecam(5);qz = posecam(6);

  v0 = (-cx*(-2*pow(qy,2) - 2*pow(qz,2) + 1) - cy*(2*qw*qz + 2*qx*qy) - cz*(-2*qw*qy + 2*qx*qz) + x*(-2*pow(qy,2) - 2*pow(z,2) + 1) + y*(2*qw*qz + 2*qx*qy) + z*(-2*qw*qy + 2*qx*qz));
  v1 = (-cx*(2*qw*qy + 2*qx*qz) - cy*(-2*qw*qx + 2*qy*qz) - cz*(-2*pow(qx,2) - 2*pow(qy,2) + 1) + x*(2*qw*qy + 2*qx*qz) + y*(-2*qw*qx + 2*qy*qz) + z*(-2*pow(qx,2) - 2*pow(qy,2) + 1));
  v1_2 = pow(v1,2);
  v3 = (-cx*(-2*qw*qz + 2*qx*qy) - cy*(-2*pow(qx,2) - 2*pow(qz,2) + 1) - cz*(2*qw*qx + 2*qy*qz) + x*(-2*qw*qz + 2*qx*qy) + y*(-2*pow(qx,2) - 2*pow(qz,2) + 1) + z*(2*qw*qx + 2*qy*qz));

  Jacob_C.block<1,7>(0,0) << fx*(2*qw*qy + 2*qx*qz)*v0/v1_2 + fx*(2*pow(qy,2) + 2*pow(qz,2) - 1)/v1,
    fx*(-2*qw*qx + 2*qy*qz)*v0/v1_2 + fx*(-2*qw*qz - 2*qx*qy)/v1,
    fx*(2*qw*qy - 2*qx*qz)/v1 + fx*(-2*pow(qx,2) - 2*pow(qy,2) + 1)*v0/v1_2,
    fx*(2*cx*qy - 2*cy*qx + 2*qx*y - 2*qy*x)*v0/v1_2 + fx*(-2*cy*qz + 2*cz*qy - 2*qy*z + 2*qz*y)/v1,
    fx*(-2*cy*qy - 2*cz*qz + 2*qy*y + 2*qz*z)/v1 + fx*(2*cx*qz - 2*cy*qw - 4*cz*qx + 2*qw*y + 4*qx*z - 2*qz*x)*v0/v1_2,
    fx*(2*cx*qw + 2*cy*qz - 4*cz*qy - 2*qw*x + 4*qy*z - 2*qz*y)*v0/v1_2 + fx*(4*cx*qy - 2*cy*qx + 2*cz*qw - 2*qw*z + 2*qx*y - 4*qy*x)/v1,
    fx*(2*cx*qx + 2*cy*qy - 2*qx*x - 2*qy*y)*v0/v1_2 + fx*(4*cx*qz - 2*cy*qw - 2*cz*qx + 2*qw*y + 2*qx*z - 4*qz*x)/v1;

  Jacob_C.block<1,7>(1,0) << fy*(2*qw*qy + 2*qx*qz)*v3/v1_2 + fy*(2*qw*qz - 2*qx*qy)/v1,
    fy*(-2*qw*qx + 2*qy*qz)*v3/v1_2 + fy*(2*pow(qx,2) + 2*pow(qz,2) - 1)/v1,
    fy*(-2*qw*qx - 2*qy*qz)/v1 + fy*(-2*pow(qx,2) - 2*pow(qy,2) + 1)*v3/v1_2,
    fy*(2*cx*qy - 2*cy*qx + 2*qx*y - 2*qy*x)*v3/v1_2 + fy*(2*cx*qz - 2*cz*qx + 2*qx*z - 2*qz*x)/v1,
    fy*(-2*cx*qy + 4*cy*qx - 2*cz*qw + 2*qw*z - 4*qx*y + 2*qy*x)/v1 + fy*(2*cx*qz - 2*cy*qw - 4*cz*qx + 2*qw*y + 4*qx*z - 2*qz*x)*v3/v1_2,
    fy*(-2*cx*qx - 2*cz*qz + 2*qx*x + 2*qz*z)/v1 + fy*(2*cx*qw + 2*cy*qz - 4*cz*qy - 2*qw*x + 4*qy*z - 2*qz*y)*v3/v1_2,
    fy*(2*cx*qx + 2*cy*qy - 2*qx*x - 2*qy*y)*v3/v1_2 + fy*(2*cx*qw + 4*cy*qz - 2*cz*qy - 2*qw*x + 2*qy*z - 4*qz*y)/v1;

  cov_f = Jacob_C*cov_p*(Jacob_C.transpose());
  cov_r.block<2,2>(0,0) << cov_f;
  return cov_r;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sba_node");
  SBANode sbanode;
  sbanode.addFrame();

  ros::spin();
  return 0;
}

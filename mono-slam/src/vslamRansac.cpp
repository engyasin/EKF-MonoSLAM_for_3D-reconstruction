//YY: Modified Comments only
//Yasin Yousif : engyasin.github.io 2018/1/2

#include <ros/ros.h>

#include "vslamRansac.hpp"
#include "utils.hpp"

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <algorithm>



#define STATE_DIM 14

/**
 * Prototipes of the used funcitons
 *
 */


/**
 *
 * @param St
 * @param sigma_size
 * @return
 */

/* Compute the ellispoides parameters from features prediction covariance and store it
*  in a matrix of which each row represent a features ellipoide in form (a,b,theta), where
*  a and b are the semiaxis and theta are the rotation angle
*/
inline MatrixXi computeEllipsoidParameters(MatrixXf St, int sigma_size);

/**
 * Converts a Vector3f to a Quaternion
 * @param vec Vector to convert
 * @return Quaternion
 * perhaps used in the system equations
 * to convert an updated angle to state
 * unknown math , to search
 */
inline Vector4f vec2quat(Vector3f vec);

/**
 * Computes the rotation matrix associated to a quaternion
 * @param quat The quaternion
 * @return The rotation matrix
 * cool but where to apply
 * model uknown
 */
inline Matrix3f quat2rot(Vector4f quat);

/**
 * Computes the quaternion complement associated to a given quaternion
 * @param quat The quaternion
 * @return The complement of quat
 */
inline Vector4f quatComplement(Vector4f quat);

/*
* a function to convert 3d points encodings
*
*/
inline Vector3f inverse2XYZ4_projecting(VectorXf f, Vector3f r, MatrixXf &J_hp_f , bool give_J=false);


inline Matrix3f diffQuat2rot(Vector4f quat, int index);
// compute the partial derivate matrix of q2r(q) for q_index
// general f formula (jacopian for r) at index

inline Vector4f quatZero();
inline Vector4f quatCrossProduct(Vector4f h, Vector4f g);
// that's for predict state

// return Jacobian d(g(x))/dx
inline MatrixXf System_model_jacobian(VectorXf X_old, double dT , Vector3f Rotational_Speed_Control);

// compute the Jacobian between q(t) and q(t-1) knowing the rotation h = quat(w*dT)

inline Matrix4f Jacobian_qt_qt1(Vector4f h);

// Computer the Jacobian df/dhW
inline MatrixXf Jacobain_inv_feature_to_hW(Vector3f hW);

// compute the Jacobian between q(t) and w(t-1) knowing the actual rotation q, w and dT
inline MatrixXf Jacobian_qt_w(Vector4f q, Vector3f w, double dT);

// return the Jacobian d(q^c)/d(q)

inline Matrix4f d_qbar_q();

// return Jacobian d(R(q)d)/dq
inline MatrixXf Jacobian_hW_to_qantrion(Vector4f q, Vector3f d);

// take state and variance and return what?(do what)
inline void normalizeQuaternion(VectorXf &mu, MatrixXf &Sigma);

/* State camera Prediction
	This function predicts the State of the camera using kwnoledge on Motion
	and the time distance from the last update.

	Xv: last state of the camera
	dT: time distance

    return: Predicted state of the camera
*/

inline VectorXf Predict_State(VectorXf Xv,Vector3f T_speed,Vector3f R_speed, double dT);


inline bool isInsideImage(Vector2f hi, cv::Size size, int windowsSize);
// in patch.findmatch() there's use for this


// VSLAM Class Implementation


int VSlamFilter::numOfFeatures() {
	return this->patches.size();
}

MatrixXf VSlamFilter::getSigma(){
 	return Sigma.block<STATE_DIM,STATE_DIM>(0,0);
 }

VectorXf VSlamFilter::getState() {
	VectorXf system_state(STATE_DIM);
	system_state = mu.segment<STATE_DIM>(0);

    return system_state;
}

VSlamFilter::VSlamFilter(char *file) : config(file) {
    // to review intializing a var not to input but to an attr
    // member initializer ,(setting a value to your parameters)
	old_ts = -1;
	float sigma_vv = 0.0004, sigma_ww = 0.0004;
  //initalize with very small covaraiance to calculate Jacobians
	patchnumbre = 1;
	kernel_min_size = config.kernel_size;
	sigma_pixel = config.sigma_pixel;
	sigma_pixel_2 = sigma_pixel*sigma_pixel;
	sigma_size = config.sigma_size;
	windowsSize = config.window_size;

 	dT = 1;
	T_camera = config.T_camera;
	scale = config.scale;
	cam.setParam(config.camParams);
    noise_cov_factor = 0;

    //akaze = cv::ORB::create(30);

  mu = VectorXf::Zero(STATE_DIM); mu(3) = 1;
  mu(13) = 1; // initialization mu
  //hard code tum data values
  ///////
  Vector4f qs_f_odo,q90y,qn90z;
  Vector3f tilt;
  tilt << -9*3.14159/180,0.0,0.0;
  //wxyz
  qs_f_odo << 0.995396,0.0,0.0,0.095846;//360 data
  //qs_f_odo << -0.121869,0.0,0.0,0.992546;//slam data
  //qsx << 0,1,0,0;
  q90y << 0.7071,0,0.7071,0;
  qn90z << 0.7071,0,0,-0.7071;
  //making the camera coords
  //mu.segment<4>(3) = quatCrossProduct(quatCrossProduct(
  //                   quatCrossProduct(qs_f_odo,q90y),qn90z),vec2quat(tilt));
  //mu.segment<4>(3) = quatCrossProduct(qs_f_odo,qsz);
  mu.segment<4>(3) << 0.0,0.0,-0.707106781,0.707106781;

  //mu.segment<3>(0) << 1.354,-2.106,0.0;//qx qy inverted, 360
  //mu.segment<3>(0) << 3.448,-1.614,0.0;//qx qy inverted, slam
  /*
  //plant bag
  Vector4f qs;
  Vector4f qsz;
  qsz << 0,0,0,1;
  qs << 0.3124,-0.2849,-0.5960,0.6826;
  mu.segment<4>(3) = quatCrossProduct(qs,qsz);
  mu.segment<3>(0) << 0.8939,-0.4399,0.7370;//qx qy inverted
  */
  ///////
    Vmax = MatrixXf::Identity(6,6);
	Vmax(0,0) = config.sigma_vx*config.sigma_vx;
	Vmax(1,1) = config.sigma_vy*config.sigma_vy;
	Vmax(2,2) = config.sigma_vz*config.sigma_vz;//very small for planar robot
	Vmax(3,3) = config.sigma_wx*config.sigma_wx;
	Vmax(4,4) = config.sigma_wy*config.sigma_wy;
	Vmax(5,5) = config.sigma_wz*config.sigma_wz;

    Vmax_n = Vmax*2;

	/*Vmax_n(0,0) = 0.02*0.02;
	Vmax_n(1,1) = 0.02*0.02;
	Vmax_n(3,3) = 0.005*0.005;
	Vmax_n(4,4) = 0.02*0.02;
	Vmax_n(5,5) = 0.005*0.005;*/

    // all r ,q very small covariance to start
	Sigma = 0.0000000004*MatrixXf::Identity(STATE_DIM,STATE_DIM);
	Sigma(13,13) = 0.09;

    // bigger covariance for speeds (zeros)
	Sigma.block<3,3>(7,7) = sigma_vv*sigma_vv*MatrixXf::Identity(3,3);
	Sigma.block<3,3>(10,10) = sigma_ww*sigma_ww*MatrixXf::Identity(3,3);

    nInitFeatures = config.nInitFeatures;
	MatrixX3i Point4sba = MatrixX3i::Zero(1,3);
	//Point4sba(0) = -1;
    lastImgPatches.push_back(0);

}


void VSlamFilter::captureNewFrame(cv::Mat newFrame, double time_stamp) {
	if (old_ts > 0) {
		dT = (time_stamp - old_ts);
		std::cout << "my Dt = " << dT*1000 << " milisecond"<<std::endl;
	}
	old_ts = time_stamp;
	captureNewFrame(newFrame);
}

void VSlamFilter::captureNewFrame(cv::Mat newFrame) {
	cv::resize(newFrame,newFrame,cv::Size(newFrame.size().width/scale, newFrame.size().height/scale));
	old_frame = frame.clone();
    drawedImg = newFrame.clone();// for painting purposes
    //cvtColor( newFrame, frame, CV_BGR2GRAY );
    if(newFrame.channels()!=1){
        cvtColor(newFrame, frame , CV_BGR2GRAY );
    }else{
        frame = newFrame.clone();
    }
}

double VSlamFilter::getDt(){ return dT; }

bool VSlamFilter::seedWithFirstImg(cv::Mat img,std::vector<int> Pnts,std::vector<cv::Point2f> FImgCorners){

    std::vector<cv::Point2f> oldCorners;
    std::vector<uchar> staut;
    cv::calcOpticalFlowPyrLK(img,frame,
                            FImgCorners,oldCorners,
                            staut,
                            cv::noArray(),
                            cv::Size( 20, 20 ), // Search window size
                            3,//pyramid levels
                            cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                                    35,0.03
                                    ));

    // a term to assure matching
    int m=0;
    for(int i = 0; i < staut.size(); i++)
    {
        if(staut[i]) m++;
    }
    if(((float)m/FImgCorners.size())<0.88) {
        std::cout << "Number of feats to tot_feats: " << std::endl;
        std::cout << m << std::endl;
        std::cout << FImgCorners.size() << std::endl;
        return false;
    }
    

    for(int i=0; i<deleted_patches.size(); i++){
        if (std::find(Pnts.begin(),Pnts.end(),deleted_patches[i].real_index)!=Pnts.end()){
            if(staut[deleted_patches[i].real_index]){
                // change u,v 
                Patch P = deleted_patches[i]; // it's a copy
                P.center = oldCorners[P.real_index];
                P.z(0) = P.center.x;
                P.z(1) = P.center.y;
                P.matching_patch = P.patch.clone();
                //change mu
                int pos = mu.size();
                P.position_in_state = pos;
        		mu = Concat(mu,P.XYZ_pos);

                //change Sigma
                Matrix3f Patch_sigma = Matrix3f::Zero(3,3);
                MatrixXf Sigma1 = MatrixXf::Zero(pos+3,pos+3);
			    for (int i = 0; i < Patch_sigma.size(); i++) {
			    	Patch_sigma(i) = P.cov_4_delete(i);
			    }
                Sigma1.block<3,3>(pos,pos) = Patch_sigma.transpose().eval();
                Sigma1.block(0,0,pos,pos) = Sigma;
                Sigma = Sigma1;
                //change Patches
                patches.push_back(P);

            }
        }
    }
    return true;
}

int VSlamFilter::addFeature(cv::Point2f pf) {

    Vector2f hd;
    hd << (float)pf.x, (float)pf.y;

    if(isInsideImage(hd,frame.size(),windowsSize)) {

    	int pos = mu.size();

        Patch newPat(cv::Mat(frame, cv::Rect(pf.x-windowsSize/2, pf.y-windowsSize/2, windowsSize,windowsSize)), pf, pos,patchnumbre);
		patchnumbre +=1;
        patches.push_back(newPat);

        Vector3f r = mu.segment<3>(0);
        Vector4f q = mu.segment<4>(3);

        MatrixXf Jacobian_3dpointto_2dpoint;
        Vector3f hC = cam.UndistortAndDeproject(hd, Jacobian_3dpointto_2dpoint);

        Matrix3f Rot = quat2rot(q);

        Vector3f hW = Rot*hC;// in world cord

        float hx = hW(0);
        float hy = hW(1);
        float hz = hW(2);

        float ro = config.rho_0;// initialize depth


        float theta = atan2(hx,hz);
        float phi = atan2(-hy, sqrt(hx*hx+hz*hz));

		// Updating state and Sigma

		VectorXf f(6);
		f << r, theta, phi, ro;
		mu = Concat(mu,f);

        // apply equation 5.72
        int nOld = Sigma.rows();
        MatrixXf Js = MatrixXf::Zero(nOld+6, nOld+3);// why the jacobian 6*3
        Js.block(0,0,nOld,nOld) = MatrixXf::Identity(nOld,nOld);
        Js.block<3,3>(nOld,0) = MatrixXf::Identity(3,3);// for r?


        MatrixXf Jacobain_feature_to_hW = Jacobain_inv_feature_to_hW(hW);//what it does d_f_d_hW
        MatrixXf Jacobian_hW_2q = Jacobian_hW_to_qantrion(q,hC);

        Js.block<6,4>(nOld,3) = Jacobain_feature_to_hW*Jacobian_hW_2q;
        Js.block<6,2>(nOld,nOld) = Jacobain_feature_to_hW*Rot*Jacobian_3dpointto_2dpoint;
        Js.block<6,1>(nOld, nOld+2) << 0,0,0,0,0,1;

        MatrixXf S = sigma_pixel_2*MatrixXf::Identity(nOld+3, nOld+3);
        S.block(0,0, nOld, nOld) = Sigma;

        S(nOld + 2, nOld + 2) =  config.sigma_rho_0;

		Sigma = Js*S*Js.transpose();
        return 1;
    }
    return 0;
}

void VSlamFilter::removeFeature(int index) {
    // TODO: could be more readable

    Patch p = patches[index];
    int pos = p.position_in_state;

    int first = pos;
	int psize;
	VectorXf feature;
    if(p.isXYZ()){
		psize = 3;
		feature = mu.segment<3>(pos);
	}else{
		psize = 6;
		feature = mu.segment<6>(pos);
	}


    int last = mu.rows() - pos - psize;// read like [-1*last] in python

    // a segment to save good features
    if (p.n_find > 5 && psize==3 ) {
			MatrixXf J;
            p.XYZ_pos = inverseDepth2XyzWorld(feature,J,0);
			Matrix3f Patch_sigma = Matrix3f::Zero(3,3);
			Patch_sigma = Sigma.block<3,3>(pos,pos).transpose();
			for (int i = 0; i < Patch_sigma.size(); i++) {
				p.cov_4_delete(i) = Patch_sigma(i);
				/* code */
			}
	    deleted_patches.push_back(p);// could be used for loop closing
    }
    //end of segment


    if (index != patches.size() - 1) {
        mu = Concat(mu.head(first), mu.tail(last));
        Sigma = vConcat(Sigma.topRows(first), Sigma.bottomRows(last));
        Sigma = hConcat(Sigma.leftCols(first), Sigma.rightCols(last));
    } else {
        mu = mu.head(pos).eval();
        Sigma = Sigma.block(0,0,pos,pos).eval();
    }

    for(int i = index + 1; i < patches.size(); i++) {
    	patches[i].change_position(-psize);
    }
    patches.erase(patches.begin()+index);
}

void VSlamFilter::rts_epoch(VectorXf &MU,MatrixXf &SIGMA,VectorXf MU_S ,MatrixXf SIGMA_S,Vector3f dTspeed,Vector3f dRspeed ,double deltaT){

    //MU is 13 * 13;
    //1. Prediction of Covariance
	MatrixXf Ft_complete = System_model_jacobian( MU, deltaT , dRspeed);

    MatrixXf Qtot;
    MatrixXf SIGMA_P(STATE_DIM,STATE_DIM);
    VectorXf MU_P(STATE_DIM);
    MatrixXf K;

    // it's supposed covariance of accelaration not speed,so convert to speed cov
    Qtot = Ft_complete.middleCols<6>(7)*(Vmax/deltaT/deltaT)*Ft_complete.middleCols<6>(7).transpose();

    SIGMA_P = (Ft_complete*SIGMA*Ft_complete.transpose() + Qtot).eval();

    //2. Prediction of state
    MU_P = Predict_State(MU,dTspeed,dRspeed, deltaT);

    K = SIGMA * Ft_complete.transpose() * (SIGMA_P.inverse());

    //3.update rts:
    MU = MU+ K * (MU_S - MU_P);
    //std::cout << MU << std::endl;
    SIGMA = SIGMA + K*(SIGMA_S - SIGMA_P)*(K.transpose());

}

void VSlamFilter::predict(Vector3f Translation_Speed_Control, Vector3f Rotational_Speed_Control,bool Vcontrol) {

    //1. Prediction of Covariance
	Ft = System_model_jacobian(mu.segment<13>(0), dT , Rotational_Speed_Control);
    const int n = Sigma.cols();

    MatrixXf Ft_complete = MatrixXf::Identity(n,n);
    Ft_complete.block<13,13>(0,0) = Ft;
    MatrixXf Q;

    // it's supposed covariance of accelaration not speed,so convert to speed cov
    
    if (Vcontrol) {
        Q = Ft.middleCols<6>(7)*(Vmax/dT/dT)*Ft.middleCols<6>(7).transpose();
        noise_cov_factor = 0;
    }else{
        noise_cov_factor++;
        //std::cout << "here0" << std::endl;
        //std::cout << pow(1.5,noise_cov_factor)<< std::endl;//2.1624
        //Q = Ft.middleCols<6>(7)*(Vmax*pow(2,noise_cov_factor)/dT/dT)*Ft.middleCols<6>(7).transpose();
        Q = Ft.middleCols<6>(7)*(Vmax_n/dT/dT)*Ft.middleCols<6>(7).transpose();

    }
    
    MatrixXf Qtot = MatrixXf::Zero(n,n);
    Qtot.block<13,13>(0,0) = Q;
    Sigma = (Ft_complete*Sigma*Ft_complete.transpose() + Qtot).eval();

    //2. Prediction of state
    mu.segment<13>(0) = Predict_State(mu.segment<13>(0),Translation_Speed_Control,Rotational_Speed_Control, dT);

    //3. Prediction of Patches
    Vector3f r = mu.segment<3>(0);
    Vector4f q = mu.segment<4>(3);
    Vector3f v = mu.segment<3>(7);
    Vector3f w = mu.segment<3>(10);
    Vector4f h = vec2quat(w);

    map_scale = mu(13);
    //TODO: intialize it to the real map scale
    //TODO: a way to change it

    Vector4f qc = quatComplement(q);
    Matrix3f RotCW = quat2rot(qc);

    // blurring components
    // make different r , and q for blurring
    Matrix3f RotCW_blurring = quat2rot(quatComplement(quatCrossProduct(q,vec2quat(w*T_camera*dT))));
    Vector3f r_blurring = r + v*T_camera*dT;
    Vector2f hi_out_blurred;

    VectorXf hi_out;
    MatrixXf Hit;

   int featurer_counter = 0;
   // predict the new positon of each patch and the position for covariance
   // for matching..
    for (int i = 0; i < patches.size(); i++) {
		int pos = patches[i].position_in_state;
        Hit = MatrixXf::Zero(2, mu.rows());//u,v vs mu

        if (!patches[i].isXYZ()) {

            VectorXf f = mu.segment<6>(pos);

			//check if patch is at inifnity ==> unpredictable
            if (f(5) <= 0) {
            	ROS_ERROR("Feature %d at infinity: %f", i, f(5));
            	patches[i].setRemove();
            	patches[i].setIsInInnovation(false);
            	continue;
            }

            MatrixXf J_hW_f;
            Vector3f d = inverse2XYZ4_projecting(f, r, J_hW_f,true);
            Vector3f hC = RotCW*d;
            MatrixXf J_h_hC;//jacobian to projection function
            hi_out = cam.projectAndDistort(hC, J_h_hC);
            bool InsideImage_InFront_Of_Camera = isInsideImage(hi_out, frame.size(), windowsSize) && hC(2) >= 0 ;
            // is there any need to f(5)? hC>0?? not behind the camera?

			patches[i].setIsInInnovation(InsideImage_InFront_Of_Camera);
            if (!InsideImage_InFront_Of_Camera) continue;
            // messedup with the above if
			// feauter predict ok , ready to match
			featurer_counter++;
            MatrixXf Jacobian_hC_2q = Jacobian_hW_to_qantrion(qc,d)*d_qbar_q();

            Hit.middleCols<3>(0) = -f(5)*J_h_hC*RotCW;
            Hit.middleCols<4>(3) = J_h_hC*Jacobian_hC_2q;
            Hit.middleCols<6>(pos) = J_h_hC*RotCW*J_hW_f;
            patches[i].h = hi_out;
			patches[i].H = Hit;

			// Blurring
			hi_out_blurred = cam.projectAndDistort(RotCW_blurring*inverse2XYZ4_projecting(f,r_blurring, J_hW_f,false), J_h_hC);
            // ok here he multiply with half speed T_camera, so for blurring purposes
			patches[i].blur(hi_out, hi_out_blurred,kernel_min_size );

			//#define TEST_BLURR
			// that would text T_camera value
        } else {
            Vector3f y = mu.segment<3>(pos);

            Vector3f d = y-r;
            Vector3f hC = RotCW*d;
            MatrixXf J_h_hC;//jacobian to projection function
            hi_out = cam.projectAndDistort(hC, J_h_hC);
            bool InsideImage_InFront_Of_Camera = isInsideImage(hi_out, frame.size(), windowsSize) && hC(2) >= 0;

			patches[i].setIsInInnovation(InsideImage_InFront_Of_Camera);
            if (!InsideImage_InFront_Of_Camera) continue;

			featurer_counter++;
            // the Jacobian H as in Update stage page 71;
            MatrixXf Jacobian_hC_2q = Jacobian_hW_to_qantrion(qc,d)*d_qbar_q();

            Hit.middleCols<3>(0) = -J_h_hC*RotCW;
            Hit.middleCols<4>(3) = J_h_hC*Jacobian_hC_2q;
            Hit.middleCols<3>(pos) = J_h_hC*RotCW;
            patches[i].h = hi_out;
			patches[i].H = Hit;

			// Blurring
			hi_out_blurred = cam.projectAndDistort(RotCW_blurring*(y-r_blurring), J_h_hC);
			patches[i].blur(hi_out, hi_out_blurred,kernel_min_size );

        }
    }

    VectorXf temp_h_out;
    MatrixXf temp_Ht;

	for (int i = 0, j = 0; i < patches.size(); i++) {
		if (patches[i].patchIsInInnovation()) {
			temp_h_out = Concat(temp_h_out,patches[i].h);
            // h_out can be omited with modification to drawprediction
			temp_Ht = vConcat(temp_Ht,patches[i].H);
			patches[i].position_in_z = 2*j;
			j++;
		}
	}

    h_out = temp_h_out;
    Ht = temp_Ht;

    if (h_out.rows() > 0) {
        St = (Ht*Sigma*Ht.transpose() + sigma_pixel_2*MatrixXf::Identity(Ht.rows(), Ht.rows())).eval();
		// alogarthem 5.4 inciate there's a M matrix
        // h_out,St global attr to Vslamfilter
        this->drawPrediction();
    }
}

void VSlamFilter::loopUpdate() {

    std::vector<cv::Point2f> lastCorners,newCorners;
    std::vector<uchar> features_found;

    for (int i = 0; i < patches.size(); i++) {
        if (patches[i].patchIsInInnovation()) {
            lastCorners.push_back(patches[i].center);
        }
    }

    if(old_frame.channels()!=1){
        cvtColor(old_frame, old_frame , CV_BGR2GRAY );
    }


    cv::calcOpticalFlowPyrLK(old_frame,frame,
                            lastCorners,newCorners,
                            features_found,
                            cv::noArray(),
                            cv::Size( 25, 25 ), // Search window size
                            2,//pyramid levels
                            cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                                    25,0.03
                                    ));
    int j=0;
    //number of mathces that considerd ok for update
    for (int i = 0; i < patches.size(); i++) {
        if (patches[i].patchIsInInnovation()) {
            if(features_found[j]){
                patches[i].center = newCorners[j];
        		patches[i].z(0) = patches[i].center.x;
        		patches[i].z(1) = patches[i].center.y;
            }else{
                patches[i].center = cv::Point2f(-1,-1);
                //patches[i].founded = false;
                patches[i].setIsInInnovation(false);
            }
            j++;
        }
    }

    VectorXf z_hi;
    MatrixXf H_hi;
    VectorXf h_hi;
	for (int i = 0, j = 0; i < patches.size(); i++) {
		if (patches[i].patchIsInInnovation()) {
			h_hi = Concat(h_hi,  patches[i].h);//so it's already counted H
			H_hi = vConcat(H_hi, patches[i].H);
			z_hi = Concat(z_hi,  patches[i].z);
			patches[i].position_in_z = 2*j;
			j++;
		}
	}

	MatrixXf Sigma_tmp = Sigma;
	VectorXf mu_tmp = mu;

    if (z_hi.rows() > 0) {
    	int p = H_hi.rows();
    	St = H_hi*Sigma_tmp*H_hi.transpose();

    	MatrixXf St_error = sigma_pixel_2*MatrixXf::Identity(p,p);
    	//if (flag_odom) St_error(p-1,p-1) = 0.2;
    	St += St_error;

    	Kt = Sigma_tmp*H_hi.transpose()*St.inverse();
        //lu().solve(Matrix<float,Dynamic,Dynamic>::Identity(p,p));
    	mu_tmp = mu_tmp + Kt*(z_hi - h_hi);
        Sigma_tmp = ((MatrixXf::Identity(Sigma_tmp.rows(),Sigma_tmp.rows()) - Kt*H_hi)*Sigma_tmp).eval();
       	normalizeQuaternion(mu_tmp,Sigma_tmp);
    }

    mu = mu_tmp;
    Sigma = Sigma_tmp;

    //paint
    for (int i = 0; i < patches.size(); i++) {
    	patches[i].drawUpdate(drawedImg, i);
	    //std::cout << drawedImg.size() << std::endl;
    }

    
}

Vector3f VSlamFilter::inverseDepth2XyzWorld(VectorXf f, MatrixXf &J ,int compute_Jacobian) {
    /*
        Compute tha Jacobian either:
        0 : Don't
        1 : Compute it
        2 : auto using linear index bigger than thresh
            to compute or setremove if quality index bigger than another thresh
    */
    if (f.rows()==3) {
        return f;
    }else{
        float lineraty_conversion_threshold = 0.01;
        const float theta = f(3);
        const float phi = f(4);
        const float ro = f(5);

        Vector3f m;
        m(0) = sin(theta)*cos(phi);
        m(1) = -sin(phi);
        m(2) = cos(theta)*cos(phi);

        Vector3f y = f.segment<3>(0) + m/ro;

        if(compute_Jacobian > 1) {
            // calculate L_d : lineraty index:
            Vector3f d = y - mu.segment<3>(0);
			int pos = compute_Jacobian-2;
            float sigma_rho = Sigma(pos+5,pos+5);
            float t = d.transpose()*m;
            float L_d = 4*sigma_rho*abs(t)/(ro*ro*d.squaredNorm());
            compute_Jacobian = (L_d < lineraty_conversion_threshold ?1:0);
        }
        if(compute_Jacobian == 1){
            J = MatrixXf::Zero(3,6);
            J.middleCols<3>(0) = Matrix3f::Identity();

            J(0,3) = cos(theta)*cos(phi)/ro;
            J(1,3) = 0;
            J(2,3) = -sin(theta)*cos(phi)/ro;

            J(0,4) = -sin(theta)*sin(phi)/ro;
            J(1,4) = -cos(phi)/ro;
            J(2,4) = -cos(theta)*sin(phi)/ro;

            J.col(5) = -m/(ro*ro);
        }
        return y;
    }
}


void VSlamFilter::convert2XYZ_ifLinear(int index) {

	int pos = patches[index].position_in_state;
	VectorXf f = mu.segment<6>(pos);
    MatrixXf J_hp_f = MatrixXf::Zero(3,6);
    Vector3f y = inverseDepth2XyzWorld( f, J_hp_f, 2+pos);

    if(J_hp_f(0,0) == 1){
        int first = pos;
        int last = mu.rows() - pos - 3;
        int n = Sigma.cols();

        MatrixXf J = MatrixXf::Zero(n-3,n);
        J.topLeftCorner(pos,pos) = MatrixXf::Identity(pos,pos);
        J.block<3,6>(pos,pos) = J_hp_f;

        if (index != patches.size() - 1) {
        	J.bottomRightCorner(n-pos-6,n-pos-6) = MatrixXf::Identity(n-pos-6,n-pos-6);
            mu = Concat(mu.head(first), mu.tail(last));
            mu.segment<3>(pos) = y;
        }else{
            mu = Concat(mu.head(first), y);
        }

        Sigma = J*Sigma*J.transpose();
        patches[index].convertInXYZ();

        for(int i = index + 1; i < patches.size(); i++) {
        	patches[i].change_position(-3);
        }
   }
}



void VSlamFilter::convert2XYZ_ifLinearAll() {
	for (int i = 0; i < patches.size(); i++) {
		if (!patches[i].isXYZ()) convert2XYZ_ifLinear(i);
	}
}


void VSlamFilter::findNewFeatures(int num) {
	ROS_INFO("Finding new Feature");

	if (num <= 0) num = nInitFeatures;

    int winSize = 2*windowsSize+1;
    cv::Mat mask = cv::Mat(frame.size(),CV_8UC1 );
    mask.setTo(cv::Scalar(0));
	int estrem = windowsSize;
    cv::Mat(mask, cv::Rect(estrem, estrem,mask.size().width-2*estrem, mask.size().height - 2*estrem)).setTo(cv::Scalar(255));

    // black out all the already exiting patches
    for (int i = 0; i < patches.size(); i++) {
    	if (patches[i].center.x > windowsSize &&
    		patches[i].center.y > windowsSize &&
    		patches[i].center.x < mask.size().width - windowsSize &&
    		patches[i].center.y < mask.size().height - windowsSize)
    		{
    		int x = (patches[i].center.x-winSize/2 > 0 ?
    				patches[i].center.x-winSize/2 : 0);
    		int y = (patches[i].center.y-winSize/2 > 0 ?
    				patches[i].center.y-winSize/2 : 0);
    		int width = winSize -
    			(patches[i].center.x+winSize/2 <= frame.size().width ?
    			0 : frame.size().width - patches[i].center.x-winSize/2);

			int height = winSize -
    			(patches[i].center.y+winSize/2 <= frame.size().height ?
    			0 : frame.size().height - patches[i].center.y-winSize/2);

    		cv::Rect roi = cv::Rect(x,y, width, height);
    		cv::Mat(mask, roi).setTo(cv::Scalar(0));

    	}
    }
    std::vector<cv::Point2f> features;
    std::vector<cv::KeyPoint> kpts;
    //what about 0.01f and 5??
    // 5 for minmum dist , maybe TODO: make it 12
    // quality level and mindist
    // TODO: another dector: KASE, ORB , FAST
    // cv::sum(mask)
    /*akaze->detect(frame, kpts, mask);
    std::cout << "here 2 " << std::endl;
    for (int i = 0; i < kpts.size(); i++) {
	    //cv::Point2f newFeature = cv::Point2f(features[i].x, features[i].y);
        this->addFeature(kpts[i].pt);
    }*/
    goodFeaturesToTrack(frame,features,num,0.01f,12,mask);
    for (int i = 0; i < features.size(); i++) {
	    cv::Point2f newFeature = cv::Point2f(features[i].x, features[i].y);
        this->addFeature(newFeature);
    }

	ROS_DEBUG("Found %d Feature[s]", features.size());

}

float VSlamFilter::Covariance_Parameter(){
	float P_cov_all = 0;
    /*
	int XYZ_Points=0;
	for (int i = 0; i < patches.size(); i++) {
		if(patches[i].patchIsInInnovation() ){
			if( patches[i].isXYZ()){
				XYZ_Points++;
			}
		}
	}
    */
	// Adding Camera Covariance in Position
	P_cov_all += Sigma(0,0)+Sigma(1,1)+Sigma(2,2);
	P_cov_all += Sigma(4,4)+Sigma(5,5)+Sigma(6,6)+Sigma(3,3);
    /*
	int Number_of_min_XYZ_Ps = 0;
	if (XYZ_Points> Number_of_min_XYZ_Ps){
        //std::cout <<"XYZn: "<< XYZ_Points << " cov: "<<P_cov_all << std::endl; 
		return P_cov_all;// /(XYZ_Points);
	}else{
		return  P_cov_all;//*2;//1000000;
	}
    */
    return P_cov_all;
}

void VSlamFilter::update(float v_x, float w_z) {

    int matchedFeatures = 0;

    std::vector<Patch> patches_copy = patches;
    for (int i = 0; i < patches.size(); i++) {
        if (patches[i].patchIsInInnovation()) {
            patches[i].findMatch(frame, St.block<2,2>(patches[i].position_in_z,patches[i].position_in_z),sigma_size, false);
            if (patches[i].patchIsInInnovation()){
                matchedFeatures++;
            }
        }
    }
    //number of mathces that considerd ok for update
    int okmatches = 6;
    /*
    if (matchedFeatures>okmatches){
        // save last ok img and the patches as in the vector for it
        lastOkImg = frame.clone();
        //TODO: earse lastImgPatches
        lastImgPatches.clear();
        //lastImgPatches taken
        //fix z values
        for (int i = 0; i < patches.size(); i++) {
            if(patches[i].patchIsInInnovation()){
                lastImgPatches.push_back(patches[i].real_index);
                patches[i].zLastImg(0) = patches[i].center.x;
                patches[i].zLastImg(1) = patches[i].center.y;
            }
        }

    } else if (lastImgPatches.size()>okmatches){
        std::vector<cv::Point2f> lastCorners,newCorners;
        std::vector<int> lastIndx;
        std::vector<uchar> features_found;
        for(int i=0; i<patches.size(); i++){
            if (std::find(lastImgPatches.begin(),lastImgPatches.end(),patches[i].real_index)!=lastImgPatches.end()){
                if(!patches[i].patchIsInInnovation()){
                lastCorners.push_back(cv::Point2f(patches[i].zLastImg(0),patches[i].zLastImg(1)));
                lastIndx.push_back(i);
                }
            }
        }

        if(lastCorners.size()>2) {

        cv::calcOpticalFlowPyrLK(lastOkImg,frame,
                                lastCorners,newCorners,
                                features_found,
                                cv::noArray(),
                                cv::Size( 25, 25 ), // Search window size
                                3,//pyramid levels
                                cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                                        15,0.3
                                        ));

        int matchPs = 0;
        for(int j; j<features_found.size(); j++){
            if(features_found[j])matchPs++;
            }
        //if matches less than 5 (too mush speed), stay on last

        if(matchPs>4){
            lastImgPatches.clear();
            for(int i; i<lastIndx.size(); i++){
                if(!features_found[i])
                {continue;}
                //TODO as in find matches
                patches[lastIndx[i]].center   = newCorners[i];//copy?
                patches[lastIndx[i]].z(0)     = newCorners[i].x;
                patches[lastIndx[i]].z(1)     = newCorners[i].y;
                patches[lastIndx[i]].zLastImg = patches[lastIndx[i]].z;
                //TODO write lastpatches
                lastImgPatches.push_back(patches[lastIndx[i]].real_index);
            }
            //adding the correlation matches
            for(int i=0; i<patches.size(); i++){
                if (std::find(lastIndx.begin(),lastIndx.end(),i)!=lastIndx.end()){
                    patches[i].setIsInInnovation(true);
                }else{
                    if(patches[i].patchIsInInnovation()){
                        lastImgPatches.push_back(patches[i].real_index);
                        patches[i].zLastImg = patches[i].z;
                    }
                }
            }
            lastOkImg = frame.clone();

        }
        // make sparese opt flow with last ok img
        // make changes to patches as in findmatches
        // save last ok img if matches found ,or don't
        }
    }
    */

#ifdef USE_RANSAC

	int nhyp = 10000;
	float p = 0.99;// propality
	float th_low_innovation_inliers = 2*sigma_pixel;

	srand(time(NULL));

	int num_zli = 0;

	std::vector<int> ransacindex;


	for (int i = 0; i < patches.size(); i++) {
		if (patches[i].patchIsInInnovation()){
			// Innovation after correct matching
            ransacindex.push_back(i);
        }
	}

    matchedFeatures = ransacindex.size();

	for (int i = 0; i < nhyp && ransacindex.size() > 0; i++) {
		int actual_num_zli = 0;

		int posRansac = rand()%ransacindex.size();//random feature
		int selectPatch = ransacindex[posRansac];
		ransacindex.erase(ransacindex.begin() + posRansac);

        // one update with one point , hence the name ,only mu
		MatrixXf S_i = patches[selectPatch].H*Sigma*patches[selectPatch].H.transpose() + sigma_pixel_2*MatrixXf::Identity(patches[selectPatch].H.rows(), patches[selectPatch].H.rows());
        MatrixXf K_i = Sigma*patches[selectPatch].H.transpose()*S_i.inverse();
		VectorXf mu_i = mu + K_i*(patches[selectPatch].z - patches[selectPatch].h);

		Vector3f r = mu_i.segment<3>(0);
		Vector4f q = mu_i.segment<4>(3)/mu_i.segment<4>(3).norm();//normlaize to 1 len
    	Matrix3f RotCW = quat2rot(quatComplement(q));

        //check for consest set
    	int searched_features = 0;
		for (int i = 0; i < patches.size(); i++) {
            // is it quicker if we use a clone to ransacindex!
			if (!patches[i].patchIsInInnovation()) continue;
			Vector2f hi_i;
			int pos = patches[i].position_in_state;

			if (!patches[i].isXYZ()) {
				VectorXf f = mu_i.segment<6>(pos);
                MatrixXf J;
				Vector3f d = inverse2XYZ4_projecting(f, r , J,false);
				hi_i = cam.projectAndDistort(RotCW*d);
			} else {
	            Vector3f y = mu.segment<3>(pos);
	            Vector3f d = y-r;
	            Vector3f hC = RotCW*d;
	            hi_i = cam.projectAndDistort(hC);
			}

			patches[i].setIsInLi((patches[i].z - hi_i).norm() <= th_low_innovation_inliers);
			if (patches[i].patchIsInLi()) actual_num_zli++;//misnaming again
			searched_features++;//why count if not used?

    	}

    	if (actual_num_zli > num_zli) {
    		num_zli = actual_num_zli;
    		nhyp = log(1-p)/(log(1-(num_zli/(matchedFeatures+0.0f))));//what lib for log? changed here[YY]
            //std::cout << "Thats the number of hyposese for 1p-ransac line 726 : " << nhyp << std::endl;
            // before confirm, sound useless as the last is correct so , no need , just ckeck isInLi
    	}
	}//end of low-innovation search

	VectorXf z_li;
    MatrixXf H_li;
    VectorXf h_li;

	for (int i = 0, j = 0; i < patches.size(); i++) {
		if (patches[i].patchIsInLi()) {
			h_li = Concat(h_li,patches[i].h);
			H_li = vConcat(H_li,patches[i].H);
			z_li = Concat(z_li, patches[i].z);
			patches[i].position_in_z = 2*j;
			j++;
		}
	}

	MatrixXf Sigma_tmp = Sigma;
	VectorXf mu_tmp = mu;
    // one update for all low innovation inliers points
    if (z_li.rows() > 0) {
    	int p = H_li.rows();
    	St = H_li*Sigma_tmp*H_li.transpose() + sigma_pixel_2*MatrixXf::Identity(p,p);

    	Kt = Sigma_tmp*H_li.transpose()*St.inverse();//

    	mu_tmp = mu + Kt*(z_li - h_li);
    	Sigma_tmp = ((MatrixXf::Identity(Sigma_tmp.rows(),Sigma_tmp.rows()) - Kt*H_li)*Sigma_tmp).eval();
       	normalizeQuaternion(mu_tmp,Sigma_tmp);
    } else {
        ROS_ERROR("No Matching li");
    }
    //////////////////////////////////////////////////////////
	float th_high_innovasion_inliers = 1;// for high innovation
    // maybe 2 is an option

    Vector3f r = mu.segment<3>(0);
    Vector4f q = mu.segment<4>(3);
    Vector4f qc = quatComplement(q);
    Matrix3f RotCW = quat2rot(qc);
    Matrix2f S_hi;
    Vector2f hi_hi;

    // why recalcluating H here?, what the difference of the offical predict mode
    // is patches[i].h the same, shouldn't we use mu_temp instead of mu!?
    // cahnge mu ==> mu_tmp
    int counter = 0;
    for (int i = 0; i < patches.size(); i++) {
        if (!patches[i].patchIsInLi() && patches[i].patchIsInInnovation()) {
            int pos = patches[i].position_in_state;
            if (!patches[i].isXYZ()) {
                MatrixXf Hi_hi = MatrixXf::Zero(2, mu_tmp.rows());
                VectorXf f = mu_tmp.segment<6>(pos);
                MatrixXf J_hp_f;
                Vector3f d = inverse2XYZ4_projecting(f, r, J_hp_f, true);
                MatrixXf J_hf_hi;
                patches[i].h = cam.projectAndDistort(RotCW*d, J_hf_hi);

                MatrixXf d_h_q = Jacobian_hW_to_qantrion(qc,d)*d_qbar_q();
                Hi_hi.middleCols<3>(0) = -f(5)*J_hf_hi*RotCW;
                Hi_hi.middleCols<4>(3) = J_hf_hi*d_h_q;
                Hi_hi.middleCols<6>(pos) = J_hf_hi*RotCW*J_hp_f;
                patches[i].H = Hi_hi;

            } else {
                MatrixXf Hi_hi = MatrixXf::Zero(2, mu_tmp.rows());
                Vector3f y = mu_tmp.segment<3>(pos);
                Vector3f d = y-r;
                Vector3f hC = RotCW*d;
                MatrixXf J_hf_hC;
                patches[i].h = cam.projectAndDistort(hC, J_hf_hC);

                MatrixXf d_h_q = Jacobian_hW_to_qantrion(qc,d)*d_qbar_q();

                Hi_hi.middleCols<3>(0) = -J_hf_hC*RotCW;
                Hi_hi.middleCols<4>(3) = J_hf_hC*d_h_q;
                Hi_hi.middleCols<3>(pos) = J_hf_hC*RotCW;
                patches[i].H = Hi_hi;

            }
            S_hi = patches[i].H*Sigma_tmp*patches[i].H.transpose();// 4 the new covaruance
            patches[i].setIsInHi((patches[i].h - patches[i].z).transpose()*S_hi.inverse()*(patches[i].h - patches[i].z) <= th_high_innovasion_inliers);
            counter++;//no need
        }
    }

    VectorXf z_hi;
    MatrixXf H_hi;
    VectorXf h_hi;
	for (int i = 0, j = 0; i < patches.size(); i++) {
		if (patches[i].patchIsInHi()) {
			h_hi = Concat(h_hi,  patches[i].h);
			H_hi = vConcat(H_hi, patches[i].H);
			z_hi = Concat(z_hi,  patches[i].z);
			patches[i].position_in_z = 2*j;
			j++;
		}
	}
    //here test if ransac give many points or opt-flow

    /*if(z_li.rows() + z_hi.rows() <8 ){
        //out
        //std::cout << "1-p ransac failed switching to opt-flow" << std::endl;
        patches = patches_copy;
        std::vector<cv::Point2f> lastCorners,newCorners,backlast;
        std::vector<uchar> features_found;

        for (int i = 0; i < patches.size(); i++) {
            if (patches[i].patchIsInInnovation()) {
                lastCorners.push_back(patches[i].center);
            }
        }

        if(old_frame.channels()!=1){
            cvtColor(old_frame, old_frame , CV_BGR2GRAY );
        }

        cv::calcOpticalFlowPyrLK(old_frame,frame,
                            lastCorners,newCorners,
                            features_found,
                            cv::noArray(),
                            cv::Size( 10, 10 ), // Search window size
                            3,//pyramid levels
                            cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                                    15,0.03
                                    ));

        cv::calcOpticalFlowPyrLK(frame,old_frame,
                            newCorners,backlast,
                            features_found,
                            cv::noArray(),
                            cv::Size( 10, 10 ), // Search window size
                            3,//pyramid levels
                            cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                                    15,0.03
                                    ));

        float back_thresh = 1.2;
        int j=0,k=0;
        //number of mathces that considerd ok for update
        for (int i = 0; i < patches.size(); i++) {
            if (patches[i].patchIsInInnovation()) {
                if(cv::norm(lastCorners[j] - backlast[j])<back_thresh){
                    patches[i].center = newCorners[j];
            		patches[i].z(0) = patches[i].center.x;
            		patches[i].z(1) = patches[i].center.y;
                    k++;
                }else{
                    patches[i].center = cv::Point2f(-1,-1);
                    //patches[i].founded = false;
                    patches[i].setIsInInnovation(false);
                }
                j++;
            }
        }*/
        /*int j=0,k=0;
        //number of mathces that considerd ok for update
        for (int i = 0; i < patches.size(); i++) {
            if (patches[i].patchIsInInnovation()) {
                if(features_found[j]){
                    patches[i].center = newCorners[j];
            		patches[i].z(0) = patches[i].center.x;
            		patches[i].z(1) = patches[i].center.y;
                    k++;
                }else{
                    patches[i].center = cv::Point2f(-1,-1);
                    //patches[i].founded = false;
                    patches[i].setIsInInnovation(false);
                }
                j++;
            }
        }*/
        //std::cout << "numof flow pnts " << k << std::endl;
        /*
        VectorXf z_hi;
        MatrixXf H_hi;
        VectorXf h_hi;
	    for (int i = 0, j = 0; i < patches.size(); i++) {
	    	if (patches[i].patchIsInInnovation()) {
	    		h_hi = Concat(h_hi,  patches[i].h);//so it's already counted H
	    		H_hi = vConcat(H_hi, patches[i].H);
	    		z_hi = Concat(z_hi,  patches[i].z);
	    		patches[i].position_in_z = 2*j;
	    		j++;
	    	}
	    }

    	MatrixXf Sigma_tmp = Sigma;
    	VectorXf mu_tmp = mu;
        
    }*/

#endif


#ifndef USE_RANSAC
    VectorXf z_hi;
    MatrixXf H_hi;
    VectorXf h_hi;
	for (int i = 0, j = 0; i < patches.size(); i++) {
		if (patches[i].patchIsInInnovation()) {
			h_hi = Concat(h_hi,  patches[i].h);//so it's already counted H
			H_hi = vConcat(H_hi, patches[i].H);
			z_hi = Concat(z_hi,  patches[i].z);
			patches[i].position_in_z = 2*j;
			j++;
		}
	}
#endif



	bool flag_correction = false;
	Matrix3f corr_ass_sigma = 0.00001*Matrix3f::Identity();
// what forsPlane do exactly ... defult zero
// seems adding 3pt data to z and H , and covariance to small value ,
// why?
	if (config.forsePlane) {
		Vector3f corr_ass = Vector3f::Zero();
		MatrixXf corr_ass_H = MatrixXf::Zero(3,mu_tmp.size());
		corr_ass_H(0,1) = 1;
		corr_ass_H(1,4) = 1;
		corr_ass_H(2,6) = 1;

		Vector3f h_corr_ass;
		h_corr_ass << mu_tmp(1), mu_tmp(4),  mu_tmp(6);
		h_hi = Concat(h_hi,  h_corr_ass);
		z_hi = Concat(z_hi,  corr_ass);
		H_hi = vConcat(H_hi, corr_ass_H);
		flag_correction = true;
	}


    if (z_hi.rows() > 0) {
    	int p = H_hi.rows();
    	St = H_hi*Sigma_tmp*H_hi.transpose();

    	MatrixXf St_error = sigma_pixel_2*MatrixXf::Identity(p,p);
    	//if (flag_odom) St_error(p-1,p-1) = 0.2;
    	if (flag_correction){St_error.block<3,3>(p-3,p-3) = corr_ass_sigma;}

    	St += St_error;

    	Kt = Sigma_tmp*H_hi.transpose()*St.inverse();
        //lu().solve(Matrix<float,Dynamic,Dynamic>::Identity(p,p));
    	mu_tmp = mu_tmp + Kt*(z_hi - h_hi);
        Sigma_tmp = ((MatrixXf::Identity(Sigma_tmp.rows(),Sigma_tmp.rows()) - Kt*H_hi)*Sigma_tmp).eval();
       	normalizeQuaternion(mu_tmp,Sigma_tmp);
    }

    mu = mu_tmp;
    Sigma = Sigma_tmp;
	//**********

	//*********
    //paint
    for (int i = 0; i < patches.size(); i++) {
    	patches[i].drawUpdate(drawedImg, i);
	//std::cout << drawedImg.size() << std::endl;
    }

    //delete bad patches
    // why not compyte the qualityindex in the loop above??
	for (int i = patches.size()-1; i >= 0; --i){
		patches[i].update_quality_index();
		if (patches[i].mustBeRemove()) this->removeFeature(i);
	}

    int nVisibleFeature = 0;// maybe it's better to make it attr to the class
    //also with the same loop
    for (int i = 0; i < patches.size(); i++) if (patches[i].patchIsInInnovation()) nVisibleFeature++;

    //std::cout << "matches after ransac: "<< std::endl;
    //std::cout << nVisibleFeature << std::endl;
    // good, rename to min_visiblae_features and
    // maybe group into near memory patches and far memory patches
    //how about adding features to fixed number always! TODO
    // config desired_number of feauters

    if (nVisibleFeature < config.min_features) {
    	if (patches.size() > config.max_features) this->removeFeature(0);// must be outside if ?
    	this->findNewFeatures(config.min_features-nVisibleFeature);
    }
// Finally Convert whatever to XYZ
	convert2XYZ_ifLinearAll();
    /*
    Point4sba = MatrixX3i::Zero(1,3) ;//and ignore first row
    RowVector3i np ;
    for (int i = 0; i < patches.size(); i++){
      if (patches[i].patchIsInInnovation() && patches[i].isXYZ()){
	      if(Point4sba(0)==0){
					Point4sba <<patches[i].real_index,patches[i].z(0),patches[i].z(1);
		  	}else{
					np(0) = patches[i].real_index;
					np(1) = 640; np(1) = patches[i].z(0);
					np(2) = 480; np(2) = patches[i].z(1);
					//std::cout << "np: " << np<< std::endl;
					if (np(1)<640 && np(2)<480 ){
							//std::cout << "Point4sba: " << Point4sba<< std::endl;
		    			MatrixX3i concatMatrix(Point4sba.rows()+1,3);
		    			concatMatrix << Point4sba,np;
							//std::cout << "concatMatrix: " << concatMatrix<< std::endl;
		    			Point4sba = concatMatrix;
					}
		  	}
      }
    }
    */
}


void VSlamFilter::drawUpdate(cv::Point f) {
    //blue square
    cv::rectangle(drawedImg, cv::Point(f.x - windowsSize/2, f.y - windowsSize/2), cv::Point(f.x + windowsSize/2, f.y + windowsSize/2), CV_RGB(0,0,255), 1);
    cv::circle(drawedImg, f, 2, CV_RGB(0,0,255),2);
}

void VSlamFilter::drawPrediction() {

    MatrixXi sigma_mis = computeEllipsoidParameters(St, sigma_size);
    // St and h_out form the outer scope , .. predict
    for (int i = 0; i < St.rows()/2; i++) {
        cv::Point2i predictFeature(h_out(2*i), h_out(2*i+1));
        cv::ellipse(drawedImg, predictFeature, cv::Size(sigma_mis(i,0),sigma_mis(i,1)), (double)sigma_mis(i,2), 0, 360, CV_RGB(32, 32, 255));
    }
}

cv::Mat VSlamFilter::returnImageDrawed() {
    return drawedImg.clone();
}

cv::Mat VSlamFilter::returnGrayImg() {
    return frame.clone();
}

MatrixXi computeEllipsoidParameters(MatrixXf St, int sigma_size) {
    int nFeatures = St.cols()/2;// what St looks like . cout?
    // number of points
    MatrixXi sigma_mis(nFeatures,3);
    for (int i = 0; i < nFeatures; i++) {
        SelfAdjointEigenSolver<MatrixXf> eigenSolver(St.block<2,2>(2*i,2*i));
        Vector2f eigs = eigenSolver.eigenvalues();
        Matrix2f vecs = eigenSolver.eigenvectors();

        sigma_mis(i,0) = (eigs(0) > 0 ? (int)sigma_size*sqrt((double)eigs(0)) : 1);
        sigma_mis(i,1) = (eigs(1) > 0 ? (int)sigma_size*sqrt((double)eigs(1)) : 1);
        sigma_mis(i,2) = (int)(180/3.14*atan2((double)vecs(1,0), (double)vecs(0,0)));
    }
    return sigma_mis;
}

/*************************/
/****helpping functions***/
/*************************/

Vector4f vec2quat(Vector3f vec) {
    float alpha = vec.norm();

    if (alpha != 0) {
        Vector4f quat;
        quat(0) = cos(alpha/2);
        quat.segment<3>(1) = vec*sin(alpha/2)/alpha;
        return quat;
    }
    else {
        return quatZero();
    }
}

Vector4f quatZero() {
    Vector4f quat;
    quat << 1,0,0,0;
    return quat;
}

Matrix3f quat2rot(Vector4f quat) {
    float qr = quat(0);
    float qi = quat(1);
    float qj = quat(2);
    float qk = quat(3);

    Matrix3f rot;

	rot <<	qr*qr + qi*qi - qj*qj - qk*qk, 		-2*qr*qk+2*qi*qj,					 2*qr*qj+2*qi*qk,
		    2*qr*qk+2*qi*qj,				    qr*qr - qi*qi + qj*qj - qk*qk,		 -2*qr*qi+2*qj*qk,
		    -2*qr*qj+2*qi*qk,				    2*qr*qi+2*qj*qk,					 qr*qr - qi*qi - qj*qj + qk*qk;

    return rot;
}

Matrix4f YupsilonMatric(Vector4f q1) {
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


Matrix4f YupsilonMatricComplementar(Vector4f q1) {
	Matrix4f res;
    const float r1=q1(0);
    const float x1=q1(1);
    const float y1=q1(2);
   	const float z1=q1(3);

	res <<  r1, -x1, -y1, -z1,
		    x1,  r1,  z1, -y1,
		    y1, -z1,  r1,  x1,
		    z1,  y1, -x1,  r1;

	return res;

}

Vector4f quatCrossProduct(Vector4f q1, Vector4f q2) {

    return YupsilonMatric(q1)*q2;
}

Vector3f inverse2XYZ4_projecting(VectorXf f, Vector3f r, MatrixXf &J_hp_f,bool give_J) {

    const float theta = f(3);
    const float phi = f(4);
    const float ro = f(5);

    Vector3f m;
    m(0) = sin(theta)*cos(phi);
    m(1) = -sin(phi);
    m(2) = cos(theta)*cos(phi);

    if(give_J){
        J_hp_f = MatrixXf(3,6);
        J_hp_f.middleCols<3>(0) = ro*Matrix3f::Identity();

        J_hp_f(0,3) = cos(theta)*cos(phi);
        J_hp_f(1,3) = 0;
        J_hp_f(2,3) = -sin(theta)*cos(phi);

        J_hp_f(0,4) = -sin(theta)*sin(phi);
        J_hp_f(1,4) = -cos(phi);
        J_hp_f(2,4) = -cos(theta)*sin(phi);

        J_hp_f.col(5) = f.segment<3>(0)-r;
    }

    return ro*(f.segment<3>(0)-r) + m;
}

// return Jacobian d(g(x))/dx
MatrixXf System_model_jacobian(VectorXf mu,double dT ,Vector3f Rotational_Speed_Control) {
	Vector3f r = mu.segment<3>(0);
    Vector4f q = mu.segment<4>(3);
    Vector3f v = mu.segment<3>(7);
    Vector3f w = mu.segment<3>(10);
    Vector4f h = vec2quat(dT*(w + Rotational_Speed_Control)); // h = quat(w)

	MatrixXf Ft;
    Ft = MatrixXf::Identity(13,13);
    Ft.block<4,4>(3,3) = Jacobian_qt_qt1(h);
    Ft.block<4,3>(3,10) = Jacobian_qt_w(q,w + Rotational_Speed_Control,dT);

	Ft.block<3,3>(0,7) = dT*MatrixXf::Identity(3,3);

	return Ft;
}

// compute the Jacobian between q(t) and q(t-1) knowing the rotation h = quat(w*T)
Matrix4f Jacobian_qt_qt1(Vector4f h) {

    return YupsilonMatricComplementar(h);
}

// compute the Jacobian between q(t) and w(t-1) knowing the actual rotation q, w and dT
MatrixXf Jacobian_qt_w(Vector4f q, Vector3f w, double dT) {

    const float n = w.norm();
    const float s = sin(dT*n/2);
    const float c = cos(dT*n/2);
    const float Sinc = (n == 0 ? 1: 2*sin(dT*n/2)/(dT*n));

    Vector3f n_w;
    if (n > 0) n_w = w/n;

    Matrix4f t1;

	t1 = YupsilonMatric(q);
    // relation 5.28
    MatrixXf t2 = MatrixXf::Zero(4,3);
    t2.row(0) = -dT*0.5*s*n_w.transpose();
    t2.middleRows<3>(1) = dT*0.5*(Sinc*Matrix3f::Identity() + (c-Sinc)*n_w*n_w.transpose());

    return t1*t2;
}

Matrix3f diffQuat2rot(Vector4f quat, int index) {
    Matrix3f dR;
    const float q0_2 = 2*quat(0);
    const float qx_2 = 2*quat(1);
    const float qy_2 = 2*quat(2);
    const float qz_2 = 2*quat(3);

    if (index == 0) {
        dR << 	 q0_2, 	-qz_2,	 qy_2,
        		 qz_2,	 q0_2,	-qx_2,
        		-qy_2,	 qx_2,	 q0_2;

    } else if (index == 1) {
        dR << 	qx_2, 	 qy_2, 	 qz_2,
        		qy_2,  	-qx_2, 	-q0_2,
        		qz_2,	 q0_2,	-qx_2;

    } else if (index == 2) {
        dR <<	-qy_2,     qx_2,      q0_2,
        		 qx_2,     qy_2,      qz_2,
        		-q0_2,     qz_2,     -qy_2;


    } else if (index == 3) {
        dR <<	-qz_2,		-q0_2, 	qx_2,
        		 q0_2,      -qz_2,  qy_2,
        		 qx_2,       qy_2,  qz_2;
    }
    return dR;
}

Vector4f quatComplement(Vector4f quat) {
    Vector4f qC;
    qC << quat(0), -quat(1), -quat(2), -quat(3);
    return qC;
}

// state camera Update
VectorXf Predict_State(VectorXf Xv,Vector3f T_speed,Vector3f R_speed, double dT){

	Vector3f v = Xv.segment<3>(7);
	Vector3f w = Xv.segment<3>(10);

    v = v + T_speed;
    w = w + R_speed;// in radian.s-1

    Xv.segment<3>(0) += dT*v;
    Xv.segment<4>(3) = quatCrossProduct( Xv.segment<4>(3),vec2quat(dT*w));

    Xv.segment<3>(7) = v;
	Xv.segment<3>(10) = w;
    return Xv;
}

Matrix4f d_qbar_q() {
    Matrix4f J = -Matrix4f::Identity();
    J(0,0) = 1;
    return J;
}


// Compute the Jacobian df/dhW
MatrixXf Jacobain_inv_feature_to_hW(Vector3f hW) {
    const float hx = hW(0);
    const float hy = hW(1);
    const float hz = hW(2);

    // d_Theta_hW
    MatrixXf J_Theta_hW(1,3);
    float normal = hx*hx+hz*hz;
    J_Theta_hW(0,0) = hz/normal;
    J_Theta_hW(0,1) = 0;
    J_Theta_hW(0,2) = -hx/normal;

    // d_Phi_hW
    float normal2 = hx*hx+hy*hy+hz*hz;
    MatrixXf J_Phi_hW(1,3);
    J_Phi_hW(0,0) = hx*hy/sqrt(normal)/normal2;
    J_Phi_hW(0,1) = -sqrt(normal)/normal2;
    J_Phi_hW(0,2) = hz*hy/sqrt(normal)/normal2;

    MatrixXf J_f_hW = MatrixXf::Zero(6,3);
    J_f_hW.row(3) = J_Theta_hW;
    J_f_hW.row(4) = J_Phi_hW;

    return J_f_hW;
}

void normalizeQuaternion(VectorXf &mu, MatrixXf &Sigma) {
    Vector4f q = mu.segment<4>(3);

    const float norma = q.norm();
    mu.segment<4>(3) = q/norma;

    MatrixXf Q;
    // realation 5.52

    Q = norma*norma*Matrix4f::Identity() - q*q.transpose();

    Q = (Q*(1/(norma*norma*norma))).eval();

    MatrixXf Qc = MatrixXf::Identity(Sigma.rows(), Sigma.cols());
    Qc.block<4,4>(3,3) = Q;

	Sigma = (Qc*Sigma*Qc.transpose()).eval();
}

bool isInsideImage(Vector2f hi, cv::Size size, int windowsSize) {
    float i = hi(0);
    float j = hi(1);

    if (i > windowsSize/2 && j > windowsSize/2 && i < size.width - windowsSize/2 && j < size.height - windowsSize/2) {
        return true;
    }
    return false;
}

MatrixXf Jacobian_hW_to_qantrion(Vector4f q, Vector3f d) {
    //compute hC/qk Jacobian
    MatrixXf diff_Rq_times_dq(3,4);
    for (int j = 0; j < 4; j++) {
    	diff_Rq_times_dq.col(j) = diffQuat2rot(q,j)*d;
    }
    return diff_Rq_times_dq;
}

//Dock node and Filter
#include "ff_estimate/base_mocap_estimator.hpp"


#include <Eigen/Dense>
#include "ff_estimate/base_mocap_estimator.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using ff_msgs::msg::FreeFlyerState;
using ff_msgs::msg::Pose2DStamped;
using ff_msgs::msg::Pose2D;

using namespace std;

class ConstVelKalmanFilterNode : public ff::BaseMocapEstimator {

    public:
	using Qmatrix = Eigen::Matrix<double, 6, 6>;
	using Rmatrix Eigen::Matrix3d;
	
        ConstVelKalmanFilterNode() : ff::BaseMocapEstimator("const_vel_kalman_filter_node") {
            this->declare_parameter("min_dt", 0.005);
		P.setIdentity();
		Q =  (Qmatrix() << 1e-5, 0, 0, 0, 0, 0,  //Process Noise Covariance Matrix
				  0, 1e-5, 0, 0, 0, 0,
	          		  0, 0, 1e-5, 0, 0, 0,
			 	  0, 0, 0, 1e-3, 0, 0,
				  0, 0, 0, 0, 1e-3, 0,
				  0, 0, 0, 0, 0, 1e-3
		).finished();

		R = (Rmatrix() <<    2.4445e-3,   0.0    ,   0.0    ,   0.0   ,   0.0    ,   0.0    ,  
	        			0.0   , 1.2527e-3,   0.0    ,   0.0   ,   0.0    ,   0.0    ,
	        			0.0   ,   0.0    , 4.0482e-3,   0.0   ,   0.0    ,   0.0    ,
					0.0   ,   0.0    ,   0.0    ,   0.0   ,   0.0    ,   0.0    ,
					0.0   ,   0.0    ,   0.0    ,   0.0   ,   0.0    ,   0.0    ,
					0.0   ,   0.0    ,   0.0    ,   0.0   ,   0.0    ,   0.0    
		    ).finished();
		
		flag = 0;
        }

        void EstimatewithPose2D(const Pose2DStamped & pose_stamped) override {

	FreeFlyerState state{};
	//initial declaration cycle
	if (flag == 0) {
		
		Eigen::Vector3d pose = Eigen::Map<Eigen::Vector3d>(pose_stamped.pose, 3);
		x.block<3, 1>(0,0) = pose;
		flag++;
	} else {
		Eigen::Vector3d pose = Eigen::Map<Eigen::Vector3d>(pose_stamped.pose, 3);
		prev_.header = pose_stamped.header;
		const rclcpp::Time now = pose_stamped.header.stamp;
            	const rclcpp::Time last = prev_.header.stamp;
           	double dt = (now - last).seconds();
		process_update(dt);
		measurement_update(pose);
		state.pose.x = x.block<1, 1>(0,0);
		state.pose.y = x.block<1, 1>(1,0);
		state.pose.theta = x.block<1, 1>(2,0);
		SendStateEstimate(state);
		
	}
         //R = Eigen::Matrix<3,3>::Identity(3, 3) * 2.4445e-3, 1.2527e-3, 4.0482e-3;
        
        

	
	/* In order to use new state prediction, velocity must be discovered, and because there is constant vel
 	   We can do (current state  - previous state)/ change in time
     	
        if (prev_state_ready_) {
            const rclcpp::Time now = pose_stamped.header.stamp;
            const rclcpp::Time last = prev_.header.stamp;
            double dt = (now - last).seconds();

            if (dt < (this->get_parameter("min_dt").as_double())) {
                return;
            }
			
	    double vx = (pose_stamped.pose.x - prev_.state.pose.x) / dt;
	    double vy = (pose_stamped.pose.y - prev_.state.pose.y) / dt;
			
			// wrap angle delta to [-pi, pi]
      	    double dtheta = std::remainder(pose_stamped.pose.theta - prev_.state.pose.theta, 2 * M_PI);
      	    double wz = dtheta / dt;
			
	        //estimated velocities for naive state estimation
	        Eigen::Vector3d vel(vx, vy, wz);

	        //get position vector from pose_stamped where vector is pose
	        //not sure if this is how to get the positions into an Eigen vector
	        Eigen::Vector3d pose = Eigen::Map<Eigen::Vector3d>(pose_stamped.pose, 3);

			//combine position vector and velocity vector for initial state vector
			Eigen::Matrix<double, 6, 1> x  = Eigen::Matrix<double, 6, 1>::Zero(6, 1);
			x.block<3, 1>(3,0) = vel;
			x.block<3, 1>(0,0) = pose;

		
				

        } else {
            prev_state_ready_ = true;
        }

        prev_.state = state;
        prev_.header = pose_stamped.header;
	*/
        //SendStateEstimate(state);
    }


        void process_update(double dt) {
            if (dt <= 0.) {
                return;
            }
	    /*A matrix used for prediction of next state matrix -- just for factoring in naive estimation into the 
     		next position state matrix
	    	Format: { 
      		{ 1 0 0 d 0 0}
	 	{ 0 1 0 0 d 0}
    		{ 0 0 1 0 0 d}
       		{ 0 0 0 1 0 0}
		{ 0 0 0 0 1 0}
   		{ 0 0 0 0 0 1}
     			}
			where d = dt 
     		*/
            Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity(6, 6);
            A.block<3, 3>(0, 3).setIdentity() * dt;

	//[6 x 6] * [6 x 1] = [6 x 1]
            x = A * x;
	//  P = [6 x 6] * [6 x 6] * [6 x 6] + [6 x 6]
            P = A * P * A.transpose() + Q * dt;
        }

        void measurement_update(Eigen::VectorXd z) {
	    /*H matrix 
	    	Format:{ 
      			{ 1 0 0 0 0 0}
	 		{ 0 1 0 0 0 0}
    			{ 0 0 1 0 0 0}
     			}
     		*/
            Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero(3, 6);
            H.block<3, 3>(0, 0).setIdentity();
	    /* S is divisor for Kalman Gain separated to be able to use inverse function
     	       H    *    P    *  H.inv     +    R
	    [3 x 6] * [6 x 6] * [6 x 3]  + [3 x 3]
     	         [3 x 6] * [6 x 3]       + [3 x 3]
	       	      [3 x 3]      +       [3 x 3]
	    S = 	        [3 x 3]
     		*/
            Eigen::Matrix3d S = (H * P) * H.transpose() + R;
	    /* K is kalman gain 
	       K =   P     *   H.inv   /     S    
		  [6 x 6]  *  [6 x 3]  /  [3 x 3]
 	                [6 x 3]        /  [3 x 3]
	       K =              [6 x 3]
 		*/
            Eigen::Matrix3d K = P * H.transpose() * S.inverse();
		/* y = [3 x 1] - [3 x 6] * [6 x 1]
  			[3 x 1] - [3 x 1]
	    z.block<1,1>(1,0) = wrap_angle(z.block<1,1>(1,0));
            Eigen::Matrix<double, 6, 1> y = z - H * x;
            y.block<1,1>(1,0) = wrap_angle(y.block<1,1>(1,0));
		/* x = [6 x 1] + [6 x 3] * ([3 x 1])
  			[6 x 1] + [6 x 3] *([3 x 1])
     			[6 x 1] + [6 x 1]
  		*/
	    //New State x
            x = x + K * y;
		
		/* P = ([6 x 6] - [6 x 6] * [6 x 6]) * [6 x 6]
			     [6 x 6] * [6 x 6]
		   P =            [6 x 6]
		*/
	//I is identity matrix <6 , 6> 
	Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity(6, 6);
	/*
 		P = [(6 x 6) - (6 x 3) * (3 x 6)] * (6 x 6)
  	*/
            P = (I - K * H) * P;
        }

        double wrap_angle(double theta) {
            return atan2(sin(theta), cos(theta));
        }
private:
		Eigen::Matrix<double, 6, 1> x = Eigen::Matrix<double, 6, 1>::Zero(6, 1);

		/*Find out state variance-covariance matrix = P-- using identity matrix for now
   			6 x 6 because of three states and three velocities for each of those states
			P Format: 
   				1  0  0  0  0  0 
       				0  1  0  0  0  0
				0  0  1  0  0  0
    				0  0  0  1  0  0
				0  0  0  0  1  0
    				0  0  0  0  0  1
   			*/
		Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Identity(6, 6);

		//6 x 6 accounts for velocity, so even though we aren't fed in velocity we can calculate it and already have a variance for it
        	const Eigen::Matrix<double, 6, 6> Q; 

		// R is Measurement Covariance Matrix. Can be thought of as observation error
        	const Eigen::Matrix<double, 3, 3> R;
			
		
       		//double MAX_DT = 1e-3;

		int flag;


};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConstantVelKalmanFilterNode>());
    rclcpp::shutdown();
}

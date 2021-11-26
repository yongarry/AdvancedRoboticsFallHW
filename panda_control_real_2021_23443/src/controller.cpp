#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>


void ArmController::compute()
{
    // Kinematics and dynamics calculation ------------------------------
    Eigen::Affine3d transform = model_interface_.getTransform(franka::Frame::kEndEffector, q_);
    Eigen::Affine3d transform2 = model_interface_.getTransform(franka::Frame::kJoint4, q_);
    x_ = transform.translation();
    rotation_ = transform.linear();
    j_ = model_interface_.getJacobianMatrix(franka::Frame::kEndEffector, q_);
    m_ = model_interface_.getMassMatrix(q_);
    m_inverse_ = m_.inverse();

	x_dot_ = j_ * qdot_;
    // For CLIK
    Eigen::Affine3d transform_from_q_desired = model_interface_.getTransform(franka::Frame::kEndEffector, q_desired_);
    x_from_q_desired_ = transform_from_q_desired.translation();
    rotation_from_q_desired_ = transform_from_q_desired.linear();
    j_from_q_desired_ = model_interface_.getJacobianMatrix(franka::Frame::kEndEffector, q_desired_);
    // -----------------------------------------------------

	
    
	if (is_mode_changed_)
	{
		is_mode_changed_ = false;
		write_ = false;
		write_init_ = true;

		control_start_time_ = play_time_;

		q_init_ = q_;
		qdot_init_ = qdot_;
		q_error_sum_.setZero();
		q_desired_ = q_;

		x_init_ = x_;
		x_init_2_ = x_2;
		x_dot_init_ = x_dot_.block<3,1>(0,0);
		x_cubic_old_ = x_;
		rotation_init_ = rotation_;
		if (control_mode_ == "HW6-1")
		{
			writeFile.open(path);
		}
		else if (control_mode_ == "HW6-2(1)")
		{
			writeFile1.open(path1);
		}
		else if (control_mode_ == "HW6-2(2)")
		{
			writeFile2.open(path2);
		}
		else if (control_mode_ == "HW6-2(3)")
		{
			writeFile3.open(path3);
		}
	}
	

	if (control_mode_ == "joint_ctrl_home")
	{
        double duration = 5.0;
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, -M_PI / 4, 0.0, M_PI / 2, 0;
		q_desired_ = cubicVector<7>(play_time_, control_start_time_, control_start_time_ + duration, q_init_, target_position, qdot_init_, qdot_target_);
	}
	else if(control_mode_ == "joint_ctrl_init")
	{
        double duration = 5.0;
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, M_PI / 2, 0;
		q_desired_ = cubicVector<7>(play_time_, control_start_time_, control_start_time_ + duration, q_init_, target_position, qdot_init_, qdot_target_);
	}
	else if(control_mode_ == "torque_ctrl_init1")
	{
        double duration = 5.0;

		target_q << 0.0, 0.0, 0.0, -30*DEG2RAD, 0.0, 90*DEG2RAD, 0.0;

		for (int i = 0; i < dof_; i++)
		{
			q_desired_(i) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + duration, q_init_(i), target_q(i), 0.0, 0.0);
			qdot_desired_(i) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + duration, q_init_(i), target_q(i), 0.0, 0.0, hz_);
		}

		double K_p, K_v;

		K_p = 400;
		K_v = 40;

		torque_desired_ = m_ * (K_p * (q_desired_ - q_) + K_v * (qdot_desired_ - qdot_));
	}
	else if(control_mode_ == "torque_ctrl_init2")
	{
        double duration = 5.0;

		target_q << 0.0, 0.0, 0.0, -90*DEG2RAD, 0.0, 90*DEG2RAD, 0.0;

		for (int i = 0; i < dof_; i++)
		{
			q_desired_(i) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + duration, q_init_(i), target_q(i), 0.0, 0.0);
			qdot_desired_(i) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + duration, q_init_(i), target_q(i), 0.0, 0.0, hz_);
		}

		double K_p, K_v;

		K_p = 400;
		K_v = 40;

		torque_desired_ = m_ * (K_p * (q_desired_ - q_) + K_v * (qdot_desired_ - qdot_));
	}
	else if (control_mode_ == "HW6-1")
	{
		double duration = 5.0;
		x_target_ << 0.25, 0.28, 0.65;
		rotation_target_ << 0.7071, 0.7071, 0, 0.7071, -0.7071, 0, 0, 0,-1;

		for (int i = 0; i < 3; i++) {
			x_dot_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + duration, x_init_(i), x_target_(i), 0, 0, hz_);
			x_desired(i) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + duration, x_init_(i), x_target_(i), 0, 0);
		}

		rotation_desired_ = rotation_init_.transpose() * rotation_target_;
		angle_axis_.fromRotationMatrix(rotation_desired_);

		double theta_ = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + duration, 0, angle_axis_.angle(), 0, 0);
		double theta_dot_ = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + duration, 0, angle_axis_.angle(), 0, 0, hz_);
		
		k_hat_ = angle_axis_.axis();

		x_dot_desired.block<3,1>(3,0) = theta_dot_*k_hat_;
		
		// x_desired.block<3,1>(3,0) = DyrosMath::rot2Euler(rotation_) + dt_ * x_dot_desired.block<3,1>(3,0);
		// get rot(desired - from qdesired)
		angle_axis_desired = AngleAxisd(theta_, k_hat_);
		rotation_desired_ = angle_axis_desired.toRotationMatrix();
		x_desired.block<3,1>(3,0) = DyrosMath::getPhi(rotation_desired_, rotation_from_q_desired_);
		
       		// Jacobian: pseudo inverse
		Matrix<double, 7, 6> j_pseudo_inverse;
		j_pseudo_inverse = j_from_q_desired_.transpose() * (j_from_q_desired_ * j_from_q_desired_.transpose()).inverse();

		Matrix6d K_p;
		K_p.setIdentity();
		K_p = 0.1 * K_p;

		Vector6d x_from_q_desired_with_rot;
		x_from_q_desired_with_rot.block<3,1>(0,0) = x_from_q_desired_;
		//x_from_q_desired_with_rot.block<3,1>(3,0) = DyrosMath::rot2Euler(rotation_from_q_desired_);
		x_from_q_desired_with_rot.block<3,1>(3,0) << 0.0, 0.0, 0.0;

		qdot_desired_ = j_pseudo_inverse * (x_dot_desired + K_p * (x_desired - x_from_q_desired_with_rot));
       		q_desired_ = q_desired_ + dt_ * qdot_desired_ ;
		
		Matrix3d rotation_current = rotation_.transpose() * rotation_target_;
		AngleAxisd angle_axis_current.fromRotationMatrix(rotation_current);
		double theta_current = angle_axis_current.angle();
		writeFile << fixed << (play_time_ - control_start_time_) << "\t" <<  x_(0) << "\t" << x_(1) << "\t" << x_(2) << "\t" << x_desired(0) << "\t" << x_desired(1)<< "\t" << x_desired(2) << theta_current << theta_ << std::endl;
	}
    	else if (control_mode_ == "HW6-2(1)")
    	{
		double duration = 5.0;

		Matrix7d K_p, K_v;

		K_p.setIdentity();
		K_v.setIdentity();

		K_p(0, 0) = 400;
		K_p(1, 1) = 400;
		K_p(2, 2) = 400;
		K_p(3, 3) = 400;
		K_p(4, 4) = 400;
		K_p(5, 5) = 400;
		K_p(6, 6) = 400;

		K_v(0, 0) = 40;
		K_v(1, 1) = 40;
		K_v(2, 2) = 40;
		K_v(3, 3) = 40;
		K_v(4, 4) = 40;
		K_v(5, 5) = 40;
		K_v(6, 6) = 40;

		q_desired_ = q_init_;
		qdot_desired_.setZero();

		double target_position = -60 * DEG2RAD;
		q_desired_(3) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + duration, q_init_(3), target_position, 0.0, 0.0);

		torque_desired_ = m_ * (K_p * (q_desired_ - q_) + K_v * (qdot_desired_ - qdot_));

		writeFile1 << fixed << (play_time_ - control_start_time_) << "\t" <<  q_(3) << "\t" << q_desired_(3) << std::endl;

   	}
	else if (control_mode_ == "HW6-2(2)")
	{
		double duration = 5.0;

		double K_p, K_v, K_p_, K_v_;

		K_p = 400;
		K_v = 40;
		K_p_ = 400;
		K_v_ = 40;

		for (int i = 0; i < 3; i++)
		{
			x_dot_desired_(i) = 0.0;
			if (i == 1)
			{
				x_desired_(i) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + duration, x_init_(i), x_init_(i) + 0.1, 0.0, 0.0);
			}
			else
				x_desired_(i) = x_init_(i);
		}

		direction_cosine_[0] = rotation_.block<3, 1>(0, 0);
		direction_cosine_[1] = rotation_.block<3, 1>(0, 1);
		direction_cosine_[2] = rotation_.block<3, 1>(0, 2);

		direction_cosine_desired_[0] = rotation_init_.block<3, 1>(0, 0);
		direction_cosine_desired_[1] = rotation_init_.block<3, 1>(0, 1);
		direction_cosine_desired_[2] = rotation_init_.block<3, 1>(0, 2);

		delPhi_ = -0.5 * (direction_cosine_[0].cross(direction_cosine_desired_[0]) + direction_cosine_[1].cross(direction_cosine_desired_[1]) + direction_cosine_[2].cross(direction_cosine_desired_[2]));
		omega_ = j_.block<3, DOF>(3, 0) * qdot_;

		f_star = K_p_ * (x_desired_ - x_) + K_v_ * (x_dot_desired_ - x_dot_.block<3, 1>(0, 0));
		m_star = -K_p_ * delPhi_ - K_v_ * omega_;

		F_star_zero_.block<3, 1>(0, 0) = f_star;
		F_star_zero_.block<3, 1>(3, 0) = m_star;

		torque_init_ = m_ * (K_p * (q_init_ - q_) - K_v * qdot_);

		Lambda = (j_ * m_inverse_ * j_.transpose()).inverse();

		j_t_dyn_cons_inv = Lambda * j_ * m_inverse_;

		Matrix7d I_;
		I_.setIdentity();

		torque_desired_ = j_.transpose() * Lambda * F_star_zero_ + (I_ - j_.transpose() * j_t_dyn_cons_inv) * torque_init_;

		writeFile2 << fixed << (play_time_ - control_start_time_) << "\t" <<  x_(0) << "\t" << x_(1) << "\t" << x_(2) << "\t" << x_desired_(0) << "\t" << x_desired_(1)<< "\t" << x_desired_(2) << std::endl;

	}
	else if (control_mode_ == "HW6-2(3)")
	{
		double duration = 5.0;

		double K_p, K_v, K_p_, K_v_, xdot_max;

		K_p = 400;
		K_v = 40;
		K_p_ = 400;
		K_v_ = 40;
		xdot_max = 0.3;

		target_x << 0.3, -0.012, 0.52;
		for (int i = 0; i < 3; i++)
		{
			x_desired_(i) = target_x(i);
			if (abs(K_p_ / K_v_ * (x_desired_(i) - x_(i))) < xdot_max)
			{
				x_dot_desired_(i) = K_p_ / K_v_ * (x_desired_(i) - x_(i));
			}
			else
			{
				x_dot_desired_(i) = xdot_max / abs(x_desired_(i) - x_(i)) * (x_desired_(i) - x_(i));
			}
		}

		direction_cosine_[0] = rotation_.block<3, 1>(0, 0);
		direction_cosine_[1] = rotation_.block<3, 1>(0, 1);
		direction_cosine_[2] = rotation_.block<3, 1>(0, 2);

		direction_cosine_desired_[0] = rotation_init_.block<3, 1>(0, 0);
		direction_cosine_desired_[1] = rotation_init_.block<3, 1>(0, 1);
		direction_cosine_desired_[2] = rotation_init_.block<3, 1>(0, 2);

		delPhi_ = -0.5 * (direction_cosine_[0].cross(direction_cosine_desired_[0]) + direction_cosine_[1].cross(direction_cosine_desired_[1]) + direction_cosine_[2].cross(direction_cosine_desired_[2]));
		omega_ = j_.block<3, DOF>(3, 0) * qdot_;

		f_star = K_v_ * (x_dot_desired_ - x_dot_.block<3, 1>(0, 0));
		m_star = -K_p_ * delPhi_ - K_v_ * omega_;

		F_star_zero_.block<3, 1>(0, 0) = f_star;
		F_star_zero_.block<3, 1>(3, 0) = m_star;

		torque_init_ = m_ * (K_p * (q_init_ - q_) - K_v * qdot_);

		Lambda = (j_ * m_inverse_ * j_.transpose()).inverse();

		j_t_dyn_cons_inv = Lambda * j_ * m_inverse_;

		Matrix7d I_;
		I_.setIdentity();

		torque_desired_ = j_.transpose() * Lambda * F_star_zero_ + (I_ - j_.transpose() * j_t_dyn_cons_inv) * torque_init_ + g_;

		writeFile3 << fixed << (play_time_ - control_start_time_) << "\t" <<  x_(0) << "\t" << x_(1) << "\t" << x_(2) << "\t" << x_desired_(0) << "\t" << x_desired_(1)<< "\t" << x_desired_(2) << std::endl;
	}
	else 
	{
		torque_desired_.setZero();
	}

	printState();

	tick_++;
}


void ArmController::printState()
{
	// TODO: Modify this method to debug your code

	static int DBG_CNT = 0;
	if (DBG_CNT++ > hz_ / 20.)
	{
		DBG_CNT = 0;

		cout << "q desired:\t";
		cout << std::fixed << std::setprecision(3) << q_desired_.transpose() << endl;
		cout << "q now    :\t";
		cout << std::fixed << std::setprecision(3) << q_.transpose() << endl;
		cout << "x desired:\t";
		cout << x_desired_.transpose() << endl;
		cout << "x        :\t";
		cout << x_.transpose() << endl;

	}

	if (play_time_ == control_start_time_ + 3)
	{
		cout << "---------------------------------------------------------------------" << endl;
		cout << "                     control time finished                           " << endl;
		cout << "---------------------------------------------------------------------" << endl;
	}
}



// Controller Core Methods ----------------------------

void ArmController::setMode(const std::string & mode)
{
	is_mode_changed_ = true;
	control_mode_ = mode;
	cout << "Current mode (changed) : " << mode << endl;
}
void ArmController::initDimension()
{
	dof_ = DOF;
	q_temp_.resize(DOF);
	j_temp_.resize(6, DOF);
	j_temp_2.resize(6, DOF);
	j_temp_2.setZero();

	qddot_.setZero();

	x_target_.setZero();
	q_desired_.setZero();
	qdot_target_.setZero();
	torque_desired_.setZero();
	x_dot_init_.setZero();
	x_dot_target_.setZero();

	g_temp_.resize(DOF);
	m_temp_.resize(DOF, DOF);

	Kp_.setZero();
	Kv_.setZero();
	Kp_joint_.setZero();
	Kv_joint_.setZero();
}

void ArmController::readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = torque(i);
	}
}
void ArmController::readData(const Vector7d &position, const Vector7d &velocity)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = 0;
	}
}

const Vector7d & ArmController::getDesiredPosition()
{
	return q_desired_;
}

const Vector7d & ArmController::getDesiredTorque()
{
	return torque_desired_;
}

void ArmController::closeFile()
{
	if(writeFile.is_open())
		writeFile.close();
}

void ArmController::initPosition(franka::RobotState state)
{
    q_init_ = q_;
    q_desired_ = q_init_;
    model_interface_.setRobotState(state);
}

void ArmController::updateTime(double dt)
{
    dt_ = dt;
    play_time_ += dt;
}
// ----------------------------------------------------


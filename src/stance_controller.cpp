#include "stance_controller.hpp"

//Helper functoion to convert Eigen VectorXd type to stdl vectors
vector<double> eigenToStlVec(VectorXd vec) {
    std::vector<double> stl_vec(
        vec.data(), vec.data() + vec.rows() * vec.cols());
    return stl_vec;
}

vector<int> eigenToStlVec(VectorXi vec) {
    std::vector<int> stl_vec(
        vec.data(), vec.data() + vec.rows() * vec.cols());
    return stl_vec;
}

StanceController::StanceController(
    A1* _robot,
    GaitGenerator* _gait_generator,
    VectorXd _desired_speed,
    double _desired_twisting_speed,
    double _desired_body_height,
    double _body_mass)

    :robot(_robot),
    gait_generator(_gait_generator),
    desired_speed(_desired_speed),
    desired_body_height(_desired_body_height),
    body_mass(_body_mass) {};

std::map<int,double> StanceController::getAction(std::vector<double> weights, double mass, vector<double> inertial) {
    //Initialize solver
    int planning_horizon_steps = 10;
    double planning_timestep = 0.025;
    //vector<double> inertia_vec = {inertia[0], 0, 0, 0, inertia[1], 0, 0, 0, inertia[2]};

    VMC_QP vmc_qp(mass, inertial,
                  friction_coeff,
                  kp_torque,
                  kd_torque,
                  kp_force,
                  kd_force,
                  weights,
                  alpha);

    //Get desired states
    vector<double> desired_com_position {
        0, 0, desired_body_height};
    vector<double> desired_com_velocity {
        desired_speed[0], desired_speed[1], 0};
    vector<double> desired_com_roll_pitch_yaw {0, 0, 0};
    vector<double> desired_com_angular_velocity {
        0, 0, desired_twisting_speed};
    
    //Get current states
    vector<int> foot_contact_state = 
        eigenToStlVec(gait_generator->desired_leg_state);
    
    vector<double> com_position {{0, 0, robot->getComPosition()[2]}};
    vector<double> com_roll_pitch_yaw = eigenToStlVec(
        robot->getBaseRollPitchYaw());
    com_roll_pitch_yaw[2] = 0;

    vector<double> com_velocity = eigenToStlVec(robot->getComVelocity());
    vector<double> com_angular_velocity = eigenToStlVec(
        robot->getBaseRollPitchYawRate());

    VectorXd foot_tmp = robot->getFootPositionsInBaseFrame().reshaped<
        Eigen::StorageOptions::RowMajor>();
    vector<double> foot_positions_base_frame = eigenToStlVec(foot_tmp);

    //Compute contact forces
    int control_mode = 0;
    vector<double> robot_position = eigenToStlVec(robot->getComPosition());
    auto predicted_contact_forces_tmp = vmc_qp.getContactForce(control_mode,robot_position,
              com_roll_pitch_yaw,com_velocity,com_angular_velocity,foot_positions_base_frame,
              foot_contact_state,desired_com_position,desired_com_roll_pitch_yaw,desired_com_velocity,
              desired_com_angular_velocity);
    Eigen::VectorXd predicted_contact_forces = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        predicted_contact_forces_tmp.data(), predicted_contact_forces_tmp.size());
    
    //Convert forces to joint torques and format as a dictionary
    std::map<int,VectorXd> contact_forces;
    for (int i = 0; i < num_legs; i++) {
        VectorXd force = predicted_contact_forces(seq(3*i,3*i + 2));
        if ( foot_contact_state[i] != 0) {
            contact_forces[i] = force;
        }
        else {
            //Set small values to zero
            contact_forces[i] = VectorXd::Zero(3);
        }
    }

    std::map<int,double>action;
    for (auto &[leg_id, force] : contact_forces) {
        auto motor_torques = robot->mapContactForceToJointTorques(leg_id, force);

        for (auto &[joint_id, torque] : motor_torques) {
            //Only add non-zero values
            if (torque) {
                action[joint_id] = torque;
            }
        }
    }
    
    last_action = action;
    return action;
}
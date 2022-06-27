#include "a1.hpp"
#include "gait_generator.hpp"
#include "VMC.h"

using std::vector;

class StanceController {
    public:
    A1* robot;
    GaitGenerator* gait_generator;
    VectorXd desired_speed;
    double desired_twisting_speed;
    double desired_body_height;
    double body_mass;
    int num_legs = 4;
    double friction_coeff =0.4;
    double alpha = 1e-5;
    vector<double> body_inertia {0.07335, 0, 0, 0, 0.25068,
        0, 0, 0, 0.25447};
    map<int,double> last_action;
    vector<double> kp_force {10,10,100} ;
    vector<double> kd_force {6.3,6.3,20};
    vector<double> kp_torque {1000, 1000, 1000} ;
    vector<double> kd_torque {63, 63, 63};

    StanceController(){};
    StanceController(
        A1* _robot,
        GaitGenerator* _gait_generator,
        VectorXd _desired_speed,
        double _desired_twisting_speed,
        double _desired_body_height,
        double _body_mass);
    
    std::map<int,double> getAction(std::vector<double> weights, double mass, vector<double> inertia);
};
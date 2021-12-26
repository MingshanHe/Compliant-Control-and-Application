#ifndef MPC_H
#define MPC_H

#include <vector>
#include <map>

#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>

using CppAD::AD;

using namespace std;

class FG_eval 
{
    public:
        // Fitted polynomial coefficients
        Eigen::VectorXd coeffs;
        //reference values
        double _dt;
        double _ref_q;
        // weight of coefficient
        double _weight_q;
        double _weight_tau;
        // search values
        int _mpc_steps;
        int _q_start;
        int _qdot_start;
        int _tau_start;

        // AD value
        AD<double> q0, qdot0, tau0;
        AD<double> q, qdot, tau;
        AD<double> cost_q, cost_tau;
    public:
        // Constructor
        FG_eval(Eigen::VectorXd coeffs);

        // Load parameters for constraints
        void LoadParams(const std::map<string, double> &params);

        // MPC implementation (cost func & constraints)
        typedef CPPAD_TESTVECTOR(AD<double>) ADvector; 
        // fg: function that evaluates the objective and constraints using the syntax       
        void operator()(ADvector& fg, const ADvector& vars);
};

class MPC
{
    public:
        MPC();
        ~MPC(){}
        // Solve the model given an initial state and polynomial coefficients.
        // Return the first actuatotions.
        vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
        void LoadParams(const std::map<string, double> &params);

    public:
        vector<double> q;
        vector<double> qdot;
        vector<double> tau;

        double _mpc_totalcost;
        double _mpc_cost_q;
        double _mpc_cost_tau;

    private:
        // Parameters for mpc solver
        double _max_qdot;
        double _max_tau;
        double _bound_value;

        int _mpc_steps;
        int _q_start;
        int _qdot_start;
        int _tau_start;

        std::map<string, double> _params;

        unsigned int dis_cnt;
};
#endif /* MPC_H */

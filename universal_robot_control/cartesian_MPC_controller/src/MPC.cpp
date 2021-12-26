#include "cartesian_MPC_controller/MPC.h"

using CppAD::AD;

// =========================================
// FG_eval class definition implementation.
// =========================================

// Constructor
FG_eval::FG_eval(Eigen::VectorXd coeffs) 
{ 
    this->coeffs = coeffs; 

    // Set default value    
    _dt         = 0.1;  // in sec
    _ref_q      = 0.0;
    _weight_q   = 100;
    _weight_tau = 100;

    _mpc_steps  = 40;
    _q_start    = 0;
    _qdot_start = _q_start + _mpc_steps;
    _tau_start  = _qdot_start + _mpc_steps;
}

// Load parameters for constraints
void FG_eval::LoadParams(const std::map<string, double> &params)
{
    _dt         = params.find("DT") != params.end() ? params.at("DT") : _dt;
    _ref_q      = params.find("REF_Q") != params.end()  ? params.at("REF_Q") : _ref_q;

    _weight_q   = params.find("WEIGHT_Q") != params.end()  ? params.at("WEIGHT_Q") : _weight_q;
    _weight_tau = params.find("WEIGHT_TAU") != params.end()  ? params.at("WEIGHT_TAU") : _weight_tau;

    _mpc_steps  = params.find("STEPS") != params.end()  ? params.at("STEPS") : _mpc_steps;
    _q_start    = 0;
    _qdot_start = _q_start + _mpc_steps;
    _tau_start  = _qdot_start + _mpc_steps;

}

// fg: function that evaluates the objective and constraints using the syntax      
void FG_eval::operator()(ADvector& fg, const ADvector& vars) 
{
    // fg[0] for cost function
    fg[0] = 0;
    cost_q =  0;
    cost_tau = 0;

    for (int i = 0; i < _mpc_steps; i++) 
    {
        fg[0] += _weight_q * CppAD::pow(vars[_q_start + i] - _ref_q, 2); // cross deviation error
        fg[0] += _weight_tau * CppAD::pow(vars[_tau_start + i], 2); // heading error

        cost_q      +=  _weight_q * CppAD::pow(vars[_q_start + i] - _ref_q, 2);
        cost_tau    +=  _weight_tau * CppAD::pow(vars[_tau_start + i], 2); 
    }
    cout << "-----------------------------------------------" <<endl;
    cout << "cost_q, cost_tau: " << cost_q << ", " << cost_tau << endl;

    // fg[x] for constraints
    // Initial constraints
    fg[1 + _q_start]    = vars[_q_start];
    fg[1 + _qdot_start] = vars[_qdot_start];
    fg[1 + _tau_start]  = vars[_tau_start];

    // Add system dynamic model constraint
    for (int i = 0; i < _mpc_steps - 1; i++)
    {
        // The state at time t+1 .
        q       = vars[_q_start + i + 1];
        qdot    = vars[_qdot_start + i + 1];

        // The state at time t.
        q0      = vars[_q_start + i];
        qdot0   = vars[_qdot_start + i];

        // Here's `x` to get you started.
        // The idea here is to constraint this value to be 0.
        //
        // NOTE: The use of `AD<double>` and use of `CppAD`!
        // This is also CppAD can compute derivatives and pass
        // these to the solver.
        // TODO: Setup the rest of the model constraints
        fg[2 + _q_start + i]    = q - (q0 + qdot0 * _dt);
        fg[2 + _qdot_start + i] = q - (q0 + qdot0 * _dt);
    }
}

// ====================================
// MPC class definition implementation.
// ====================================
MPC::MPC() 
{
    // Set default value    
    _mpc_steps      = 20;
    _max_qdot       = 3.0; // Maximal angvel radian (~30 deg)
    _max_tau        = 1.0; // Maximal throttle accel
    _bound_value    = 1.0e3; // Bound value for other variables

    _q_start        = 0;
    _qdot_start     = _q_start + _mpc_steps;
    _tau_start      = _qdot_start + _mpc_steps;

}

void MPC::LoadParams(const std::map<string, double> &params)
{
    _params = params;
    //Init parameters for MPC object
    _mpc_steps      = _params.find("STEPS") != _params.end() ? _params.at("STEPS") : _mpc_steps;
    _max_qdot       = _params.find("MAX_QDOT") != _params.end() ? _params.at("ANGVEL") : _max_qdot;
    _max_tau        = _params.find("MAX_TAU") != _params.end() ? _params.at("MAXTHR") : _max_tau;
    _bound_value    = _params.find("BOUND") != _params.end()  ? _params.at("BOUND") : _bound_value;

    cout << "\n!! MPC Obj parameters updated !! " << endl; 
}


vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) 
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    const double q      = state[0];
    const double qdot   = state[1];
    const double tau    = state[2];

    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9
    size_t n_vars = _mpc_steps * 3 + (_mpc_steps - 1) * 1;
    
    // Set the number of constraints
    size_t n_constraints = _mpc_steps * 3;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) 
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[_q_start]      = q;
    vars[_qdot_start]   = qdot;
    vars[_tau_start]    = tau;

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < _qdot_start; i++) 
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] = _bound_value;
    }
    // The upper and lower limits of angvel are set to -25 and 25
    // degrees (values in radians).
    for (int i = _qdot_start; i < _tau_start; i++) 
    {
        vars_lowerbound[i] = -_max_qdot;
        vars_upperbound[i] = _max_qdot;
    }
    // Acceleration/decceleration upper and lower limits
    for (int i = _tau_start; i < n_vars; i++)  
    {
        vars_lowerbound[i] = -_max_tau;
        vars_upperbound[i] = _max_tau;
    }


    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[_q_start]    = q;
    constraints_lowerbound[_qdot_start] = qdot;
    constraints_lowerbound[_tau_start]  = tau;
    constraints_upperbound[_q_start]    = q;
    constraints_upperbound[_qdot_start] = qdot;
    constraints_upperbound[_tau_start]  = tau;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);
    fg_eval.LoadParams(_params);

    fg_eval.cost_q;

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    double cost     = solution.obj_value;
    std::cout << "------------ Total Cost(solution): " << cost << "------------" << std::endl;
    cout << "-----------------------------------------------" <<endl;
    _mpc_totalcost  = cost;
    _mpc_cost_q     = Value(fg_eval.cost_q);
    _mpc_cost_tau   = Value(fg_eval.cost_tau);

    // this->q     = {};
    // this->qdot  = {};
    // this->tau   = {};
    this->q.clear();
    this->qdot.clear();
    this->tau.clear();
    for (int i = 0; i < _mpc_steps; i++) 
    {
        this->q.push_back(solution.x[_q_start+i]);
        this->qdot.push_back(solution.x[_qdot_start+i]);
        this->tau.push_back(solution.x[_tau_start+i]);
    }
    
    vector<double> result;
    result.push_back(solution.x[_q_start]);
    result.push_back(solution.x[_qdot_start]);
    result.push_back(solution.x[_tau_start]);
    return result;
}
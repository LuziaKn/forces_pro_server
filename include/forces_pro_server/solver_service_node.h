#ifndef __SOLVER_SERVICE_NODE_H__
#define __SOLVER_SERVICE_NODE_H__

// To switch the solver type ctrl+h pointRobotFORCESNLPsolver_fixed replace with
#include <pointRobotFORCESNLPsolver_fixed.h>
#include <pointRobotFORCESNLPsolver_fixed_memory.h>

#include <forces_pro_server/srv/call_forces_pro.hpp>

#define SOLVER_ADTOOL2FORCES pointRobotFORCESNLPsolver_fixed_adtool2forces
#define SOLVER_EXT_FUNC pointRobotFORCESNLPsolver_fixed_extfunc
#define SOLVER_EXTERNAL_MEM(x, y, z) pointRobotFORCESNLPsolver_fixed_external_mem(x, y, z)
#define SOLVER_GET_MEM_SIZE() pointRobotFORCESNLPsolver_fixed_get_mem_size()
#define SOLVER_SOLVE(x, y, z, a, b, c) pointRobotFORCESNLPsolver_fixed_solve(x, y, z, a, b, c)
extern "C"
{
    extern solver_int32_default pointRobotFORCESNLPsolver_fixed_adtool2forces(pointRobotFORCESNLPsolver_fixed_float *x,       /* primal vars                                         */
                                                                    pointRobotFORCESNLPsolver_fixed_float *y,       /* eq. constraint multiplers                           */
                                                                    pointRobotFORCESNLPsolver_fixed_float *l,       /* ineq. constraint multipliers                        */
                                                                    pointRobotFORCESNLPsolver_fixed_float *p,       /* parameters                                          */
                                                                    pointRobotFORCESNLPsolver_fixed_float *f,       /* objective function (scalar)                         */
                                                                    pointRobotFORCESNLPsolver_fixed_float *nabla_f, /* gradient of objective function                      */
                                                                    pointRobotFORCESNLPsolver_fixed_float *c,       /* dynamics                                            */
                                                                    pointRobotFORCESNLPsolver_fixed_float *nabla_c, /* Jacobian of the dynamics (column major)             */
                                                                    pointRobotFORCESNLPsolver_fixed_float *h,       /* inequality constraints                              */
                                                                    pointRobotFORCESNLPsolver_fixed_float *nabla_h, /* Jacobian of inequality constraints (column major)   */
                                                                    pointRobotFORCESNLPsolver_fixed_float *hess,    /* Hessian (column major)                              */
                                                                    solver_int32_default stage,           /* stage number (0 indexed)                            */
                                                                    solver_int32_default iteration,       /* iteration number of solver                          */
                                                                    solver_int32_default threadID /* Id of caller thread 								   */);
}

struct Solver
{

    // Memory used in solving
    char *solver_memory_;
    pointRobotFORCESNLPsolver_fixed_mem *solver_memory_handle_;

    // Parameters / info / output
    pointRobotFORCESNLPsolver_fixed_params forces_params_;
    pointRobotFORCESNLPsolver_fixed_output forces_output_;
    pointRobotFORCESNLPsolver_fixed_info forces_info_;

    Solver();
    virtual ~Solver();

    int Solve();

    void SolveServiceCallback(
        const std::shared_ptr<forces_pro_server::srv::CallForcesPro::Request> request,
        std::shared_ptr<forces_pro_server::srv::CallForcesPro::Response> response);
};

#endif
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * TrajOptimizer_codable.c
 *
 * Code generation for function 'TrajOptimizer_codable'
 *
 */

/* Include files */
#include "TrajOptimizer_codable.h"
#include "TrajOptimizer_codable_types.h"
#include "rt_nonfinite.h"
#include "Traj3DoF.h"

/* Function Definitions */
void TrajOptimizer_codable(const emlrtStack *sp, const real_T x_initial[6],
                           const real_T x_guess[2400],
                           const real_T u_guess[800], const Vehicle *vehicle,
                           real_T u_opt[800], real_T x_opt[2400])
{
  const casadi_real *arg[407];
  casadi_real w[100855];
  casadi_real *res[12];
  int32_T mem;
  (void)sp;
  (void)vehicle;
  /* input current state and vehicle information, output control vector and
   * potential state */
  /*  TODO */
  /*  - Add free final time through having time be normalized and adding a */
  /*  parameter for time dialation (the time scaling factor) */
  /*        - Hopefully should greatly increase the robustness of the MPC */
  /*  - Generalize to higher degree of freedom models if possible */
  /*  - Account for delay from computation time when setting initial */
  /*  condition */
  /*  - Add glideslope constraint */
  /*  Any pre-processing using pure Matlab operations can go here */
  /*  Set the number of steps and the timestep (dt) */
  /*  Make sure data-types and sizes are known */
  /*  Anything CasADi related goes here */
  /*  This gets executed when Matlab Coder is parsing the file */
  /*  Hooks up Matlab Coder with CasADi generated C code */
  /*  Connect .c and .h file */
  /*  Set link and include path */
  /*  Link with IPOPT */
  /*  Setting up working space */
  mem = Traj3DoF_checkout();
  /*  Call the generated CasADi code */
  Traj3DoF_unrolled(&x_initial[0], &x_guess[0], &u_guess[0], &u_opt[0],
                    &x_opt[0], &arg[0], &res[0], NULL, &w[0], mem);
  /*  % Adapt to as many inputs arguments as your CasADi Function has */
  /*  % Adapt to as many outputs as your CasADi Function has */
  /*   */
  Traj3DoF_release(mem);
  /*  Any post-processing using pure Matlab operations can go here */
  /*     %{ */
  /*     x_opt = zeros(size(x_opt_sol)); */
  /*     u_opt = zeros(size(u_opt_sol)); */
  /*     x_opt(:, :) = x_opt_sol(:, :); */
  /*     u_opt(:, :) = u_opt_sol(:, :); */
  /*  */
  /*     %Plots */
  /*  */
  /*     figure('Name', 'Optimization Results', 'NumberTitle', 'off', 'Color',
   * 'w'); */
  /*  */
  /*     % Plot state variables */
  /*     subplot(2,1,1); */
  /*     hold on; */
  /*     plot(x_opt(:,1), 'LineWidth', 1.5, 'DisplayName', 'x (Position)'); */
  /*     plot(x_opt(:,2), 'LineWidth', 1.5, 'DisplayName', 'y (Position)'); */
  /*     plot(x_opt(:,3), 'LineWidth', 1.5, 'DisplayName', 'x\_dot (Velocity)');
   */
  /*     plot(x_opt(:,4), 'LineWidth', 1.5, 'DisplayName', 'y\_dot (Velocity)');
   */
  /*     plot(x_opt(:,5), 'LineWidth', 1.5, 'DisplayName', 'theta (Angle)'); */
  /*     plot(x_opt(:,6), 'LineWidth', 1.5, 'DisplayName', 'theta\_dot (Angular
   * Velocity)'); */
  /*     hold off; */
  /*     legend('Location', 'best'); */
  /*     xlabel('Time Step'); */
  /*     ylabel('State Values'); */
  /*     title('State Variables'); */
  /*     grid on; */
  /*  */
  /*     % control inputs */
  /*     subplot(2,1,2);  */
  /*     hold on; */
  /*     plot(u_opt(:,1), 'LineWidth', 1.5, 'DisplayName', 'Thrust %'); */
  /*     plot(u_opt(:,2), 'LineWidth', 1.5, 'DisplayName', 'Thrust Angle
   * (rad)'); */
  /*     hold off; */
  /*     legend('Location', 'best'); */
  /*     xlabel('Time Step'); */
  /*     ylabel('Control Inputs'); */
  /*     title('Control Inputs'); */
  /*     grid on; */
  /*  */
  /*     % times */
  /*  */
  /*     final_time_step = t_step; */
  /*     duration = t_step * steps; */
  /*  */
  /*     fprintf('Final Time Step: %.4f seconds\n', final_time_step); */
  /*     fprintf('Total Duration: %.4f seconds\n', duration); */
  /*  */
  /*     fprintf("Complete state matrix: \n______________________________\n") */
  /*     disp(x_opt) */
  /*     fprintf("Complete control matrix: \n______________________________\n")
   */
  /*     %} */
}

/* End of code generation (TrajOptimizer_codable.c) */

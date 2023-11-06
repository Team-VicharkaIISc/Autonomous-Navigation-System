/*
 * vehicle.cc
 *
 *  created: Oct 2016
 *   author: Matthias Rungger
 */

/*
 * information about this example is given in
 * http://arxiv.org/abs/1503.03715
 * doi: 10.1109/TAC.2016.2593947
 */
#include <iostream>
#include <array>

/* SCOTS header */
#include "scots.hh"

/* ode solver */
#include "RungeKutta4.hh"

/* time profiling */
#include "TicToc.hh"

/* memory profiling */
#include <sys/time.h>
#include <sys/resource.h>

struct rusage usage;

/* state space dim */
const int state_dim = 3;
/* input space dim */
const int input_dim = 2;

/* sampling time */
const double tau = 2.1;

/*
 * data types for the state space elements and input space
 * elements used in uniform grid and ode solvers
 */
using state_type = std::array<double, state_dim>;
using input_type = std::array<double, input_dim>;

/* abbrev of the type for abstract states and inputs */
using abs_type = scots::abs_type;

/* we integrate the vehicle ode by tau sec (the result is stored in x)  */
auto  vehicle_post = [](state_type &x, const input_type &u) {
  /* the ode describing the vehicle */
  auto rhs =[](state_type& xx,  const state_type &x, const input_type &u) {
    xx[0] = u[0] * std::cos(x[2]);
    xx[1] = u[0] * std::sin(x[2]);
    xx[2] = u[1];
  };
  /* simulate (use 10 intermediate steps in the ode solver) */
  scots::runge_kutta_fixed4(rhs, x, u, state_dim, tau, 10);
};

/* we integrate the growth bound by 0.3 sec (the result is stored in r)  */
auto radius_post = [](state_type &r, const state_type &, const input_type &u) {
  r[0] = r[0] + r[2] * std::abs(u[0]) * tau;
  r[1] = r[1] + r[2] * std::abs(u[0]) * tau;
};

int main() {
  /* to measure time */
  TicToc tt;

  /* setup the workspace of the synthesis problem and the uniform grid */
  /* lower bounds of the hyper rectangle */
  state_type s_lb={{0, 0, -3.5}};
  
  /* upper bounds of the hyper rectangle */
  state_type s_ub={{5.0, 4.55, 3.5}};
  
  /* grid node distance diameter */
  state_type s_eta={{.1, .1, .2}};

  scots::UniformGrid ss(state_dim, s_lb, s_ub, s_eta);
  std::cout << "Uniform grid details:" << std::endl;
  ss.print_info();
  
  /* construct grid for the input space */
  /* lower bounds of the hyper rectangle */
  input_type i_lb={{-0.22, -0.1}};
  
  /* upper bounds of the hyper rectangle */
  input_type i_ub={{ 0.22, 0.1}};
  
  /* grid node distance diameter */
  input_type i_eta={{.02, .01}};
  
  scots::UniformGrid is(input_dim, i_lb, i_ub, i_eta);
  is.print_info();

  /* set up constraint functions with obtacles */
  int num_obs = 6;
  double H[num_obs][4] = {
    { 0.95, 1.15, 2.65, 3.05 },
    { 1.45, 2.35, 1.95, 2.10 },
    { 2.95, 3.85, 1.95, 2.10 },
    { 2.45, 2.85, 1.15, 1.35 }, 
    { 3.95, 4.15, 2.65, 3.05 }, 
    { 1.85, 3.35, 3.20, 3.35 }
  };

  /* avoid function returns 1 if x is in avoid set  */
  auto avoid = [&H, ss, s_eta, num_obs](const abs_type& idx) {
    state_type x;
    ss.itox(idx, x); 
    double c1 = s_eta[0] / 2.0+1e-10;
    double c2 = s_eta[1] / 2.0+1e-10;
    for(size_t i = 0; i < num_obs; i++) {
      if ((H[i][0] - c1) <= x[0] && x[0] <= (H[i][1] + c1) && 
          (H[i][2] - c2) <= x[1] && x[1] <= (H[i][3] + c2))
        return true;
    }
    return false;
  };
  
  /* write obstacles to file */
  write_to_file(ss, avoid, "obstacles_turtlebot");

  std::cout << "Computing the transition function: " << std::endl;
  
  /* transition function of symbolic model */
  scots::TransitionFunction tf;
  scots::Abstraction<state_type,input_type> abs(ss, is);

  tt.tic();
  abs.compute_gb(tf, vehicle_post, radius_post, avoid);
  // abs.compute_gb(tf, vehicle_post, radius_post);
  tt.toc();

  if(!getrusage(RUSAGE_SELF, &usage))
    std::cout << "Memory per transition: " << usage.ru_maxrss / (double)tf.get_no_transitions() << std::endl;
  std::cout << "Number of transitions: " << tf.get_no_transitions() << std::endl;

  /* define target set */
  auto target_1 = [&ss, &s_eta](const abs_type& idx) {
    state_type x;
    ss.itox(idx,x);
    /* function returns 1 if cell associated with x is in target set  */
    if (2.5 <= (x[0] - s_eta[0] / 2.0) && (x[0] + s_eta[0] / 2.0) <= 2.85 && 
        0.65 <= (x[1] - s_eta[1] / 2.0) && (x[1] + s_eta[1] / 2.0) <= 1.0)
      return true;
    return false;
  };
   /* write target to file */
  write_to_file(ss, target_1, "target_tb_1");

  auto target_2 = [&ss, &s_eta](const abs_type& idx) {
    state_type x;
    ss.itox(idx, x);
    /* function returns 1 if cell associated with x is in target set  */
    if (2.5 <= (x[0] - s_eta[0] / 2.0) && (x[0] + s_eta[0] / 2.0) <= 2.85 && 
        3.8 <= (x[1] - s_eta[1] / 2.0) && (x[1] + s_eta[1] / 2.0) <= 4.15)
      return true;
    return false;
  };
  /* write target to file */
  write_to_file(ss, target_2, "target_tb_1");

 
  std::cout << "\nSynthesis 1: " << std::endl;
  tt.tic();
  scots::WinningDomain win_1 = scots::solve_reachability_game(tf, target_1);
  tt.toc();
  std::cout << "Winning domain size for tb 1: " << win_1.get_size() << std::endl;

  std::cout << "\nSynthesis 2: " << std::endl;
  tt.tic();
  scots::WinningDomain win_2 = scots::solve_reachability_game(tf, target_2);
  tt.toc();
  std::cout << "Winning domain size for tb 2: " << win_2.get_size() << std::endl;

  std::cout << "\nWrite controller to controller_tb_1.scs \n";
  if(write_to_file(scots::StaticController(ss, is, std::move(win_1)),"controller_tb_1"))
    std::cout << "Done. \n";

  std::cout << "\nWrite controller to controller_tb_2.scs \n";
  if(write_to_file(scots::StaticController(ss, is, std::move(win_2)),"controller_tb_2"))
    std::cout << "Done. \n";

  return 0;
}

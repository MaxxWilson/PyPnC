#include <cmath>
#include <iostream>

#include <ifopt/ipopt_solver.h>

#include <configuration.h>
#include <towr_plus/locomotion_solution.h>
#include <towr_plus/locomotion_task.h>
#include <towr_plus/models/robot_model.h>
#include <towr_plus/nlp_formulation.h>

int main() {
  YAML::Node cfg =
      YAML::LoadFile(THIS_COM "config/towr_plus/atlas_four_step.yaml");
  Clock clock = Clock();
  double time_solving(0.);

  // Locomotion Task
  LocomotionTask task = LocomotionTask("atlas_four_step");
  task.from_yaml(cfg["locomotion_task"]);

  // Locomotion Solution
  LocomotionSolution sol =
      LocomotionSolution("atlas_four_step", cfg["locomotion_param"]);

  // Construct NLP from locomotion task
  NlpFormulation formulation;
  formulation.model_ = RobotModel(RobotModel::Atlas);
  formulation.params_.from_yaml(cfg["locomotion_param"]);
  formulation.from_locomotion_task(task);
  // formulation.initialize_from_dcm_planner("dubins");

  // Solve
  ifopt::Problem nlp;
  SplineHolder solution;
  for (auto c : formulation.GetVariableSets(solution)) {
    nlp.AddVariableSet(c);
  }
  for (auto c : formulation.GetConstraints(solution)) {
    nlp.AddConstraintSet(c);
  }
  for (auto c : formulation.GetCosts()) {
    nlp.AddCostSet(c);
  }

  // Eigen::VectorXd initial_vars = nlp.GetVariableValues();
  // sol.from_one_hot_vector(initial_vars);
  // sol.to_yaml();
  // nlp.PrintCurrent();
  // exit(0);

  auto solver = std::make_shared<ifopt::IpoptSolver>();
  // solver->SetOption("derivative_test", "first-order");
  // solver->SetOption("derivative_test_tol", 1e-3);
  // nlp.PrintCurrent();
  // exit(0);
  solver->SetOption("jacobian_approximation", "exact");
  solver->SetOption("max_cpu_time", 1000.0);
  clock.start();
  solver->Solve(nlp);
  time_solving = clock.stop();

  nlp.PrintCurrent();

  Eigen::VectorXd vars = nlp.GetVariableValues();
  sol.from_one_hot_vector(vars);
  // sol.print_solution();
  sol.to_yaml();
  printf("Takes %f seconds\n", 1e-3 * time_solving);

  std::cout << "---------- Stats ----------" << std::endl;
  std::cout << "Iteration Count: " << nlp.GetIterationCount() << std::endl;
  std::cout << "Length of Motion: " << formulation.params_.GetTotalTime() << std::endl;
  std::cout << "Length of Motion: " << formulation.params_.GetTotalTime() << std::endl;

  int i = 4;
  // for(int i = 0; i < nlp.GetIterationCount(); i++){
    nlp.SetOptVariables(i);
    auto variable_values = nlp.GetVariableValues();
    auto constraints = nlp.EvaluateConstraints(variable_values.data());

    std::cout << std::endl << "Accessing Opt Variables" << std::endl;
    auto variables = nlp.GetOptVariables()->GetComponents();
    for(auto &v: variables){
      std::cout << v->GetName() << ", " << v->GetRows() << std::endl;
      // if(v->GetName() == "ee-motion-lin_0"){
      //   int i = 0;
      //   std::cout << v->GetValues() << std::endl;
      // }
    }

    std::cout << std::endl << "Accessing Costs" << std::endl;
    auto costs = nlp.GetCosts().GetComponents();
    for(auto &c: costs){
      std::cout << c->GetName() << ", " << c->GetRows() << std::endl;
    }

    std::cout << std::endl << "Accessing Constraints" << std::endl;
    auto constraint_components = nlp.GetConstraints().GetComponents();
    for(auto &constraint: constraint_components){
      std::cout << constraint->GetName() << ", " << constraint->GetRows() << std::endl;
    }

    std::cout << "Solution" << std::endl;
    std::cout << solution.base_linear_->GetPoint(2).p() << std::endl;
    nlp.SetOptVariables(10);
    std::cout << std::endl << solution.base_linear_->GetPoint(2).p() << std::endl;
    nlp.SetOptVariables(4);
    std::cout << std::endl << solution.base_linear_->GetPoint(2).p() << std::endl;

  return 0;
}

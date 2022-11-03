add_library(towr_plus SHARED
  # sample formulation usage
  src/nlp_formulation.cc
  src/parameters.cc
  # variables
  src/variables/nodes_variables.cc
  src/variables/nodes_variables_all.cc
  src/variables/nodes_variables_phase_based.cc
  src/variables/phase_durations.cc
  # models
  src/models/robot_model.cc
  src/models/dynamic_model.cc
  src/models/composite_rigid_body_dynamics.cc
  src/models/composite_rigid_body_inertia.cc
  src/models/crbi_helper.cc
  src/models/examples/atlas_crbi_helper.c
  src/models/examples/atlas_composite_rigid_body_inertia.cc
  src/models/examples/valkyrie_crbi_helper.c
  src/models/examples/valkyrie_composite_rigid_body_inertia.cc
  src/models/examples/nao_crbi_helper.c
  src/models/examples/nao_composite_rigid_body_inertia.cc
  # constraints
  src/constraints/time_discretization_constraint.cc
  src/constraints/base_motion_constraint.cc
  src/constraints/terrain_constraint.cc
  src/constraints/swing_constraint.cc
  src/constraints/force_constraint.cc
  src/constraints/total_duration_constraint.cc
  src/constraints/dynamic_constraint.cc
  src/constraints/range_of_motion_circle_constraint.cc
  src/constraints/range_of_motion_box_constraint.cc
  src/constraints/spline_acc_constraint.cc
  src/constraints/linear_constraint.cc
  # costs
  src/costs/node_cost.cc
  src/costs/node_difference_cost.cc
  src/costs/final_node_cost.cc
  src/costs/intermediate_node_cost.cc
  src/costs/soft_constraint.cc
  # initialization
  src/initialization/footstep.cpp
  src/initialization/dcm_planner.cpp
  # terrain
  src/terrain/examples/height_map_examples.cc
  src/terrain/height_map.cc
  # helpers
  src/variables/state.cc
  src/variables/polynomial.cc
  src/variables/spline.cc
  src/variables/phase_spline.cc
  src/variables/node_spline.cc
  src/variables/nodes_observer.cc
  src/variables/spline_holder.cc
  src/variables/euler_converter.cc
  src/variables/phase_durations_observer.cc
  # high level interface
  src/locomotion_task.cc
  src/locomotion_solution.cc
)

target_link_libraries(towr_plus PUBLIC ifopt::ifopt_core my_util dubins
    my_goldfarb)


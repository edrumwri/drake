# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:lint.bzl", "add_lint_tests")
load("//tools:install_data.bzl", "install_data")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "valkyrie_simulator",
    hdrs = ["valkyrie_simulator.h"],
    deps = [
        "//drake/common:find_resource",
        "//drake/examples/valkyrie:actuator_effort_to_rigid_body_plant_input_converter",
        "//drake/examples/valkyrie:robot_command_to_desired_effort_converter",
        "//drake/examples/valkyrie:robot_state_encoder",
        "//drake/examples/valkyrie:valkyrie_constants",
        "//drake/lcm",
        "//drake/lcmtypes:contact_info_for_viz",
        "//drake/lcmtypes:contact_results_for_viz",
        "//drake/multibody/dev:time_stepping_rigid_body_plant",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/framework",
        "//drake/systems/lcm",
        "//drake/systems/lcm:lcmt_drake_signal_translator",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:pass_through",
    ],
)

drake_cc_binary(
    name = "valkyrie_simulation",
    srcs = ["valkyrie_simulation.cc"],
    data = ["//drake/examples/valkyrie:models"],
    deps = [":valkyrie_simulator",
            "//drake/common:text_logging_gflags",
            "@com_github_gflags_gflags//:gflags",
],
)

# === test/ ===

#drake_cc_googletest(
#    name = "valkyrie_simulation_test",
#    srcs = ["test/valkyrie_simulation_test.cc"],
#    data = [":models"],
#    deps = [":valkyrie_simulator"],
#)

install_data()

add_lint_tests()

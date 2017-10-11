# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
    "drake_cc_googletest",
)
load("//tools:lint.bzl", "add_lint_tests")

drake_cc_binary(
    name = "time_stepping_kuka_sim",
    srcs = ["time_stepping_kuka_sim.cc"],
    data = [
        "//drake/examples/kuka_iiwa_arm:models",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        "//drake/common:find_resource",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/lcm",
        "//drake/multibody/dev:time_stepping_rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/primitives:trajectory_source",
        "@com_github_gflags_gflags//:gflags",
    ],
)

add_lint_tests()

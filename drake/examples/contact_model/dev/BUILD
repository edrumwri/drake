# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_binary",
)
load("//tools:install_data.bzl", "install_data")
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_binary(
    name = "sliding_bricks",
    srcs = ["sliding_bricks.cc"],
    data = ["//drake/examples/contact_model:sliding_bricks"],
    deps = [
        "//drake/common:find_resource",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/dev:time_stepping_rigid_body_plant",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/primitives:constant_vector_source",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "rigid_gripper",
    srcs = ["rigid_gripper.cc"],
    data = [
	"//drake/examples/contact_model:rigid_gripper",
#        "rigid_gripper.urdf",
        "//drake/multibody:models",
    ],
    deps = [
        "//drake/common:find_resource",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/multibody/dev:time_stepping_rigid_body_plant",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "bowling_ball",
    srcs = ["bowling_ball.cc"],
    add_test_rule = 1,
    data = [
        "//drake/examples/contact_model:bowling_models",
    ],
    test_rule_args = [
        " --sim_duration=0.01",
        " --playback=false",
    ],
    deps = [
        "//drake/common:find_resource",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/multibody/dev:time_stepping_rigid_body_plant",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/lcm",
        "@com_github_gflags_gflags//:gflags",
    ],
)

filegroup(
    name = "bowling_models",
    srcs = [
        "bowling_ball.urdf",
        "pin.obj",
        "pin.urdf",
    ],
)

install_data()

add_lint_tests()

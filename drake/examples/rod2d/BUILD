# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library", 
     "drake_cc_binary")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "rod2d_state_vector",
    srcs = ["gen/rod2d_state_vector.cc"],
    hdrs = ["gen/rod2d_state_vector.h"],
    deps = [
      "//drake/systems/framework:vector",
    ]
)

drake_cc_binary(
    name = "rod2d_sim",
    srcs = ["rod2d_sim.cc"],
    deps = [
        ":rod2d",
        ":rod2d_state_vector",
        "//drake/common",
        "//drake/solvers:mathematical_program",
        "//drake/systems/framework:leaf_system",
        "//drake/lcmtypes:viewer",
        "//drake/lcm",
        "//drake/systems/analysis",
        "//drake/systems/framework",
        "//drake/systems/lcm",
        "//drake/systems/rendering:drake_visualizer_client",
        "//drake/systems/rendering:pose_aggregator",
        "//drake/systems/rendering:pose_bundle_to_draw_message",
        "@gflags//:gflags",
    ],
)

drake_cc_library(
    name = "rod2d",
    srcs = ["rod2d.cc"],
    hdrs = [
        "rigid_contact.h",
        "rod2d.h",
        "rod2d-inl.h",
        "separating_accel_witness.h",
        "signed_distance_witness.h",
        "sliding_dot_witness.h",
        "sticking_friction_forces_slack_witness.h",
    ],
    deps = [
        ":rod2d_state_vector",
        "//drake/common",
        "//drake/solvers:mathematical_program",
        "//drake/systems/framework:leaf_system",
        "//drake/systems/rendering:pose_vector",
    ],
)


# === test/ ===

drake_cc_googletest(
    name = "rod2d_test",
    deps = [
        ":rod2d",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/analysis",
    ],
)

cpplint()

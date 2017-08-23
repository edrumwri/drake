# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "time_stepping_rigid_body_plant",
    hdrs = [
        "time_stepping_rigid_body_plant.h",
    ],
    srcs = ["time_stepping_rigid_body_plant.cc"],
    deps = [
        "//drake/common",
	"//drake/multibody/rigid_body_plant",
	"//drake/multibody/constraint",
	"//drake/multibody/constraint:constraint_solver",
    ],
)


# === test/ ===

add_lint_tests()

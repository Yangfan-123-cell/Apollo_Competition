load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "best_parking_space_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libbest_parking_space.so",
    srcs = [
        "stage_approaching_parking_spot.cc",
        "stage_parking.cc",
        "best_parking_space.cc",
    ],
    hdrs = [
        "stage_approaching_parking_spot.h",
        "stage_parking.h",
        "best_parking_space.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/best_parking_space/proto:best_parking_space_proto",
    ],
)

apollo_package()

cpplint()

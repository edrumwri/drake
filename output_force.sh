#!/bin/bash

# Output the piecewise DAE
./bazel-bin/examples/rod2d/rod2d_sim --sim_duration=.5 --output_force --simulation_type=pDAE
mv force.output force.output.pDAE

# Output the compliant model
./bazel-bin/examples/rod2d/rod2d_sim --stiffness=1e6 --dissipation=1e2 --sim_duration=.5 --output_force --simulation_type=compliant
mv force.output force.output.compliant

# Output the time stepping model
./bazel-bin/examples/rod2d/rod2d_sim --stiffness=1e6 --dissipation=1e2 --dt=1e-4 --sim_duration=.5 --output_force --simulation_type=timestepping
mv force.output force.output.timestepping



#!/bin/bash

# Output the piecewise DAE
./bazel-bin/examples/rod2d/rod2d_sim --sim_duration=1 --output_state --simulation_type=pDAE
mv state.output state.output.pDAE

# Output the compliant model
./bazel-bin/examples/rod2d/rod2d_sim --sim_duration=1 --output_state --simulation_type=compliant
mv state.output state.output.compliant

# Output the time stepping model
./bazel-bin/examples/rod2d/rod2d_sim --sim_duration=1 --output_state --simulation_type=timestepping
mv state.output state.output.timestepping



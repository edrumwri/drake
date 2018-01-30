#!/bin/bash

# Output the piecewise DAE
./bazel-bin/examples/rod2d/rod2d_sim --state="0 0.707106 0.7854 -1 0 0" --mu_c=0.01 --mu_s=0.01 --sim_duration=1 --output_state --simulation_type=pDAE
mv state.output state.output.pDAE

# Output the compliant model
./bazel-bin/examples/rod2d/rod2d_sim --state="0 0.707106 0.7854 -1 0 0" --mu_c=0.01 --mu_s=0.01 --sim_duration=1 --output_state --simulation_type=compliant
mv state.output state.output.compliant

# Output the time stepping model
./bazel-bin/examples/rod2d/rod2d_sim --state="0 0.707106 0.7854 -1 0 0" --mu_c=0.01 --mu_s=0.01 --dt=1e-3 --sim_duration=1 --output_state --simulation_type=timestepping
mv state.output state.output.timestepping



#!/bin/bash

# Output the piecewise DAE
time ./bazel-bin/examples/rod2d/rod2d_sim --state="0 0 0 -1 0 0" --mu_c=0.1 --mu_s=0.1 --sim_duration=1.5 --output_state --simulation_type=pDAE --accuracy=1e-8
mv state.output state.output.pDAE

# Output the compliant model
time ./bazel-bin/examples/rod2d/rod2d_sim --state="0 0 0 -1 0 0" --mu_c=0.1 --mu_s=0.1 --sim_duration=1.5 --stiffness=1e10 --dissipation=1 --output_state --simulation_type=compliant --accuracy=1e-2
mv state.output state.output.compliant

# Output the time stepping model
time ./bazel-bin/examples/rod2d/rod2d_sim --state="0 0 0 -1 0 0" --mu_c=0.1 --mu_s=0.1 --dt=1e-7 --sim_duration=1.5 --output_state --simulation_type=timestepping
mv state.output state.output.timestepping



#!/bin/bash

# Generates the source files for the Rod2DStateVector.

me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
examples=$(dirname "$mydir")
drake=$(dirname "$examples")

namespace="drake::examples::rod2d"

source $drake/tools/lcm_vector_gen.sh

gen_vector_proto $mydir/rod2d_state.named_vector


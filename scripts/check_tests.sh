#!/bin/bash
set -o pipefail
set -e

# see testing.bash for documentation

DIR=$(dirname "$0")

source ${DIR}/testing.bash

ABS_DIR=$(get_script_path)

( cd ${ABS_DIR} && check_tests $@ )
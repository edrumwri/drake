#!/bin/bash
set -Eeo pipefail
trap ERR

# see testing.bash for documentation

DIR=$(dirname "$0")

source ${DIR}/testing.bash

ABS_DIR=$(get_script_path)

( cd ${ABS_DIR} && check_tests )
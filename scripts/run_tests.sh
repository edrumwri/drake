#!/bin/bash
set -Eeo pipefail
trap ERR

# see testing.bash for documentation

DIR=$(realpath $(dirname ${0}))

source ${DIR}/testing.bash

( cd ${DIR} && run_tests $@ )

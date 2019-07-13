# @returns the absolute path to the directory of the script.
function get_script_path()
{
    # writes the path to this script to attribute at $1
    local  __RESULTVAR=$1
    # otherwise echos result

    local SOURCE="${BASH_SOURCE[0]}"
    while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
        DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
        SOURCE="$(readlink "$SOURCE")"
        [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
    done
    local DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"

    if [[ "$__RESULTVAR" ]]; then
        eval $__RESULTVAR="'$DIR'"
    else
        echo "$DIR"
    fi
}

# This script generates an MD5 hash of each source file (*.h, *.cc), 
# makes the project and then runs unit tests.
# $1: should be the path to the build directory.
# @returns make result or (if project builds) GTest status.
function run_tests()
{
  USAGE_STR="usage: run_tests.sh <build dir>"
  if [ "$#" -ne "1" ]; then
    echo ${USAGE_STR}
    exit 1
  fi

  BUILD_DIR=$1

  if [ ! -d "${BUILD_DIR}" ]; then
    echo ${USAGE_STR}
    echo "<build dir>:'${BUILD_DIR}' is not a valid directory!"
    exit 1
  fi

  GIT_ROOT=$(git rev-parse --show-toplevel)
  # Make a new test results file.
  (cd ${GIT_ROOT} && echo "Test Results:" > ./test_results.txt)

  # Generate MD5 sum from all source code.
  # Check MD5 hash of source code against MD5 of test results file.
  for i in `(cd ${GIT_ROOT} && git ls-tree -r HEAD --name-only | grep '\.cc\|\.h' | sort -k 3)` 
  do
    (cd ${GIT_ROOT} && md5sum "$i" >> ./test_results.txt)
  done

  # Add test results to test results file.
  export CTEST_OUTPUT_ON_FAILURE=1
  (cd ${BUILD_DIR} && make -j && make test >> ${GIT_ROOT}/test_results.txt)
  # Record return status for command line exit status.
  status=$?

  echo "$(cat ${GIT_ROOT}/test_results.txt)"

  exit $status
}

# This script generates an MD5 hash of each source file (*.h, *.cc), 
# compares generated hash against the stored hashes in 'test_results.txt'
# and then checks that all unit tests passed in 'test_results.txt'.
# @returns !=0 if the hashes do not match.
#       OR 1 if the hashes match but tests fail.
#       OR 0 if hashes match and tests pass.
function check_tests(){
  GIT_ROOT=$(git rev-parse --show-toplevel)
  TEST_RESULT_FILE="${GIT_ROOT}/test_results.txt"
  echo "$(cat ${TEST_RESULT_FILE})"

  # Check MD5 hash of source code against MD5 of test results file.
  for i in `(cd ${GIT_ROOT} && git ls-tree -r HEAD --name-only | grep '\.cc\|\.h' | sort -k 3)` 
  do
    echo "Checking MD5 of file $i"
    (cd ${GIT_ROOT} && grep "$i" ${TEST_RESULT_FILE} | md5sum --strict -c)
    status=$?
    [ $status -eq 0 ] && echo "" || exit $status
  done

  echo "MD5 hash of test results file matches source code MD5 hash."

  # Check that test results file has passed to all tests.
  TEST_RESULT_PASS_STRING="100% tests passed, 0 tests failed"
  if grep -q "${TEST_RESULT_PASS_STRING}" "${TEST_RESULT_FILE}"; then
    echo "[PASS] All tests passed"
    exit 0;
  fi

  echo "[FAIL] NOT all tests passed, expected string:"
  echo "'${TEST_RESULT_PASS_STRING}'"
  exit 1;

}
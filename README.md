## Drake
### Building 
```
export WORKDIR=${HOME}
cd ${WORKDIR}
git clone https://github.com/RobotLocomotion/drake.git
mkdir drake-build
cd drake-build
cmake -DWITH_PYTHON_VERSION="3.6" -DCMAKE_BUILD_TYPE="RelWithDebInfo" ../drake
make -j
```

## DR

### Building

```
cd ${WORKDIR}
git clone git@github.com:edrumwri/DR.git
mkdir DR-build
cd DR-build
cmake -DCMAKE_BUILD_TYPE="RelWithDebInfo" -DCMAKE_PREFIX_PATH=$(realpath ${WORKDIR})/drake-build/install ../DR
make -j
```

### Running Test Code

```
export CTEST_OUTPUT_ON_FAILURE=1
make -j && make test
```

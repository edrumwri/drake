# Dextrous Robotics (DR)

## Drake (from source)

See: https://drake.mit.edu/from_source.html
for resolving any issues with this step.

### Get Source Code
```
cd ~/
git clone https://github.com/RobotLocomotion/drake.git
cd ~/drake
git remote set-url origin git@github.com:[your github user name]/drake.git
git remote add upstream git@github.com:RobotLocomotion/drake.git
git remote set-url --push upstream no_push
```

### Install Prereqs (Ubuntu 18.04)
```
cd ~/drake
./setup/ubuntu/install_prereqs.sh
```

### Building (cmake)
```
cd ~/
mkdir ~/drake-build
cd ~/drake-build
cmake -DWITH_PYTHON_VERSION="3.6" -DCMAKE_BUILD_TYPE="RelWithDebInfo" ~/drake
make -j
```

### Building (bazel)
```
cd ~/drake
bazel build //...
```

### Running Visualizer (requires Drake bazel build)
```
cd ~/drake
bazel run //tools:drake_visualizer
```

## DR

### Get Source Code
```
cd ~/
git clone git@github.com:DextrousRobotics/DR.git
cd ~/DR
git remote set-url origin git@github.com:[your github user name]/DR.git
git remote add upstream git@github.com:DextrousRobotics/DR.git
git remote set-url --push upstream no_push
```

Make a private fork of `DR` to your personal github account, your fork of `DR` should be at address `git@github.com:[your github user name]/DR.git`

### Building (cmake, requires Drake cmake build)

```
mkdir ~/DR-build
cd ~/DR-build
cmake -DCMAKE_BUILD_TYPE="RelWithDebInfo" -DCMAKE_PREFIX_PATH=$(realpath ${WORKDIR})/drake-build/install ~/DR
make -j
```

### Running Test Code

```
export CTEST_OUTPUT_ON_FAILURE=1
make -j && make test
```

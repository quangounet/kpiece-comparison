# kpiece-comparison
Comparison of AVP-RRT with OMPL-KPIECE

## kpiece_superbot
KPIECE with a 12-DOF robot

First, from inside `kpiece_superbot` folder, create new directories and compile C++ files
```
mkdir -p build/data
cd build
cmake ..
make
```
Then to run the test, run (inside build directory)
```
./SuperBotBenchmark_AllDOFs <cellSize>
```
and replace `<cellSize>` with the desired value of cell size.

## avprrt_superbot
AVP-RRT with a 12-DOF robot

First, from inside `avprrt_superbot` folder, create a directory to store results
```
mkdir data
```
Then to run the test, open `ipython` and run
```
run test_varying_dof.py
```

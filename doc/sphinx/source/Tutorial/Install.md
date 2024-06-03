# Install
## Setup the environment
### suggested platform
- operate sysem: Ubuntu 22.04.4 LTS
- compile: GCC 11.4.0
### install dependencies
This repository simulates the "AzureLoong" humanoid robot. Some dependencies including mujoco, pinocchio, eigen, quill, GLFW, jsoncpp are already contained in the code, but the simulation still needs openGL, which can be installed by executing the following instruction:
```bash
sudo apt install libglu1-mesa-dev freeglut3-dev
```
## Code Fetch and Compilation
The code can be downloaded in gitee
or you can clone the repository using git
```bash
# Clone
git clone https://atomgit.com/openloong/openloong-dyn-control.git

# Build
cd openloong-dyn-control
mkdir build
cd build
cmake ..
make

# mujoco simulation
./Walk_mpc_wbc #or ./Walk_wbc or ./Jump_mpc
```
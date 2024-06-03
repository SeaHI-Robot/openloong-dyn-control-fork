# Develop

## Developer's Guide

### Abbreviations


| prefix/suffix    | meaning                                   |
| ---------------- | ----------------------------------------- |
| *_L, _W*         | Local frame, World frame                  |
| *fe_*            | foot end                                  |
| *_L, _l, _R, _r* | Left, Right                               |
| *swing,* *sw*    | swing leg                                 |
| *stance,* *st*   | stance leg                                |
| *eul, rpy*       | angular position expressed by euler angle |
| *omega*          | angular velocity                          |
| *pos*            | linear position                           |
| *vel*            | linear velocity                           |
| *tor*, *tau*     | torque at joint                           |
| *base*           | baselink                                  |
| *_des*           | desired value                             |
| *_cur*           | current value                             |
| *_rot*           | rotation matrix                           |


### key control parameters
- MPC parameters
```C++
//MPC.h
void    set_weight(double u_weight, Eigen::MatrixXd L_diag, Eigen::MatrixXd K_diag);
//*u_weight* : the minimal weight of control input
//*L_diag* : the weight of error compared to desired values, following the order (eul, pos, omega, vel)
//*K_diag* : the weight of control input, following the order (fl, tl, fr, tr)
```

- WBC priority
```C++
//WBC_QP.cpp
std::vector<std::string taskOrder;
taskOrder.emplace_back("RedundantJoints");
taskOrder.emplace_back("static_Contact");
taskOrder.emplace_back("Roll_Pitch_Yaw_Pz");
taskOrder.emplace_back("PxPy");
taskOrder.emplace_back("SwingLeg");
taskOrder.emplace_back("HandTrack");
// add task or adjust the priority here
```

- WBC weight
```C++
//PriorityTasks.h
Eigen::MatrixXd Kp;                //weight of position error
Eigen::MatrixXd Kd;                //weight of velocity eror
//WBC_QP.h
Eigen::MatrixXd Q1;                //weight of the contact force error compared to desired, following the order (fl, tl, fr, tr)
Eigen::MatrixXd Q2;                //weight of the acceleration tracking error
```

- Swing leg trajectory
```C++
//FootPlacement.h
double kp_vx;                                 //x-direction footplacement parameter
double kp_vy;                                 //y-direction footplacement parameter
double kp_wz;                                 //z-direction posture parameter
double stepHeight;                            //the maximal step height

//FootPlacement.cpp
double    FootPlacement::Trajectory(double phase, double des1, double des2);        //z-direction posture trajectory
//phase：the phase when reaching the highest
//des1：the highest position along the trajectory
//des2：the final position of the trajectory
```

- Gait control
```C++
//GaitScheduler.h
double tSwing;                                         //the time of one step
double FzThrehold;                                     //the maximal force when touching the ground

//GaitScheduler.cpp
DataBus::LegState legState=DataBus::RSt;                //the first step state
```

- Joint parameter
```JavaScript
//JointCtrConfig.json
   "Joint-ankle-l-pitch" : {
      "PVT_LPF_Fc" : 20,
      "kd" : 5.0,
      "kp" : 50.0,
      "maxPos" : 0.61087,
      "maxSpeed" : 48.8,
      "maxTorque" : 58.5,
      "minPos" : -0.43644
   }
```

### Instructions to replace robot model
#### model file

**1. xml file preparation**

prepare the urdf (.urdf) file and mesh file (.stl) of the robot for adding the mujoco compiling tags
```XML
<mujoco>
<compiler
        meshdir="meshes/"
        balanceinertia="true"
        discardvisual="false" />
</mujoco>
```

change the working directory to `mujoco-3.x.x/bin`, run the command:
```Bash
./simulate
```

drag the urdf file into the simulation interface, after the model displaying correctly save the xml file. You should note the path of the mesh files.

You can also reference the Mujoco [documentation](https://mujoco.readthedocs.io/en/stable/XMLreference.html) to set tags like `compiler`, `option` or `asset` to customize `body`, `actuator` and `sensor` etc.


| parent label | child label                                                                                      |
| -----------  | -------------------------------------------------------------------------------------------------|
| *worldbody*  | define light, camera, floor and robot(inertial, joint, freejoint, geom, site, camera, light etc) |
| *actuator*   | define actuators (motor, position, velocity etc)                                                 |
| *sensor*     | define the sensors and adjust the sensors parameters like noises                                 |

**2. replace the model**

Take the "AzureDragon robot" as an example: under *base_link*, there are four tandem connections in parallel: the head *Link_head_*, the waist *Link\_waist\_*, the left arm *Link\_arm\_l\_*, and the right arm *Link\_arm\_r\_* branches. The left arm and right arm branches each have 7 degrees of freedom and the head branch has 2 degrees of freedom. The waist branch has 3 degrees of freedom including pitch *Link\_waist\_pitch*, roll *Link\_waist\_roll*, yaw *Link\_waist\_yaw*, etc., and the left leg and right leg branches are connected in parallel, and each leg is connected with three hip joints *Link\_hip* and one knee joint *Link\_knee\_* in turn, two ankle joints *Link\_ankle\_* and in total 6 degrees of freedom. This completes the configuration of all the 31 degrees of freedom.

You can reference this configuration and try to customize your configuration.

```xml
<worldbody>
    <body name="base_link" pos="x x x">
        <freejoint name="float_base" />
        <body name="body1" pos="x x x">
            <inertial pos="x x x" quat="x x x" mass="x" diaginertia="x x x"/>
            <joint name="joint1" pos="0 0 0" axis="1 0 0" limited="true" range="x x"/>
            <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="x x x 1" mesh="body1"/>
            <geom type="mesh" rgba="x x x 1" mesh="body1"/>
            <body name="body2" pos="x x x">
                ...
                <joint name="joint2" .../>
                ...
            </body>
        </body>
        <body name="body3" pos="x x x">
            ...
            <joint name="joint3" .../>
            ...
        </body>
    </body>
</worldbody>
```

In this case, two branches are connected in parallel under *base_link*, one branch consists of *body1* and *body2* in series, and the other branch consists of *body3*. If the robot has a floating base, add the free joint *freejoint* under *body* named *base_link* above. If the robot is a fixed base, remove the *freejoint* . Optionally, *freejoint* can be masked out during the model configuration phase, if desired.

This project sets actuator for each of the 31 joints.

```XML
<actuator>
    <motor name="motor1"  joint="joint1" gear="x" ctrllimited="true" ctrlrange="x x"/>
    ...
</actuator>
```

The user can define the corresponding actuators at the active joints depending on the degrees of freedom of the robot.

The project is configured with sensors such as quaternion *framequat*, velocimeter *velocimeter*, angular velocimeter *gyro*, accelerometer *accelerometer*, which are mounted at the *site* already defined in the *body* tag, and can be added according to the needs of *touch*, *force*, *torque*, *jointpos*, *jointvel*, *actuatorfrc* and other sensors can be added as required.

```XML
<sensor>
    <framequat name="xx" objtype="site" objname="imu" />
    <velocimeter name="xx" site="imu" />
    <gyro name="xx" site="imu" />
    <accelerometer name="xx" site="imu" />
</sensor>
```
In addition to the degree of freedom configuration, actuator configuration, sensor configuration, other more specific parameter modifications can refer to the Mojoco official [documentation](https://mujoco.readthedocs.io/en/stable/XMLreference.html).


#### Control code and Mujoco interface
Use fuction `mj_loadXML`, `mj_makeData`to get `mjModel`, `mjData` struct. You can reference the [documentation](https://mujoco.readthedocs.io/en/stable/XMLreference.html) for more details of `mjModel`, `mjData`, `mjOption`.
```C++
mjModel* mj_model = mj_loadXML("../Models/xxx.xml", 0, error, 1000);
mjData* mj_data = mj_makeData(mj_model);
```

`mj_model->nv` is the dimension of generalized velocity coordinate, i.e. the linear velocity, angular velocity of the floating base, and the velocity of the 31 joints of the rotational type. The variables related to the degrees of freedom in the program framework of the project are corresponding to `mj_model->nv-6`, and the dynamics library will automatically get the dimensions of the degrees of freedom of the robot according to the URDF, where all the dimension information are defined. Thus users don't have to modify it in the program manually.

As the access to the addresses of *body*, *joint*, *motor* and other components of this project relies on querying the name string and locking the address, when a component is modified, it will not affect the data reading and writing of other *body*, *joint*, and provide convenience for modifying the model compared to the direct indexing number. When modifying the control parameters of a certain degree of freedom in the model, you only need to modify the `JointName` of *MJ_Interface.h*, the `motorName` of *Pin_KinDyn.h*, the `motorName` of *PVT_Ctr.h*, and the variables corresponding to the name of a certain degree of freedom in the *JointCtrConfig.json* file. For example, to modify the stiffness of `J_waist_pitch`, you need to modify `J_waist_pitch` and the corresponding PD parameter in *JointCtrConfig.json*, and the name of `J_waist_pitch` corresponds to the *joint name*, *motor name* in the xml file.

The sensor data address is also accessed by querying the name string to find the address, adding or deleting sensors can be done by modifying the corresponding sensor name in *MJ_Interface.h*.
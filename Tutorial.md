# OpenLoong Dynamics Control

# 模型替换说明

*本文档主要为用户提供更换机器人模型的详细说明，OpenLoong Dynamics Control仓库地址如下，欢迎大家访问！*

atomgit：https://atomgit.com/openloong/openloong-dyn-control

github：https://github.com/loongOpen/openloong-dyn-control



## 1. urdf文件

urdf文件在该运动控制框架下，主要用于pinocchio动力学解算，相关修改步骤如下：

a. Solidworks导出用户urdf文件，并将其存放于*models*文件夹下；

b. pinocchio的变量定义处，更换urdf文件的路径：

```c++
    Pin_KinDyn kinDynSolver("../models/xxx.urdf");                        // kinematics and dynamics solver
```



## 2. xml文件

xml文件主要用于Mujoco仿真，相关修改步骤如下：

a. **xml类型模型文件导出**

准备机器人的URDF文件和mesh文件(STL格式)，添加用于mujoco编译的标签：

```XML
<mujoco>
<compiler
        meshdir="meshes/"
        balanceinertia="true"
        discardvisual="false" />
</mujoco>
```

切换到mujoco-3.x.x/bin目录，运行指令

```Bash
./simulate
```

将urdf文件拖拽进simulate预览界面中，保存xml，注意，mesh文件目录需要对应。

可参考Mojoco官方[文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html)，设置*compiler*,*option*等标签，定义*asset*标签将STL文件导入，定义(whold)body, *actuator, sensor*等：

| 父标签      | 子标签                                                       |
| ----------- | ------------------------------------------------------------ |
| *worldbody* | 定义灯光，相机，地板，机器人(*inertial*、*joint*、*freejoint*、*geom*、*site*、*camera*、*light*)等 |
| *actuator*  | 定义执行器motor、position、velocity等                        |
| *sensor*    | 定义需要的传感器，可添加传感器噪声                           |

其中，*sensor*安装在上述定义的*site*处，*site*并不参与碰撞以及物体质量和惯性的计算，无需担心附加的site会对仿真产生不利的影响。



 b. **关于模型的修改及替换**

以此项目的青龙机器人为例：*base_link*下并联了头*Link_head_*、腰*Link_waist_*、左臂*Link_arm_l*、右臂*Link_arm_r*等四个串联分支。其中左臂、右臂分支依次串联了7个自由度，头部分支串联了2个自由度。腰分支串联了俯仰*Link_waist_pitch*、滚转*Link_waist_roll、*偏航*Link_waist_yaw*等3个自由度后，并联了左腿、右腿两个分支，每条腿上依次串联了三个髋关节*Link_hip*、一个膝关节*Link_knee*、两个踝关节*Link_ankle*等6个自由度。至此，完成了31个自由度的配置。

可参考该串联系统，修改其中某些自由度的配置：

```XML
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

其中，*base_link*下并联了两个分支，其中一条分支由*body1*和*body2*串联而成，另外一条分支由*body3*构成。如果机器人为浮动基座，可以在上述名为*base_link*的*body*下添加自由关节*freejoint* 。如果机器人是固定基座，去掉*freejoint* 。可以根据需要，在模型配置阶段，选择将*freejoint* 屏蔽掉。

本项目为31个关节中的每一个都设置了motor类型的执行器。

```XML
<actuator>
    <motor name="motor1"  joint="joint1" gear="x" ctrllimited="true" ctrlrange="x x"/>
    ...
</actuator>
```

用户可以根据机器人的自由度的情况，在主动关节处定义相应执行器。

本项目配置了四元数*framequat*、速度计*velocimeter*、角速度计*gyro*、加速度计*accelerometer*等传感器，安装在*body*标签中已定义好的*site*处，可根据需要，添加 *touch*、*force*、*torque*、*jointpos*、*jointvel*、*actuatorfrc*等传感器。

```XML
<sensor>
    <framequat name="xx" objtype="site" objname="imu" />
    <velocimeter name="xx" site="imu" />
    <gyro name="xx" site="imu" />
    <accelerometer name="xx" site="imu" />
</sensor>
```

除自由度配置、执行器配置、传感器配置，其他更具体的参数修改可参考Mojoco官方[文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html)。



c. **与Mujoco接口**

可参考[文档](https://mujoco.readthedocs.io/en/stable/APIreference/index.html)中关于`mjModel`、`mjData`、`mjOption`等结构类型的定义，使用`mj_loadXML`、`mj_makeData`函数得到`mjModel`、`mjData`。

```C++
mjModel* mj_model = mj_loadXML("../Models/xxx.xml", 0, error, 1000);
mjData* mj_data = mj_makeData(mj_model);
```

其中`mj_model->nv`为广义坐标速度维度，为浮动基的线速度、角速度以及31个旋转类型关节的速度，本项目程序框架中与自由度相关变量均与`mj_model->nv-6`相对应，动力学库会根据URDF自动获取机器人自由度维数，与自由度相关变量的维数均据此定义，无需用户自己更改。



## 3. 控制代码

由于本项目的*body*、*joint*、*motor*等组件地址的访问方式为，依靠查询名称字符串并锁定地址，当某一组件修改时不会影响其他*body*、*joint*的数据读取及指令下发。修改模型中的某一自由度的控制参数时，只需要修改*MJ_Interface.h*的`JointName`、*Pin_KinDyn.h*的`motorName`、*PVT_Ctr.h*的`motorName`、*JointCtrConfig.json*等文件某一自由度名称对应的变量即可，例如修改`J_waist_pitch`的刚度，需修改*JointCtrConfig.json*中`J_waist_pitch`与对应的PD参数，`J_waist_pitch`名称与xml文件中的*joint name*、*motor name*相对应。

但算法中仍有根据关节序号索引的部分，青龙模型中的关节编号如下表所示，涉及广义坐标$$q$$、$$dq$$的编号需要在下表标号基础上加base的位姿信息，即$$q$$加7，$$dq$$加6，相关代码可根据用户实际关节编号更改。

| arm 1 left | arm 2 left | arm 3 left | arm 4 left | arm 5 left | arm 6 left | arm 7 left |
| :--------: | :--------: | :--------: | :--------: | :--------: | :--------: | :--------: |
|     0      |     1      |     2      |     3      |     4      |     5      |     6      |

| arm 1 right | arm 2 right | arm 3 right | arm 4 right | arm 5 right | arm 6 right | arm 7 right |
| :---------: | :---------: | :---------: | :---------: | :---------: | :---------: | :---------: |
|      7      |      8      |      9      |     10      |     11      |     12      |     13      |

| head  yaw | head pitch | waist pitch | waist roll | waist yaw |
| :-------: | :--------: | :---------: | :--------: | :-------: |
|    14     |     15     |     16      |     17     |    18     |

| hip roll left | hip yaw left | hip pitch left | knee pitch left | ankle pitch left | ankle roll left |
| :-----------: | :----------: | :------------: | :-------------: | :--------------: | :-------------: |
|      19       |      20      |       21       |       22        |        23        |       24        |

| hip roll right | hip yaw right | hip pitch right | knee pitch right | ankle pitch right | ankle roll right |
| :------------: | :-----------: | :-------------: | :--------------: | :---------------: | :--------------: |
|       25       |      26       |       27        |        28        |        29         |        30        |



```c++
//pino_kin_dyn.cpp
Pin_KinDyn::IkRes
Pin_KinDyn::computeInK_Leg(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R,
                           const Eigen::Vector3d &Pdes_R)
{
                               ...
	qIk[22] = -0.1;//left knee swing back
	qIk[28] = -0.1;//right knee
                               ...
	JL.block(0, 16, 6, 3).setZero();//16,waist pitch
	JR.block(0, 16, 6, 3).setZero();//16,waist pitch
}
```



```C++
//wbc_priority.cpp
void WBC_priority::computeDdq(Pin_KinDyn &pinKinDynIn){
    ...
    id = kin_tasks_walk.getId("RedundantJoints");
    kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(5);
    kin_tasks_walk.taskLib[id].errX(0) = 0 - q(21);//21: head yaw
    kin_tasks_walk.taskLib[id].errX(1) = 0 - q(22);//22: head pitch
    kin_tasks_walk.taskLib[id].errX(2) = 0 - q(23);//23: waist pitch
    kin_tasks_walk.taskLib[id].errX(3) = 0 - q(24);//24: waist roll
    kin_tasks_walk.taskLib[id].errX(4) = 0 - q(25);//25: waist yaw
    ...
    kin_tasks_walk.taskLib[id].J(0, 20) = 1;//20: head yaw
    kin_tasks_walk.taskLib[id].J(1, 21) = 1;//22: head pitch
    kin_tasks_walk.taskLib[id].J(2, 22) = 1;//23: waist pitch
    kin_tasks_walk.taskLib[id].J(3, 23) = 1;//24: waist roll
    kin_tasks_walk.taskLib[id].J(4, 24) = 1;//25: waist yaw
    ...
        
    id = kin_tasks_walk.getId("SwingLeg");
    ...
    kin_tasks_walk.taskLib[id].J.block(0, 22, 6, 3).setZero(); // exculde waist joints
	...
    kin_tasks_walk.taskLib[id].dJ.block(0, 22, 6, 3).setZero(); // exculde waist joints
    ...
    
    // task 6: hand track
    double l_hip_pitch = q(28) - q(34);//28: hip pitch left, 34: knee pitch right
    double r_hip_pitch = q(34) - q(28);
    ...
    target_arm_q.resize(14);//14: arm degree
    ...
    id = kin_tasks_walk.getId("HandTrackJoints");
    kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(14);
    kin_tasks_walk.taskLib[id].errX = target_arm_q - q.block<14, 1>(7, 0);
    kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(14);
    kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(14);
    kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(14);
    kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(14, 14) * 200; // 100
    kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(14, 14) * 10;
    kin_tasks_walk.taskLib[id].J = Eigen::MatrixXd::Zero(14, model_nv);
    kin_tasks_walk.taskLib[id].J.block(0, 6, 14, 14) = Eigen::MatrixXd::Identity(14, 14);
    kin_tasks_walk.taskLib[id].dJ = Eigen::MatrixXd::Zero(14, model_nv);
    kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);
    ...
        
    int id = kin_tasks_stand.getId("static_Contact");
    kin_tasks_stand.taskLib[id].J.block(0, 22, 12, 3).setZero(); // exculde waist joints
	...
       
    id = kin_tasks_stand.getId("HipRPY");
    kin_tasks_stand.taskLib[id].J.block(0, 22, 3, 3).setZero();//exculde waist joints
    kin_tasks_stand.taskLib[id].J.block(0, 6, 3, 14).setZero();//exculde arm joints
    ...
    
    id = kin_tasks_stand.getId("Pz");
    ...
    kin_tasks_stand.taskLib[id].J.block(0, 22, 1, 3).setZero();// exculde waist joints
    ...
    kin_tasks_stand.taskLib[id].dJ.block(0, 22, 1, 3).setZero();// exculde waist joints
    ...
        
    id = kin_tasks_stand.getId("CoMTrack");
    ...
    kin_tasks_stand.taskLib[id].J.block(0, 6, 2, 14).setZero();//exculde arm joints
    ...
        
    id = kin_tasks_stand.getId("CoMXY_HipRPY");
    ...
    kin_tasks_stand.taskLib[id].W.diagonal()(22) = 200;//22: waist pitch
    kin_tasks_stand.taskLib[id].W.diagonal()(23) = 200;//23: waist roll
    ...
        
    // define swing arm motion
    Eigen::VectorXd target_arm_q;
    target_arm_q.resize(14);//14: arm degree
    ...
    id = kin_tasks_stand.getId("HandTrackJoints");
    kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(14);
    kin_tasks_stand.taskLib[id].errX = target_arm_q - q.block<14, 1>(7, 0);
    kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(14);
    kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(14);
    kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(14);
    kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(14, 14) * 2000; // 100
    kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(14, 14) * 100;
    kin_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(14, model_nv);
    kin_tasks_stand.taskLib[id].J.block(0, 6, 14, 14) = Eigen::MatrixXd::Identity(14, 14);
    kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(14, model_nv);
    kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);
    ...
    
    id = kin_tasks_stand.getId("HeadRP");
    kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(2);
    kin_tasks_stand.taskLib[id].errX(0) = 0 - q(21); //21: head yaw
    kin_tasks_stand.taskLib[id].errX(1) = base_rpy_cur(1) - q(22);//22: head pitch
    ...
    kin_tasks_stand.taskLib[id].J(0, 20) = 1;//20: head yaw
    kin_tasks_stand.taskLib[id].J(1, 21) = 1;//21: head pitch
    ...
        
    id = kin_tasks_stand.getId("fixedWaist");
    kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(3);
    kin_tasks_stand.taskLib[id].errX(0) = 0 - q(23);//23: waist pitch
    kin_tasks_stand.taskLib[id].errX(1) = 0 - q(24);//24: waist roll
    kin_tasks_stand.taskLib[id].errX(2) = 0 - q(25);//25: waist yaw
    ...
    kin_tasks_stand.taskLib[id].J(0, 22) = 1;//22: waist pitch
    kin_tasks_stand.taskLib[id].J(1, 23) = 1;//23: waist roll
    kin_tasks_stand.taskLib[id].J(2, 24) = 1;//24: waist yaw
}

```


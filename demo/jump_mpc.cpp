/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "MJ_interface.h"
#include "GLFW_callbacks.h"
#include <string>
#include <iostream>
#include "data_logger.h"
#include "PVT_ctrl.h"
#include "mpc.h"
#include "pino_kin_dyn.h"
#include "useful_math.h"
#include "data_logger.h"

const double dt = 0.001;
// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel *mj_model = mj_loadXML("../models/scene.xml", 0, error, 1000);
mjData *mj_data = mj_makeData(mj_model);

int main(int argc, char **argv) {
    // initialize classes
    UIctr uiController(mj_model, mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco
    Pin_KinDyn kinDynSolver("../models/AzureLoong.urdf"); // kinematics and dynamics solver
    MPC mpc_force(dt);  // mpc controller
    PVT_Ctr pvtCtr(mj_model->opt.timestep, "../common/joint_ctrl_config.json");// PVT joint control
    DataBus RobotState(kinDynSolver.model_nv); // data bus
    DataLogger logger("../record/datalog.log"); // data logger
	int model_nv=kinDynSolver.model_nv;

    // initialize UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo",
                              false); // NOTE: if the saveVideo is set to true, the raw recorded file could be 2.5 GB for 15 seconds!


    // ini data logger quill file logger
    logger.addIterm("simTime", 1);
    logger.addIterm("motor_pos_des", model_nv  - 6);
    logger.addIterm("motor_pos_cur", model_nv - 6);
    logger.addIterm("motor_vel_cur", model_nv  - 6);
    logger.addIterm("motor_tor_des", model_nv - 6);
    logger.addIterm("motor_tor_out",model_nv  - 6);
    logger.addIterm("rpyVal", 3);
    logger.addIterm("gpsVal", 3);
    logger.addIterm("fe_l_pos_L_des", 3);
    logger.addIterm("fe_r_pos_L_des", 3);
	logger.addIterm("fe_l_pos_L", 3);
	logger.addIterm("fe_r_pos_L", 3);
	logger.addIterm("Ufe", 12);
    logger.finishItermAdding();

    // ini position and posture for foot-end and hand
    Eigen::Vector3d fe_l_pos_W_des, fe_r_pos_W_des;
    Eigen::Vector3d fe_l_pos_L_des = {-0.018, 0.113, -1.01};
    Eigen::Vector3d fe_r_pos_L_des = {-0.018, -0.116, -1.01};
    Eigen::Vector3d fe_l_eul_L_des = {-0.000, -0.008, -0.000};
    Eigen::Vector3d fe_r_eul_L_des = {0.000, -0.008, 0.000};
    Eigen::Matrix3d fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

	Eigen::Vector3d fe_l_pos_L_base, fe_r_pos_L_base;

    Eigen::Vector3d hd_l_pos_L_des={-0.02, 0.32, -0.159};
    Eigen::Vector3d hd_r_pos_L_des={-0.02, -0.32, -0.159};
    Eigen::Vector3d hd_l_eul_L_des={-1.7581, 0.2129, 2.9581};
    Eigen::Vector3d hd_r_eul_L_des={1.7581, 0.2129, -2.9581};
    Eigen::Matrix3d hd_l_rot_des= eul2Rot(hd_l_eul_L_des(0),hd_l_eul_L_des(1),hd_l_eul_L_des(2));
    Eigen::Matrix3d hd_r_rot_des= eul2Rot(hd_r_eul_L_des(0),hd_r_eul_L_des(1),hd_r_eul_L_des(2));
    auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);
    auto resHand = kinDynSolver.computeInK_Hand(hd_l_rot_des, hd_l_pos_L_des, hd_r_rot_des, hd_r_pos_L_des);

    // print info to the console
    double startJumpingTime = 8.5; // start jumping time
    double prepareTime = 3;
    uint16_t jump_state = 0;
    double stand_z = -0.8;
    double jump_z = 0.2;
    double count = 0.0;

    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;
    double simEndTime = 13;

    // Main loop:
    while (!glfwWindowShouldClose(uiController.window)) {
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim) {
            mj_step(mj_model, mj_data);
            simTime = mj_data->time;
            // Read the sensors:
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState);

            // update kinematics and dynamics info
            kinDynSolver.dataBusRead(RobotState);
            kinDynSolver.computeJ_dJ();
            kinDynSolver.computeDyn();
            kinDynSolver.dataBusWrite(RobotState);

            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Jac_stand(12, model_nv - 6);
            Jac_stand.setZero();
			Jac_stand.block(0, 0, 6, model_nv - 6) = RobotState.J_l.block(0, 6, 6, model_nv - 6);
			Jac_stand.block(6, 0, 6, model_nv - 6) = RobotState.J_r.block(0, 6, 6, model_nv - 6);

            // Enter here functions to send actuator commands, like:
            if (simTime <= prepareTime) {
                fe_l_pos_L_des = RobotState.fe_l_pos_L;
                fe_r_pos_L_des = RobotState.fe_r_pos_L;
                fe_l_pos_W_des = RobotState.base_rot * fe_l_pos_L_des;
                fe_r_pos_W_des = RobotState.base_rot * fe_r_pos_L_des;
                RobotState.motors_pos_des = eigen2std(resLeg.jointPosRes + resHand.jointPosRes);
                RobotState.motors_vel_des.assign(model_nv - 6, 0);
                RobotState.motors_tor_des.assign(model_nv - 6, 0);
				fe_l_pos_L_base = fe_l_pos_L_des;
				fe_r_pos_L_base = fe_r_pos_L_des;
            } else if (simTime < startJumpingTime && simTime > prepareTime) {
                fe_l_pos_L_des(2) = Ramp(fe_l_pos_L_des(2), stand_z, 0.1 * 0.001); // 0.5
                fe_r_pos_L_des(2) = Ramp(fe_r_pos_L_des(2), stand_z, 0.1 * 0.001);

                auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);
                auto resHand = kinDynSolver.computeInK_Hand(hd_l_rot_des, hd_l_pos_L_des, hd_r_rot_des, hd_r_pos_L_des);

                RobotState.base_pos_stand = RobotState.base_pos;
                RobotState.pfeW_stand.block<3, 1>(0, 0) = fe_l_pos_W_des;
                RobotState.pfeW_stand.block<3, 1>(3, 0) = fe_r_pos_W_des;

                RobotState.motors_pos_des = eigen2std(resLeg.jointPosRes + resHand.jointPosRes);
                RobotState.motors_vel_des.assign(model_nv - 6, 0);
                RobotState.motors_tor_des.assign(model_nv - 6, 0);

//				RobotState.motors_pos_des[19+4] = RobotState.motors_pos_des[19+4] + 0.01*RobotState.base_rpy(1);
//				RobotState.motors_pos_des[19+10] = RobotState.motors_pos_des[19+10] + 0.01*RobotState.base_rpy(1);

				RobotState.legState = DataBus::DSt;
            } else if (simTime >= startJumpingTime) {
                double jump_vel_des[3] = {0.0, 0.0, sqrt(2.0 * 9.8 * jump_z)};
                double jump_acc_t = 2.0 * (0.9 + stand_z) / jump_vel_des[2];//0.2;
                double jump_eul_des[3] = {0.0, -10.0 / 180.0 * 3.1415926, 0.0};

                if (jump_state == 0) {// Jump
                    mpc_force.enable();
                    Eigen::Matrix<double, 1, nx> L_diag;
                    Eigen::Matrix<double, 1, nu> K_diag;
                    L_diag <<
                           1.0, 10.0, 1.0,//eul
                            10.0, 1.0, 1.0,//pCoM
                            1e-5, 1e-5, 1e-5,//w
                            0.1, 0.01, 10.0;//vCoM
                    K_diag <<
                           1.0, 1.0, 1.0,//fl
                            1.0, 1.0, 1.0,
                            1.0, 1.0, 1.0,//fr
                            1.0, 1.0, 1.0, 1.0;
                    mpc_force.set_weight(1e-8, L_diag, K_diag);
                    RobotState.js_eul_des(1) = Ramp(RobotState.js_eul_des(1), jump_eul_des[1],
                                                    fabs(jump_eul_des[1] / jump_acc_t * dt));

                    RobotState.js_vel_des(0) = Ramp(RobotState.js_vel_des(0), jump_vel_des[0],
                                                    fabs(jump_vel_des[0] / jump_acc_t * dt));
                    RobotState.js_vel_des(2) = Ramp(RobotState.js_vel_des(2), jump_vel_des[2],
                                                    fabs(jump_vel_des[2] / jump_acc_t * dt));
                    RobotState.js_pos_des(2) = RobotState.js_pos_des(2) + RobotState.dq(2) * dt;//

                    fe_l_pos_L_des = RobotState.base_rot * RobotState.fe_l_pos_L;
                    fe_r_pos_L_des = RobotState.base_rot * RobotState.fe_r_pos_L;

                    if (simTime > startJumpingTime + jump_acc_t) {//改条件，腿离地
//                    if ((RobotState.fL[1] + RobotState.fR[1] > 260.0) || simTime < startJumpingTime + jump_acc_t - 0.1){{
                        jump_state = 3;
                        RobotState.pfeW0.block<3, 1>(0, 0) = fe_l_pos_W_des;
                        RobotState.pfeW0.block<3, 1>(3, 0) = fe_r_pos_W_des;
                    }
                } else if (jump_state == 3) { //up
                    mpc_force.disable();
                    fe_l_pos_W_des[0] = Ramp(fe_l_pos_W_des[0], RobotState.pfeW0[0] + 0.05,
                                             fabs(0.05 / (jump_vel_des[2] / 9.8) * dt));
                    fe_l_pos_W_des[2] = Ramp(fe_l_pos_W_des[2], stand_z, 5.0 * dt);
                    fe_r_pos_W_des[0] = Ramp(fe_r_pos_W_des[0], RobotState.pfeW0[3] + 0.05,
                                             fabs(0.05 / (jump_vel_des[2] / 9.8) * dt));
                    fe_r_pos_W_des[2] = Ramp(fe_r_pos_W_des[2], stand_z, 5.0 * dt);

                    fe_l_pos_L_des = RobotState.base_rot.inverse() * fe_l_pos_W_des;
                    fe_r_pos_L_des = RobotState.base_rot.inverse() * fe_r_pos_W_des;

                    auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des,
                                                              fe_r_pos_L_des);
                    auto resHand = kinDynSolver.computeInK_Hand(hd_l_rot_des, hd_l_pos_L_des, hd_r_rot_des,
                                                                hd_r_pos_L_des);

                    Eigen::Matrix<double, 31, 1> IKRes;
                    IKRes = resLeg.jointPosRes + resHand.jointPosRes;
                    IKRes(model_nv-6 - 8) = IKRes(model_nv-6 - 8) + RobotState.base_rpy(1) + 1.0 / 180.0 * 3.1415926;
                    IKRes(model_nv-6 - 2) = IKRes(model_nv-6 - 2) + RobotState.base_rpy(1) + 1.0 / 180.0 * 3.1415926;

                    if (RobotState.dq(2) < 0.1) {
                        jump_state = 4;
                        RobotState.pfeW0.block<3, 1>(0, 0) = RobotState.base_rot * RobotState.fe_l_pos_L;
                        RobotState.pfeW0.block<3, 1>(3, 0) = RobotState.base_rot * RobotState.fe_r_pos_L;
                    }

                    pvtCtr.enablePV();
                    RobotState.motors_pos_des = eigen2std(IKRes);
                    RobotState.motors_vel_des.assign(model_nv - 6, 0);
                    RobotState.motors_tor_des.assign(model_nv - 6, 0);

                } else if (jump_state == 4) { // down
                    mpc_force.disable();
                    fe_l_pos_W_des[0] = Ramp(fe_l_pos_W_des[0], RobotState.pfeW0[0] + 0.02, fabs(2.0 * dt));
                    fe_l_pos_W_des[2] = Ramp(fe_l_pos_W_des[2], stand_z, 10.0 * dt);
                    fe_r_pos_W_des[0] = Ramp(fe_r_pos_W_des[0], RobotState.pfeW0[3] + 0.02, fabs(2.0 * dt));
                    fe_r_pos_W_des[2] = Ramp(fe_r_pos_W_des[2], stand_z, 10.0 * dt);

                    fe_l_pos_L_des = RobotState.base_rot.inverse() * fe_l_pos_W_des;
                    fe_r_pos_L_des = RobotState.base_rot.inverse() * fe_r_pos_W_des;

                    auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des,
                                                              fe_r_pos_L_des);
                    auto resHand = kinDynSolver.computeInK_Hand(hd_l_rot_des, hd_l_pos_L_des, hd_r_rot_des,
                                                                hd_r_pos_L_des);

                    Eigen::MatrixXd IKRes;
                    IKRes = resLeg.jointPosRes + resHand.jointPosRes;
                    IKRes(model_nv-6 - 8) = IKRes(model_nv-6 - 8) + RobotState.base_rpy(1) + 1.0 / 180.0 * 3.1415926;
                    IKRes(model_nv-6 - 2) = IKRes(model_nv-6 - 2) + RobotState.base_rpy(1) + 1.0 / 180.0 * 3.1415926;

                    if (simTime > startJumpingTime + 0.65) {
                        jump_state = 5;
                        RobotState.pfeW0.block<3, 1>(0, 0) = RobotState.base_rot * RobotState.fe_l_pos_L;
                        RobotState.pfeW0.block<3, 1>(3, 0) = RobotState.base_rot * RobotState.fe_r_pos_L;
                        RobotState.js_pos_des(0) = RobotState.base_pos(0);
                        RobotState.js_pos_des(1) = RobotState.base_pos(1);
                        RobotState.js_pos_des(2) = RobotState.base_pos(2);
                    }

                    pvtCtr.enablePV();
                    RobotState.motors_pos_des = eigen2std(IKRes);
                    RobotState.motors_vel_des.assign(model_nv - 6, 0);
                    RobotState.motors_tor_des.assign(model_nv - 6, 0);
                } else if (jump_state == 5) {
                    mpc_force.enable();
                    Eigen::Matrix<double, 1, nx> L_diag;
                    Eigen::Matrix<double, 1, nu> K_diag;
                    L_diag <<
                           2.0, 8.0, 1.0,//eul 1, 100,  1
                            100.0, 100.0, 200.0,//pCoM
                            1e-4, 1e-4, 2e-4,//w
                            0.5, 0.01, 0.3;//vCoM
                    K_diag <<
                           1.0, 0.5, 1.0,//fl
                            1.0, 10.0, 1.0,
                            1.0, 0.5, 1.0,//fr
                            1.0, 10.0, 1.0, 1.0;
                    mpc_force.set_weight(1e-8, L_diag, K_diag);

                    double tt = 0.4;
                    RobotState.js_pos_des(2) = Ramp(RobotState.js_pos_des(2), 1.08, 1 * dt);

                    for (int i = 0; i < 3; i++)
                        RobotState.js_eul_des(i) = Ramp(RobotState.js_eul_des(i), 0.0,
                                                        fabs(30.0 / 180.0 * 3.1415926 * dt));
                    for (int i = 0; i < 3; i++)
                        RobotState.js_omega_des(i) = Ramp(RobotState.js_omega_des(i), 0.0,
                                                          fabs(50.0 / 180.0 * 3.1415926 * dt));
                    RobotState.js_vel_des(0) = Ramp(RobotState.js_vel_des(0), 0.0, 10.0 * dt);
                    RobotState.js_vel_des(1) = Ramp(RobotState.js_vel_des(1), 0.0, 10.0 * dt);
                    RobotState.js_vel_des(2) = Ramp(RobotState.js_vel_des(2), 0.0, 50.0 * dt);
                    if (count < tt / dt)
                        count = count + 1.0;

                }
            }
            // ------------- MPC ------------
            mpc_force.dataBusRead(RobotState);
            mpc_force.cal();
            mpc_force.dataBusWrite(RobotState);
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Uje(model_nv-6, 1);
            Uje.setZero();
            if (mpc_force.get_ENA()) {
                Uje = Jac_stand.transpose() * (-1.0) * RobotState.fe_react_tau_cmd.block<nu - 1, 1>(nu * 0, 0);
                double jTor_max[6] = {400.0, 100.0, 400.0, 400.0, 80.0, 10.0};
                double jTor_min[6] = {-400.0, -100.0, -400.0, -400.0, -80.0, -10.0};
                for (int i = 0; i < 6; i++) {
                    Limit(Uje(i + model_nv-6-12), jTor_max[i], jTor_min[i]);
                    Limit(Uje(i + model_nv-6-12 + 6), jTor_max[i], jTor_min[i]);
                }
                for (int i = 0; i < 12; i++) {
                    pvtCtr.disablePV(i + model_nv-6-12);
                    RobotState.motors_tor_des[i + model_nv-6-12] = Uje(i + model_nv-6-12);
                }
            }

            // joint PVT controller
            pvtCtr.dataBusRead(RobotState);
            pvtCtr.setJointPD(1500.0,50,"J_hip_l_pitch");
			pvtCtr.setJointPD(2000.0,80,"J_ankle_l_pitch");
            pvtCtr.setJointPD(1500.0,50,"J_hip_r_pitch");
			pvtCtr.setJointPD(2000.0,80,"J_ankle_r_pitch");
            if (simTime <= startJumpingTime) {
                pvtCtr.calMotorsPVT(110.0 / 1000.0 / 180.0 * 3.1415);
            } else {
                pvtCtr.calMotorsPVT();
            }
            pvtCtr.dataBusWrite(RobotState);

            // give the joint torque command to Webots
            mj_interface.setMotorsTorque(RobotState.motors_tor_out);
            // data save
            logger.startNewLine();
            logger.recItermData("simTime", simTime);
            logger.recItermData("motor_pos_des", RobotState.motors_pos_des);
            logger.recItermData("motor_pos_cur", RobotState.motors_pos_cur);
            logger.recItermData("motor_vel_cur", RobotState.motors_vel_cur);
            logger.recItermData("motor_tor_des", RobotState.motors_tor_des);
            logger.recItermData("motor_tor_out", RobotState.motors_tor_out);
            logger.recItermData("rpyVal", RobotState.rpy);
            logger.recItermData("gpsVal", RobotState.base_pos);
            logger.recItermData("fe_l_pos_L_des", fe_l_pos_L_des);
            logger.recItermData("fe_r_pos_L_des", fe_r_pos_L_des);
			logger.recItermData("fe_l_pos_L", RobotState.fe_l_pos_W);
			logger.recItermData("fe_r_pos_L", RobotState.fe_r_pos_W);
			logger.recItermData("Ufe", RobotState.fe_react_tau_cmd.block<nu - 1, 1>(nu * 0, 0));
            logger.finishLine();
        };
        if (mj_data->time >= simEndTime)
            break;

        uiController.updateScene();
    };

    // free visualization storage
    uiController.Close();

    return 0;
}

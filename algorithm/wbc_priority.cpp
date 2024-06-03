/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "wbc_priority.h"
#include "iostream"

// QP_nvIn=18, QP_ncIn=22
WBC_priority::WBC_priority(int model_nv_In, int QP_nvIn, int QP_ncIn, double miu_In, double dt): QP_prob(QP_nvIn, QP_ncIn) {
    timeStep=dt;
    model_nv=model_nv_In;
    miu=miu_In;
    QP_nc=QP_ncIn;
    QP_nv=QP_nvIn;
    Sf=Eigen::MatrixXd::Zero(6,model_nv);
    Sf.block<6,6>(0,0)=Eigen::MatrixXd::Identity(6,6);
    St_qpV2=Eigen::MatrixXd::Zero(model_nv,model_nv-6); // 6 means the dims of floating base
    St_qpV2.block(6,0,model_nv-6,model_nv-6)=Eigen::MatrixXd::Identity(model_nv-6,model_nv-6);

    St_qpV1=Eigen::MatrixXd::Zero(model_nv,6); // 6 means the dims of delta_b
    St_qpV1.block<6,6>(0,0)=Eigen::MatrixXd::Identity(6,6);

    // defined in body frame
    f_z_low=10;
    f_z_upp=1400;

    tau_upp_L<<30,10,40;
    tau_low_L<<-30,-10,-40;

    qpOASES::Options options;
    options.setToMPC();
    //options.setToReliable();
    options.printLevel=qpOASES::PL_LOW;
    QP_prob.setOptions(options);

    eigen_xOpt=Eigen::VectorXd::Zero(QP_nv);
    eigen_ddq_Opt=Eigen::VectorXd::Zero(model_nv);
    eigen_fr_Opt=Eigen::VectorXd::Zero(12);
    eigen_tau_Opt=Eigen::VectorXd::Zero(model_nv-6);

    delta_q_final_kin=Eigen::VectorXd::Zero(model_nv);
    dq_final_kin=Eigen::VectorXd::Zero(model_nv);;
    ddq_final_kin=Eigen::VectorXd::Zero(model_nv);

    base_rpy_cur=Eigen::VectorXd::Zero(3);

    //  WBC task defined and order build
    kin_tasks.addTask("static_Contact");
    kin_tasks.addTask("Roll_Pitch_Yaw_Pz");
    kin_tasks.addTask("RedundantJoints");
    kin_tasks.addTask("PxPy");
    kin_tasks.addTask("PosRot");
    kin_tasks.addTask("SwingLeg");
    kin_tasks.addTask("HandTrack");
    kin_tasks.addTask("HandTrackJoints");

    std::vector<std::string> taskOrder;
    taskOrder.emplace_back("RedundantJoints");
    taskOrder.emplace_back("static_Contact");
//    taskOrder.emplace_back("Roll_Pitch_Yaw_Pz");
//    taskOrder.emplace_back("PxPy");
    taskOrder.emplace_back("PosRot");
    taskOrder.emplace_back("SwingLeg");
//    taskOrder.emplace_back("HandTrack");
    taskOrder.emplace_back("HandTrackJoints");

    kin_tasks.buildPriority(taskOrder);
}

void WBC_priority::dataBusRead(const DataBus &robotState) {
    // foot-end offset posture
    fe_L_rot_L_off=robotState.fe_L_rot_L_off;
    fe_R_rot_L_off=robotState.fe_R_rot_L_off;

    // deisred values
    base_rpy_des=robotState.base_rpy_des;
    base_rpy_cur<<robotState.rpy[0],robotState.rpy[1],robotState.rpy[2];
    base_pos_des=robotState.base_pos_des;
    swing_fe_pos_des_W=robotState.swing_fe_pos_des_W;
    swing_fe_rpy_des_W=robotState.swing_fe_rpy_des_W;
    stance_fe_pos_cur_W=robotState.stance_fe_pos_cur_W;
    stance_fe_rot_cur_W=robotState.stance_fe_rot_cur_W;
    stanceDesPos_W=robotState.stanceDesPos_W;
    hd_l_pos_cur_W=robotState.hd_l_pos_W;
    hd_r_pos_cur_W=robotState.hd_r_pos_W;
    hd_l_rot_cur_W=robotState.hd_l_rot_W;
    hd_r_rot_cur_W=robotState.hd_r_rot_W;
    des_ddq=robotState.des_ddq;
    des_dq=robotState.des_dq;
    des_delta_q=robotState.des_delta_q;
    des_q = robotState.des_q;

    // state update
    J_base=robotState.J_base;
    dJ_base=robotState.dJ_base;
    base_rot=robotState.base_rot;
    base_pos=robotState.base_pos;

    Jfe=Eigen::MatrixXd::Zero(12,model_nv);
    Jfe.block(0,0,6,model_nv)=robotState.J_l;
    Jfe.block(6,0,6,model_nv)=robotState.J_r;
    J_hd_l=robotState.J_hd_l;
    J_hd_r=robotState.J_hd_r;
    dJ_hd_l=robotState.J_hd_l;
    dJ_hd_r=robotState.J_hd_r;
    Fr_ff=robotState.Fr_ff;
    dyn_M=robotState.dyn_M;
    dyn_M_inv=robotState.dyn_M_inv;
    dyn_Ag=robotState.dyn_Ag;
    dyn_dAg=robotState.dyn_dAg;
    dyn_Non=robotState.dyn_Non;
    dq=robotState.dq;
    q=robotState.q;
    legStateCur=robotState.legState;

    if (legStateCur==DataBus::LSt) {
        Jc = robotState.J_l;
        dJc = robotState.dJ_l;
        Jsw = robotState.J_r;
        dJsw = robotState.dJ_r;
        fe_pos_sw_W = robotState.fe_r_pos_W;
        fe_rot_sw_W =robotState.fe_r_rot_W;
    }
    else {
        Jc = robotState.J_r;
        dJc = robotState.dJ_r;
        Jsw = robotState.J_l;
        dJsw = robotState.dJ_l;
        fe_pos_sw_W = robotState.fe_l_pos_W;
        fe_rot_sw_W =robotState.fe_l_rot_W;
    }

}

void WBC_priority::dataBusWrite(DataBus &robotState) {
    robotState.wbc_tauJointRes=tauJointRes;
    robotState.wbc_FrRes=eigen_fr_Opt;
    robotState.qp_cpuTime=cpu_time;
    robotState.qp_nWSR=nWSR;
    robotState.qp_status=qpStatus;

    robotState.wbc_delta_q_final=delta_q_final_kin;
    robotState.wbc_dq_final=dq_final_kin;
    robotState.wbc_ddq_final=ddq_final_kin;
}

// QP problem contains joint torque, QP_nv=6+12, QP_nc=22;
void WBC_priority::computeTau() {
    // constust the QP problem, refer to the md file for more details
    Eigen::MatrixXd eigen_qp_A1=Eigen::MatrixXd::Zero(6, QP_nv);// 18 means the sum of dims of delta_r and delta_Fr
    eigen_qp_A1.block<6,6>(0, 0)= Sf * dyn_M * St_qpV1;

    eigen_qp_A1.block<6,12>(0, 6)= -Sf * Jfe.transpose();

    Eigen::VectorXd eqRes=Eigen::VectorXd::Zero(6);
    eqRes=-Sf*dyn_M*ddq_final_kin-Sf*dyn_Non+Sf*Jfe.transpose()*Fr_ff;

    Eigen::MatrixXd W=Eigen::MatrixXd::Zero(16,12);
    W(0,0)=1;W(0,2)= sqrt(2)/2.0*miu;
    W(1,0)=-1;W(1,2)= sqrt(2)/2.0*miu;
    W(2,1)=1;W(2,2)= sqrt(2)/2.0*miu;
    W(3,1)=-1;W(3,2)= sqrt(2)/2.0*miu;
    W.block<4,4>(4,2)=Eigen::MatrixXd::Identity(4,4);
    W.block<8,6>(8,6)=W.block<8,6>(0,0);
    Eigen::VectorXd f_low=Eigen::VectorXd::Zero(16);
    Eigen::VectorXd f_upp=Eigen::VectorXd::Zero(16);
    Eigen::Vector3d tau_upp_fe, tau_low_fe;
    tau_upp_fe=fe_L_rot_L_off.transpose()*tau_upp_L;
    tau_low_fe=fe_L_rot_L_off.transpose()*tau_low_L;
    Eigen::Vector3d tau_upp_W, tau_low_W;
    tau_upp_W=stance_fe_rot_cur_W*tau_upp_fe;
    tau_low_W=stance_fe_rot_cur_W*tau_low_fe;

    f_upp.block<8,1>(0,0)<<1e10,1e10,1e10,1e10,
            f_z_upp,std::max(tau_upp_W(0),tau_low_W(0)),std::max(tau_upp_W(1),tau_low_W(1)),std::max(tau_upp_W(2),tau_low_W(2));
    f_upp.block<8,1>(8,0)=f_upp.block<8,1>(0,0);
    f_low.block<8,1>(0,0)<<0,0,0,0,
            f_z_low,std::min(tau_upp_W(0),tau_low_W(0)),std::min(tau_upp_W(1),tau_low_W(1)),std::min(tau_upp_W(2),tau_low_W(2));
    f_low.block<8,1>(8,0)=f_low.block<8,1>(0,0);

    if (legStateCur==DataBus::LSt)
    {
        f_upp(12)=0;
        f_upp(13)=0;
        f_upp(14)=0;
        f_upp(15)=0;

        f_low(12)=0;
        f_low(13)=0;
        f_low(14)=0;
        f_low(15)=0;
    }
    else if (legStateCur==DataBus::RSt)
    {
        f_upp(4)=0;
        f_upp(5)=0;
        f_upp(6)=0;
        f_upp(7)=0;

        f_low(4)=0;
        f_low(5)=0;
        f_low(6)=0;
        f_low(7)=0;
    }

    Eigen::MatrixXd eigen_qp_A2=Eigen::MatrixXd::Zero(16, 18);
    eigen_qp_A2.block<16,12>(0,6)=W;
    Eigen::VectorXd neqRes_low=Eigen::VectorXd::Zero(16);
    Eigen::VectorXd neqRes_upp=Eigen::VectorXd::Zero(16);

    neqRes_low=f_low-W*Fr_ff;
    neqRes_upp=f_upp-W*Fr_ff;

    Eigen::MatrixXd eigen_qp_A_final=Eigen::MatrixXd::Zero(QP_nc,QP_nv);
    eigen_qp_A_final.block<6,18>(0,0)=eigen_qp_A1;
    eigen_qp_A_final.block<16,18>(6,0)=eigen_qp_A2;

    Eigen::VectorXd eigen_qp_lbA=Eigen::VectorXd::Zero(22);
    Eigen::VectorXd eigen_qp_ubA=Eigen::VectorXd::Zero(22);

    eigen_qp_lbA.block<6,1>(0,0)=eqRes;
    eigen_qp_lbA.block<16,1>(6,0)=neqRes_low;
    eigen_qp_ubA.block<6,1>(0,0)=eqRes;
    eigen_qp_ubA.block<16,1>(6,0)=neqRes_upp;

    Eigen::MatrixXd eigen_qp_H=Eigen::MatrixXd::Zero(QP_nv,QP_nv);
    Q2=Eigen::MatrixXd::Identity(6,6);
    Q1=Eigen::MatrixXd::Identity(12,12);
    eigen_qp_H.block<6,6>(0,0)=Q2*2.0*1e8;
    eigen_qp_H.block<12,12>(6,6)=Q1*2.0;

    // obj: (1/2)x'Hx+x'g
    // s.t. lbA<=Ax<=ubA
    //       lb<=x<=ub
//    qpOASES::real_t qp_H[QP_nv*QP_nv];
//    qpOASES::real_t qp_A[QP_nc*QP_nv];
//    qpOASES::real_t qp_g[QP_nv];
//    qpOASES::real_t qp_lbA[QP_nc];
//    qpOASES::real_t qp_ubA[QP_nc];
//    qpOASES::real_t xOpt_iniGuess[QP_nv];

    copy_Eigen_to_real_t(qp_H,eigen_qp_H,eigen_qp_H.rows(),eigen_qp_H.cols());
    copy_Eigen_to_real_t(qp_A,eigen_qp_A_final,eigen_qp_A_final.rows(),eigen_qp_A_final.cols());
    copy_Eigen_to_real_t(qp_lbA,eigen_qp_lbA,eigen_qp_lbA.rows(),eigen_qp_lbA.cols());
    copy_Eigen_to_real_t(qp_ubA,eigen_qp_ubA,eigen_qp_ubA.rows(),eigen_qp_ubA.cols());

    qpOASES::returnValue res;
    for (int i=0;i<QP_nv;i++) {
        xOpt_iniGuess[i] = 0;
//        xOpt_iniGuess[i] =eigen_xOpt(i);
        qp_g[i] = 0;
    }
    nWSR=200;
    cpu_time=timeStep;
//    QP_prob.reset();
    res=QP_prob.init(qp_H,qp_g,qp_A,NULL,NULL,qp_lbA,qp_ubA,nWSR,&cpu_time,xOpt_iniGuess);
    qpStatus=qpOASES::getSimpleStatus(res);
//    if (res==qpOASES::SUCCESSFUL_RETURN)
//        printf("WBC-QP: successful_return\n");
//    else if (res==qpOASES::RET_MAX_NWSR_REACHED)
//        printf("WBC-QP: max_nwsr\n");
//    else if (res==qpOASES::RET_INIT_FAILED)
//        printf("WBC-QP: init_failed\n");

    qpOASES::real_t xOpt[QP_nv];
    QP_prob.getPrimalSolution(xOpt);
    if (res==qpOASES::SUCCESSFUL_RETURN)
        for (int i=0;i<QP_nv;i++)
            eigen_xOpt(i)=xOpt[i];

    eigen_ddq_Opt=ddq_final_kin;
    eigen_ddq_Opt.block<6,1>(0,0)+=eigen_xOpt.block<6,1>(0,0);
    eigen_fr_Opt=Fr_ff+eigen_xOpt.block<12,1>(6,0);

    Eigen::VectorXd tauRes;
    tauRes=dyn_M*eigen_ddq_Opt+dyn_Non-Jfe.transpose()*eigen_fr_Opt;

    tauJointRes=tauRes.block(6,0,model_nv-6,1);

    last_nWSR=nWSR;
    last_cpu_time=cpu_time;
}

void WBC_priority::computeDdq(Pin_KinDyn &kinSolver) {
    // task definition
    int id=kin_tasks.getId("static_Contact");
    kin_tasks.taskLib[id].errX=Eigen::VectorXd::Zero(3);
    kin_tasks.taskLib[id].derrX=Eigen::VectorXd::Zero(3);
    kin_tasks.taskLib[id].ddxDes=Eigen::VectorXd::Zero(3);
    kin_tasks.taskLib[id].dxDes=Eigen::VectorXd::Zero(3);
    kin_tasks.taskLib[id].kp=Eigen::MatrixXd::Identity(3,3)*0;
    kin_tasks.taskLib[id].kd=Eigen::MatrixXd::Identity(3,3)*0;
    kin_tasks.taskLib[id].J=Jc.block(0,0,3,model_nv);
    kin_tasks.taskLib[id].dJ=dJc.block(0,0,3,model_nv);
    kin_tasks.taskLib[id].W.diagonal()=Eigen::VectorXd::Ones(model_nv);

    id=kin_tasks.getId("RedundantJoints");
    kin_tasks.taskLib[id].errX=Eigen::VectorXd::Zero(5);
        kin_tasks.taskLib[id].errX(0)=0-q(21);
        kin_tasks.taskLib[id].errX(1)=0-q(22);
        kin_tasks.taskLib[id].errX(2)=0-q(23);
        kin_tasks.taskLib[id].errX(3)=0-q(24);
        kin_tasks.taskLib[id].errX(4)=0-q(25);
    kin_tasks.taskLib[id].derrX=Eigen::VectorXd::Zero(5);
    kin_tasks.taskLib[id].ddxDes=Eigen::VectorXd::Zero(5);
    kin_tasks.taskLib[id].dxDes=Eigen::VectorXd::Zero(5);
    kin_tasks.taskLib[id].kp=Eigen::MatrixXd::Identity(5,5)*200;
    kin_tasks.taskLib[id].kd=Eigen::MatrixXd::Identity(5,5)*20;
    kin_tasks.taskLib[id].J=Eigen::MatrixXd::Zero(5,model_nv);
        kin_tasks.taskLib[id].J(0,20)=1;
        kin_tasks.taskLib[id].J(1,21)=1;
        kin_tasks.taskLib[id].J(2,22)=1;
        kin_tasks.taskLib[id].J(3,23)=1;
        kin_tasks.taskLib[id].J(4,24)=1;
    kin_tasks.taskLib[id].dJ=Eigen::MatrixXd::Zero(5,model_nv);
    kin_tasks.taskLib[id].W.diagonal()=Eigen::VectorXd::Ones(model_nv);

    id=kin_tasks.getId("Roll_Pitch_Yaw_Pz");
    kin_tasks.taskLib[id].errX=Eigen::VectorXd::Zero(4);
        Eigen::Matrix3d desRot=eul2Rot(base_rpy_des(0),base_rpy_des(1),base_rpy_des(2));
        kin_tasks.taskLib[id].errX.block<3,1>(0, 0)=diffRot(base_rot, desRot);
        kin_tasks.taskLib[id].errX(3)=base_pos_des(2)-q(2);
    kin_tasks.taskLib[id].derrX=Eigen::VectorXd::Zero(4);
        kin_tasks.taskLib[id].derrX.block<3,1>(0,0)=-dq.block<3,1>(3,0);
        kin_tasks.taskLib[id].derrX(3)=0-dq(2);
    kin_tasks.taskLib[id].ddxDes=Eigen::VectorXd::Zero(4);
    kin_tasks.taskLib[id].dxDes=Eigen::VectorXd::Zero(4);
    kin_tasks.taskLib[id].kp=Eigen::MatrixXd::Identity(4,4)*2000;
    kin_tasks.taskLib[id].kd=Eigen::MatrixXd::Identity(4,4)*10;
        Eigen::MatrixXd taskMap=Eigen::MatrixXd::Zero(4, 6);
        taskMap(0, 3)=1;
        taskMap(1, 4)=1;
        taskMap(2, 5)=1;
        taskMap(3, 2)=1;
    kin_tasks.taskLib[id].J=taskMap*J_base;
    kin_tasks.taskLib[id].dJ=taskMap*dJ_base;
    kin_tasks.taskLib[id].W.diagonal()=Eigen::VectorXd::Ones(model_nv);

    id=kin_tasks.getId("PxPy");
    kin_tasks.taskLib[id].errX=Eigen::VectorXd::Zero(2);
        kin_tasks.taskLib[id].errX(1)=base_pos_des(1) - q(1);
	kin_tasks.taskLib[id].derrX=Eigen::VectorXd::Zero(2);
//    	kin_tasks.taskLib[id].derrX(0)=des_dq(0) - dq(0);//
    kin_tasks.taskLib[id].ddxDes=Eigen::VectorXd::Zero(2);
    kin_tasks.taskLib[id].dxDes=Eigen::VectorXd::Zero(2);
    kin_tasks.taskLib[id].kp=Eigen::MatrixXd::Identity(2,2)*1.0; //100
    kin_tasks.taskLib[id].kd=Eigen::MatrixXd::Identity(2,2)*0.0;
        taskMap=Eigen::MatrixXd::Zero(2, 6);
		taskMap(0, 0)=1;
        taskMap(1, 1)=1;
    kin_tasks.taskLib[id].J=taskMap*J_base;
    kin_tasks.taskLib[id].dJ=taskMap*dJ_base;
    kin_tasks.taskLib[id].W.diagonal()=Eigen::VectorXd::Ones(model_nv);

    id = kin_tasks.getId("PosRot");
        kin_tasks.taskLib[id].errX = Eigen::VectorXd::Zero(6);
            kin_tasks.taskLib[id].errX.block(0,0,3,1) = base_pos_des - q.block(0,0,3,1);
            if (fabs(kin_tasks.taskLib[id].errX(0))>0.02)
                kin_tasks.taskLib[id].errX(0)=0.02* sign(kin_tasks.taskLib[id].errX(0));
            if (fabs(kin_tasks.taskLib[id].errX(1))>0.01)
               kin_tasks.taskLib[id].errX(1)=0.01* sign(kin_tasks.taskLib[id].errX(1));
                desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
            kin_tasks.taskLib[id].errX.block<3, 1>(3, 0) = diffRot(base_rot, desRot);
        kin_tasks.taskLib[id].derrX = -dq.block(0, 0,6,1);
        kin_tasks.taskLib[id].ddxDes = Eigen::VectorXd::Zero(6);
        kin_tasks.taskLib[id].dxDes = Eigen::VectorXd::Zero(6);
        kin_tasks.taskLib[id].kp = Eigen::MatrixXd::Identity(6, 6) * 10;
//			kin_tasks.taskLib[id].kp.block(1,1,1,1)=Eigen::MatrixXd::Identity(1, 1) * 1000;
            kin_tasks.taskLib[id].kp.block(3,3,3,3)=Eigen::MatrixXd::Identity(3, 3) * 2000;
        kin_tasks.taskLib[id].kd = Eigen::MatrixXd::Identity(6, 6) * 1;
//			kin_tasks.taskLib[id].kd.block(1,1,1,1)=Eigen::MatrixXd::Identity(1, 1) * 500;
            kin_tasks.taskLib[id].kd.block(3,3,3,3)=Eigen::MatrixXd::Identity(3, 3) * 100;
        kin_tasks.taskLib[id].J = J_base;
        kin_tasks.taskLib[id].dJ = dJ_base;
        kin_tasks.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

    id=kin_tasks.getId("SwingLeg");
    kin_tasks.taskLib[id].errX=Eigen::VectorXd::Zero(6);
        kin_tasks.taskLib[id].errX.block<3,1>(0,0)=swing_fe_pos_des_W-fe_pos_sw_W;
        desRot= eul2Rot(swing_fe_rpy_des_W(0),swing_fe_rpy_des_W(1),swing_fe_rpy_des_W(2));
        kin_tasks.taskLib[id].errX.block<3,1>(3,0)=diffRot(fe_rot_sw_W, desRot);
    kin_tasks.taskLib[id].derrX=Eigen::VectorXd::Zero(6);
//        kin_tasks.taskLib[id].derrX=-Jsw*dq;
    kin_tasks.taskLib[id].ddxDes=Eigen::VectorXd::Zero(6);
    kin_tasks.taskLib[id].dxDes=Eigen::VectorXd::Zero(6);
    kin_tasks.taskLib[id].kp=Eigen::MatrixXd::Identity(6,6)*3000;
    kin_tasks.taskLib[id].kp.block<1,1>(2,2) = kin_tasks.taskLib[id].kp.block<1,1>(2,2)*0.1;
    kin_tasks.taskLib[id].kd=Eigen::MatrixXd::Identity(6,6)*20;
    kin_tasks.taskLib[id].J=Jsw;
    kin_tasks.taskLib[id].dJ=dJsw;
    kin_tasks.taskLib[id].W.diagonal()=Eigen::VectorXd::Ones(model_nv);

    // task 6: hand track
    // define swing arm motion
//    hd_l_pos_L_des<<-0.02, 0.32, -0.159;
//    hd_r_pos_L_des<<-0.02, -0.32, -0.159;
//    hd_l_eul_L_des<<-1.7581, 0.2129, 2.9581;
//    hd_r_eul_L_des<<1.7581, 0.21291, -2.9581;

    Eigen::Vector3d hd_l_eul_L_des={-1.7581, 0.2129, 2.9581};
    Eigen::Vector3d hd_r_eul_L_des={1.7581, 0.2129, -2.9581};
    Eigen::Matrix3d hd_l_rot_des= eul2Rot(hd_l_eul_L_des(0),hd_l_eul_L_des(1),hd_l_eul_L_des(2));
    Eigen::Matrix3d hd_r_rot_des= eul2Rot(hd_r_eul_L_des(0),hd_r_eul_L_des(1),hd_r_eul_L_des(2));

    Eigen::Vector3d base2shoulder_l_pos_L_des={0.0040, 0.1616, 0.3922};
    Eigen::Vector3d shoulder2hand_l_pos_L_des={-0.0240, 0.1584, -0.5512};
    Eigen::Vector3d base2shoulder_r_pos_L_des={0.0040, -0.1616, 0.3922};
    Eigen::Vector3d shoulder2hand_r_pos_L_des={-0.0240, -0.1584, -0.5512};
    double l_hip_pitch=q(28)-qIniDes(28);
    double r_hip_pitch=q(34)-qIniDes(34);
    double k=0.8;
    hd_l_rot_des= eul2Rot(0,-k*r_hip_pitch,0)*eul2Rot(hd_l_eul_L_des(0),hd_l_eul_L_des(1),hd_l_eul_L_des(2));
    hd_r_rot_des= eul2Rot(0,-k*l_hip_pitch,0)*eul2Rot(hd_r_eul_L_des(0),hd_r_eul_L_des(1),hd_r_eul_L_des(2));

    Eigen::Vector3d hd_l_pos_W_des=eul2Rot(0,-k*r_hip_pitch,0)*shoulder2hand_l_pos_L_des+base2shoulder_l_pos_L_des+base_pos;
    Eigen::Vector3d hd_r_pos_W_des=eul2Rot(0,-k*l_hip_pitch,0)*shoulder2hand_r_pos_L_des+base2shoulder_r_pos_L_des+base_pos;
    Eigen::Matrix3d hd_l_rot_W_des= hd_l_rot_des;
    Eigen::Matrix3d hd_r_rot_W_des= hd_r_rot_des;

    id=kin_tasks.getId("HandTrack");
    kin_tasks.taskLib[id].errX=Eigen::VectorXd::Zero(12);
	    kin_tasks.taskLib[id].errX.block<3,1>(0,0)=hd_l_pos_W_des-hd_l_pos_cur_W;
	    kin_tasks.taskLib[id].errX.block<3,1>(3,0)=diffRot(hd_l_rot_cur_W, hd_l_rot_W_des);
	    kin_tasks.taskLib[id].errX.block<3,1>(6,0)=hd_r_pos_W_des-hd_r_pos_cur_W;
	    kin_tasks.taskLib[id].errX.block<3,1>(9,0)=diffRot(hd_r_rot_cur_W, hd_r_rot_W_des);
    kin_tasks.taskLib[id].derrX=Eigen::VectorXd::Zero(12);
    kin_tasks.taskLib[id].ddxDes=Eigen::VectorXd::Zero(12);
    kin_tasks.taskLib[id].dxDes=Eigen::VectorXd::Zero(12);
    kin_tasks.taskLib[id].kp=Eigen::MatrixXd::Identity(12,12)*2000;
    kin_tasks.taskLib[id].kd=Eigen::MatrixXd::Identity(12,12)*20;
    kin_tasks.taskLib[id].J=Eigen::MatrixXd::Zero(12,model_nv);
        kin_tasks.taskLib[id].J.block(0,0,6,model_nv)=J_hd_l;
        kin_tasks.taskLib[id].J.block(6,0,6,model_nv)=J_hd_r;
    kin_tasks.taskLib[id].dJ=Eigen::MatrixXd::Zero(12,model_nv);
        kin_tasks.taskLib[id].dJ.block(0,0,6,model_nv)=dJ_hd_l;
        kin_tasks.taskLib[id].dJ.block(6,0,6,model_nv)=dJ_hd_r;
    kin_tasks.taskLib[id].W.diagonal()=Eigen::VectorXd::Ones(model_nv);

    // define swing arm motion
    hd_l_eul_L_des = {-1.7581, 0.2129, 2.9581};
    hd_r_eul_L_des = {1.7581, 0.2129, -2.9581};
    hd_l_rot_des = eul2Rot(hd_l_eul_L_des(0), hd_l_eul_L_des(1), hd_l_eul_L_des(2));
    hd_r_rot_des = eul2Rot(hd_r_eul_L_des(0), hd_r_eul_L_des(1), hd_r_eul_L_des(2));

    base2shoulder_l_pos_L_des = {0.0040, 0.1616, 0.3922};
    shoulder2hand_l_pos_L_des = {-0.0240, 0.1584, -0.5512};
    base2shoulder_r_pos_L_des = {0.0040, -0.1616, 0.3922};
    shoulder2hand_r_pos_L_des = {-0.0240, -0.1584, -0.5512};
//        double l_hip_pitch = 0; //q(28) - qIniDes(28);
//        double r_hip_pitch = 0; //q(34) - qIniDes(34);
    k = 0.8;
    hd_l_rot_des =
            eul2Rot(0, -k * r_hip_pitch, 0) * eul2Rot(hd_l_eul_L_des(0), hd_l_eul_L_des(1), hd_l_eul_L_des(2));
    hd_r_rot_des =
            eul2Rot(0, -k * l_hip_pitch, 0) * eul2Rot(hd_r_eul_L_des(0), hd_r_eul_L_des(1), hd_r_eul_L_des(2));
    Eigen::Vector3d hd_l_pos_L_des =
            eul2Rot(0, -k * r_hip_pitch, 0) * shoulder2hand_l_pos_L_des + base2shoulder_l_pos_L_des;//base_pos;
    Eigen::Vector3d hd_r_pos_L_des =
            eul2Rot(0, -k * l_hip_pitch, 0) * shoulder2hand_r_pos_L_des + base2shoulder_r_pos_L_des; //+ base_pos;

    auto resLeg=kinSolver.computeInK_Hand(hd_l_rot_des,hd_l_pos_L_des,hd_r_rot_des,hd_r_pos_L_des);

    id = kin_tasks.getId("HandTrackJoints");
    kin_tasks.taskLib[id].errX = Eigen::VectorXd::Zero(14);
    kin_tasks.taskLib[id].errX=resLeg.jointPosRes.block<14,1>(0,0)-q.block<14,1>(7,0);
    kin_tasks.taskLib[id].derrX = Eigen::VectorXd::Zero(14);
    kin_tasks.taskLib[id].ddxDes = Eigen::VectorXd::Zero(14);
    kin_tasks.taskLib[id].dxDes = Eigen::VectorXd::Zero(14);
    kin_tasks.taskLib[id].kp = Eigen::MatrixXd::Identity(14, 14) * 10; //100
    kin_tasks.taskLib[id].kd = Eigen::MatrixXd::Identity(14, 14) * 5;
    kin_tasks.taskLib[id].J = Eigen::MatrixXd::Zero(14,model_nv);
    kin_tasks.taskLib[id].J.block(0,6,14,14)=Eigen::MatrixXd::Identity(14,14);
    kin_tasks.taskLib[id].dJ = Eigen::MatrixXd::Zero(14,model_nv);
    kin_tasks.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

    kin_tasks.computeAll(des_delta_q,des_dq,des_ddq,dyn_M, dyn_M_inv, dq);

    // final WBC output collection

    delta_q_final_kin=kin_tasks.out_delta_q;
    dq_final_kin=kin_tasks.out_dq;
    ddq_final_kin=kin_tasks.out_ddq;

}

void WBC_priority::copy_Eigen_to_real_t(qpOASES::real_t *target, const Eigen::MatrixXd &source, int nRows, int nCols) {
        int count = 0;

        for (int i = 0; i < nRows; i++) {
            for (int j = 0; j < nCols; j++) {
                target[count++] = isinf(source(i, j)) ? qpOASES::INFTY : source(i, j);
            }
        }
}

void WBC_priority::setQini(const Eigen::VectorXd &qIni) {
    qIniDes=qIni;
}
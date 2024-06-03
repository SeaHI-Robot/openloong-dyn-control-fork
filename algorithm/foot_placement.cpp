/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "foot_placement.h"
#include "bezier_1D.h"

void FootPlacement::dataBusRead(DataBus &robotState) {
    posStart_W=robotState.swingStartPos_W;
    desV_W=robotState.js_vel_des;
    desWz_W=robotState.js_omega_des(2);
    curV_W=robotState.base_rot*robotState.dq.block<3,1>(0,0);
    phi=robotState.phi;
    hipPos_W=robotState.posHip_W;
    base_pos=robotState.base_pos;
    tSwing= robotState.tSwing;
    theta0=robotState.theta0;
    yawCur=robotState.rpy[2];
    omegaZ_W=robotState.base_omega_W(2);
    hip_width=robotState.width_hips;
    legState=robotState.legState;
}

void FootPlacement::dataBusWrite(DataBus &robotState) {
    robotState.swing_fe_rpy_des_W<<0,0,robotState.base_rpy_des(2); // WARNING! ThetaZ!
    robotState.swing_fe_pos_des_W<<pDesCur[0],pDesCur[1],pDesCur[2];
}
void FootPlacement::getSwingPos() {
    Eigen::Matrix3d KP;
    KP.setZero();
    KP(0,0)=kp_vx;KP(1,1)=kp_vy;KP(2,2)=0;

    // for linear velocity
    posDes_W=hipPos_W+KP*(desV_W-curV_W)*(-1)+0.5*tSwing*curV_W+
            curV_W*(1-phi)*tSwing;

    // for angular veloctity
    double thetaF;
    thetaF=yawCur+theta0+omegaZ_W*(1-phi)*tSwing+0.5*omegaZ_W*tSwing+kp_wz*(omegaZ_W-desWz_W);
    posDes_W(0)+=0.5*hip_width* (cos(thetaF)-cos(yawCur+theta0));
    posDes_W(1)+=0.5*hip_width* (sin(thetaF)-sin(yawCur+theta0));

    posDes_W(0)=posDes_W(0) - 0.02;
//    posDes_W(2)=STPos_W(2)-0.04;
    posDes_W(2)=base_pos(2)-legLength-0.03;

    double yOff=0.01; // positive for moving the leg inside
    if (legState==DataBus::LSt)
        posDes_W(1)+=yOff;
    else if (legState==DataBus::RSt)
        posDes_W(1)-=yOff;

    // cycloid trajectories
    if (phi < 1.0){
        pDesCur[0]=posStart_W(0)+(posDes_W(0)-posStart_W(0))/(2*3.1415)*(2*3.1415*phi-sin(2*3.1415*phi));
        pDesCur[1]=posStart_W(1)+(posDes_W(1)-posStart_W(1))/(2*3.1415)*(2*3.1415*phi-sin(2*3.1415*phi));
    }
//    pDesCur[2]=posStart_W(2)+stepHeight*0.5*(1-cos(2*3.1415*phi))+(posDes_W(2)-posStart_W(2))/(2*3.1415)*(2*3.1415*phi-sin(2*3.1415*phi));

    pDesCur[2]=posStart_W(2)+Trajectory(0.2, stepHeight, posDes_W(2)-posStart_W(2));
}

double    FootPlacement::Trajectory(double phase, double des1, double des2){
    Bezier_1D Bswpid;
    double  para0 = 5, para1 = 3;
    for (int i = 0; i < para0; i ++)
        Bswpid.P.push_back(0.0);
    for (int i = 0; i < para1; i ++)
        Bswpid.P.push_back(1.0);

    double Bsw1, Bsw2;
    Bsw1 = Bswpid.getOut(phi/phase);
    Bsw2 = Bswpid.getOut((1.4 - phi)/(1.4 - phase));

    double output;
    if (phi < phase)
        output = des1*Bsw1;
    else{
        if (Bsw2 > 0)
            output = des1 * Bsw2 + des2 * (1.0 - Bsw2);
        else
            output = des2;
    }

    return output;
}


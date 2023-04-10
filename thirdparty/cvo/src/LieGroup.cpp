/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>, Tzu-yuan Lin <tzuyuan@umich.edu>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   LieGroup.cpp
 *  @author Ross Hartley, Tzu-yuan Lin
 *  @brief  Source file for various Lie Group functions 
 *  @date   September 18, 2019
 **/

#include "LieGroup.h"

using namespace std;

const float TOLERANCE = 1e-6;

Eigen::Matrix3f skew(const Eigen::Vector3f& v) {
    // Convert vector to skew-symmetric matrix
    Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
    M << 0, -v[2], v[1],
         v[2], 0, -v[0], 
        -v[1], v[0], 0;
        return M;
}

Eigen::Vector3f unskew(const Eigen::Matrix3f& M){
    Eigen::Vector3f v = Eigen::Vector3f::Zero();
    v << M(2,1), M(0,2), M(1,0);
        return v;
}

Eigen::Matrix4f hat2(const Eigen::VectorXf& x){
    Eigen::Matrix4f X = Eigen::Matrix4f::Zero();
    X.block<3,3>(0,0) = skew(x.head(3));
    X.block<3,1>(0,3) = x.tail(3);
    return X;
}

Eigen::VectorXf wedge(const Eigen::Matrix4f& X){
    Eigen::VectorXf x(6);
    x.head(3) = unskew(X.block<3,3>(0,0));
    x.tail(3) = X.block<3,1>(0,3);
    return x;
}

Eigen::Matrix3f LeftJacobian_SO3(const Eigen::Vector3f& w){
    Eigen::Matrix3f A = skew(w);
    float theta = w.norm();
    if (theta < TOLERANCE) {
        return Eigen::Matrix3f::Identity();
    } 
    // eye(3) + ((1-cos(theta))/theta^2)*A + ((theta-sin(theta))/theta^3)*A^2;
    Eigen::Matrix3f R =  Eigen::Matrix3f::Identity() + ((1-cos(theta))/(theta*theta))*A \
                            + ((theta-sin(theta))/(theta*theta*theta))*A*A;
    return R;
}

Eigen::MatrixXf LeftJacobianInverse_SO3(const Eigen::Vector3f& w){
    Eigen::Matrix3f A = skew(w);
    float theta = w.norm();
    if (theta < TOLERANCE) {
        return Eigen::Matrix3f::Identity();
    } 
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity()-0.5*A+((1.0/(theta*theta))-(1+cos(theta))/(2.0*theta*sin(theta)))*A*A;
    return R;
}

Eigen::MatrixXf LeftJacobian_SE3(Eigen::VectorXf& v){
    Eigen::MatrixXf output = Eigen::MatrixXf::Zero(6,6);
    Eigen::VectorXf Phi = v.head(3);
    float phi = Phi.norm();
    Eigen::VectorXf Rho = v.tail(3);
    Eigen::Matrix3f Phi_skew = skew(Phi);
    Eigen::Matrix3f Rho_skew = skew(Rho);
    Eigen::Matrix3f J = LeftJacobian_SO3(Phi);
    Eigen::Matrix3f Q = Eigen::Matrix3f::Zero();

    if(phi < TOLERANCE)
        Q = 0.5*Rho_skew;
    else
    {   
        float phi2 = phi*phi;
        float phi3 = phi*phi*phi;
        float phi4 = phi*phi*phi*phi;
        float phi5 = phi*phi*phi*phi*phi;
        Q = 0.5*Rho_skew\
            + (phi-sin(phi))/phi3 * (Phi_skew*Rho_skew + Rho_skew*Phi_skew + Phi_skew*Rho_skew*Phi_skew)\
            - (1-0.5*phi2-cos(phi))/phi4 * (Phi_skew*Phi_skew*Rho_skew + Rho_skew*Phi_skew*Phi_skew \
            - 3*Phi_skew*Rho_skew*Phi_skew) - 0.5*((1-0.5*phi2-cos(phi))/phi4 \
            - 3*(phi-sin(phi)-(phi3)/6)/phi5) \
            * (Phi_skew*Rho_skew*Phi_skew*Phi_skew + Phi_skew*Phi_skew*Rho_skew*Phi_skew);
    }
    output.block<3,3>(0,0) = J;
    output.block<3,3>(0,3) = Q;
    output.block<3,3>(3,3) = J;
    
    return output;
}

Eigen::MatrixXf RightJacobian_SE3(Eigen::VectorXf& v){
    if(v.norm() < TOLERANCE)
        return Eigen::MatrixXf::Identity(6,6);
    else
        return Adjoint_SEK3(hat2(-v).exp()) * LeftJacobian_SE3(v);
}


Eigen::MatrixXf RightJacobianInverse_SE3(Eigen::VectorXf& v){
    Eigen::MatrixXf Jr = RightJacobian_SE3(v);
    if(v.norm() < TOLERANCE)
        return Eigen::MatrixXf::Identity(6,6);
    else
        return Jr.inverse()*Eigen::MatrixXf::Identity(6,6);
}


Eigen::Vector3f Log_SO3(const Eigen::Matrix3f& M){
    float theta = acos((M.trace()-1)/2);
    if(theta<TOLERANCE){
        return Eigen::Vector3f::Zero();
    }
    return unskew(theta*(M-M.transpose())/(2*sin(theta)));
}

Eigen::VectorXf Log_SE3(const Eigen::MatrixXf& X){
    Eigen::Vector3f w = Log_SO3(X.block<3,3>(0,0));
    Eigen::Vector3f v = LeftJacobianInverse_SO3(w)*X.block<3,1>(0,3);
    Eigen::VectorXf s(6);
    s << w , v;

    return s;

}


Eigen::MatrixXf Exp_SE3(const Eigen::VectorXf& v){
    Eigen::Vector3f w = v.head(3);
    Eigen::Vector3f u = v.tail(3);
    Eigen::MatrixXf X = Eigen::Matrix4f::Identity();
    X.block<3,3>(0,0) = Exp_SO3(w);
    X.block<3,1>(0,3) = LeftJacobian_SO3(w)*u;
    return X;
}

Eigen::Matrix3f Exp_SO3(const Eigen::Vector3f& w) {
    // Computes the vectorized exponential map for SO(3)
    Eigen::Matrix3f A = skew(w);
    float theta = w.norm();
    if (theta < TOLERANCE) {
        return Eigen::Matrix3f::Identity();
    } 
    Eigen::Matrix3f R =  Eigen::Matrix3f::Identity() + (sin(theta)/theta)*A + ((1-cos(theta))/(theta*theta))*A*A;
    return R;
}

Eigen::MatrixXf Exp_SEK3(const Eigen::VectorXf& v, float dt) {
    // Computes the vectorized exponential map for SE_K(3)
    int K = (v.size()-3)/3;
    Eigen::MatrixXf X = Eigen::MatrixXf::Identity(3+K,3+K);
    Eigen::Matrix3f R;
    Eigen::Matrix3f Jl;
    Eigen::Vector3f w = v.head(3);
    float theta = w.norm();
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    if (theta < TOLERANCE) {
        R = I;
        Jl = I;
    } else {
        Eigen::Matrix3f A = skew(w);
        float theta2 = theta*theta;
        float stheta = sin(dt*theta);
        float ctheta = cos(dt*theta);
        float oneMinusCosTheta2 = (1-ctheta)/(theta2);
        Eigen::Matrix3f A2 = A*A;
        R =  I + (stheta/theta)*A + oneMinusCosTheta2*A2;
        Jl = dt*I + oneMinusCosTheta2*A + ((dt*theta-stheta)/(theta2*theta))*A2;
    }
    X.block<3,3>(0,0) = R;
    for (int i=0; i<K; ++i) {
        X.block<3,1>(0,3+i) = Jl * v.segment<3>(3+3*i);
    }
    return X;
}

Eigen::MatrixXf Adjoint_SEK3(const Eigen::MatrixXf& X) {
    // Compute Adjoint(X) for X in SE_K(3)
    int K = X.cols()-3;
    Eigen::MatrixXf Adj = Eigen::MatrixXf::Zero(3+3*K, 3+3*K);
    Eigen::Matrix3f R = X.block<3,3>(0,0);
    Adj.block<3,3>(0,0) = R;
    for (int i=0; i<K; ++i) {
        Adj.block<3,3>(3+3*i,3+3*i) = R;
        Adj.block<3,3>(3+3*i,0) = skew(X.block<3,1>(0,3+i))*R;
    }
    return Adj;
}

#include "vertex_and_edge.h"

namespace g2o
{

VertexSE3ExpmapInv::VertexSE3ExpmapInv() : BaseVertex<6, SE3Quat>()
{
}

EdgeSE3Projection::EdgeSE3Projection(const Matrix3D &intrinsic) : BaseBinaryEdge<2, Vector2D, VertexPointXYZ, VertexSE3ExpmapInv>()
{
    _intrinsic = intrinsic;
}

void EdgeSE3Projection::computeError()
{
    // std::cout << "start projection computeError" << std::endl;
    const VertexPointXYZ *vi = static_cast<const VertexPointXYZ *>(_vertices[0]);
    const VertexSE3ExpmapInv *vj = static_cast<const VertexSE3ExpmapInv *>(_vertices[1]);
    _error = _measurement - projection(vi->estimate(), vj->estimate());
    // std::cout << "error: " << _error(0) << "," << _error(1) << std::endl;
    // std::cout << "finish projection computeError" << std::endl;
}

bool EdgeSE3Projection::isDepthPositive()
{
    const VertexPointXYZ *vi = static_cast<const VertexPointXYZ *>(_vertices[0]);
    const VertexSE3ExpmapInv *vj = static_cast<const VertexSE3ExpmapInv *>(_vertices[1]);
    Vector3D point = vi->estimate();
    SE3Quat pose = vj->estimate();
    
    Matrix3D rotation = pose.rotation().toRotationMatrix();
    Vector3D translation = pose.translation();
    Vector3D point_in_camera = rotation * point + translation;

    return point_in_camera(2) > 0.0;
}

Vector2D EdgeSE3Projection::projection(const Vector3D &point, const SE3Quat &pose)
{
    // std::cout << "finish projection" << std::endl;
    Matrix3D rotation = pose.rotation().toRotationMatrix();
    Vector3D translation = pose.translation();
    Vector3D project = _intrinsic * (rotation * point + translation);
    Vector2D project_2d(project(0) / project(2), project(1) / project(2));
    // std::cout << "finish projection" << std::endl;
    return project_2d;
}

void EdgeSE3Projection::linearizeOplus()
{
    // std::cout << "start projection linearizeOplus" << std::endl;
    const VertexPointXYZ *vi = static_cast<const VertexPointXYZ *>(_vertices[0]);
    const VertexSE3ExpmapInv *vj = static_cast<const VertexSE3ExpmapInv *>(_vertices[1]);
    SE3Quat pose(vj->estimate());
    Vector3D point(vi->estimate());
    Matrix3D rotation = pose.rotation().toRotationMatrix();
    Vector3D translation = pose.translation();

    Vector3D transformed_point = rotation * point + translation;
    Eigen::Matrix<double, 2, 3> A;
    A << _intrinsic(0, 0), 0.0, -(_intrinsic(0, 0) * transformed_point(0)) / transformed_point(2),
        0.0, _intrinsic(1, 1), -(_intrinsic(1, 1) * transformed_point(1)) / transformed_point(2);

    Matrix3D transformed_point_skew = skew(-1.0 * transformed_point);
    Eigen::Matrix<double, 3, 6> B;
    B.block<3, 3>(0, 0) = transformed_point_skew;
    B.block<3, 3>(0, 3) = Matrix3D::Identity();

    _jacobianOplusXi = -1.0 / transformed_point(2) * A * rotation;
    _jacobianOplusXj = -1.0 / transformed_point(2) * A * B;
    // std::cout << "finish projection linearizeOplus" << std::endl;
}

EdgeSE3ExpmapInv::EdgeSE3ExpmapInv() : BaseBinaryEdge<6, SE3Quat, VertexSE3ExpmapInv, VertexSE3ExpmapInv>()
{
}

void EdgeSE3ExpmapInv::computeError()
{
    // std::cout << "start relative pose computeError" << std::endl;
    const VertexSE3ExpmapInv *vi = static_cast<const VertexSE3ExpmapInv *>(_vertices[0]);
    const VertexSE3ExpmapInv *vj = static_cast<const VertexSE3ExpmapInv *>(_vertices[1]);
    _error = (_measurement.inverse() * vi->estimate() * (vj->estimate()).inverse()).log();
    // std::cout << "start relative pose computeError" << std::endl;
}

void EdgeSE3ExpmapInv::linearizeOplus(){

    // // assuming error is small (does not always holds)

    // _jacobianOplusXi = _measurement.inverse().adj();
    // _jacobianOplusXj = -1.0 * Eigen::Matrix<double,6,6>::Identity();

    // std::cout << "start relative pose linearizeOplus" << std::endl;
    const VertexSE3ExpmapInv* vi = static_cast<const VertexSE3ExpmapInv*>(_vertices[0]);
    const VertexSE3ExpmapInv* vj = static_cast<const VertexSE3ExpmapInv*>(_vertices[1]);
    
    SE3Quat error_transformation = _measurement.inverse() * vi->estimate() * vj->estimate().inverse();
    Vector6d error = error_transformation.log();
    Vector3D phi(error(0),error(1),error(2));
    Vector3D rho(error(3),error(4),error(5));

    // std::cout << "phi: " << phi.transpose() << std::endl << "rho: " << rho.transpose() << std::endl; 

    double theta = phi.norm();
    // std::cout << "phi angle: " << theta << std::endl;
    // std::cout << "sin: " << sin(theta) << std::endl;
    // std::cout << "cos: " << cos(theta) << std::endl;
    // std::cout << "tan: " << tan(theta) << std::endl;

    Matrix3D rho_skew = skew(rho);
    // std::cout << "rho skew: " << std::endl;
    // std::cout << rho_skew(0,0) << "," << rho_skew(0,1) << "," << rho_skew(0,2) << std::endl;
    // std::cout << rho_skew(1,0) << "," << rho_skew(1,1) << "," << rho_skew(1,2) << std::endl;
    // std::cout << rho_skew(2,0) << "," << rho_skew(2,1) << "," << rho_skew(2,2) << std::endl << std::endl;

    Eigen::Matrix<double,6,6> J_l_inv_SE3;

    if (theta > 0.001){
        // Vector3D axis = phi / theta;
        // std::cout << "axis: " << axis.transpose() << std::endl;

        // double theta_cot = theta/2.0 * tan(2.0/theta);
        // std::cout << "theta_cot: " << theta_cot << std::endl;

        Matrix3D phi_skew = skew(phi);
        // std::cout << "phi skew: " << std::endl;
        // std::cout << phi_skew(0,0) << "," << phi_skew(0,1) << "," << phi_skew(0,2) << std::endl;
        // std::cout << phi_skew(1,0) << "," << phi_skew(1,1) << "," << phi_skew(1,2) << std::endl;
        // std::cout << phi_skew(2,0) << "," << phi_skew(2,1) << "," << phi_skew(2,2) << std::endl << std::endl;

        // Get left Jacobian of SE(3) through assembly of left Jacobian of SO(3) and Q
        // Matrix3D J_l_inv = theta_cot * Matrix3D::Identity() + (1.0 - theta_cot)*axis*axis.transpose() - theta/2.0 * skew(axis);
        Matrix3D J_l_inv = Matrix3D::Identity() - 0.5 * phi_skew + ((1/(theta*theta))-((1+cos(theta))/(2*theta*sin(theta))))*phi_skew*phi_skew;
        // std::cout << "J_l_inv: " << std::endl;
        // std::cout << J_l_inv(0,0) << "," << J_l_inv(0,1) << "," << J_l_inv(0,2) << std::endl;
        // std::cout << J_l_inv(1,0) << "," << J_l_inv(1,1) << "," << J_l_inv(1,2) << std::endl;
        // std::cout << J_l_inv(2,0) << "," << J_l_inv(2,1) << "," << J_l_inv(2,2) << std::endl << std::endl;

        Matrix3D Q = 0.5 * rho_skew + (theta-sin(theta))/(theta*theta*theta)*(phi_skew*rho_skew + rho_skew*phi_skew + phi_skew*rho_skew*phi_skew) + \
                    (theta*theta+2.0*cos(theta)-2.0)/(2.0*theta*theta*theta*theta)*(phi_skew*phi_skew*rho_skew + rho_skew*phi_skew*phi_skew - \
                    3.0*phi_skew*rho_skew*phi_skew) + ((2.0*theta-3.0*sin(theta)+theta*cos(theta))/(2.0*theta*theta*theta*theta*theta))*(phi_skew*rho_skew*phi_skew*phi_skew + phi_skew*phi_skew*rho_skew*phi_skew);
        // std::cout << "Q: " << std::endl;
        // std::cout << Q(0,0) << "," << Q(0,1) << "," << Q(0,2) << std::endl;
        // std::cout << Q(1,0) << "," << Q(1,1) << "," << Q(1,2) << std::endl;
        // std::cout << Q(2,0) << "," << Q(2,1) << "," << Q(2,2) << std::endl << std::endl;


        J_l_inv_SE3.block<3,3>(0,0) = J_l_inv;
        J_l_inv_SE3.block<3,3>(3,3) = J_l_inv;
        J_l_inv_SE3.block<3,3>(3,0) = -1.0 * J_l_inv * Q * J_l_inv;
        J_l_inv_SE3.block<3,3>(0,3) = Matrix3D::Zero();

        // Get left Jacobian of SE(3) through equation (7.94) in the book State Estimation for Robotics
        // Eigen::Matrix<double,6,6> error_adj;
        // error_adj.block<3,3>(0,0) = skew(phi);
        // error_adj.block<3,3>(3,3) = skew(phi);
        // error_adj.block<3,3>(3,0) = skew(rho);

        // Eigen::Matrix<double,6,6> J_l_SE3;
        // J_l_SE3 = Eigen::Matrix<double,6,6>::Identity() + ((4.0-theta*sin(theta)-4.0*cos(theta))/(2.0*theta*theta))*error_adj + ((4.0*theta-5.0*sin(theta)+theta*cos(theta))/(2.0*theta*theta*theta))*error_adj*error_adj + \
        //           ((2.0-theta*sin(theta)-2.0*cos(theta))/(2.0*theta*theta*theta*theta))*error_adj*error_adj*error_adj + ((2.0*theta-3.0*sin(theta)+theta*cos(theta))/(2.0*theta*theta*theta*theta*theta))*error_adj*error_adj*error_adj*error_adj;

        // _jacobianOplusXi = J_l_SE3.inverse() * _measurement.inverse().adj();

    }
    else{
        J_l_inv_SE3 = Eigen::Matrix<double,6,6>::Identity();
        J_l_inv_SE3.block<3,3>(3,0) = -0.5 * rho_skew;
    }

    // std::cout << "J_l_inv_SE3: " << std::endl;
    // std::cout << J_l_inv_SE3(0,0) << "," << J_l_inv_SE3(0,1) << "," << J_l_inv_SE3(0,2) << "," << J_l_inv_SE3(0,3) << "," << J_l_inv_SE3(0,4) << "," << J_l_inv_SE3(0,5) << std::endl;
    // std::cout << J_l_inv_SE3(1,0) << "," << J_l_inv_SE3(1,1) << "," << J_l_inv_SE3(1,2) << "," << J_l_inv_SE3(1,3) << "," << J_l_inv_SE3(1,4) << "," << J_l_inv_SE3(1,5) << std::endl;
    // std::cout << J_l_inv_SE3(2,0) << "," << J_l_inv_SE3(2,1) << "," << J_l_inv_SE3(2,2) << "," << J_l_inv_SE3(2,3) << "," << J_l_inv_SE3(2,4) << "," << J_l_inv_SE3(2,5) << std::endl;
    // std::cout << J_l_inv_SE3(3,0) << "," << J_l_inv_SE3(3,1) << "," << J_l_inv_SE3(3,2) << "," << J_l_inv_SE3(3,3) << "," << J_l_inv_SE3(3,4) << "," << J_l_inv_SE3(3,5) << std::endl;
    // std::cout << J_l_inv_SE3(4,0) << "," << J_l_inv_SE3(4,1) << "," << J_l_inv_SE3(4,2) << "," << J_l_inv_SE3(4,3) << "," << J_l_inv_SE3(4,4) << "," << J_l_inv_SE3(4,5) << std::endl;
    // std::cout << J_l_inv_SE3(5,0) << "," << J_l_inv_SE3(5,1) << "," << J_l_inv_SE3(5,2) << "," << J_l_inv_SE3(5,3) << "," << J_l_inv_SE3(5,4) << "," << J_l_inv_SE3(5,5) << std::endl << std::endl;

    _jacobianOplusXi = J_l_inv_SE3 * _measurement.inverse().adj();
    _jacobianOplusXj = -1.0 * J_l_inv_SE3 * error_transformation.inverse().adj().inverse();

    // Eigen::Matrix<double,4,4> M = _measurement.inverse().to_homogeneous_matrix();
    // Eigen::Matrix<double,4,4> N = error_transformation.inverse().to_homogeneous_matrix();

    // std::cout << "measurement inverse: " << std::endl;
    // std::cout << M(0,0) << "," << M(0,1) << "," << M(0,2) << "," << M(0,3) << std::endl;
    // std::cout << M(1,0) << "," << M(1,1) << "," << M(1,2) << "," << M(1,3) << std::endl;
    // std::cout << M(2,0) << "," << M(2,1) << "," << M(2,2) << "," << M(2,3) << std::endl;
    // std::cout << M(3,0) << "," << M(3,1) << "," << M(3,2) << "," << M(3,3) << std::endl << std::endl;

    // std::cout << "error_transformation inverse: " << std::endl;
    // std::cout << N(0,0) << "," << N(0,1) << "," << N(0,2) << "," << N(0,3) << std::endl;
    // std::cout << N(1,0) << "," << N(1,1) << "," << N(1,2) << "," << N(1,3) << std::endl;
    // std::cout << N(2,0) << "," << N(2,1) << "," << N(2,2) << "," << N(2,3) << std::endl;
    // std::cout << N(3,0) << "," << N(3,1) << "," << N(3,2) << "," << N(3,3) << std::endl << std::endl;

    // std::cout << "_jacobianOplusXi: " << std::endl;
    // std::cout << _jacobianOplusXi(0,0) << "," << _jacobianOplusXi(0,1) << "," << _jacobianOplusXi(0,2) << "," << _jacobianOplusXi(0,3) << "," << _jacobianOplusXi(0,4) << "," << _jacobianOplusXi(0,5) << std::endl;
    // std::cout << _jacobianOplusXi(1,0) << "," << _jacobianOplusXi(1,1) << "," << _jacobianOplusXi(1,2) << "," << _jacobianOplusXi(1,3) << "," << _jacobianOplusXi(1,4) << "," << _jacobianOplusXi(1,5) << std::endl;
    // std::cout << _jacobianOplusXi(2,0) << "," << _jacobianOplusXi(2,1) << "," << _jacobianOplusXi(2,2) << "," << _jacobianOplusXi(2,3) << "," << _jacobianOplusXi(2,4) << "," << _jacobianOplusXi(2,5) << std::endl;
    // std::cout << _jacobianOplusXi(3,0) << "," << _jacobianOplusXi(3,1) << "," << _jacobianOplusXi(3,2) << "," << _jacobianOplusXi(3,3) << "," << _jacobianOplusXi(3,4) << "," << _jacobianOplusXi(3,5) << std::endl;
    // std::cout << _jacobianOplusXi(4,0) << "," << _jacobianOplusXi(4,1) << "," << _jacobianOplusXi(4,2) << "," << _jacobianOplusXi(4,3) << "," << _jacobianOplusXi(4,4) << "," << _jacobianOplusXi(4,5) << std::endl;
    // std::cout << _jacobianOplusXi(5,0) << "," << _jacobianOplusXi(5,1) << "," << _jacobianOplusXi(5,2) << "," << _jacobianOplusXi(5,3) << "," << _jacobianOplusXi(5,4) << "," << _jacobianOplusXi(5,5) << std::endl << std::endl;

    // std::cout << "_jacobianOplusXj: " << std::endl;
    // std::cout << _jacobianOplusXj(0,0) << "," << _jacobianOplusXj(0,1) << "," << _jacobianOplusXj(0,2) << "," << _jacobianOplusXj(0,3) << "," << _jacobianOplusXj(0,4) << "," << _jacobianOplusXj(0,5) << std::endl;
    // std::cout << _jacobianOplusXj(1,0) << "," << _jacobianOplusXj(1,1) << "," << _jacobianOplusXj(1,2) << "," << _jacobianOplusXj(1,3) << "," << _jacobianOplusXj(1,4) << "," << _jacobianOplusXj(1,5) << std::endl;
    // std::cout << _jacobianOplusXj(2,0) << "," << _jacobianOplusXj(2,1) << "," << _jacobianOplusXj(2,2) << "," << _jacobianOplusXj(2,3) << "," << _jacobianOplusXj(2,4) << "," << _jacobianOplusXj(2,5) << std::endl;
    // std::cout << _jacobianOplusXj(3,0) << "," << _jacobianOplusXj(3,1) << "," << _jacobianOplusXj(3,2) << "," << _jacobianOplusXj(3,3) << "," << _jacobianOplusXj(3,4) << "," << _jacobianOplusXj(3,5) << std::endl;
    // std::cout << _jacobianOplusXj(4,0) << "," << _jacobianOplusXj(4,1) << "," << _jacobianOplusXj(4,2) << "," << _jacobianOplusXj(4,3) << "," << _jacobianOplusXj(4,4) << "," << _jacobianOplusXj(4,5) << std::endl;
    // std::cout << _jacobianOplusXj(5,0) << "," << _jacobianOplusXj(5,1) << "," << _jacobianOplusXj(5,2) << "," << _jacobianOplusXj(5,3) << "," << _jacobianOplusXj(5,4) << "," << _jacobianOplusXj(5,5) << std::endl << std::endl;




    // _jacobianOplusXj = -1.0 * Eigen::Matrix<double,6,6>::Identity();

    // Get left Jacobian of SE(3) through equation (7.94) in the book State Estimation for Robotics
    // Eigen::Matrix<double,6,6> error_adj_minus;
    // error_adj_minus.block<3,3>(0,0) = skew(-1.0 * phi);
    // error_adj_minus.block<3,3>(3,3) = skew(-1.0 * phi);
    // error_adj_minus.block<3,3>(3,0) = skew(-1.0 * rho);

    // Eigen::Matrix<double,6,6> J_l_SE3_minus;
    // J_l_SE3_minus = Eigen::Matrix<double,6,6>::Identity() + ((4.0-theta*sin(theta)-4.0*cos(theta))/(2.0*theta*theta))*error_adj_minus + ((4.0*theta-5.0*sin(theta)+theta*cos(theta))/(2.0*theta*theta*theta))*error_adj_minus*error_adj_minus + \
    //           ((2.0-theta*sin(theta)-2.0*cos(theta))/(2.0*theta*theta*theta*theta))*error_adj_minus*error_adj_minus*error_adj_minus + ((2.0*theta-3.0*sin(theta)+theta*cos(theta))/(2.0*theta*theta*theta*theta*theta))*error_adj_minus*error_adj_minus*error_adj_minus*error_adj_minus;

    // _jacobianOplusXj = -1.0 * J_l_SE3_minus.inverse();
    // std::cout << "finish relative pose linearizeOplus" << std::endl;

    // Eigen::Matrix<double,6,6> Jxi_T_Jxi = _jacobianOplusXi.transpose() * _jacobianOplusXi;
    // Eigen::Matrix<double,6,6> Jxj_T_Jxj = _jacobianOplusXj.transpose() * _jacobianOplusXj;
    // std::cout << "determinant of Jxi_T_Jxi: " << Jxi_T_Jxi.determinant() << std::endl;
    // std::cout << "determinant of Jxj_T_Jxj: " << Jxj_T_Jxj.determinant() << std::endl << std::endl;
    
    // std::cout << "theta: " << theta << std::endl;
    // std::cout << "J_l_inv_SE3: " << std::endl;
    // std::cout << J_l_inv_SE3(0,0) << "," << J_l_inv_SE3(0,1) << "," << J_l_inv_SE3(0,2) << std::endl;
    // std::cout << J_l_inv_SE3(1,0) << "," << J_l_inv_SE3(1,1) << "," << J_l_inv_SE3(1,2) << std::endl;
    // std::cout << J_l_inv_SE3(2,0) << "," << J_l_inv_SE3(2,1) << "," << J_l_inv_SE3(2,2) << std::endl << std::endl;
}

EdgeSE3ProjectionOnlyPose::EdgeSE3ProjectionOnlyPose(const Matrix3D& intrinsic, const Vector3D& point_position) :  BaseUnaryEdge<2, Vector2D, VertexSE3ExpmapInv>()
{
    _intrinsic = intrinsic;
    _point_position = point_position;
}

void EdgeSE3ProjectionOnlyPose::computeError()
{
    // std::cout << "start projection computeError" << std::endl;
    const VertexSE3ExpmapInv *vi = static_cast<const VertexSE3ExpmapInv *>(_vertices[0]);
    _error = _measurement - projection(vi->estimate());
    // std::cout << "error: " << _error(0) << "," << _error(1) << std::endl;
    // std::cout << "finish projection computeError" << std::endl;
}

Vector2D EdgeSE3ProjectionOnlyPose::projection(const SE3Quat &pose)
{
    // std::cout << "finish projection" << std::endl;
    Matrix3D rotation = pose.rotation().toRotationMatrix();
    Vector3D translation = pose.translation();
    Vector3D project = _intrinsic * (rotation * _point_position + translation);
    Vector2D project_2d(project(0) / project(2), project(1) / project(2));
    // std::cout << "finish projection" << std::endl;
    return project_2d;
}

void EdgeSE3ProjectionOnlyPose::linearizeOplus()
{
    // std::cout << "start projection linearizeOplus" << std::endl;
    const VertexSE3ExpmapInv *vi = static_cast<const VertexSE3ExpmapInv *>(_vertices[0]);
    SE3Quat pose(vi->estimate());
    Matrix3D rotation = pose.rotation().toRotationMatrix();
    Vector3D translation = pose.translation();

    Vector3D transformed_point = rotation * _point_position + translation;
    Eigen::Matrix<double, 2, 3> A;
    A << _intrinsic(0, 0), 0.0, -(_intrinsic(0, 0) * transformed_point(0)) / transformed_point(2),
        0.0, _intrinsic(1, 1), -(_intrinsic(1, 1) * transformed_point(1)) / transformed_point(2);

    Matrix3D transformed_point_skew = skew(-1.0 * transformed_point);
    Eigen::Matrix<double, 3, 6> B;
    B.block<3, 3>(0, 0) = transformed_point_skew;
    B.block<3, 3>(0, 3) = Matrix3D::Identity();

    _jacobianOplusXi = -1.0 / transformed_point(2) * A * B;
    // std::cout << "finish projection linearizeOplus" << std::endl;
}

} // namespace g2o
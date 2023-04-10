#include <g2o/core/base_vertex.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/core/eigen_types.h>

#ifndef VERTEX_AND_EDGE_H
#define VERTEX_AND_EDGE_H

namespace g2o{

class VertexSE3ExpmapInv : public BaseVertex<6, SE3Quat>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexSE3ExpmapInv();

    virtual bool read(std::istream& is) {}

    virtual bool write(std::ostream& os) const {}

    virtual void setToOriginImpl() {_estimate = SE3Quat();}

    virtual void oplusImpl(const double* update_) {
      Eigen::Map<const Vector6d> update(update_);
      setEstimate(SE3Quat::exp(update)*estimate());
    }

    void setEstimateInv(const SE3Quat& pose) {setEstimate(pose.inverse());}

    SE3Quat estimateInv() {return _estimate.inverse();}
};

class EdgeSE3Projection: public BaseBinaryEdge<2, Vector2D, VertexPointXYZ, VertexSE3ExpmapInv>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE3Projection(const Matrix3D& intrinsic);

    virtual bool read(std::istream& is) {}

    virtual bool write(std::ostream& os) const {}

    virtual void computeError();

    bool isDepthPositive();

    virtual void linearizeOplus();

    Vector2D projection(const Vector3D& point, const SE3Quat& pose);

    double squaredError() {return _error(0)*_error(0)+_error(1)*_error(1);}

private:
    Matrix3D _intrinsic;
};

class EdgeSE3ExpmapInv: public BaseBinaryEdge<6, SE3Quat, VertexSE3ExpmapInv, VertexSE3ExpmapInv>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE3ExpmapInv();

    virtual bool read(std::istream& is) {}

    virtual bool write(std::ostream& os) const {}

    virtual void computeError();

    void getMeasurementData(Vector7d& data) {data = _measurement.toVector();}

    virtual void linearizeOplus();
};

class EdgeSE3ProjectionOnlyPose: public BaseUnaryEdge<2, Vector2D, VertexSE3ExpmapInv>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE3ProjectionOnlyPose(const Matrix3D& intrinsic, const Vector3D& point_position);

    virtual bool read(std::istream& is) {}

    virtual bool write(std::ostream& os) const {}

    virtual void computeError();

    virtual void linearizeOplus();

    Vector2D projection(const SE3Quat& pose);

private:
    Matrix3D _intrinsic;
    Vector3D _point_position;
};

} // namespace g2o

#endif

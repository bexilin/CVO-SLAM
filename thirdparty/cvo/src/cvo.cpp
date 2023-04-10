/* ----------------------------------------------------------------------------
 * Copyright 2019, Tzu-yuan Lin <tzuyuan@umich.edu>, Maani Ghaffari <maanigj@umich.edu>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   cvo.cpp
 *  @author Tzu-yuan Lin, Maani Ghaffari 
 *  @brief  Source file for contineuous visual odometry
 *  @date   september 20, 2019
 **/

#include "cvo.hpp"

namespace cvo{

cvo::cvo(const string& calib_file):
    // initialize parameters
    init(false),           // initialization indicator
    // ptr_fixed_fr(new frame),
    // ptr_moving_fr(new frame),
    // ptr_previous_fr(new frame),
    // ptr_fixed_pcd(new point_cloud),
    // ptr_moving_pcd(new point_cloud),
    // ptr_previous_pcd(new point_cloud),
    ptr_fixed_fr(new frame),
    ptr_moving_fr(nullptr),
    ptr_previous_fr(nullptr),
    ptr_fixed_pcd(new point_cloud),
    ptr_moving_pcd(nullptr),
    ptr_previous_pcd(nullptr),
    first_frame(true),
    pre_pc_init(false),
    ell(0.15),             // kernel characteristic length-scale
    sigma(0.1),            // kernel signal variance (set as std)      
    sp_thres(8e-3),        // kernel sparsification threshold       
    c(7.0),                // so(3) inner product scale     
    d(7.0),                // R^3 inner product scale
    color_scale(1.0e-5),   // color space inner product scale
    c_ell(200),             // kernel characteristic length-scale for color kernel
    c_sigma(1),
    r_weight(1),
    g_weight(1),
    b_weight(1),
    dx_weight(1),
    dy_weight(1),
    MAX_ITER(2000),        // maximum number of iteration
    min_step(2*1.0e-1),    // minimum integration step
    eps(5*1.0e-5),         // threshold for stopping the function
    eps_2(1.0e-5)         // threshold for se3 distance
    // R(Eigen::Matrix3f::Identity(3,3)), // initialize rotation matrix to I
    // T(Eigen::Vector3f::Zero()),        // initialize translation matrix to zeros
    // transform(Eigen::Affine3f::Identity()),    // initialize transformation to I
    // prev_transform(Eigen::Affine3f::Identity()),
    // accum_transform(Eigen::Affine3f::Identity())
{
    cv::FileStorage fSettings(calib_file, cv::FileStorage::READ);

    cam_info.fx = fSettings["Camera.fx"];
    cam_info.fy = fSettings["Camera.fy"];
    cam_info.cx = fSettings["Camera.cx"];
    cam_info.cy = fSettings["Camera.cy"];
    cam_info.scaling_factor = fSettings["DepthMapFactor"];
    
    R = Eigen::Matrix3f::Identity(3,3); // initialize rotation matrix to I
    T = Eigen::Vector3f::Zero();        // initialize translation matrix to zeros
    transform = Eigen::Affine3f::Identity();    // initialize transformation to I
    prev_transform = Eigen::Affine3f::Identity();
    accum_transform = Eigen::Affine3f::Identity();
}

cvo::~cvo(){
}

inline Eigen::VectorXcf cvo::poly_solver(const Eigen::VectorXf& coef){
    // extract order
    int order = coef.size()-1;
    Eigen::VectorXcf roots;
    
    // create M = diag(ones(n-1,1),-1)
    Eigen::MatrixXf M = Eigen::MatrixXf::Zero(order,order);
    M.bottomLeftCorner(order-1,order-1) = Eigen::MatrixXf::Identity(order-1,order-1);
    
    // M(1,:) = -p(2:n+1)./p(1)
    M.row(0) = -(coef/coef(0)).segment(1,order).transpose();

    // eigen(M) and get the answer
    roots = M.eigenvalues();

    return roots;
}

inline float cvo::dist_se3(const Eigen::Matrix3f& R, const Eigen::Vector3f& T){
    // create transformation matrix
    Eigen::Matrix4f temp_transform = Eigen::Matrix4f::Identity();
    temp_transform.block<3,3>(0,0)=R;
    temp_transform.block<3,1>(0,3)=T;
    
    // distance = frobenius_norm(logm(trans))
    float d = temp_transform.log().norm();
    
    return d;
}

inline void cvo::update_tf(){
    // transform = [R', -R'*T; 0,0,0,1]
    transform.matrix().block<3,3>(0,0) = R.transpose();
    transform.matrix().block<3,1>(0,3) = -R.transpose()*T;
}


inline float cvo::color_kernel(const int i, const int j){
    Eigen::Matrix<float,5,1> feature_x = ptr_fixed_pcd->features.row(i).transpose();
    Eigen::Matrix<float,5,1> feature_y = ptr_moving_pcd->features.row(j).transpose();

    return((feature_x-feature_y).squaredNorm());
}



void cvo::se_kernel(const float l, const float s2){
    A_trip_concur.clear();
    // convert k threshold to d2 threshold (so that we only need to calculate k when needed)
    float d2_thres = -2.0*l*l*log(sp_thres/s2);
    float d2_c_thres = -2.0*c_ell*c_ell*log(sp_thres/c_sigma/c_sigma);


    /** 
     * kdtreeeeeeeeeeeeeeeeeeeee
     **/

    typedef KDTreeVectorOfVectorsAdaptor<cloud_t, float>  kd_tree_t;

	kd_tree_t mat_index(3 /*dim*/, (*cloud_y), 10 /* max leaf */ );
	mat_index.index->buildIndex();

    // loop through points
    tbb::parallel_for(int(0),num_fixed,[&](int i){
    // for(int i=0; i<num_fixed; ++i){

        const float search_radius = d2_thres;
		std::vector<std::pair<size_t,float>>  ret_matches;

		nanoflann::SearchParams params;
		//params.sorted = false;

		const size_t nMatches = mat_index.index->radiusSearch(&(*cloud_x)[i](0), search_radius, ret_matches, params);

        // Eigen::Vector3f cloud_xi = (*cloud_x)[i];
        Eigen::Matrix<float,5,1> feature_x = ptr_fixed_pcd->features.row(i).transpose();
        
        // std::cout<<"nMatches: "<<nMatches<<std::endl;

        // for(int j=0; j<num_moving; j++){
        for(size_t j=0; j<nMatches; ++j){
            int idx = ret_matches[j].first;
            float d2 = ret_matches[j].second;
            // d2 = (x-y)^2
            float k = 0;
            float ck = 0;
            float d2_color = 0;
            float a = 0;
            // float d2 = 100;
            // d2 = (cloud_xi-(*cloud_y)[j]).squaredNorm();
            if(d2<d2_thres){
                // d2_color = color_kernel(i,j);
                Eigen::Matrix<float,5,1> feature_y = ptr_moving_pcd->features.row(idx).transpose();
                d2_color = ((feature_x-feature_y).squaredNorm());

                if(d2_color<d2_c_thres){
                    k = s2*exp(-d2/(2.0*l*l));
                    ck = c_sigma*c_sigma*exp(-d2_color/(2.0*c_ell*c_ell));
                    a = ck*k;
                    if (a > sp_thres) A_trip_concur.push_back(Trip_t(i,idx,a));
                }
            }
        }
    });
    // }
    // form A
    A.setFromTriplets(A_trip_concur.begin(), A_trip_concur.end());
    A.makeCompressed();
}


void cvo::compute_flow(){
    // compute SE kernel
    se_kernel(ell, sigma*sigma);

    // some initialization of the variables
    omega = Eigen::Vector3f::Zero();
    v = Eigen::Vector3f::Zero();
    Eigen::Vector3d double_omega = Eigen::Vector3d::Zero(); // this is omega in double precision
    Eigen::Vector3d double_v = Eigen::Vector3d::Zero();

    A_nonzero = 0;

    tbb::spin_mutex omegav_lock;

    // loop through points in cloud_x
    tbb::parallel_for(int(0),num_fixed,[&](int i){
        // initialize reused varaibles
        int num_non_zeros = A.innerVector(i).nonZeros();
        Eigen::MatrixXf Ai = Eigen::MatrixXf::Zero(1,num_non_zeros);
        Eigen::MatrixXf cross_xy = Eigen::MatrixXf::Zero(num_non_zeros,3);
        Eigen::MatrixXf diff_yx = Eigen::MatrixXf::Zero(num_non_zeros,3);
        Eigen::Matrix<double, 1, 3> partial_omega;
        Eigen::Matrix<double, 1, 3> partial_v;

        int j = 0;
        // loop through non-zero ids in ith row
        for(Eigen::SparseMatrix<float,Eigen::RowMajor>::InnerIterator it(A,i); it; ++it){
            int idx = it.col();
            Ai(0,j) = it.value();    // extract current value in A
            cross_xy.row(j) = ((*cloud_x)[i].transpose().cross((*cloud_y)[idx].transpose()));
            diff_yx.row(j) = ((*cloud_y)[idx]-(*cloud_x)[i]).transpose();

            ++j;
        }

        partial_omega = (1/c*Ai*cross_xy).cast<double>();
        partial_v = (1/d*Ai*diff_yx).cast<double>();

        // sum them up
        omegav_lock.lock();
        double_omega += partial_omega.transpose();
        double_v += partial_v.transpose();
        A_nonzero += num_non_zeros;
        omegav_lock.unlock();
    });

    // update them to class-wide variables
    omega = double_omega.cast<float>();
    v = double_v.cast<float>();
}


void cvo::compute_step_size(){
    // compute skew matrix
    Eigen::Matrix3f omega_hat = skew(omega);
    
    // compute xi*z+v, xi^2*z+xi*v, xi^3*z+xi^2*v, xi^4*z+xi^3*v
    Eigen::MatrixXf xiz(num_moving,3);
    Eigen::MatrixXf xi2z(num_moving,3);
    Eigen::MatrixXf xi3z(num_moving,3);
    Eigen::MatrixXf xi4z(num_moving,3);
    Eigen::MatrixXf normxiz2(num_moving,1);
    Eigen::MatrixXf xiz_dot_xi2z(num_moving,1);
    Eigen::MatrixXf epsil_const(num_moving,1);

    tbb::parallel_for( int(0), num_moving, [&]( int j ){
        Eigen::Vector3f cloud_yi = (*cloud_y)[j];
        xiz.row(j) = omega.transpose().cross(cloud_yi.transpose())+v.transpose(); // (xi*z+v)
        xi2z.row(j) = (omega_hat*omega_hat*cloud_yi\
                            +(omega_hat*v)).transpose();    // (xi^2*z+xi*v)
        xi3z.row(j) = (omega_hat*omega_hat*omega_hat*cloud_yi\
                            +(omega_hat*omega_hat*v)).transpose();  // (xi^3*z+xi^2*v)
        xi4z.row(j) = (omega_hat*omega_hat*omega_hat*omega_hat*cloud_yi\
                            +(omega_hat*omega_hat*omega_hat*v)).transpose();    // (xi^4*z+xi^3*v)
        normxiz2(j,0) = xiz.row(j).squaredNorm();
        xiz_dot_xi2z(j,0) = (-xiz.row(j).dot(xi2z.row(j)));
        epsil_const(j,0) = xi2z.row(j).squaredNorm()+2*xiz.row(j).dot(xi3z.row(j));
    });

    // initialize coefficients
    float temp_coef = 1/(2.0*ell*ell);   // 1/(2*l^2)
    double B = 0;
    double C = 0;
    double D = 0;
    double E = 0;

    tbb::spin_mutex BCDE_lock;
    
    tbb::parallel_for(int(0),num_fixed,[&](int i){
        // loop through used index in ith row
        double Bi=0;
        double Ci=0;
        double Di=0;
        double Ei=0;

        for(Eigen::SparseMatrix<float,Eigen::RowMajor>::InnerIterator it(A,i); it; ++it){
            int idx = it.col();
            
            // diff_xy = x[i] - y[used_idx[j]]
            auto diff_xy = ((*cloud_x)[i] - (*cloud_y)[idx]);    
            // beta_i = -1/l^2 * dot(xiz,diff_xy)
            float beta_ij = (-2.0*temp_coef * xiz.row(idx)*diff_xy).value();
            // gamma_i = -1/(2*l^2) * (norm(xiz).^2 + 2*dot(xi2z,diff_xy))
            float gamma_ij = (-temp_coef * (normxiz2.row(idx)\
                            + 2.0*xi2z.row(idx)*diff_xy)).value();
            // delta_i = 1/l^2 * (dot(-xiz,xi2z) + dot(-xi3z,diff_xy))
            float delta_ij = (2.0*temp_coef * (xiz_dot_xi2z.row(idx)\
                            + (-xi3z.row(idx)*diff_xy))).value();
            // epsil_i = -1/(2*l^2) * (norm(xi2z).^2 + 2*dot(xiz,xi3z) + 2*dot(xi4z,diff_xy))
            float epsil_ij = (-temp_coef * (epsil_const.row(idx)\
                            + 2.0*xi4z.row(idx)*diff_xy)).value();

            float A_ij = it.value();

            Bi += double(A_ij * beta_ij);
            Ci += double(A_ij * (gamma_ij+beta_ij*beta_ij/2.0));
            Di += double(A_ij * (delta_ij+beta_ij*gamma_ij + beta_ij*beta_ij*beta_ij/6.0));
            Ei += double(A_ij * (epsil_ij+beta_ij*delta_ij+1/2.0*beta_ij*beta_ij*gamma_ij\
                        + 1/2.0*gamma_ij*gamma_ij + 1/24.0*beta_ij*beta_ij*beta_ij*beta_ij));
        }
        
        // sum them up
        BCDE_lock.lock();
        B+=Bi;
        C+=Ci;
        D+=Di;
        E+=Ei;
        BCDE_lock.unlock();
    });

    Eigen::VectorXf p_coef(4);
    p_coef << 4.0*float(E),3.0*float(D),2.0*float(C),float(B);
    
    // solve polynomial roots
    Eigen::VectorXcf rc = poly_solver(p_coef);
    
    // find usable step size
    float temp_step = numeric_limits<float>::max();
    for(int i=0;i<rc.real().size();i++)
        if(rc(i,0).real()>0 && rc(i,0).real()<temp_step && rc(i,0).imag()==0)
            temp_step = rc(i,0).real();
    
    // if none of the roots are suitable, use min_step
    step = temp_step==numeric_limits<float>::max()? min_step:temp_step;

    // if step>0.8, just use 0.8 as step
    step = step>0.8 ? 0.8:step;
}

void cvo::transform_pcd(){
    tbb::parallel_for(int(0), num_moving, [&]( int j ){
        (*cloud_y)[j] = transform.linear()*ptr_moving_pcd->positions[j]+transform.translation();
    });
    
}



void cvo::set_pcd(const cv::Mat& RGB_img,const cv::Mat& dep_img){

    // create pcd_generator class
    pcd_generator pcd_gen;
    pcd_gen.set_calib(cam_info);
    
    // if it's the first image
    if(init == false){
        // std::cout<<"initializing cvo..."<<std::endl;
        pcd_gen.load_image(RGB_img,dep_img,ptr_fixed_fr.get());
        pcd_gen.create_pointcloud(1,ptr_fixed_fr.get(), ptr_fixed_pcd.get());
        // std::cout<<"first pcd generated!"<<std::endl;
        init = true;
        // write_pcl_point_cloud_to_disk(ptr_fixed_pcd.get(),pcd_pth,pcd_dso_pth);
        return;
    }

    ptr_moving_fr.reset(new frame);
    ptr_moving_pcd.reset(new point_cloud);

    pcd_gen.load_image(RGB_img,dep_img,ptr_moving_fr.get());
    pcd_gen.create_pointcloud(1,ptr_moving_fr.get(), ptr_moving_pcd.get());
    // write_pcl_point_cloud_to_disk(ptr_moving_pcd.get(),pcd_pth,pcd_dso_pth);

    // get total number of points
    num_fixed = ptr_fixed_pcd->num_points;
    num_moving = ptr_moving_pcd->num_points;
    // std::cout<<"num fixed: "<<num_fixed<<std::endl;
    // std::cout<<"num moving: "<<num_moving<<std::endl;

    // extract cloud x and y
    cloud_x = &(ptr_fixed_pcd->positions);
    cloud_y = new std::vector<Eigen::Vector3f>(ptr_moving_pcd->positions);

    // initialization of parameters
    A_trip_concur.reserve(num_moving*20);
    A.resize(num_fixed,num_moving);
    A.setZero();
    // ell = 0.15;

    A_nonzero = 0;
}

const inn_p cvo::function_inner_product(point_cloud* cloud_a, point_cloud* cloud_b){
    
    double sum_A = 0;
    double sum = 0;
    double sum_e = 0;

    // convert k threshold to d2 threshold (so that we only need to calculate k when needed)
    float d2_thres = -2.0*ell*ell*log(sp_thres/sigma/sigma);
    float d2_c_thres = -2.0*c_ell*c_ell*log(sp_thres/c_sigma/c_sigma);

    /** 
     * kdtreeeeeeeeeeeeeeeeeeeee
     **/
    typedef KDTreeVectorOfVectorsAdaptor<cloud_t, float>  kd_tree_t;
	kd_tree_t mat_index(3 /*dim*/, (cloud_b->positions), 10 /* max leaf */ );
	mat_index.index->buildIndex();

    tbb::spin_mutex sum_A_lock;
    // loop through points
    tbb::parallel_for(int(0),cloud_a->num_points,[&](int i){

        const float search_radius = d2_thres;
		std::vector<std::pair<size_t,float>>  ret_matches;
		nanoflann::SearchParams params;
		//params.sorted = false;
		const size_t nMatches = mat_index.index->radiusSearch(&(cloud_a->positions)[i](0), search_radius, ret_matches, params);
        Eigen::Matrix<float,5,1> feature_a = cloud_a->features.row(i).transpose();

        for(size_t j=0; j<nMatches; ++j){
            int idx = ret_matches[j].first;
            float d2 = ret_matches[j].second;
            float k = 0;
            float ck = 0;
            float d2_color = 0;
            float a = 0;
            if(d2<d2_thres){
                // d2_color = color_kernel(i,j);
                Eigen::Matrix<float,5,1> feature_b = cloud_b->features.row(idx).transpose();
                d2_color = ((feature_a-feature_b).squaredNorm());

                if(d2_color<d2_c_thres){
                    k = sigma*sigma*exp(-d2/(2.0*ell*ell));
                    ck = c_sigma*c_sigma*exp(-d2_color/(2.0*c_ell*c_ell));
                    a = ck*k;
                    sum_A_lock.lock();
                        sum_A+=a;
                        sum += 1;
                    sum_A_lock.unlock();
                    // if (a > 1e-5){
                    //     sum_A_lock.lock();
                    //         sum_A+=a;
                    //         sum += 1;
                    //     sum_A_lock.unlock();
                    // }
                    // else
                    // {
                    //     sum_e += 1;
                    // }
                    
                }
            }
        }
    });

    // std::cout << "inn_p mean: " << sum_A / sum << std::endl;
    // std::cout << "inn_p inliers: " << sum << std::endl;

    if (sum == 0)
        sum = 1;
    const inn_p result(sum_A, sum, sum_e);
    return result;
}

void cvo::match_odometry(const cv::Mat& RGB_img,const cv::Mat& dep_img, Eigen::Affine3d& transformd)
{
    if(init == false){
        std::cout << "cvo not initialized !" << "\n";
        return;
    }

    set_pcd(RGB_img, dep_img);

    align(); 

    transformd = transform.cast<double>();
}

void cvo::compute_innerproduct(inn_p& inn_pre, inn_p& inn_post, Eigen::Matrix<double, 6, 6>& post_hessian, \
                                    Eigen::Affine3f& tran, int& inliers, inn_p& inn_fixed_pcd, inn_p& inn_moving_pcd, float& cos_angle)
{

    unique_ptr<point_cloud> transformed_pcd(new point_cloud);

    transformed_pcd->num_points = ptr_moving_pcd -> num_points;
    transformed_pcd->positions = ptr_moving_pcd -> positions;
    transformed_pcd->features = ptr_moving_pcd -> features;

    tbb::parallel_for(int(0), transformed_pcd->num_points, [&]( int i ){
        transformed_pcd->positions[i] = (tran.linear()*ptr_moving_pcd->positions[i] + tran.translation());
        });

    inn_pre.copy(function_inner_product(ptr_moving_pcd.get(), ptr_fixed_pcd.get()));
    
    inn_post.copy(function_inner_product(transformed_pcd.get(), ptr_fixed_pcd.get()));

    // pre_hessian = se3_Hessian(ptr_moving_pcd.get(), ptr_fixed_pcd.get());

    // compute cosine of angle
    inn_fixed_pcd.copy(function_inner_product(ptr_fixed_pcd.get(), ptr_fixed_pcd.get()));
    inn_moving_pcd.copy(function_inner_product(ptr_moving_pcd.get(), ptr_moving_pcd.get()));
    cos_angle = inn_post.value / (sqrt(inn_fixed_pcd.value) * sqrt(inn_moving_pcd.value));

    post_hessian = se3_Hessian(transformed_pcd.get(), ptr_fixed_pcd.get(), inliers);

    return;
}

void cvo::compute_innerproduct_lc(inn_p& inn_prior, inn_p& inn_lc_prior, inn_p& inn_lc_pre, inn_p& inn_lc_post, \
                                       Eigen::Matrix<double, 6, 6>& post_hessian, Eigen::Affine3f& prior_tran, \
                                       Eigen::Affine3f& lc_prior_tran, Eigen::Affine3f& lc_prior_tran_2, Eigen::Affine3f& lc_tran, \
                                       int& inliers_svd, int& inliers_pnpransac, inn_p& inn_fixed_pcd, inn_p& inn_moving_pcd, float& cos_angle)
{

    unique_ptr<point_cloud> lc_transformed_pcd(new point_cloud);
    unique_ptr<point_cloud> prior_transformed_pcd(new point_cloud);
    unique_ptr<point_cloud> lc_prior_transformed_pcd(new point_cloud);
    unique_ptr<point_cloud> lc_prior_transformed_pcd_2(new point_cloud);

    lc_transformed_pcd->num_points = ptr_moving_pcd -> num_points;
    lc_transformed_pcd->positions = ptr_moving_pcd -> positions;
    lc_transformed_pcd->features = ptr_moving_pcd -> features;

    prior_transformed_pcd->num_points = ptr_moving_pcd -> num_points;
    prior_transformed_pcd->positions = ptr_moving_pcd -> positions;
    prior_transformed_pcd->features = ptr_moving_pcd -> features;

    lc_prior_transformed_pcd->num_points = ptr_moving_pcd -> num_points;
    lc_prior_transformed_pcd->positions = ptr_moving_pcd -> positions;
    lc_prior_transformed_pcd->features = ptr_moving_pcd -> features;

    lc_prior_transformed_pcd_2->num_points = ptr_moving_pcd -> num_points;
    lc_prior_transformed_pcd_2->positions = ptr_moving_pcd -> positions;
    lc_prior_transformed_pcd_2->features = ptr_moving_pcd -> features;

    tbb::parallel_for(int(0), lc_transformed_pcd->num_points, [&]( int i ){
        lc_transformed_pcd->positions[i] = (lc_tran.linear()*ptr_moving_pcd->positions[i] + lc_tran.translation());
        prior_transformed_pcd->positions[i] = (prior_tran.linear()*ptr_moving_pcd->positions[i] + prior_tran.translation());
        lc_prior_transformed_pcd->positions[i] = (lc_prior_tran.linear()*ptr_moving_pcd->positions[i] + lc_prior_tran.translation());
        lc_prior_transformed_pcd_2->positions[i] = (lc_prior_tran_2.linear()*ptr_moving_pcd->positions[i] + lc_prior_tran_2.translation());
        });
    
    inn_prior.copy(function_inner_product(prior_transformed_pcd.get(), ptr_fixed_pcd.get()));

    inn_lc_prior.copy(function_inner_product(lc_prior_transformed_pcd.get(), ptr_fixed_pcd.get()));

    inn_lc_pre.copy(function_inner_product(ptr_moving_pcd.get(), ptr_fixed_pcd.get()));

    inn_lc_post.copy(function_inner_product(lc_transformed_pcd.get(), ptr_fixed_pcd.get()));
    
    // pre_hessian = se3_Hessian(ptr_moving_pcd.get(), ptr_fixed_pcd.get());

    // compute cosine of angle
    inn_fixed_pcd.copy(function_inner_product(ptr_fixed_pcd.get(), ptr_fixed_pcd.get()));
    inn_moving_pcd.copy(function_inner_product(ptr_moving_pcd.get(), ptr_moving_pcd.get()));
    cos_angle = inn_lc_post.value / (sqrt(inn_fixed_pcd.value) * sqrt(inn_moving_pcd.value));

    inliers_svd = 0;
    post_hessian = se3_Hessian(lc_transformed_pcd.get(), ptr_fixed_pcd.get(), inliers_svd);

    inliers_pnpransac = 0;
    se3_Hessian(lc_prior_transformed_pcd_2.get(), ptr_fixed_pcd.get(), inliers_pnpransac);

    return;
}

void cvo::match_keyframe(const cv::Mat& RGB_img,const cv::Mat& dep_img, Eigen::Affine3d& transformd)
{
    if(init == false){
        std::cout << "cvo not initialized !" << "\n";
        return;
    }

    set_pcd(RGB_img, dep_img);

    align();

    transformd = transform.cast<double>();

}

void cvo::update_fixed_pcd()
{
    ptr_fixed_fr = std::move(ptr_moving_fr);
    ptr_fixed_pcd = std::move(ptr_moving_pcd);
}

void cvo::update_previous_pcd()
{
    ptr_previous_fr = std::move(ptr_moving_fr);
    ptr_previous_pcd = std::move(ptr_moving_pcd);
    pre_pc_init = true;
}

void cvo::reset_keyframe(Eigen::Affine3f& odometry)
{
    if(!pre_pc_init){
        ptr_fixed_fr = std::move(ptr_moving_fr);
        ptr_fixed_pcd = std::move(ptr_moving_pcd);
    }
    else{
        ptr_fixed_fr = std::move(ptr_previous_fr);
        ptr_fixed_pcd = std::move(ptr_previous_pcd);
        update_previous_pcd();
    }
    reset_transform(odometry);

}

void cvo::reset_transform(Eigen::Affine3f& odometry)
{
    transform = odometry;
}

Eigen::Affine3f cvo::reset_initial(Eigen::Affine3f& odometry)
{
    Eigen::Affine3f init = (transform * odometry).inverse();
    R = init.rotation();
    T = init.translation();

    return init.inverse();
}

Eigen::Matrix<double,6,6> cvo::se3_Hessian(point_cloud* cloud_a, point_cloud* cloud_b, int& inliers)
{
    Eigen::Matrix<float,6,6> Hessian(Eigen::Matrix<float,6,6>::Zero());
    // int inliers = 0;
    
    // convert k threshold to d2 threshold (so that we only need to calculate k when needed)
    float d2_thres = -2.0*ell*ell*log(sp_thres/sigma/sigma);
    float d2_c_thres = -2.0*c_ell*c_ell*log(sp_thres/c_sigma/c_sigma);

    /** 
     * kdtreeeeeeeeeeeeeeeeeeeee
     **/
    typedef KDTreeVectorOfVectorsAdaptor<cloud_t, float>  kd_tree_t;
	kd_tree_t mat_index(3 /*dim*/, (cloud_b->positions), 10 /* max leaf */ );
	mat_index.index->buildIndex();

    tbb::spin_mutex hessian_lock;
    // loop through points
    tbb::parallel_for(int(0),cloud_a->num_points,[&](int i){

        const float search_radius = d2_thres;
		std::vector<std::pair<size_t,float>>  ret_matches;
		nanoflann::SearchParams params;
		//params.sorted = false;
		const size_t nMatches = mat_index.index->radiusSearch(&(cloud_a->positions)[i](0), search_radius, ret_matches, params);
        Eigen::Vector3f position_a = (cloud_a->positions)[i];
        Eigen::Matrix<float,5,1> feature_a = cloud_a->features.row(i).transpose();

        for(size_t j=0; j<nMatches; ++j){
            int idx = ret_matches[j].first;
            float d2 = ret_matches[j].second;
            float k = 0;
            float cdot = 0;
            float d2_color = 0;
            if(d2<d2_thres){
                // d2_color = color_kernel(i,j);
                Eigen::Matrix<float,5,1> feature_b = cloud_b->features.row(idx).transpose();
                d2_color = ((feature_a-feature_b).squaredNorm());

                if(d2_color<d2_c_thres){
                    Eigen::Vector3f position_b = (cloud_b->positions)[idx];
                    k = sigma*sigma*exp(-d2/(2.0*ell*ell));
                    cdot = feature_a.dot(feature_b);
                    Eigen::Vector3f cross_ab = position_a.cross(position_b);
                    
                    // block A
                    Eigen::Matrix<float,3,3> A;
                    float dot1 = position_a(1)*position_b(1) + position_a(2)*position_b(2);
                    float dot2 = position_a(0)*position_b(0) + position_a(2)*position_b(2);
                    float dot3 = position_a(0)*position_b(0) + position_a(1)*position_b(1);
                    A(0,0) = 1/(ell*ell)*cross_ab(0)*cross_ab(0)-dot1;
                    A(1,1) = 1/(ell*ell)*cross_ab(1)*cross_ab(1)-dot2;
                    A(2,2) = 1/(ell*ell)*cross_ab(2)*cross_ab(2)-dot3;
                    A(0,1) = A(1,0) = 1/(ell*ell)*cross_ab(0)*cross_ab(1)+0.5*(position_a(0)*position_b(1)+position_a(1)*position_b(0));
                    A(0,2) = A(2,0) = 1/(ell*ell)*cross_ab(0)*cross_ab(2)+0.5*(position_a(0)*position_b(2)+position_a(2)*position_b(0));
                    A(1,2) = A(2,1) = 1/(ell*ell)*cross_ab(1)*cross_ab(2)+0.5*(position_a(1)*position_b(2)+position_a(2)*position_b(1));

                    // block C
                    Eigen::Matrix<float,3,3> C;
                    Eigen::Vector3f diff_ba = position_b - position_a;
                    C(0,0) = 1/(ell*ell)*cross_ab(0)*diff_ba(0);
                    C(1,1) = 1/(ell*ell)*cross_ab(1)*diff_ba(1);
                    C(2,2) = 1/(ell*ell)*cross_ab(2)*diff_ba(2);
                    C(1,0) = position_a(2) + 1/(ell*ell)*diff_ba(1)*cross_ab(0);
                    C(2,0) = -position_a(1) + 1/(ell*ell)*diff_ba(2)*cross_ab(0);
                    C(0,1) = -position_a(2) + 1/(ell*ell)*diff_ba(0)*cross_ab(1);
                    C(2,1) = position_a(0) + 1/(ell*ell)*diff_ba(2)*cross_ab(1);
                    C(0,2) = position_a(1) + 1/(ell*ell)*diff_ba(0)*cross_ab(2);
                    C(1,2) = -position_a(0) + 1/(ell*ell)*diff_ba(1)*cross_ab(2);

                    // block D
                    Eigen::Matrix<float,3,3> D;
                    D(0,0) = 1/(ell*ell)*diff_ba(0)*diff_ba(0)-1;
                    D(1,1) = 1/(ell*ell)*diff_ba(1)*diff_ba(1)-1;
                    D(2,2) = 1/(ell*ell)*diff_ba(2)*diff_ba(2)-1;
                    D(0,1) = D(1,0) = 1/(ell*ell)*diff_ba(0)*diff_ba(1);
                    D(0,2) = D(2,0) = 1/(ell*ell)*diff_ba(0)*diff_ba(2);
                    D(1,2) = D(2,1) = 1/(ell*ell)*diff_ba(1)*diff_ba(2);

                    // compute hessian
                    Eigen::Matrix<float,6,6> Blocks;
                    Blocks.block<3,3>(0,0) = A;
                    Blocks.block<3,3>(0,3) = C.transpose();
                    Blocks.block<3,3>(3,0) = C;
                    Blocks.block<3,3>(3,3) = D;

                    hessian_lock.lock();
                        Hessian += 1/(ell*ell)*cdot*k*Blocks;
                        inliers++;
                        // std::cout << "Blocks(1,1): " << Blocks(0,0) << std::endl; 
                        // std::cout << "d2_color: " << d2_color << std::endl;
                        // std::cout << "feature_a: " << feature_a.transpose() << std::endl;
                        // std::cout << "feature_b: " << feature_b.transpose() << std::endl << std::endl;
                    hessian_lock.unlock();
                    
                    // std::cout << "cdot: " << cdot << " k: " << k << " Blocks max: " << Blocks.maxCoeff() << std::endl;
                    // std::cout << "diff_ba: " << diff_ba(0) << "," << diff_ba(1) << "," << diff_ba(2) << std::endl;
                }
            }
        }
    });

    // std::cout << "hessian inliers: " << inliers << std::endl;

    // if (inliers) Hessian *= -1.0;

    if (inliers){
        Hessian *= -1.0 / 100000;
        Eigen::Matrix<std::complex<float>,6,1> eigenvalues_c = Hessian.eigenvalues();
        Eigen::Matrix<float,6,1> eigenvalues;
        for(size_t i = 0;i < 6; i++) eigenvalues(i) = eigenvalues_c(i).real();
        // std::cout << "scaled Hessian eigenvalues: " << eigenvalues.transpose() << std::endl;
        
        float sufficient_scale = 0.0;
        Eigen::Matrix<float,6,1> addition_vector;
        addition_vector << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        int min_idx;
        eigenvalues.cwiseAbs().minCoeff(&min_idx);
        float min_eigen = eigenvalues(min_idx);
        // std::cout << "current eigenvalue with min abs: " << min_eigen << std::endl;
        while(abs(min_eigen) < 1.0){
            sufficient_scale += (1.0 - min_eigen);
            eigenvalues += (1.0 - min_eigen) * addition_vector;
            eigenvalues.cwiseAbs().minCoeff(&min_idx);
            min_eigen = eigenvalues(min_idx);
            // std::cout << "current eigenvalue with min abs: " << min_eigen << std::endl;
        }
        Hessian += sufficient_scale * Eigen::Matrix<float,6,6>::Identity();

        
        Eigen::Matrix<std::complex<float>,6,1> eigenvalues_check_c = Hessian.eigenvalues();
        Eigen::Matrix<float,6,1> eigenvalues_check;
        for(size_t i = 0;i < 6; i++) eigenvalues_check(i) = eigenvalues_check_c(i).real();
        // std::cout << "Hessian eigenvalues check: " << eigenvalues_check.transpose() << std::endl;
    }
    else Hessian = Eigen::Matrix<float,6,6>::Identity();
    // else Hessian = 5000 * Eigen::Matrix<float,6,6>::Identity();

    return Hessian.cast<double>();
}



void cvo::align(){
    int n = tbb::task_scheduler_init::default_num_threads();
    // std::cout<<"num_thread: "<<n<<std::endl;

    // loop until MAX_ITER
    for(int k=0; k<MAX_ITER; k++){
        // update transformation matrix
        update_tf();

        // apply transform to the point cloud
        transform_pcd();
        
        // compute omega and v
        compute_flow();

        // compute step size for integrating the flow
        compute_step_size();

        // stop if the step size is too small
        if(omega.norm()<eps && v.norm()<eps){
            iter = k;
            // std::cout<<"norm, omega: "<<omega.norm()<<", v: "<<v.norm()<<std::endl;
            break;
        }

        // stacked omega and v for finding dtrans
        Eigen::VectorXf vec_joined(omega.size()+v.size());
        vec_joined << omega, v;

        // find the change of translation matrix dtrans
        Eigen::MatrixXf dtrans = Exp_SEK3(vec_joined, step);

        // extract dR and dT from dtrans
        Eigen::Matrix3f dR = dtrans.block<3,3>(0,0);
        Eigen::Vector3f dT = dtrans.block<3,1>(0,3);

        // calculate new R and T
        T = R * dT + T;
        R = R * dR;

        // if the se3 distance is smaller than eps2, break
        if(dist_se3(dR,dT)<eps_2){
            iter = k;
            // std::cout<<"dist: "<<dist_se3(dR,dT)<<std::endl;
            break;
        }

        ell = (k>2)? 0.10:ell;
        ell = (k>9)? 0.06:ell;
        ell = (k>19)? 0.03:ell;
        
    }
    prev_transform = transform.matrix();
    accum_transform = accum_transform * transform.matrix();
    update_tf();

    // ptr_fixed_pcd = std::move(ptr_moving_pcd);
    delete cloud_y;
}

// void cvo::run_cvo(const int dataset_seq,const cv::Mat& RGB_img,const cv::Mat& dep_img, string pcd_pth, string pcd_dso_pth){

//     if(init == false){
//         set_pcd(dataset_seq,RGB_img, dep_img, pcd_pth, pcd_dso_pth);
//     }
//     else{
//         set_pcd(dataset_seq, RGB_img, dep_img, pcd_pth, pcd_dso_pth);
//         align();
        
//         std::cout<<"Total iterations: "<<iter<<std::endl;
//         std::cout<<"RKHS-SE(3) Object Transformation Estimate: \n"<<transform.matrix()<<std::endl;
//     }

// }
}
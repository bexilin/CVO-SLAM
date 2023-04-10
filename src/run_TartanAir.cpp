#include "keyframe_tracker.h"
#include <opencv2/core/core.hpp>
#include <vector>
#include <string>
#include <filesystem>

using namespace std;

void load_file_name(string folder, vector<string> &filenames);

void load_img(string& RGB_pth, string& dep_pth, string& timestamp, cvo_slam::Image_Ptr& current);

void configure(cvo_slam::cfg& cfg_, string& config_file);

int main(int argc, char** argv){

    string cvo_slam_cfg;
    string orb_vocabulary;
    string calib_orb_tracking_cfg;
    string folder;
    string association;
    // int dataset_seq;
    cvo_slam::cfg cfg_;
    
    if(argc!=5){
        std::cout << "Please the following terms:\n"
                  << "1. CVO SLAM configuration file\n"
                  << "2. Visual vocabulary file\n"
                  << "3. Calibration and ORB feature configuration file\n"
                  << "4. Directory of a TartanAir sequence\n";
        exit (EXIT_FAILURE);
    } 
    else{
        cvo_slam_cfg = argv[1];
        std::cout << std::endl << "CVO SLAM Parameters:" << std::endl; 
        configure(cfg_,cvo_slam_cfg);
        orb_vocabulary = argv[2];
        calib_orb_tracking_cfg = argv[3];
        folder = argv[4];
    }

    vector<string> filenames;
    load_file_name(folder+"image_left/",filenames);
    int num_img = filenames.size();

    cvo_slam::KeyframeTracker keyframe_tracker(orb_vocabulary, calib_orb_tracking_cfg, folder);

    keyframe_tracker.init();
    keyframe_tracker.configure(cfg_);
    keyframe_tracker.configureTracker(cfg_);
    if(!cfg_.OnlyTracking){
        keyframe_tracker.configureGraph(cfg_);
    }
    cvo_slam::Image_Ptr current;
    cvo_slam::Transform trajectory;
    

    ofstream fPoseQtTxt;
    fPoseQtTxt.open(folder+"Tracking_trajectory.txt");

    string optimized_trajectory_file = folder+"SLAM_trajectory.txt";
    string loop_closure_file = folder+"loop_closure.txt";

   
    for(int i=0;i<num_img;i++){

        std::cout << "Current frame number: " << i+1 << std::endl << std::endl;

        string RGB_pth = folder + "image_left/" + filenames[i] + ".png";
        string dep_pth = folder + "depth_left_image/" + filenames[i] + "_depth.png";
        
        load_img(RGB_pth, dep_pth, filenames[i], current);

        if(i == num_img-1) keyframe_tracker.forceKeyframe();

        keyframe_tracker.update(current, trajectory);

        Eigen::Quaterniond q(trajectory.rotation());
        fPoseQtTxt << current->timestamp << " ";
        fPoseQtTxt << trajectory.translation()(0) << " " << trajectory.translation()(1) << " " << trajectory.translation()(2) << " ";
        fPoseQtTxt << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }

    fPoseQtTxt.close();

    if(!cfg_.OnlyTracking){
        // std::cout << "\n\nFinal optimization start\n\n";

        // keyframe_tracker.finish();

        // cvo_slam::FileSerializer<cvo_slam::TrajectorySerializer> serializer(optimized_trajectory_file,loop_closure_file);
        keyframe_tracker.writeSLAMTrajectoryAndLoopClosureFile(optimized_trajectory_file, loop_closure_file);
    }
}

void load_file_name(string folder, vector<string> &filenames){
    std::filesystem::path dir = folder;
    for(const auto& file: std::filesystem::directory_iterator(dir)){
        if(file.is_regular_file() && file.path().extension() == ".png"){
            filenames.push_back(file.path().stem().string());
        }
    }
    std::sort(filenames.begin(),filenames.end());
}


void load_img(string& RGB_pth, string& dep_pth, string& timestamp, cvo_slam::Image_Ptr& current){
    // std::cout << "come here" << std::endl;
    current.reset(new cvo_slam::Image());
    current->rgb = cv::imread(RGB_pth);
    current->depth = cv::imread(dep_pth,CV_LOAD_IMAGE_ANYDEPTH);
    current->timestamp = timestamp;
    
    // For SuperPoint implementation
    current->rgb_path = RGB_pth;
}

void configure(cvo_slam::cfg& cfg_, string& cvo_slam_cfg){
    std::ifstream config_file;
    config_file.open(cvo_slam_cfg.c_str());
    if (config_file.is_open()){
        string line;
        while(getline(config_file,line)){
            stringstream variable;
            variable << line;
            string name;
            variable >> name;
            if(name.compare("KFS_Distance") == 0){
                variable >> cfg_.KFS_Distance ;
                std::cout << "KFS_Distance: " << cfg_.KFS_Distance << std::endl;
            }
            else if(name.compare("KFS_Angle") == 0){
                variable >> cfg_.KFS_Angle ;
                std::cout << "KFS_Angle: " << cfg_.KFS_Angle << std::endl;
            }
            else if(name.compare("OptimizationIterations") == 0){
                variable >> cfg_.OptimizationIterations ;
                std::cout << "OptimizationIterations: " << cfg_.OptimizationIterations << std::endl;
            }
            else if(name.compare("MinConstraintDistance") == 0){
                variable >> cfg_.MinConstraintDistance ;
                std::cout << "MinConstraintDistance: " << cfg_.MinConstraintDistance << std::endl;
            }
            else if(name.compare("OptimizationRemoveOutliers") == 0){
                variable >> cfg_.OptimizationRemoveOutliers ;
                std::cout << "OptimizationRemoveOutliers: " << cfg_.OptimizationRemoveOutliers << std::endl;
            }
            else if(name.compare("UseMultiThreading") == 0){
                variable >> cfg_.UseMultiThreading ;
                std::cout << "UseMultiThreading: " << cfg_.UseMultiThreading << std::endl;
            }
            else if(name.compare("OptimizationUseDenseGraph") == 0){
                variable >> cfg_.OptimizationUseDenseGraph ;
                std::cout << "OptimizationUseDenseGraph: " << cfg_.OptimizationUseDenseGraph << std::endl;
            }
            else if(name.compare("FinalOptimizationUseDenseGraph") == 0){
                variable >> cfg_.FinalOptimizationUseDenseGraph ;
                std::cout << "FinalOptimizationUseDenseGraph: " << cfg_.FinalOptimizationUseDenseGraph << std::endl;
            }
            else if(name.compare("FinalOptimizationIterations") == 0){
                variable >> cfg_.FinalOptimizationIterations ;
                std::cout << "FinalOptimizationIterations: " << cfg_.FinalOptimizationIterations << std::endl;
            }
            else if(name.compare("UseRobustKernel") == 0){
                variable >> cfg_.UseRobustKernel ;
                std::cout << "UseRobustKernel: " << cfg_.UseRobustKernel << std::endl;
            }
            else if(name.compare("FE_InnpThreshold") == 0){
                variable >> cfg_.FE_InnpThreshold ;
                std::cout << "FE_InnpThreshold: " << cfg_.FE_InnpThreshold << std::endl;
            }
            else if(name.compare("OnlyTracking") == 0){
                variable >> cfg_.OnlyTracking ;
                std::cout << "OnlyTracking: " << cfg_.OnlyTracking << std::endl;
            }
            else if(name.compare("LC_MinMatch") == 0){
                variable >> cfg_.LC_MinMatch ;
                std::cout << "LC_MinMatch: " << cfg_.LC_MinMatch << std::endl;
            }
            else if(name.compare("LC_MatchThreshold") == 0){
                variable >> cfg_.LC_MatchThreshold ;
                std::cout << "LC_MatchThreshold: " << cfg_.LC_MatchThreshold << std::endl;
            }
            else if(name.compare("RobustKernelDelta") == 0){
                variable >> cfg_.RobustKernelDelta ;
                std::cout << "RobustKernelDelta: " << cfg_.RobustKernelDelta << std::endl;
            }
            else if(name.compare("LC_MinScoreRatio") == 0){
                variable >> cfg_.LC_MinScoreRatio ;
                std::cout << "LC_MinScoreRatio: " << cfg_.LC_MinScoreRatio << std::endl;
            }
            else if(name.compare("Min_KF_interval") == 0){
                variable >> cfg_.Min_KF_interval ;
                std::cout << "Min_KF_interval: " << cfg_.Min_KF_interval << std::endl;
            }
            else if(name.compare("Max_KF_interval") == 0){
                variable >> cfg_.Max_KF_interval ;
                std::cout << "Max_KF_interval: " << cfg_.Max_KF_interval << std::endl;
            }
            else std::cout << "not defined in configuration !" << std::endl;

        }

    }
    else std::cout << "fail to open configure file" << std::endl;

}
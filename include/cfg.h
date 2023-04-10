/* ----------------------------------------------------------------------------
 * Copyright 2019, Xi Lin <bexilin@umich.edu>, Dingyi Sun <dysun@umich.edu>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#ifndef CFG_H
#define CFG_H

namespace cvo_slam{

struct cfg
{
    // Default value for cvo slam parameters
    double KFS_Distance = 0.15;
    double KFS_Angle = 30; // in degree
    int OptimizationIterations = 50;
    int MinConstraintDistance = 1;
    bool OptimizationRemoveOutliers = true;
    bool UseMultiThreading = true;
    bool OptimizationUseDenseGraph = false;
    bool FinalOptimizationUseDenseGraph = true;
    int FinalOptimizationIterations = 1000;
    bool UseRobustKernel = true;
    float FE_InnpThreshold = 0.1;
    bool OnlyTracking = false;
    int LC_MinMatch = 50;
    float LC_MatchThreshold = 0.6;
    float RobustKernelDelta = 5.0;
    float LC_MinScoreRatio = 0.7;
    int Min_KF_interval = 10;
    int Max_KF_interval = 20;
};
 
} // namespace cvo_slam

#endif // CFG_H 
//
// Created by sc on 18-6-11.
//

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <fstream>
using namespace std;
using namespace gtsam;

int main(int argc,char ** argv){

    NonlinearFactorGraph graph;
    noiseModel::Diagonal::shared_ptr priorNoise=noiseModel::Diagonal::Sigmas(Vector3(0.3,0.3,0.1));
    graph.emplace_shared<PriorFactor<Pose2>>(1,Pose2(0,0,0),priorNoise);
    noiseModel::Diagonal::shared_ptr model =noiseModel::Diagonal::Sigmas(Vector3(0.2,0.2,0.1));
    graph.emplace_shared<BetweenFactor<Pose2>>(1,2,Pose2(2,0,0),model);
    graph.emplace_shared<BetweenFactor<Pose2>>(2,3,Pose2(2,0,M_PI_2),model);
    graph.emplace_shared<BetweenFactor<Pose2>>(3,4,Pose2(2,0,M_PI_2),model);
    graph.emplace_shared<BetweenFactor<Pose2>>(4,5,Pose2(2,0,M_PI_2),model);

    graph.emplace_shared<BetweenFactor<Pose2>>(5,2,Pose2(2,0,M_PI_2),model);

    Values initialEstamate;
    initialEstamate.insert(1,Pose2(0.5,0.0,0.2));
    initialEstamate.insert(2,Pose2(2.3,0.1,-0.2));
    initialEstamate.insert(3,Pose2(4.1,0.1,M_PI_2));
    initialEstamate.insert(4,Pose2(4.0,2.0,M_PI));
    initialEstamate.insert(5,Pose2(2.1,2.1,-M_PI_2));

    GaussNewtonParams params;
    params.relativeErrorTol=1e-5;
    params.maxIterations=100;
    GaussNewtonOptimizer optimizer(graph,initialEstamate,params);
    Values result=optimizer.optimize();
    result.print("\n final result \n");
    Marginals marginals(graph,result);
    for (int j = 1; j < 6; ++j) {
        boost::format fmt("\n x%1% covariance: \n %2% \n");
        fmt%j%marginals.marginalCovariance(j);
        cout<<fmt;

    }
    ofstream os("Pose2slam.dot");
    graph.saveGraph(os,result);
    graph.saveGraph(cout,result);
    writeG2o(graph,result,"graph2g2o.g2o");
    return 0;

}
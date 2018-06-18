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
    NonlinearFactorGraph::shared_ptr graph1;
    Values::shared_ptr initialEstamate1;
    string g2ofile=findExampleDataFile("noisyToyGraph.txt");
    bool is3D =false;
    boost::tie(graph1,initialEstamate1)=readG2o(g2ofile,is3D);
    noiseModel::Diagonal::shared_ptr priorModel=noiseModel::Diagonal::Variances(Vector3(1e-6,1e-6,16-8));
    NonlinearFactorGraph graph=*graph1;
    Values initialEstamate=*initialEstamate1;
    graph.add(PriorFactor<Pose2>(0,Pose2(),priorModel));


    GaussNewtonParams params;
    params.relativeErrorTol=1e-5;
    params.maxIterations=100;
    GaussNewtonOptimizer optimizer(graph,initialEstamate,params);
    Values result=optimizer.optimize();
//    result.print("\n final result \n");
    Marginals marginals(graph,result);
/*    for (int j = 1; j < 6; ++j) {
        boost::format fmt("\n x%1% covariance: \n %2% \n");
        fmt%j%marginals.marginalCovariance(j);
        cout<<fmt;
    }*/
    ofstream os("Pose2slam.dot");
    graph.saveGraph(os,result);
//    graph.saveGraph(cout,result);
    writeG2o(graph,result,"graph2g2o.g2o");
    return 0;

}
#include <iostream>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
using namespace std;
using namespace gtsam;
const double degree =M_PI/180;

int main() {

    Rot2 prior=Rot2::fromAngle(30*degree);
    prior.print("goal angle");
    noiseModel::Isotropic::shared_ptr model=noiseModel::Isotropic::Sigma(1,1*degree);
    Symbol key('x',1);
    PriorFactor<Rot2> factor(key,prior,model);
    NonlinearFactorGraph graph;
    graph.push_back(factor);
    graph.print("full graph");
    Values initial;
    initial.insert(key,Rot2::fromAngle(20*degree));
    initial.print("initial estimate");

    Values result=LevenbergMarquardtOptimizer(graph,initial).optimize();
    result.print("final result");

    std::cout << "Hello, World!" << std::endl;
    return 0;
}
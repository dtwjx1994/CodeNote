//
// Created by sc on 18-6-12.
//
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/dataset.h>
#include <iostream>
#include <fstream>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/sam/RangeFactor.h>
using namespace std;
using namespace gtsam;
namespace NM = gtsam::noiseModel;
typedef pair<double,Pose2> TimedOdometry;
list<TimedOdometry> readOdometry(){
    list<TimedOdometry> odometryList;
    string data_file=findExampleDataFile("Plaza2_DR.txt");
    ifstream is(data_file.c_str());

    while(is){
        double t,distance_traceled,delta_heading;
        is>>t>>distance_traceled>>delta_heading;
        cout<<boost::format("t:%1% dis:%2% del:%3%\n")%t%distance_traceled%delta_heading;
        odometryList.push_back(
                TimedOdometry(t,Pose2(distance_traceled,0,delta_heading))
                );
    }
    is.clear();
    return odometryList;
}
//boost 元组
typedef boost::tuple<double,size_t,double > RangeTriple;
vector<RangeTriple> readTriples(){
    vector<RangeTriple> triples;
    string data_file=findExampleDataFile("Plaza2_TD.txt");
    ifstream is(data_file.c_str());

    while (is){
        double t,sender,range;
        size_t receiver;
        double  WTF;
        string str;
//        getline(is,str);

        is>>t>>sender>>receiver>>WTF>>range;
       cout<<boost::format("t:%1% sender:%2% receiver:%3% range:%4% \n")%t%sender%receiver%range;
        triples.push_back(RangeTriple(t,receiver,range));
    }
    is.clear();
    return triples;

}

int main(int argc,char** argv){
    list<TimedOdometry> odometry =readOdometry();
    vector<RangeTriple> triples =readTriples();

    size_t K=triples.size();
    size_t minK=150;
    size_t incK=25;
    bool groundTruth=false;
    bool robust=true;

    Vector priorSigms=Vector3(1,1,M_PI);
    Vector odosigms=Vector3(0.05,0.01,0.2);
    double sigmaR=100;
    const NM::Base::shared_ptr priorNoise=NM::Diagonal::Sigmas(priorSigms),
            odoNoise=NM::Diagonal::Sigmas(odosigms),
                    gaussian=NM::Isotropic::Sigma(1,sigmaR),
                            tukey=NM::Robust::Create(NM::mEstimator::Tukey::Create(15),gaussian),
                                    rangeNoise=robust?tukey:gaussian;
    ISAM2 isam;
    Pose2 pose0=Pose2(-34.2086489999201, 45.3007639991120,
                      M_PI - 2.02108900000000);
    NonlinearFactorGraph newFactors,graph;
    newFactors.push_back(PriorFactor<Pose2>(0,pose0,priorNoise));
    Values initial,initialsum;
    initial.insert(0,pose0);

    if(groundTruth){
        initial.insert(symbol('L', 1), Point2(-68.9265, 18.3778));
        initial.insert(symbol('L', 6), Point2(-37.5805, 69.2278));
        initial.insert(symbol('L', 0), Point2(-33.6205, 26.9678));
        initial.insert(symbol('L', 5), Point2(1.7095, -5.8122));
    } else{
        initial.insert(symbol('L', 1), Point2(3.5784, 2.76944));
        initial.insert(symbol('L', 6), Point2(-1.34989, 3.03492));
        initial.insert(symbol('L', 0), Point2(0.725404, -0.0630549));
        initial.insert(symbol('L', 5), Point2(0.714743, -0.204966));
    }

    int i=1;
    int k=0;
    bool initialized =false;
    Pose2 lastPose =pose0;
    int countK=0;

    for (const TimedOdometry& timedOdometry:odometry) {
        double t;
        Pose2 odometry;
        boost::tie(t,odometry)=timedOdometry;
        newFactors.push_back(BetweenFactor<Pose2>(i-1,i,odometry,NM::Diagonal::Sigmas(odosigms)));
        Pose2 predictedPose =lastPose.compose(odometry);
        lastPose=predictedPose;
        initial.insert(i,predictedPose);

        while(k<K&&t>=boost::get<0>(triples[k])){
            int j=boost::get<1>(triples[k]);
            double range= boost::get<2>(triples[k]);
            newFactors.push_back(RangeFactor<Pose2,Point2>(i,symbol('L',j),range,rangeNoise));
            k=k+1;
            countK=countK+1;
        }
        if((k>minK)&&(countK>incK)){
            if (!initialized){
                newFactors.print("new factors ");
                initial.print("initial");
                LevenbergMarquardtOptimizer batchOptimizer(newFactors,initial);
                initial=batchOptimizer.optimize();
                initialized=true;
            }

            isam.update(newFactors,initial);
            Values result=isam.calculateBestEstimate();

            lastPose=result.at<Pose2>(i);
            // 重置
                    for(int f=0;f<initial.size();f++){
                        initialsum.insert(initial[f]);
                    }

                    for(auto f:newFactors)
                        graph.push_back(f);
//            graph+=newFactors;
//            initialsum+=initial;
            newFactors=NonlinearFactorGraph();
            initial=Values();
            countK=0;
        }
        i+=1;
    }
    newFactors= isam.getFactorsUnsafe();
    Values values=isam.calculateBestEstimate();
    writeG2o(newFactors,values,"123.g2o");



}

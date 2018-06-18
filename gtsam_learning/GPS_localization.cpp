//
// Created by sc on 18-6-11.
//

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
using namespace std ;
using namespace gtsam;
class UnaryFactor: public NoiseModelFactor1<Pose2> {

    // The factor will hold a measurement consisting of an (X,Y) location
    // We could this with a Point2 but here we just use two doubles
    double mx_, my_;

public:
    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<UnaryFactor> shared_ptr;

    // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
    UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
            NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

    virtual ~UnaryFactor() {}

    // Using the NoiseModelFactor1 base class there are two functions that must be overridden.
    // The first is the 'evaluateError' function. This function implements the desired measurement
    // function, returning a vector of errors when evaluated at the provided variable value. It
    // must also calculate the Jacobians for this measurement function, if requested.
    Vector evaluateError(const Pose2& q, boost::optional<Matrix&> H = boost::none) const
    {
        // The measurement function for a GPS-like measurement is simple:
        // error_x = pose.x - measurement.x
        // error_y = pose.y - measurement.y
        // Consequently, the Jacobians are:
        // [ derror_x/dx  derror_x/dy  derror_x/dtheta ] = [1 0 0]
        // [ derror_y/dx  derror_y/dy  derror_y/dtheta ] = [0 1 0]
        if (H) (*H) = (Matrix(2,3) << 1.0,0.0,0.0, 0.0,1.0,0.0).finished();
        return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
    }

    // The second is a 'clone' function that allows the factor to be copied. Under most
    // circumstances, the following code that employs the default copy constructor should
    // work fine.
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this))); }

    // Additionally, we encourage you the use of unit testing your custom factors,
    // (as all GTSAM factors are), in which you would need an equals and print, to satisfy the
    // GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these are not needed below.

}; // UnaryFactor
int main(int argc,char ** argv){
    NonlinearFactorGraph graph;

    noiseModel::Diagonal::shared_ptr odometryNoise=noiseModel::Diagonal::Sigmas(Vector3(0.2,0.2,0.1));
    graph.emplace_shared<BetweenFactor<Pose2>>(1,2,Pose2(2.0,0.0,0.0),odometryNoise); //emplace 放置
    graph.emplace_shared<BetweenFactor<Pose2>>(2,3,Pose2(2.0,0.0,0.0),odometryNoise);

    noiseModel::Diagonal::shared_ptr unaryNoise =noiseModel::Diagonal::Sigmas(Vector2(0.1,0.1));
    graph.emplace_shared<UnaryFactor>(1,0.0,0.0,unaryNoise);
    graph.emplace_shared<UnaryFactor>(2,2.0,0.0,unaryNoise);
    graph.emplace_shared<UnaryFactor>(3,4.0,0.0,unaryNoise);
    graph.print("\n Factor Graph: \n");

    Values initialEstimate;
    initialEstimate.insert(1,Pose2(0.5,0.0,0.2));
    initialEstimate.insert(2,Pose2(2.3,0.1,-0.2));
    initialEstimate.insert(3,Pose2(4.1,0.1,0.1));
    initialEstimate.print("\n Initial Estimate:\n");

    LevenbergMarquardtOptimizer optimizer(graph,initialEstimate);
    Values result=optimizer.optimize();

    result.print("\n result \n");

    Marginals marginals(graph,result);
    for (int j = 1; j < 4; ++j) {
        boost::format fmt("\n x%1% covariance: \n %2% \n");
        fmt%j%marginals.marginalFactor(j);
//        marginals.marginalFactor(j);
        cout<<fmt;
    }



}
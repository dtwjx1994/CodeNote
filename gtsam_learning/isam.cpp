//
// Created by sc on 18-6-11.
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

using namespace std;
using namespace gtsam;

std::vector<gtsam::Point3> createPoints() {

    // Create the set of ground-truth landmarks
    std::vector<gtsam::Point3> points;
    points.push_back(gtsam::Point3(10.0, 10.0, 10.0));
    points.push_back(gtsam::Point3(-10.0, 10.0, 10.0));
    points.push_back(gtsam::Point3(-10.0, -10.0, 10.0));
    points.push_back(gtsam::Point3(10.0, -10.0, 10.0));
    points.push_back(gtsam::Point3(10.0, 10.0, -10.0));
    points.push_back(gtsam::Point3(-10.0, 10.0, -10.0));
    points.push_back(gtsam::Point3(-10.0, -10.0, -10.0));
    points.push_back(gtsam::Point3(10.0, -10.0, -10.0));

    return points;
}

/* ************************************************************************* */
std::vector<gtsam::Pose3> createPoses() {

    // Create the set of ground-truth poses
    std::vector<gtsam::Pose3> poses;
    double radius = 30.0;
    int i = 0;
    double theta = 0.0;
    gtsam::Point3 up(0, 0, 1);
    gtsam::Point3 target(0, 0, 0);
    for (; i < 8; ++i, theta += 2 * M_PI / 8) {
        gtsam::Point3 position = gtsam::Point3(radius * cos(theta), radius * sin(theta), 0.0);
        gtsam::SimpleCamera camera = gtsam::SimpleCamera::Lookat(position, target, up);
        poses.push_back(camera.pose());
    }
    return poses;
}

int main(int argc, char **argv) {
    Cal3_S2::shared_ptr K(new Cal3_S2(50, 50, 0, 50, 50));
    noiseModel::Isotropic::shared_ptr noise = noiseModel::Isotropic::Sigma(2, 1);
    vector<Pose3> poses = createPoses();
    vector<Point3> points = createPoints();
    int relinearizeInterval = 3;
    NonlinearISAM isam(relinearizeInterval);

    NonlinearFactorGraph graph;
    Values initialEstiamte;
    for (int i = 0; i < poses.size(); ++i) {
        for (int j = 0; j < points.size(); ++j) {
            SimpleCamera camera(poses[i], *K);
            Point2 measurement = camera.project(points[j]);
            graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2>>(measurement, noise, Symbol('x', i),
                                                                                  Symbol('l', j), K);
        }
        Pose3 noise(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.2));
        Pose3 initial_xi = poses[i].compose(noise);
        initialEstiamte.insert(Symbol('x', i), initial_xi);
        if (i == 0) {
            noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas(
                    (Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished());
            graph.emplace_shared<PriorFactor<Pose3>>(Symbol('x', 0), poses[0], poseNoise);
            noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
            graph.emplace_shared<PriorFactor<Point3>>(Symbol('l', 0), points[0], pointNoise);
            Point3 noise(-0.25, 0.2, 0.15);
            for (int j = 0; j < points.size(); ++j) {
                Point3 initial_lj = points[j] + noise;
                initialEstiamte.insert(Symbol('l', j), initial_lj);
            }
        }else{
            isam.update(graph,initialEstiamte);
            Values currentEstimate=isam.estimate();
            boost::format fmt("\n Frame%1%:\n");
            fmt%i;
            cout<<fmt;
            currentEstimate.print("\n");
            graph.resize(0);
            initialEstiamte.clear();
        }

    }
}
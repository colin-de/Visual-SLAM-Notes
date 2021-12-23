#include <sophus/se3.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <unistd.h>
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string trajectory_file = "../trajectory.txt";
string groundtruth_file = "../groundtruth.txt";
string estimated_file = "../estimated.txt";

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

// start point is red and end point is blue
void DrawTrajectory(TrajectoryType);
void DrawTwoTrajectory(TrajectoryType, TrajectoryType);
void CalculateError(TrajectoryType, TrajectoryType );

int main(int argc, char **argv) {

    TrajectoryType poses;
    TrajectoryType groundtruth;
    TrajectoryType estimated;

    /// implement pose reading code
    std::ifstream file_reader(trajectory_file);
    std::ifstream groundtruth_reader(groundtruth_file);
    std::ifstream estimated_reader(estimated_file);
    double time, tx, ty, tz, qx, qy, qz, qw;

    // Method 1
    // while (!file_reader.eof()){
    //     file_reader >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    //     Eigen::Vector3d t(tx, ty, tz);
    //     Eigen::Quaterniond q(qw, qx, qy, qz);
    //     q.normalize();
    //     Eigen::Matrix3d Rotation_Matrix(q);
    //     Sophus::SE3d SE3_from_Eigen(Rotation_Matrix, t);
    //     poses.push_back(SE3_from_Eigen);
    // }
    // cout << "There are " << poses.size() << " timestamps." << endl;

    // Method 2
    while(file_reader >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw){
        poses.push_back(Sophus::SE3d(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz)));
    }
    cout << "There are " << poses.size() << " timestamps." << endl;

    //    DrawTrajectory(poses);

    while(groundtruth_reader >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw){
        groundtruth.push_back(Sophus::SE3d(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz)));
    }
    cout << "There are " << groundtruth.size() << " timestamps." << endl;

    while(estimated_reader >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw){
        estimated.push_back(Sophus::SE3d(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz)));
    }
    cout << "There are " << estimated.size() << " timestamps." << endl;
    DrawTwoTrajectory(groundtruth, estimated);

    CalculateError(groundtruth, estimated);

    return 0;
}

void CalculateError(TrajectoryType groundtruth, TrajectoryType estimated){
    double error(0.0);
    double rmse(0.0);
    for (size_t i = 0; i < groundtruth.size(); i++){
        error = (groundtruth[i].inverse() * estimated[i]).log().norm();
        rmse += error * error;
    }
    rmse = sqrt(rmse / groundtruth.size());
    cout << "rmse: " << rmse << endl;
}

/*******************************************************************************************/
void DrawTrajectory(TrajectoryType poses) {

    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}

void DrawTwoTrajectory(TrajectoryType poses1, TrajectoryType poses2) {
    if (poses1.empty() || poses2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Groundtruth: red, Estimated: blue", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses1.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();

            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            auto p3 = poses2[i], p4 = poses2[i + 1];
            glVertex3d(p3.translation()[0], p3.translation()[1], p3.translation()[2]);
            glVertex3d(p4.translation()[0], p4.translation()[1], p4.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <highgui.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>

class CamExtrinsicsCalculus{

public:
    CamExtrinsicsCalculus();
    ~CamExtrinsicsCalculus(){}

    void LoadXYZWORLDtags();
    void LoadXYZCAMtags();
    void LoadUVCAMtags();
    void LoadXYZCAMtags_hardcoded();
    void LoadUVCAMtags_hardcoded();

    void addGentry(Eigen::Vector3d &xyz_world, Eigen::Vector3d &xyz_cam);
    void getRotAndTfromPmatrix(Eigen::MatrixXd m_world_cam);
    Eigen::MatrixXd constructPmatrix(std::vector<Eigen::Vector3d> &xyz_world, std::vector<Eigen::Vector3d> &xyz_cam, std::vector<int> tagIds);
    std::vector<Eigen::Vector3d> cameraPixelsToUndistortXYZPoints(std::vector<Eigen::Vector3d> &uv_cam);
    Eigen::Vector3d cameraPixelsToUndistortXYZPoints(Eigen::Vector3d &uv_cam);
    Eigen::Vector3d confirmCalibration(Eigen::Vector3d xyzpoint);
    void findRigidTransformation(std::vector<Eigen::Vector3d> xyz_world, std::vector<Eigen::Vector3d> xyz_cam,
                        Eigen::MatrixXd &T);
    void transformHPoint(Eigen::Vector4d src, Eigen::Vector4d &dst, Eigen::MatrixXd T);

    bool writeMatrixToFile(Eigen::MatrixXd &m, std::string filename);
    bool readMatrixFromFile(Eigen::MatrixXd &m, std::string filename);
    void printCalibrationToFile(std::string filename);

    void addTagId(int id){this->allTagsFound_ids.push_back(id);}
    void addCamToTagPose(geometry_msgs::PoseStamped p){allTagsFound_camPoses.push_back(p);}

    bool CalculateExtrinsics();
    bool CalculateExtrinsics_WldUvPoints();

    void setInstrinsicsMatrix(Eigen::Matrix3d intrinsic_camera_matrix) {this->intrinsic_camera_matrix = intrinsic_camera_matrix;}
    void setDistorctionMatrix(Eigen::Vector4d distortion_camera_matrix) {this->distortion_camera_matrix = distortion_camera_matrix;}

private:
    Eigen::Matrix3d intrinsic_camera_matrix;
    Eigen::Vector4d distortion_camera_matrix;

    Eigen::MatrixXd G_matrix;
    Eigen::Vector3d wld_centroid ,cam_centroid;

    Eigen::MatrixXd R_ext;
    Eigen::Vector3d t_ext;
    Eigen::Vector3d tc_;

    Eigen::MatrixXd world_to_cam_T;

    std::vector<Eigen::Vector3d> xyz_world;
    std::vector<int> calibtagIds;
    std::vector<Eigen::Vector3d> xyz_cam;
    std::vector<Eigen::Vector3d> uv_cam;

    std::vector<int> allTagsFound_ids;
    std::vector<geometry_msgs::PoseStamped> allTagsFound_camPoses;

    std::ofstream storing_file;

};

double roundToNdecimalPlaces( double x , int n);

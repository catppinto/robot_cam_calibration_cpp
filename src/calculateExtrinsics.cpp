#include <../include/cat_robotcam_calibration/calculateExtrinsics.hpp>


CamExtrinsicsCalculus::CamExtrinsicsCalculus()
{
    this->intrinsic_camera_matrix << 529.6085148771313, 0, 318.3732583194404,
            0, 529.3631821305655, 243.8970374362488,
            0, 0, 1;

    this->distortion_camera_matrix << 0.06015428950370723, -0.1531946478687832,
            0.001782403245658974, 0.001616065169196731;
}

void CamExtrinsicsCalculus::addGentry(Eigen::Vector3d &xyz_world, Eigen::Vector3d &xyz_cam){

    Eigen::MatrixXd Gentry(2, 12);

    double u_cam = xyz_cam(0) / xyz_cam(2);
    double v_cam = xyz_cam(1) / xyz_cam(2);

    Gentry << xyz_world(0), xyz_world(1), xyz_world(2), 1,
            0, 0, 0, 0,
            -u_cam*xyz_world(0), -u_cam*xyz_world(1), -u_cam*xyz_world(2), -u_cam,
            0, 0, 0, 0,
            xyz_world(0), xyz_world(1), xyz_world(2), 1,
            -v_cam*xyz_world(0), -v_cam*xyz_world(1), -v_cam*xyz_world(2), -v_cam;

    //TODO : confirm this part
    if(G_matrix.rows() ==0)
    {
        G_matrix.resize(2, 12);
        G_matrix << Gentry;
    }
    else
    {
        Eigen::MatrixXd temp_matrix(G_matrix.rows(), G_matrix.cols());
        temp_matrix << G_matrix;
        G_matrix.resize(G_matrix.rows()+2, G_matrix.cols());
        G_matrix << temp_matrix, Gentry;
    }
}

void CamExtrinsicsCalculus::getRotAndTfromPmatrix(Eigen::MatrixXd projectionMatrix_computed){

    Eigen::MatrixXd B(3,3), b(3,1), K(3,3);
    B << projectionMatrix_computed.block(0,0,3,3);
    b << projectionMatrix_computed(0,3) , projectionMatrix_computed(1,3) , projectionMatrix_computed(2,3);

    Eigen::MatrixXd intrinsics(3,3);

    Eigen::MatrixXd r(3,3);
    Eigen::Vector3d t;
    intrinsics << (this->intrinsic_camera_matrix).block(0,0,3,3);

    r = intrinsics.inverse() * B;
    t = intrinsics.inverse() * b;

    if(roundToNdecimalPlaces( r.determinant() , 1) == -1)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(r, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd VUt (svd.matrixV().rows(), svd.matrixU().rows());
        VUt << svd.matrixV() * (svd.matrixU()).transpose();
        double determinant_VUt = VUt.determinant();
        double d = (determinant_VUt > 0 ? 1 : -1);
        r = -1* r;
        t = -1* t;
    }

    this->R_ext = r;
    this->t_ext = t;


}

Eigen::MatrixXd CamExtrinsicsCalculus::constructPmatrix(std::vector<Eigen::Vector3d> &xyz_world, std::vector<Eigen::Vector3d> &xyz_cam, std::vector<int> tagIds){

    // create G matrix
    for (int i=0; i< xyz_world.size(); i++)
    {
        addGentry(xyz_world[i], xyz_cam[i]);
    }

    std::cout << "  -------------------------------- " << std::endl;
    std::cout << " G matrix " << std::endl << G_matrix << std::endl;

    // compute P matrix

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(G_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
    std::cout << "Its left singular vectors are the columns of the thin U matrix:" << std::endl << svd.matrixU() << std::endl;

    Eigen::MatrixXd eigenv_gtg(svd.matrixV().rows(), svd.matrixV().cols());
    eigenv_gtg << svd.matrixV();
    std::cout << "Its right singular vectors are the columns of the thin V matrix:" << std::endl << eigenv_gtg << std::endl;

    Eigen::MatrixXd projectioMatrix(4,4);
    double cols_idx = eigenv_gtg.cols()-1;
    projectioMatrix <<  eigenv_gtg(0,cols_idx), eigenv_gtg(1,cols_idx), eigenv_gtg(2,cols_idx) , eigenv_gtg(3,cols_idx),
            eigenv_gtg(4,cols_idx), eigenv_gtg(5,cols_idx), eigenv_gtg(6,cols_idx) , eigenv_gtg(7,cols_idx),
            eigenv_gtg(8,cols_idx), eigenv_gtg(9,cols_idx), eigenv_gtg(10,cols_idx), eigenv_gtg(11,cols_idx),
            0                     , 0                     , 0                      , 1;

    double m_divisor = sqrt(projectioMatrix(2,0)*projectioMatrix(2,0)+projectioMatrix(2,1)*projectioMatrix(2,1)+projectioMatrix(2,2)*projectioMatrix(2,2));
    projectioMatrix = projectioMatrix/m_divisor;
    std::cout << "Projection Matrix Computed : " << std::endl << projectioMatrix << std::endl;

    return projectioMatrix;
}

std::vector<Eigen::Vector3d> CamExtrinsicsCalculus::cameraPixelsToUndistortXYZPoints(std::vector<Eigen::Vector3d> &uv_cam)
{
    std::vector<Eigen::Vector3d> xyz;
    for (int i=0; i< uv_cam.size(); i++)
    {

    Eigen::Matrix3d instrinsicsMatrix;
    instrinsicsMatrix << this->intrinsic_camera_matrix(0,0), 0, this->intrinsic_camera_matrix(0,2),
                    0, this->intrinsic_camera_matrix(1,1), this->intrinsic_camera_matrix(1,2),
                    0, 0, 1;
    Eigen::Vector4d distCoeffs;
    distCoeffs << this->distortion_camera_matrix(0),this->distortion_camera_matrix(1),
                  this->distortion_camera_matrix(2),this->distortion_camera_matrix(3);

    // passar de uv para xyz
    Eigen::Vector3d xyz_distorted = instrinsicsMatrix.inverse() * uv_cam[i];

//    cv::Mat src (3,1,CV_64FC);
//    src.at<double>(0,0) = xyz_distorted(0);
//    src.at<double>(1,0) = xyz_distorted(1);
//    src.at<double>(2,0) = xyz_distorted(2);

//    cv::Mat dst(3,1,CV_64FC);
//    cv::undistortPoints(src, dst, cameraMatrix, distCoeffs);

//    //distorcer o xyz
//    Eigen::Vector3d xyz_undistorted;
//    double x = xyz_distorted(0);
//    double y = xyz_distorted(1);
//    double r = sqrt(x*x+y*y);
//    double k1 = distCoeffs(0);
//    double k2 = distCoeffs(1);
//    double p1 = distCoeffs(2);
//    double p2 = distCoeffs(3);

//    double x_undistort = ( x *( 1 + (k1*r*r) + (k2 * r*r*r*r) )) +
//            2*p1*x*y + p2*(r*r + 2*x*x);

//    double y_undistort = ( y * (1 + (k1*r*r) + (k2 * r*r*r*r) )) +
//                2*p2*y*x + p1*(r*r + 2*y*y);

//    xyz_undistorted << x_undistort, y_undistort,  1.0;

    Eigen::Vector3d uv_point = uv_cam[i];
    std::cout << "uv__cam : " << uv_point(0) << " , "
              << uv_point(1) << " , "
              << uv_point(2) << std::endl;
    std::cout << "xyz_dst : " << xyz_distorted(0) << " , "
              << xyz_distorted(1) << " , "
              << xyz_distorted(2) << std::endl;
//    std::cout << "xyz_undistorted" << std::endl << xyz_undistorted << std::endl;

    xyz.push_back(xyz_distorted); // nao estou a fazer distorcao
    }


    return xyz;

}

Eigen::Vector3d CamExtrinsicsCalculus::cameraPixelsToUndistortXYZPoints(Eigen::Vector3d &uv_cam)
{
    Eigen::Matrix3d cameraMatrix;
    cameraMatrix << this->intrinsic_camera_matrix(0,0), 0, this->intrinsic_camera_matrix(0,2),
                    0, this->intrinsic_camera_matrix(1,1), this->intrinsic_camera_matrix(1,2),
                    0, 0, 1;
    Eigen::Vector4d distCoeffs;
    distCoeffs << this->distortion_camera_matrix(0),this->distortion_camera_matrix(1),
                  this->distortion_camera_matrix(2),this->distortion_camera_matrix(3);

    // passar de uv para xyz
    Eigen::Vector3d xyz_distorted = cameraMatrix.inverse() * uv_cam;

    std::cout << "xyz_distorted" << std::endl << xyz_distorted << std::endl;

    return xyz_distorted;
}

Eigen::Vector3d CamExtrinsicsCalculus::confirmCalibration(Eigen::Vector3d uv_point){

    //enter u,v coordinates

    Eigen::Vector3d xyz_normalized =cameraPixelsToUndistortXYZPoints(uv_point);

    Eigen::MatrixXd r = this->R_ext;
    Eigen::Vector3d tc  = this->tc_;

    Eigen::Vector3d world_point;
    world_point = r.inverse() * xyz_normalized + (-r.inverse()*tc);

            std::cout << "WORLD Pd : "
                      << world_point(0) << " , "
                      << world_point(1) << " , "
                      << world_point(2) << std::endl;
    return world_point;
}

void CamExtrinsicsCalculus::findRigidTransformation(std::vector<Eigen::Vector3d> xyz_world, std::vector<Eigen::Vector3d> xyz_cam,
                    Eigen::MatrixXd &T)
{
    for (int i=0; i< 8; i++)
    {
        std::cout << "w" << i << " : \n" << xyz_world[i] << std::endl;
        std::cout << "c" << i << " : \n" << xyz_cam[i] << std::endl;
    }

    std::vector<Eigen::Vector3d> xyz_world_temp = xyz_world;
    std::vector<Eigen::Vector3d> xyz_cam_temp = xyz_cam;
    // recover centroid from wlr and cam data
    double n_points  = xyz_world_temp.size();
    Eigen::Vector3d wld_centroid, cam_centroid;

    for (int i=0; i< n_points; i++)
    {
        Eigen::Vector3d temp_world, temp_cam;
        temp_world = xyz_world_temp[i];
        temp_cam = xyz_cam_temp[i];
        wld_centroid += temp_world;
        cam_centroid += temp_cam;
    }
    wld_centroid /= n_points;
    cam_centroid /= n_points;
    for (int i=0; i< n_points; i++)
    {
        xyz_world_temp[i] -= wld_centroid;
        xyz_cam_temp[i] -= cam_centroid;
    }
     std::cout << "wld_centroid" << std::endl << wld_centroid << std::endl;
     std::cout << "cam_centroid" << std::endl << cam_centroid << std::endl;


    // acumulation matrix H
    Eigen::Matrix3d H, temp_matrix;
    for (int i=0; i< n_points; i++){
        temp_matrix = (xyz_world_temp[i]) * (xyz_cam_temp[i].transpose()) ;
        H += temp_matrix;
    }

    std::cout << "H" << std::endl << H << std::endl;  // covariance matrix

    // make R orthonormal
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd vut = svd.matrixV()*(svd.matrixU().transpose());
    double d = (vut.determinant()<0)? -1.0 : 1.0;
    Eigen::MatrixXd ident(3,3);
    ident << 1,0,0,
             0,1,0,
             0,0,d;
    Eigen::MatrixXd R = svd.matrixV()*ident*(svd.matrixU()).transpose();
    Eigen::Vector3d t = -R*cam_centroid + wld_centroid;
    T.resize(4,4);
    T << R, t,
         0, 0, 0, 1;

    std::cout << "R" << std::endl << R << std::endl;
    std::cout << "det R" << std::endl << R.determinant() << std::endl;
    std::cout << "t" << std::endl << t << std::endl;
    std::cout << "T" << std::endl << T << std::endl;

    world_to_cam_T = T;
}

void CamExtrinsicsCalculus::transformHPoint(Eigen::Vector4d src, Eigen::Vector4d &dst, Eigen::MatrixXd T){

    dst = T * src;
}

bool CamExtrinsicsCalculus::writeMatrixToFile(Eigen::MatrixXd &m, std::string filename){

    if(m.cols() != 4 || m.rows() != 4)
    {
        std::cout << "Writing matrix to file failed. Wrong matrix size" << std::endl;
        return false;
    }

    std::ofstream file(filename.c_str());

    if(file.is_open()){
        file << "Transformation matrix (Cam To World): \n" ;
        for (int i=0 ; i< m.rows(); i++){
            for (int j=0; j< m.cols(); j++){
                if(j!= m.cols()-1)
                    file << m(i,j) << "  ";
                else
                    file << m(i,j) << " \n ";
            }
        }
        file << "End of Transformation matrix \n" ;
    }
    else
    {
        std::cout << "Writing matrix to file failed. Unable to open file." << std::endl;
        return false;
    }

    return true;

}

bool CamExtrinsicsCalculus::readMatrixFromFile(Eigen::MatrixXd &m, std::string filename){

    std::ifstream file(filename.c_str());
    std::string line;
    Eigen::MatrixXd T(4,4);
    if(file.is_open()){

        getline (file,line);
        for (int i=0; i< 4; i++)
        {
          getline (file,line);
          std::stringstream ss(line.c_str());
          double a;
          int counter =0;
          while (ss >> a)
          {
            T(i, counter) = a;
            counter = counter+1;
          }
        }
        file.close();
        std::cout << " matrix : " << std::endl << T << '\n';
    }
    else
    {
        std::cout << "Reading file failed. Unable to open file." << std::endl;
        return false;
    }

    return true;
}

void CamExtrinsicsCalculus::LoadXYZWORLDtags(){
    xyz_world.resize(8);
    calibtagIds.resize(8);
    xyz_cam.resize(8);

    Eigen::Vector3d xyz_world_tag;
    //tag0
    xyz_world_tag << 0.7005, 0.024, 0.0;
    xyz_world[0]= xyz_world_tag;
    calibtagIds[0] = 0;

    //tag1
    xyz_world_tag << 0.40, 0.024, 0.0;
    xyz_world[1]= xyz_world_tag;
    calibtagIds[1] = 1;

    //tag2
    xyz_world_tag << 0.2495, 0.024, 0.0;
    xyz_world[2]= xyz_world_tag;
    calibtagIds[2] = 2;

    //tag11
    xyz_world_tag << 0.402, 0.19, 0.0;
    xyz_world[3]= xyz_world_tag;
    calibtagIds[3] = 11;

    //tag4
    xyz_world_tag << 0.449, -0.004, 0.16627;
    xyz_world[4]= xyz_world_tag;
    calibtagIds[4] = 4;

    //tag5
    xyz_world_tag << 0.449, -0.004, 0.11877;
    xyz_world[5]= xyz_world_tag;
    calibtagIds[5] = 5;

    //tag6
    xyz_world_tag << 0.449, -0.004, 0.07127;
    xyz_world[6]= xyz_world_tag;
    calibtagIds[6] = 6;

    //tag7
    xyz_world_tag << 0.449, -0.004, 0.02377;
    xyz_world[7]= xyz_world_tag;
    calibtagIds[7] = 7;

    for (int i=0; i< xyz_cam.size(); i++)
    {
        std::cout << "  -------------------------------- " << std::endl;
        std::cout << "  TAG # " << calibtagIds[i]  << std::endl;
        std::cout << " WORLD " << std::endl << xyz_world[i] << std::endl;
    }
}

void CamExtrinsicsCalculus::LoadXYZCAMtags(){
    Eigen::Vector3d xyz_cam_tag;

    for (int i =0; i< calibtagIds.size(); i++)
    {
        // find
        std::vector<int>::iterator iter = std::find(this->allTagsFound_ids.begin(), this->allTagsFound_ids.end(), calibtagIds[i]);
        size_t index = std::distance(allTagsFound_ids.begin(), iter);
        if(index == allTagsFound_ids.size())
        {
            std::cout << "Cannot find tag " << calibtagIds[i] << std::endl;
            xyz_cam_tag << -1000, -1000, -1000;

            xyz_cam[i] = xyz_cam_tag;
            continue;
        }
        else
        {

            geometry_msgs::PoseStamped pos = this->allTagsFound_camPoses[index];
            xyz_cam_tag << pos.pose.position.x, pos.pose.position.y, pos.pose.position.z;

            xyz_cam[i] = xyz_cam_tag;
        }
    }

    for (int i =0; i< calibtagIds.size(); i++)
    {
        std::cout << " TAG # " << calibtagIds[i] << std::endl;
        std::cout << " XYZ_cam " <<  xyz_cam[i]<< std::endl;
    }
}

void CamExtrinsicsCalculus::LoadUVCAMtags_hardcoded(){
    uv_cam.resize(8);

    Eigen::Vector3d c1, c2, c3, c4;
    Eigen::Vector3d uv_cam_tag;

    //tag0

    c1 << 38.5722427368, 265.748321533, 1.0;
    c2 << 77.4839935303, 267.500915527, 1.0;
    c3 << 91.3074645996, 242.959609985, 1.0;
    c4 << 54.4688720703, 241.082885742, 1.0;
    uv_cam_tag= (c1+c2+c3+c4)/4;
    uv_cam[0]= uv_cam_tag;

    //tag1
    c1 << 347.844451904, 278.39666748 , 1.0;
    c2 << 386.319396973, 279.589172363, 1.0;
    c3 << 383.000671387, 255.293136597, 1.0;
    c4 << 346.686798096, 253.704772949, 1.0;
    uv_cam_tag= (c1+c2+c3+c4)/4;
    uv_cam[1]= uv_cam_tag;


    //tag2
    c1 <<  498.496002197, 284.615325928, 1.0;
    c2 <<  537.08782959, 286.589996338, 1.0;
    c3 <<  526.105773926, 261.166992188, 1.0;
    c4 <<  490.007293701, 259.998168945, 1.0;
    uv_cam_tag= (c1+c2+c3+c4)/4;
    uv_cam[2]= uv_cam_tag;


    //tag11
    c1 <<  397.319976807, 389.17175293, 1.0;
    c2 <<  349.834594727, 387.880340576, 1.0;
    c3 <<  351.817993164, 430.871856689, 1.0;
    c4 <<  403.397216797, 432.288879395, 1.0;
    uv_cam_tag= (c1+c2+c3+c4)/4;
    uv_cam[3]= uv_cam_tag;

    //tag4

    c1 <<  299.718933105, 71.432975769, 1.0;
    c2 <<  298.679748535, 112.697143555, 1.0;
    c3 <<  340.793060303, 115.227043152, 1.0;
    c4 <<  344.220672607, 73.8323745728, 1.0;
    uv_cam_tag= (c1+c2+c3+c4)/4;
    uv_cam[4]= uv_cam_tag;

    //tag5
    c1 <<  298.840209961, 123.029266357, 1.0;
    c2 <<  297.07434082, 160.063598633, 1.0;
    c3 <<  336.935760498, 161.758377075, 1.0;
    c4 <<  339.83001709, 125.438903809, 1.0;
    uv_cam_tag= (c1+c2+c3+c4)/4;
    uv_cam[5]= uv_cam_tag;

    //tag6
    c1 <<  296.99887085, 168.663223267, 1.0;
    c2 <<  296.70690918, 201.317184448, 1.0;
    c3 <<  333.610900879, 203.413085938, 1.0;
    c4 <<  336.105102539, 171.108093262, 1.0;
    uv_cam_tag= (c1+c2+c3+c4)/4;
    uv_cam[6]= uv_cam_tag;

    //tag7
    c1 <<  296.540435791, 210.223800659, 1.0;
    c2 <<  295.258178711, 238.848556519, 1.0;
    c3 <<  330.657287598, 240.20098877, 1.0;
    c4 <<  333.131134033, 211.259674072, 1.0;
    uv_cam_tag= (c1+c2+c3+c4)/4;
    uv_cam[7] = uv_cam_tag;

    for (int i=0; i< uv_cam.size(); i++)
    {
        std::cout << "  -------------------------------- " << std::endl;
        std::cout << "  TAG # " << calibtagIds[i]  << std::endl;
        std::cout << " uv_cam " << std::endl << uv_cam[i] << std::endl;
    }
}

void CamExtrinsicsCalculus::LoadXYZCAMtags_hardcoded(){
    xyz_cam.resize(8);
    calibtagIds.resize(8);

    Eigen::Vector3d xyz_cam_tag;
    //tag0
    xyz_cam_tag << -0.321184602274, 0.0128276579339, 0.673121773699;
    xyz_cam[0]= xyz_cam_tag;

    //tag1
    xyz_cam_tag << 0.0609618722214, 0.0288604380938, 0.678714156174;
    xyz_cam[1]= xyz_cam_tag;

    //tag2
    xyz_cam_tag << 0.249648532916, 0.0370654230865, 0.680061164152;
    xyz_world[2]= xyz_cam_tag;

    //tag11
    xyz_cam_tag << 0.0552627671597, 0.159918707469, 0.511924077352;
    xyz_cam[3]= xyz_cam_tag;

    //tag4
    xyz_cam_tag << 0.00272367615797, -0.166222506828, 0.586358587007;
    xyz_cam[4]= xyz_cam_tag;

    //tag5
    xyz_cam_tag << -0.00023708442727, -0.119829988825, 0.628698558112;
    xyz_cam[5]= xyz_cam_tag;

    //tag6
    xyz_cam_tag << -0.00320446563741, -0.0723873292449, 0.66744483229;
    xyz_cam[6]= xyz_cam_tag;

    //tag7,
    xyz_cam_tag << -0.00594974756818, -0.0246083493194, 0.705823648429;
    xyz_cam[7]= xyz_cam_tag;

    for (int i=0; i< xyz_cam.size(); i++)
    {
        std::cout << "  -------------------------------- " << std::endl;
        std::cout << "  TAG # " << calibtagIds[i]  << std::endl;
        std::cout << " CAM " << std::endl << xyz_cam[i] << std::endl;
    }
}


void CamExtrinsicsCalculus::printCalibrationToFile(std::string filename) {

    storing_file.open(filename.c_str());

    storing_file << " ----------------------------------------------- " << std::endl;
    storing_file << "               STORING FILE                      " << std::endl;
    storing_file << ros::Time::now() << std::endl;
    storing_file << " ----------------------------------------------- " << std::endl;

    for (int i =0; i< calibtagIds.size(); i++)
    {
        // store tag id
        storing_file << " ----------------------------------------------- " << std::endl;
        storing_file << "TAG ID : " << calibtagIds[i] << std::endl;

        storing_file << "XYZ world : " << std::endl << this->xyz_world[i] << std::endl;
        storing_file << "XYZ cam   : " << std::endl << this->xyz_cam[i] << std::endl;
    }
    storing_file << " ----------------------------------------------- " << std::endl;
    storing_file << "             CALIBRATION MATRIX                  " << std::endl;
    storing_file << " ----------------------------------------------- " << std::endl;
    storing_file << world_to_cam_T << std::endl;


    storing_file << " ----------------------------------------------- " << std::endl;
    storing_file << "             end of STORING FILE                 " << std::endl;
    storing_file << " ----------------------------------------------- " << std::endl;

    storing_file.close();

}

bool CamExtrinsicsCalculus::CalculateExtrinsics(){

    LoadXYZWORLDtags();
    LoadXYZCAMtags();

    Eigen::MatrixXd T(4,4);
    findRigidTransformation(this->xyz_world, this->xyz_cam, T);

    std::string path = "/home/cat/catkin_ws/src/biovision/cat_robotcam_calibration/calibration_matrix";

    std::string filename = path + "/world_cam_matrix.txt";
    if(writeMatrixToFile(T, filename))
    {
        std::cout << "Successfully written to file!" << std::endl ;

        Eigen::MatrixXd M(4,4);
        readMatrixFromFile(M, filename);
    }

    //confirm the transformation to wld coord
    for (int i=0; i<xyz_world.size(); i++)
    {
        Eigen::Vector4d p, p_transformed;
        Eigen::Vector3d uv_p = xyz_cam[i];
        p(0) = uv_p(0);
        p(1) = uv_p(1);
        p(2) = uv_p(2);
        p(3) = 1;
        transformHPoint(p, p_transformed, T);
        Eigen::Vector3d p_w = xyz_world[i];
        std::cout << "WORLD PD : "
                  << p_transformed(0) << " , "
                  << p_transformed(1) << " , "
                  << p_transformed(2) << std::endl;

        std::cout << "WORLD GT : "
                  << p_w(0) << " , "
                  << p_w(1) << " , "
                  << p_w(2) << std::endl;
    }

    filename = path + "/calibration.txt";
    printCalibrationToFile(filename);

    return false;

}



bool CamExtrinsicsCalculus::CalculateExtrinsics_WldUvPoints(){

    LoadXYZWORLDtags();
    LoadUVCAMtags_hardcoded();

    Eigen::MatrixXd projection_matrix_computed(4,4);
    projection_matrix_computed = constructPmatrix(this->xyz_world, this->uv_cam, this->calibtagIds);

    getRotAndTfromPmatrix(projection_matrix_computed);

    std::cout << " Projection Matrix " << std::endl << projection_matrix_computed << std::endl;

    std::cout << " Rext " << std::endl << this->R_ext << std::endl;

    std::cout << " text " << std::endl << this->t_ext << std::endl;

//    std::string path = "/home/cat/catkin_ws/src/biovision/cat_robotcam_calibration/calibration_matrix";

//    std::string filename = path + "/world_cam_matrix.txt";
//    if(writeMatrixToFile(T, filename))
//    {
//        std::cout << "Successfully written to file!" << std::endl ;

//        Eigen::MatrixXd M(4,4);
//        readMatrixFromFile(M, filename);
//    }

//    //confirm the transformation to wld coord
//    for (int i=0; i<xyz_world.size(); i++)
//    {
//        Eigen::Vector4d p, p_transformed;
//        Eigen::Vector3d uv_p = xyz_cam[i];
//        p(0) = uv_p(0);
//        p(1) = uv_p(1);
//        p(2) = uv_p(2);
//        p(3) = 1;
//        transformHPoint(p, p_transformed, T);
//        Eigen::Vector3d p_w = xyz_world[i];
//        std::cout << "WORLD PD : "
//                  << p_transformed(0) << " , "
//                  << p_transformed(1) << " , "
//                  << p_transformed(2) << std::endl;

//        std::cout << "WORLD GT : "
//                  << p_w(0) << " , "
//                  << p_w(1) << " , "
//                  << p_w(2) << std::endl;
//    }

//    filename = path + "/calibration.txt";
//    printCalibrationToFile(filename);

//    return false;

}

double roundToNdecimalPlaces( double x , int n)
{
    const double sd = pow(10, n);
    double a  =int(x*sd + (x<0? -0.5 : 0.5))/sd;
    return a;
}

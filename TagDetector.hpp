#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp> 

#include <vector>
#include <iostream>
#include <fstream>

bool LoadBoard(std::string file, int tag_size, std::vector<std::vector<cv::Point3f>>& tag_coord)
{
    std::ifstream fin(file);
    if (!fin)
    {
        return false;
    }

    for (int tag_num = 0; tag_num < tag_size; tag_num++)
    {
        std::vector<cv::Point3f> single_tag_coord;
        for (int j = 0; j < 4; j++)
        {
            cv::Point3f p;
            fin >> p.x >> p.y >> p.z;
            single_tag_coord.push_back(p);
        }
        tag_coord.push_back(single_tag_coord);
    }
    fin.close();
    return true;
}

int TagDetector(cv::Mat& image, std::vector<std::vector<cv::Point3f>>& tag_coord, 
                cv::Mat& inrinsic, cv::Mat distort, Eigen::Matrix4f& pose)
{
    std::vector<cv::Point3f> t_tag_coord;
    std::vector<cv::Point2f> t_cam_coord;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::vector<int> markerIds;

    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    if(markerIds.size() > 0)
    {
        for(uint8_t tag_num = 0; tag_num < markerIds.size(); tag_num++)
        {
            int id = markerIds[tag_num];
            for (int corner_num = 0; corner_num < 4; corner_num++)
            {
                t_tag_coord.push_back(tag_coord[id][corner_num]);
                t_cam_coord.push_back(markerCorners[tag_num][corner_num]);
            }
        }

        std::cout << t_tag_coord[0] << std::endl;
        std::cout << t_cam_coord[0] << std::endl;

        cv::Mat rvec, tvec;
        cv::solvePnPRansac(t_tag_coord, t_cam_coord, inrinsic, distort, rvec, tvec);
        
        cv::Mat R, t;
        cv::Rodrigues(rvec, R);
        t = tvec.clone();

        Eigen::Matrix3f R_eigen;
        Eigen::Vector3f t_eigen;
        cv::cv2eigen(R,R_eigen);
        cv::cv2eigen(t,t_eigen);

        pose.block<3,3>(0,0) = R_eigen;
        pose.block<3,1>(0,3) = t_eigen;
        pose.block<1,3>(3,0) = Eigen::RowVector3f::Zero();
        pose(3,3) = 1;
    }

    return markerIds.size();
}

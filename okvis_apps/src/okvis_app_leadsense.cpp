/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Jun 26, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <memory>
#include <functional>
#include <atomic>
#include <stdio.h>
#include <Eigen/Core>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Woverloaded-virtual"

#pragma GCC diagnostic pop


#include <boost/filesystem.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <okvis/VioParametersReader.hpp>
#include <okvis/ThreadedKFVio.hpp>

#include <evo_stereocamera.h>
#include <evo_matconverter.h>

using namespace std;

class PoseViewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    constexpr static const double imageSize = 800.0;
    PoseViewer()
    {
        cv::namedWindow("LeadSense Top View");
        _image.create(imageSize, imageSize, CV_8UC3);
        drawing_ = false;
        showing_ = false;
    }
    // this we can register as a callback
    void publishFullStateAsCallback(
            const okvis::Time & /*t*/, const okvis::kinematics::Transformation & T_WS,
            const Eigen::Matrix<double, 9, 1> & speedAndBiases,
            const Eigen::Matrix<double, 3, 1> & /*omega_S*/)
    {

        // just append the path
        Eigen::Vector3d r = T_WS.r();
        Eigen::Matrix3d C = T_WS.C();
    	_path.push_back(cv::Point2d(r[0], r[1]));
		_heights.push_back(r[2]);
        // maintain scaling
		if (r[0] - _frameScale < _min_x)
			_min_x = r[0] - _frameScale;
		if (r[1] - _frameScale < _min_y)
			_min_y = r[1] - _frameScale;
		if (r[2] < _min_z)
			_min_z = r[2];
		if (r[0] + _frameScale > _max_x)
			_max_x = r[0] + _frameScale;
		if (r[1] + _frameScale > _max_y)
			_max_y = r[1] + _frameScale;
		if (r[2] > _max_z)
			_max_z = r[2];
        _scale = std::min(imageSize / (_max_x - _min_x), imageSize / (_max_y - _min_y));

        // draw it
        while (showing_) {
        }
        drawing_ = true;
        // erase
        _image.setTo(cv::Scalar(10, 10, 10));
        drawPath();
        // draw axes
        Eigen::Vector3d e_x = C.col(1);
        Eigen::Vector3d e_y = C.col(0);
        Eigen::Vector3d e_z = C.col(2);
        cv::line(
                    _image,
                    convertToImageCoordinates(_path.back()),
                    convertToImageCoordinates(
                        _path.back() + cv::Point2d(e_x[0], e_x[1]) * _frameScale),
                cv::Scalar(0, 0, 255), 1, CV_AA);
        cv::line(
                    _image,
                    convertToImageCoordinates(_path.back()),
                    convertToImageCoordinates(
                        _path.back() + cv::Point2d(e_y[0], e_y[1]) * _frameScale),
                cv::Scalar(0, 255, 0), 1, CV_AA);
        cv::line(
                    _image,
                    convertToImageCoordinates(_path.back()),
                    convertToImageCoordinates(
                        _path.back() + cv::Point2d(e_z[0], e_z[1]) * _frameScale),
                cv::Scalar(255, 0, 0), 1, CV_AA);

        // some text:
        std::stringstream postext;
		postext << "position = [" << r[0] << ", " << r[1] << ", " << r[2] << "]";
        cv::putText(_image, postext.str(), cv::Point(15,25),
                    cv::FONT_HERSHEY_COMPLEX, 0.9, cv::Scalar(255,255,255), 1);
        std::stringstream veltext;
        veltext << "velocity = [" << speedAndBiases[1] << ", " << speedAndBiases[0] << ", " << speedAndBiases[2] << "]";
        cv::putText(_image, veltext.str(), cv::Point(15,60),
                    cv::FONT_HERSHEY_COMPLEX, 0.9, cv::Scalar(255,255,255), 1);
        drawing_ = false; // notify
    }
    void display()
    {
        while (drawing_) {
        }
        showing_ = true;
        cv::imshow("LeadSense Top View", _image);
        showing_ = false;
        cv::waitKey(1);
    }
private:
    cv::Point2d convertToImageCoordinates(const cv::Point2d & pointInMeters) const
    {
        cv::Point2d pt = (pointInMeters - cv::Point2d(_min_x, _min_y)) * _scale;
        return cv::Point2d(pt.x, imageSize - pt.y); // reverse y for more intuitive top-down plot
    }
    void drawPath()
    {
        for (size_t i = 0; i + 1 < _path.size(); ) {
            cv::Point2d p0 = convertToImageCoordinates(_path[i]);
            cv::Point2d p1 = convertToImageCoordinates(_path[i + 1]);
            cv::Point2d diff = p1-p0;

            if(diff.dot(diff)<2.0){
                _path.erase(_path.begin() + i + 1);  // clean short segment
                _heights.erase(_heights.begin() + i + 1);
                continue;
            }

            double rel_height = (_heights[i] - _min_z + _heights[i + 1] - _min_z)
                    * 0.5 / (_max_z - _min_z);
            cv::line(
                        _image,
                        p0,
                        p1,
                        rel_height * cv::Scalar(125, 0, 125)
                        + (1.0 - rel_height) * cv::Scalar(0, 0, 255),
                        1, CV_AA);
            i++;
        }
    }
    cv::Mat _image;
    std::vector<cv::Point2d> _path;
    std::vector<double> _heights;
    double _scale = 1.0;
    double _min_x = -0.5;
    double _min_y = -0.5;
    double _min_z = -0.5;
    double _max_x = 0.5;
    double _max_y = 0.5;
    double _max_z = 0.5;
    const double _frameScale = 0.2;  // [m]
    std::atomic_bool drawing_;
    std::atomic_bool showing_;
};


int main(int argc, char **argv)
{	
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
    FLAGS_colorlogtostderr = 1;

	evo::bino::StereoCamera camera;
	evo::RESULT_CODE res;
	
    if (argc < 2)
    {
        LOG(ERROR) << "Usage: " << "./" << argv[0] << " config-yaml-file [.evo]" << std::endl;
		return -1;
    }
    else if (argc == 2)
    {
    	res = camera.open(evo::bino::RESOLUTION_FPS_MODE_SD400_60);
    }
    else if (argc >= 3)
    {
    	res = camera.open(argv[2]);
    }
    
	if (res != evo::RESULT_CODE_OK)
	{
		LOG(ERROR) << "open camera failed.. " << evo::result_code2str(res) << std::endl;
		return 0;
	}
	
	//print rectified stereo parameters
	evo::bino::StereoParameters paras_new = camera.getStereoParameters(true);
	std::cout << "###################" << std::endl << "rectified stereo parameters: " << std::endl;
	std::cout << "left fx = " << paras_new.leftCam.focal.x << " fy = " << paras_new.leftCam.focal.y << " cx = " << paras_new.leftCam.center.x << " cy = " << paras_new.leftCam.center.y << std::endl;
	std::cout << "right fx = " << paras_new.rightCam.focal.x << " fy = " << paras_new.rightCam.focal.y << " cx = " << paras_new.rightCam.center.x << " cy = " << paras_new.rightCam.center.y << std::endl;
	std::cout << "R = " << paras_new.R.x << ", " << paras_new.R.y << ", " << paras_new.R.z << std::endl;
	std::cout << "T = " << paras_new.T.x << ", " << paras_new.T.y << ", " << paras_new.T.z << std::endl;
	std::cout << "###################" << std::endl;


    // read configuration file
    std::string configFilename(argv[1]);

	okvis::VioParametersReader vio_parameters_reader(configFilename);
	okvis::VioParameters parameters;
	vio_parameters_reader.getParameters(parameters);

	okvis::ThreadedKFVio okvis_estimator(parameters);

	PoseViewer poseViewer;
	okvis_estimator.setFullStateCallback(
               std::bind(&PoseViewer::publishFullStateAsCallback, &poseViewer,
                         std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3, std::placeholders::_4));

    okvis_estimator.setBlocking(true);
	
	camera.setIMUDataType(evo::imu::IMU_DATA_TYPE_RAW_CALIBRATED);
	camera.setIMUDataRetrieveMode(evo::imu::IMU_DATA_RETRIEVE_MODE_NEWEST_IMAGE);
	res = camera.startRetrieveIMU();
	if (res != evo::RESULT_CODE_OK)
	{		
		LOG(ERROR) << "start retrieve IMU failed.. "  << evo::result_code2str(res) << std::endl;
		return 0;
	}
   
	evo::bino::GrabParameters grab_parameters;
	grab_parameters.do_rectify = true;

    while (true)
    {
    	res = camera.grab(grab_parameters);

		if (res == evo::RESULT_CODE_OK)
		{
			std::vector<evo::imu::IMUData> imu_data = camera.retrieveIMUData();
			
			evo::Mat<unsigned char> evo_left = camera.retrieveImage(evo::bino::SIDE_LEFT);//gray 1 channel
    		evo::Mat<unsigned char> evo_right = camera.retrieveImage(evo::bino::SIDE_RIGHT);//gray 1 channel
			uint32_t img_s = (uint32_t)camera.getCurrentFrameTimeCode();
			uint32_t img_ns = (camera.getCurrentFrameTimeCode() - img_s) * 1000000000;
			okvis::Time t = okvis::Time(img_s, img_ns);
       
		    for (int i = 0; i < imu_data.size(); i++)
		    {
			   Eigen::Vector3d accel;
			   accel[0] = imu_data.at(i).accel_calibrated[0] * 9.8; 
			   accel[1] = imu_data.at(i).accel_calibrated[1] * 9.8;
			   accel[2] = imu_data.at(i).accel_calibrated[2] * 9.8;
			   Eigen::Vector3d gyro;
			   gyro[0] = imu_data.at(i).gyro_calibrated[0]; 
			   gyro[1] = imu_data.at(i).gyro_calibrated[1]; 
			   gyro[2] = imu_data.at(i).gyro_calibrated[2];

			   uint32_t imu_s = (uint32_t)imu_data.at(i).timestamp;
			   uint32_t imu_ns = (imu_data.at(i).timestamp - imu_s) * 1000000000;
			   okvis::Time t_imu = okvis::Time(imu_s, imu_ns);
			   okvis_estimator.addImuMeasurement(t_imu, accel, gyro);
			}
			
        	cv::Mat left_img = evo::evoMat2cvMat(evo_left);
        	cv::Mat right_img = evo::evoMat2cvMat(evo_right);
        	
	       	okvis_estimator.addImage(t, 0, left_img);
            okvis_estimator.addImage(t, 1, right_img);
		}
		
    	okvis_estimator.display();
		poseViewer.display();
    }

	camera.close();


    return 0;
}

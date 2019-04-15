///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


/*************************************************************************
** This sample demonstrates how to use the ZED for positional tracking  **
** and display camera motion in an OpenGL window. 		                **
**************************************************************************/

// Standard includes
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <filesystem>

// ZED includes
#include <sl/Camera.hpp>
#include "csvFile.hpp"

// OpenCV includes
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"


// Sample includes
#include "TrackingViewer.hpp"

// Using std namespace
using namespace std;
using namespace sl;
namespace fs = std::experimental::filesystem;

// Create ZED objects
sl::Camera zed;
sl::IMUData imu_data;
sl::Pose camera_pose;
std::thread zed_callback;
bool quit = false;

// OpenGL window to display camera motion
GLViewer viewer;

const int MAX_CHAR = 128;

// Sample functions
void startZED();
void run();
void runMCS();
void close();
void transformPose(sl::Transform &pose, float tx);
cv::Mat slMat2cvMat(Mat& input);

int main(int argc, char **argv) {

	// Set configuration parameters for the ZED
	InitParameters initParameters;
	initParameters.camera_resolution = RESOLUTION::RESOLUTION_HD720;
	initParameters.depth_mode = DEPTH_MODE_PERFORMANCE;
	initParameters.coordinate_units = UNIT_METER;
	initParameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;

	if (argc > 1 && std::string(argv[1]).find(".svo"))
		initParameters.svo_input_filename.set(argv[1]);

	// Open the camera
	ERROR_CODE err = zed.open(initParameters);
	if (err != sl::SUCCESS) {
		std::cout << sl::toString(err) << std::endl;
		zed.close();
		return 1; // Quit if an error occurred
	}

	
	// Set positional tracking parameters
	TrackingParameters trackingParameters;
	trackingParameters.initial_world_transform = sl::Transform::identity();
	trackingParameters.enable_spatial_memory = true;

	// Start motion tracking
	zed.enableTracking(trackingParameters);

	// Initialize OpenGL viewer
	viewer.init(zed.getCameraInformation().camera_model);

	// Start ZED callback
	startZED();

	// Set the display callback
	glutCloseFunc(close);
	glutMainLoop();
	

	return 0;
}


/**
 *   Launch ZED thread. Using a thread here allows to retrieve camera motion and display it in a GL window concurrently.
 **/
void startZED() {
	quit = false;
	zed_callback = std::thread(run);
}

/**
 *  This function loops to get image and motion data from the ZED. It is similar to a callback.
 *  Add your own code here.
 **/
void run() {

	float tx = 0, ty = 0, tz = 0;
	float rx = 0, ry = 0, rz = 0;

	// Get the distance between the center of the camera and the left eye
	float translation_left_to_center = zed.getCameraInformation().calibration_parameters.T.x * 0.5f;

	// Create text for GUI
	char text_rotation[MAX_CHAR];
	char text_translation[MAX_CHAR];
	char text_imu_angular_velocity[MAX_CHAR];
	char text_imu_acc[MAX_CHAR];

	// Create common directory to output collected data. 
	std::string output_dir = "output/";
	std::string cam0_dir = output_dir + "cam0/"; // left cam directory.  
	std::string cam1_dir = output_dir + "cam1/"; // right cam directory.
	std::string imu_dir = output_dir + "imu/";
	std::string cam_params_dir = output_dir + "camera_parameters/";

	fs::create_directory(output_dir);
	fs::create_directory(cam0_dir);
	fs::create_directory(cam1_dir);
	fs::create_directory(imu_dir);
	fs::create_directory(cam_params_dir);

	// Create a CSV file to log motion tracking data
	std::ofstream outputFile;
	ofstream outputFile_imu;
	std::string csvName = "Motion_data";
	outputFile.open(imu_dir + csvName + ".csv");
	if (!outputFile.is_open())
		cout << "WARNING: Can't create CSV file. Run the application with administrator rights." << endl;
	else
		outputFile << "Timestamp(ns);Rotation_X(rad);Rotation_Y(rad);Rotation_Z(rad);Position_X(m);Position_Y(m);Position_Z(m);" << endl;


	if (zed.getCameraInformation().camera_model == MODEL_ZED_M)
	{
		
			outputFile_imu << "#timestamp [ns];w_RS_S_x [rad s^-1];w_RS_S_y [rad s^-1];w_RS_S_z [rad s^-1];a_RS_S_x [m s^-2];a_RS_S_y [m s^-2];a_RS_S_z [m s^-2]" << std::endl;
	}

	char key = ' ';
	unsigned int i = 0;
	while ((!quit && zed.getSVOPosition() != zed.getSVONumberOfFrames() - 1) && key != 'q') {
		if (zed.grab() == SUCCESS) {
			// Get the position of the camera in a fixed reference frame (the World Frame)
			TRACKING_STATE tracking_state = zed.getPosition(camera_pose, sl::REFERENCE_FRAME_WORLD);

			if (tracking_state == TRACKING_STATE_OK) {
				// Capture images from both left and right cameras. 
				sl::Mat zed_image_left;
				sl::Mat zed_image_right;

				// retrieve images from zed camera. 
				zed.retrieveImage(zed_image_left, VIEW_LEFT);
				zed.retrieveImage(zed_image_right, VIEW_RIGHT);

				// convert images to opencv format. 
				cv::Mat ocv_left_image = slMat2cvMat(zed_image_left);
				cv::Mat ocv_right_image = slMat2cvMat(zed_image_right);

				// getPosition() outputs the position of the Camera Frame, which is located on the left eye of the camera.
				// To get the position of the center of the camera, we transform the pose data into a new frame located at the center of the camera.
				// The generic formula used here is: Pose(new reference frame) = M.inverse() * Pose (camera frame) * M, where M is the transform between two frames.
				transformPose(camera_pose.pose_data, translation_left_to_center); // Get the pose at the center of the camera (baseline/2 on X axis)

				// Update camera position in the viewing window
				viewer.updateZEDPosition(camera_pose.pose_data);

				// Get quaternion, rotation and translation
				sl::float4 quaternion = camera_pose.getOrientation();
				sl::float3 rotation = camera_pose.getEulerAngles(); // Only use Euler angles to display absolute angle values. Use quaternions for transforms.
				sl::float3 translation = camera_pose.getTranslation();

				// Display translation and rotation (pitch, yaw, roll in OpenGL coordinate system)
				snprintf(text_rotation, MAX_CHAR, "%3.2f; %3.2f; %3.2f", rotation.x, rotation.y, rotation.z);
				snprintf(text_translation, MAX_CHAR, "%3.2f; %3.2f; %3.2f", translation.x, translation.y, translation.z);

				// Save the pose data in a csv file
				if (outputFile.is_open())
					outputFile << zed.getTimestamp(sl::TIME_REFERENCE::TIME_REFERENCE_IMAGE) << "; " << text_rotation << "; " << text_translation << ";" << endl;

				// Save imu raw data 
				if (zed.getCameraInformation().camera_model == MODEL_ZED_M)
				{
					zed.getIMUData(imu_data, sl::TIME_REFERENCE::TIME_REFERENCE_IMAGE);
					auto time_stamp = imu_data.timestamp;
					auto linear_acc = imu_data.linear_acceleration;
					auto angular_vel = imu_data.angular_velocity;

					// write imu data to disk. 
					snprintf(text_imu_angular_velocity, MAX_CHAR, "%3.2f; %3.2f; %3.2f", angular_vel.x, angular_vel.y, angular_vel.z);
					snprintf(text_imu_acc, MAX_CHAR, "%3.2f; %3.2f; %3.2f", linear_acc.x, linear_acc.y, linear_acc.z);
					std::cout << "angular velocity: " << angular_vel.x << ", " << angular_vel.y << ", " << angular_vel.z << '\n';
					std::cout << "Linear accelration: " << linear_acc.x << ", " << linear_acc.y << ", " << linear_acc.z << '\n';
					if (outputFile_imu.is_open())
						outputFile_imu << time_stamp << "; " << text_imu_angular_velocity << "; " << text_imu_acc << ";" << std::endl;
				}


				// Save/show image data to screen/Disk. 
				cv::imwrite(cam0_dir + "left" + std::to_string(i) + ".jpg", ocv_left_image);
				cv::imwrite(cam1_dir + "right" + std::to_string(i) + ".jpg", ocv_right_image);
				cv::imshow("left", ocv_left_image);
				cv::imshow("right", ocv_right_image);
				key = cv::waitKey(30);
				++i;
			}

			// Update rotation, translation and tracking state values in the OpenGL window
			viewer.updateText(string(text_translation), string(text_rotation), tracking_state);
		}
		else sl::sleep_ms(1);
	}

	// write camera intrinsic parameters to disk. 
	std::ofstream cam_params_left(cam_params_dir + "left.txt");
	std::ofstream cam_params_right(cam_params_dir + "right.txt");
	auto cam_info = zed.getCameraInformation().calibration_parameters_raw;

	//output cam_left params
	cam_params_left << cam_info.left_cam.fx << " " << cam_info.left_cam.fy << " "
		<< cam_info.left_cam.cx << " " << cam_info.left_cam.cy << '\n';

	//output cam_right params 
	cam_params_right << cam_info.right_cam.fx << ' ' << cam_info.right_cam.fy << ' '
		<< cam_info.right_cam.cx << ' ' << cam_info.right_cam.cy << '\n';
}

/**
 *  Trasnform pose to create a Tracking Frame located in a separate location from the Camera Frame
 **/
void transformPose(sl::Transform &pose, float tx) {
	sl::Transform transform_;
	transform_.setIdentity();
	// Move the tracking frame by tx along the X axis
	transform_.tx = tx;
	// Apply the transformation
	pose = Transform::inverse(transform_) * pose * transform_;
}

/**
 * This function closes the ZED camera, its callback (thread) and the GL viewer
 **/
void close() {
	quit = true;
	zed_callback.join();
	zed.disableTracking("./ZED_spatial_memory"); // Record an area file

	zed.close();
	viewer.exit();
}


cv::Mat slMat2cvMat(Mat& input) {
	// Mapping between MAT_TYPE and CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
	case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}

	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}


void runMCS(/*const InitParameters& init_parameters*/)
{
	// Create common directory to output collected data.
	std::string output_dir = "output/";
	std::string cam0_dir = output_dir + "cam0/"; // left cam directory.
	std::string cam1_dir = output_dir + "cam1/"; // right cam directory.
	std::string imu_dir = output_dir + "imu/";
	std::string cam_params_dir = output_dir + "camera_parameters/";

	fs::create_directory(output_dir);
	fs::create_directory(cam0_dir);
	fs::create_directory(cam1_dir);
	fs::create_directory(imu_dir);
	fs::create_directory(cam_params_dir);

	// reading camera params. 
	auto camera_params = zed.getCameraInformation().calibration_parameters_raw;

	unsigned int i = 0;
	char key = ' ';
	while (key != 'q')
	{
		// Capture images from both left and right cameras. 
		sl::Mat zed_image_left;
		sl::Mat zed_image_right;

		// retrieve images from zed camera. 
		zed.retrieveImage(zed_image_left, VIEW_LEFT);
		zed.retrieveImage(zed_image_right, VIEW_RIGHT);

		// convert images to opencv format. 
		cv::Mat ocv_left_image = slMat2cvMat(zed_image_left);
		cv::Mat ocv_right_image = slMat2cvMat(zed_image_right);

		// Read/Save imu raw data 
		if (zed.getCameraInformation().camera_model == MODEL_ZED_M)
		{
			zed.getIMUData(imu_data, sl::TIME_REFERENCE::TIME_REFERENCE_IMAGE);
			auto time_stamp = imu_data.timestamp;
			auto linear_acc = imu_data.linear_acceleration;
			auto angular_vel = imu_data.angular_velocity;

			// write imu data to disk. 
			std::cout << "angular velocity: " << angular_vel.x << ", " << angular_vel.y << ", " << angular_vel.z << '\n';
			std::cout << "Linear accelration: " << linear_acc.x << ", " << linear_acc.y << ", " << linear_acc.z << '\n';
		}

		// Write images to Disk. 
		cv::imshow("left", ocv_left_image);
		cv::imshow("right", ocv_right_image);
		cv::imwrite(cam0_dir + std::to_string(i) + "_left.jpg", ocv_left_image);
		cv::imwrite(cam1_dir + std::to_string(i) + "_right.jpg", ocv_right_image);
		++i;
		cv::waitKey(30);
	}

	// Write camera parameters data to Disk. 
	std::ofstream cam_params_left, cam_params_right;
	cam_params_left.open(cam_params_dir + "left.txt");
	cam_params_right.open(cam_params_dir + "right.txt");
	if (cam_params_left.is_open())
		cam_params_left << camera_params.left_cam.fx << ' ' << camera_params.left_cam.fy <<
		' ' << camera_params.left_cam.cx << ' ' << camera_params.left_cam.cy << '\n';
	if (cam_params_right.is_open())
		cam_params_left << camera_params.right_cam.fx << ' ' << camera_params.right_cam.fy <<
		' ' << camera_params.right_cam.cx << ' ' << camera_params.right_cam.cy << '\n';
}

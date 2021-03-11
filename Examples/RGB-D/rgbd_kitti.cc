//
// Created by nxy on 2021/3/10.
//

/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, const string &strPathToDepth, const string &strPathToDepthUncertainty,
	vector<string> &vstrImageFilenames, vector<string> &vstrDepthFilenames, vector<string> &vstrDepthUncertFilenames,
	vector<double> &vTimestamps);

int main(int argc, char **argv)
{
	if(argc != 6)
	{
		cerr << endl << "Usage: ./rgbd_kitti path_to_vocabulary path_to_settings path_to_sequence path_to_predicted_depth path_to_predicted_depth_uncertainty" << endl;
		return 1;
	}

	// Retrieve paths to images
	vector<string> vstrImageFilenamesRGB;
	vector<string> vstrImageFilenamesD;
	vector<string> vstrImageFilenamesDepthUncert;
	vector<double> vTimestamps;
	string strPathToSequence = string(argv[3]);
	string strPathToPredictedDepth = string(argv[4]);
	string strPathToPredictedDepthUncertainty = string(argv[5]);

	LoadImages(strPathToSequence, strPathToPredictedDepth, strPathToPredictedDepthUncertainty, vstrImageFilenamesRGB,
		vstrImageFilenamesD, vstrImageFilenamesDepthUncert, vTimestamps);

	// Check consistency in the number of images and depthmaps
	int nImages = vstrImageFilenamesRGB.size();
	if(vstrImageFilenamesRGB.empty())
	{
		cerr << endl << "No images found in provided path." << endl;
		return 1;
	}
	else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
	{
		cerr << endl << "Different number of images for rgb and depth." << endl;
		return 1;
	}

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;
	cout << "Images in the sequence: " << nImages << endl << endl;

	// Main loop
	cv::Mat imRGB, imD, imDU;
	for(int ni=0; ni<nImages; ni++)
	{
		// Read image and depthmap from file
		imRGB = cv::imread(string(argv[3])+vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED);
		imD = cv::imread(string(argv[4])+"/"+vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED);
		imDU = cv::imread(string(argv[5])+"/"+vstrImageFilenamesDepthUncert[ni], cv::IMREAD_UNCHANGED);
		if(ni == 0)
		{
			cout << "图片的路径是" << string(argv[3])+"/"+vstrImageFilenamesRGB[ni] << endl;
			cout << "深度图的路径是" << string(argv[4])+"/"+vstrImageFilenamesD[ni] << endl;
			cout << "深度不确定性的路径是" << string(argv[5])+"/"+vstrImageFilenamesDepthUncert[ni] << endl;
		}
		double tframe = vTimestamps[ni];

		if(imRGB.empty())
		{
			cerr << endl << "Failed to load image at: "
				 << string(argv[3])+vstrImageFilenamesRGB[ni] << endl;
			return 1;
		}

#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

		// Pass the image to the SLAM system
		SLAM.TrackRGBD(imRGB, imD, imDU, tframe);

#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

		double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		vTimesTrack[ni]=ttrack;

		// Wait to load the next frame
		double T=0;
		if(ni<nImages-1)
			T = vTimestamps[ni+1]-tframe;
		else if(ni>0)
			T = tframe-vTimestamps[ni-1];

		if(ttrack<T)
			usleep((T-ttrack)*1e6);
	}

	// Stop all threads
	SLAM.Shutdown();

	// Tracking time statistics
	sort(vTimesTrack.begin(),vTimesTrack.end());
	float totaltime = 0;
	for(int ni=0; ni<nImages; ni++)
	{
		totaltime+=vTimesTrack[ni];
	}
	cout << "-------" << endl << endl;
	cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
	cout << "mean tracking time: " << totaltime/nImages << endl;

	// Save camera trajectory
	SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

	return 0;
}


void LoadImages(const string &strPathToSequence, const string &strPathToDepth, const string &strPathToDepthUncertainty,
	vector<string> &vstrImageFilenames, vector<string> &vstrDepthFilenames, vector<string> &vstrDepthUncertFilenames,
	vector<double> &vTimestamps)
{
	ifstream fTimes;
	string strPathTimeFile = strPathToSequence + "/times.txt";
	fTimes.open(strPathTimeFile.c_str());
	while(!fTimes.eof())
	{
		string s;
		getline(fTimes,s);
		if(!s.empty())
		{
			stringstream ss;
			ss << s;
			double t;
			ss >> t;
			vTimestamps.push_back(t);
		}
	}

	string strPrefixLeft =  "/image_2/";

	const int nTimes = vTimestamps.size();
	vstrImageFilenames.resize(nTimes);
	vstrDepthFilenames.resize(nTimes);
	vstrDepthUncertFilenames.resize(nTimes);

	for(int i=0; i<nTimes; i++)
	{
		stringstream ss;
		ss << setfill('0') << setw(6) << i;
		vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
		vstrDepthFilenames[i] =  ss.str() + ".png";
		vstrDepthUncertFilenames[i] = ss.str() + ".png";
	}
	cout << "读取文件id完成，共有 " << vstrDepthFilenames.size() << "张图片" << endl;
}
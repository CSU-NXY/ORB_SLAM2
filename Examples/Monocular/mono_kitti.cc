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
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"
#include "Converter.h"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

void LoadGroundtruth(const string &strPathToGroundtruth, vector<vector<double>> &vdGroundtruth);
void LoadUncertainty(const string &strPathToUncertainty, vector<vector<double>> &vdUncertainty);

int main(int argc, char **argv)
{
    if(argc != 6)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence path_to_groundtruth path_to_uncertainty" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    // 加载groundtruth和不确定性，记录了上一帧到当前帧的位姿变换。第一帧的内容全为零
    cout << "开始加载GT和Uncertainty" << endl;
    vector<vector<double>> vvdGroundtruths;
    LoadGroundtruth(string(argv[4]), vvdGroundtruths);
    vector<vector<double>> vvdUncertainties;
    LoadUncertainty(string(argv[5]), vvdUncertainties);

    assert(vvdGroundtruths.size() == vvdUncertainties.size());
    assert(vvdGroundtruths.size() == vstrImageFilenames.size());

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];
        vector<double> vdGroundtruth = vvdGroundtruths[ni];
        vector<double> vdUncertainty = vvdUncertainties[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
		SLAM.TrackMonocular(im, tframe, vdGroundtruth, vdUncertainty);

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
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
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

    string strPrefixLeft = strPathToSequence + "/image_2/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}

void LoadGroundtruth(const string &strPathToGroundtruth, vector<vector<double>> &vdGroundtruth)
{
	vdGroundtruth.emplace_back(vector<double>(6,0));

	ifstream fGroundtruth;
	fGroundtruth.open(strPathToGroundtruth);

	while(!fGroundtruth.eof())
	{
		vector<double> v;

		string s;
		getline(fGroundtruth,s);

		stringstream ss(s);

		double x;
		while (ss >> x)
			v.push_back(x);
		vdGroundtruth.push_back(v);
	}

//	for (auto i : vdGroundtruth[1])
//	{
//		cout << i << endl;
//	}
}

void LoadUncertainty(const string &strPathToUncertainty, vector<vector<double>> &vdUncertainty)
{
	vdUncertainty.emplace_back(vector<double>(6,0));

	ifstream fUncertainty;
	fUncertainty.open(strPathToUncertainty);

	while(!fUncertainty.eof())
	{
		vector<double> v;

		string s;
		getline(fUncertainty,s);

		stringstream ss(s);

		double x;
		while (ss >> x)
			v.push_back(x);
		vdUncertainty.push_back(v);
	}
}
#ifndef GRID_HPP
#define GRID_HPP

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <random>
#include <float.h>

#include <bits/stdc++.h>

#include <geometry_msgs/Point.h>

using std::cout;
using std::endl;
using std::cin;
using std::vector;
using std::string;
using std::ofstream;
using std::ios;

const int NO_PARENT = -1;

class TrajPlanner {
    private:
        vector<vector<bool>> occupancy_grid;
        cv::Mat gridImage;
        cv::Mat occupancyGridImage;
        cv::Mat PRMImage;

        cv::Point2i initial_position;
        cv::Point2i end_position;

        int rows;
        int cols;
        int upscale_res = 2;
        int max_nodes = 1000;

        vector<geometry_msgs::Point> shortestPath;

        void readImage(string filePath);
        void fillOccupancyMatrix();
        void setOccupancy(int row, int col, bool occupied);
        bool getOccupancy(int row, int col);
        bool drawLine(cv::Point2i a, cv::Point2i b);
        bool Dijkstra(vector<vector<double>>  graph, int src, vector<int>& shortestPath);
        
    public:
        TrajPlanner();
        ~TrajPlanner();

        void createOccupancyGrid();
        void show(int delay);
        void infos();
        void inflateGrid(int inflateRadius);
        bool PRM(cv::Point startLoc, cv::Point endLoc, int numNodes, double conectionDistance);
        void genTrajectory(cv::Point startLoc, cv::Point endLoc, int numNodes, double conectionDistance);
        vector<geometry_msgs::Point> getShortestPath();
};

#endif

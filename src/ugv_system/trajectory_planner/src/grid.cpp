#include "grid.hpp"

TrajPlanner::TrajPlanner() {
    string filePath = "src/ugv_system/trajectory_planner/src/maps/laser-map.pgm";
    
    this->readImage(filePath);
    this->rows = gridImage.rows;
    this->cols = gridImage.cols;
    this->occupancy_grid = vector<vector <bool>>(this->rows, vector<bool>(this->cols));
    this->fillOccupancyMatrix();
    this->createOccupancyGrid();
}

void TrajPlanner::setOccupancy(int row, int col, bool occupied){
    this->occupancy_grid[row][col] = occupied;
}

bool TrajPlanner::getOccupancy(int row, int col) {
    return this->occupancy_grid[row][col];
}

void TrajPlanner::readImage(string filePath) {
    this->gridImage = cv::imread(filePath, cv::IMREAD_GRAYSCALE);

    if (gridImage.empty()) {
        cout << "Image File Not Found" << endl;

        cin.get();

        exit(1);
    }

    // int upscale_y = this->gridImage.rows*this->upscale_res;
    // int upscale_width = this->gridImage.cols*this->upscale_res;

    // cv::Mat resized_up;

    // cv::resize(this->gridImage, resized_up, cv::Size(upscale_width, upscale_y), cv::INTER_LINEAR);

    // this->gridImage = resized_up;

    cv::flip(this->gridImage, this->gridImage, 0);
}

// true - free | false - occupied
void TrajPlanner::fillOccupancyMatrix() {
    for (int col=0; col < this->cols; col++) {
        for (int row=0; row < this->rows; row++) {
            uchar pixel = this->gridImage.at<uchar>(row,col);
            // cout << (int)(pixel) << " ";
            this->setOccupancy(row, col, pixel >= 250);
        }
        // cout << endl;
    }
}

void TrajPlanner::createOccupancyGrid() {
    this->occupancyGridImage = cv::Mat::zeros(this->gridImage.size(), CV_8UC3);

    for (int col=0; col < this->cols; col++) {
        for (int row=0; row < this->rows; row++) {
            if (this->getOccupancy(row,col)) {
                this->occupancyGridImage.at<cv::Vec3b>(row,col) = cv::Vec3b(255,255,255);
            }
        }
    }
}

void TrajPlanner::show(int delay) {
    imshow("Map Image", this->gridImage);
    imshow("Occupancy TrajPlanner Image", this->occupancyGridImage);
    imshow("Traj planner image", this->PRMImage);
    cv::waitKey(delay);
}

void TrajPlanner::infos() {
    cout << "gridImage: " << this->rows << "x" << this->cols << endl;
    cout << "TrajPlanner:" << this->occupancy_grid.size() << "x" << this->occupancy_grid[0].size() << " | " << this->occupancyGridImage.type() << endl;
}

void TrajPlanner::inflateGrid(int inflateRadius) {
    for (int col=0; col < this->cols; col++) {
        for (int row=0; row < this->rows; row++) {
            if (!this->getOccupancy(row, col)) {
                cv::circle(this->occupancyGridImage, cv::Point(col,row), inflateRadius, cv::Scalar(0,0,0), cv::FILLED, cv::LINE_8);
            }
        }
    }

    for (int col=0; col < this->cols; col++) {
        for (int row=0; row < this->rows; row++) {
            cv::Vec3b pixel = this->occupancyGridImage.at<cv::Vec3b>(row,col);
            // cout << (int)(pixel) << " ";
            // cout << "[" << col << "," << row << " = " << (int)(pixel) << "]";
            bool isWhitePixel = pixel[0] > 250 && pixel[1] > 250 && pixel[2] > 250;
            this->setOccupancy(row, col, isWhitePixel);
        }
        // cout << endl;
    }
}

void TrajPlanner::genTrajectory(cv::Point startLoc, cv::Point endLoc, int numNodes, double conectionDistance) {
    for(int i = 10; i < 1000; i += 10) {
        cout << "num_nodes: " << i << endl;
        
        if (this->PRM(startLoc, endLoc, i, 100))
            break;
    }
}

bool TrajPlanner::PRM(cv::Point startLoc, cv::Point endLoc, int numNodes, double conectionDistance) {
    this->PRMImage = this->occupancyGridImage.clone();
    cv::circle(this->PRMImage, startLoc, 5, cv::Scalar(0,0,255), cv::FILLED, cv::LINE_8);
    cv::circle(this->PRMImage, endLoc, 5, cv::Scalar(255,0,255), cv::FILLED, cv::LINE_8);
    
    vector<cv::Point2i> random_locs;
    vector<vector<double>> distance_matrix = vector<vector<double>>(numNodes+2, vector<double>(numNodes+2, 0));
    this->shortestPath = vector<geometry_msgs::Point>(0);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> random_col(0, this->cols-1);
    std::uniform_int_distribution<> random_row(0, this->rows-1);

    // cout << "random nodes" << endl;
    random_locs.push_back(startLoc);

    int nodesCount = 0;
    while (nodesCount < numNodes) {
        int col = random_col(gen);
        int row = random_row(gen);

        if (this->getOccupancy(row, col)) {
            random_locs.push_back(cv::Point(col, row));
            nodesCount++;
        }
    }

    random_locs.push_back(endLoc);

    // cout << "draw nodes" << endl;

    for (int i=0; i<random_locs.size(); i++) {
        cv::Point2d p1 = random_locs[i];
        // cout << p1.x << "," << p1.y << " - " << int(this->getOccupancy(p1.y, p1.x)) << endl;
        cv::circle(this->PRMImage, p1, 3, cv::Scalar(255,0,0), cv::FILLED, cv::LINE_8);
        
        for (int j=0; j<random_locs.size(); j++) {
            cv::Point2d p2 = random_locs[j];
            double distance = cv::norm(p1-p2);

            if (distance <= conectionDistance) {
                bool colision = this->drawLine(p1, p2);
                if (!colision) {
                    distance_matrix[i][j] = distance;
                }
            }
        }
    }

    vector<int> indexPath;
    bool pathFound = this->Dijkstra(distance_matrix, 0, indexPath);

    

    if (pathFound) {
        geometry_msgs::Point point;
        point.x = random_locs[indexPath[0]].x;
        point.y = random_locs[indexPath[0]].y;
        shortestPath.push_back(point);

        std::cout << 0 << " -> " << indexPath[0] << " - " << random_locs[indexPath[0]].x << "," << random_locs[indexPath[0]].y << std::endl;
        
        for (int i=0; i<indexPath.size()-1; i++) {
            int index_pt1 = indexPath[i];
            int index_pt2 = indexPath[i+1];
            cv::line(this->PRMImage, random_locs[index_pt1], random_locs[index_pt2], cv::Scalar(0,0,255), 2);

            point.x = random_locs[index_pt2].x;
            point.y = random_locs[index_pt2].y;
            shortestPath.push_back(point);

            std::cout << i << " -> " << indexPath[i] << " - " << random_locs[indexPath[i]].x << "," << random_locs[indexPath[i]].y << "   |   ";
            std::cout << i+1 << " -> " << indexPath[i+1] << " - " << random_locs[indexPath[i+1]].x << "," << random_locs[indexPath[i+1]].y << std::endl;
        }
    }

    // this->show(1);

    return pathFound;
}

bool TrajPlanner::drawLine(cv::Point2i a, cv::Point2i b) {
    cv::Mat1b mask(this->PRMImage.size(), uchar(0));
    line(mask, a, b, cv::Scalar(255));

    vector<cv::Point> points;
    findNonZero(mask, points);

    bool colision = false;
    for (cv::Point p : points) {
        if (!this->getOccupancy(p.y, p.x)) {
            colision = true;
            break;
        }
    }

    if (!colision) {
        cv::line(this->PRMImage, a, b, cv::Scalar(255,255,0));
    }

    return colision;
}

vector<geometry_msgs::Point> TrajPlanner::getShortestPath() {
    return this->shortestPath;
}

TrajPlanner::~TrajPlanner() {
}

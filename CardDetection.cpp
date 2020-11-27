#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <stdio.h>
#include <opencv2/highgui.hpp>
#include <vector>
#include <string>
#include <math.h>

using namespace cv;
using namespace std;



int main(int argc, char* argv[]) {

    //Initialisation camera
    cv::VideoCapture camera(0);

    if (!camera.isOpened()) {
        std::cout << "Camera could not be opened." << std::endl;
    }

    cv::namedWindow("Working Frame", CV_WINDOW_AUTOSIZE);
    cv::Mat frame, working_frame, working_frame_old;
    cv::Size frame_size;
    cv::Size working_frame_size(640, 480);
    int k_frame = 0;
    int num_carte = 0;
    int old_num_carte = 0;

    //Chessboard
    int board_width = 5;
    int board_height = 3;
    int number_of_boards = board_width * board_height;
    cv::Size board_size(board_width, board_height);
    bool found_chessboard;

    //Symmetrical circle 
    Size patternsize(5, 3); //number of centers
    bool found_circle;

    //Detection carte;
    Point2f p0;
    Point2f p1;
    Point2f p2;
    Point2f p3;
    Point2f p4;

    //Calibrage 1
    int successes = 0;
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<cv::Point2f> corners_chessboard;
    std::vector<cv::Point2f> corners_circle;
    vector<Point3f> obj;
    for (int j = 0; j < number_of_boards; j++)
        obj.push_back(Point3f(j / board_width, j % board_width, 0.0f));

    Mat intrinsic = Mat(3, 3, CV_32FC1);
    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;

    //Type de carte
    int CARTE = 0;  // 0 = Aucune carte
                    // 1 = Type de carte 1
                    // 2 = Type de carte 2
                    // 3 = Type de carte 3

    while (successes < number_of_boards) {

        //clavier
        char c = cv::waitKey(2);

        //affichage camera
        camera >> frame;
        if (frame.empty()) {
            continue;
        }
        //Dimensionnement
        frame_size = frame.size();
        cv::resize(frame, working_frame, working_frame_size);

        //Detection
        found_chessboard = cv::findChessboardCorners(working_frame, board_size, corners_chessboard);
        cv::drawChessboardCorners(working_frame, board_size, corners_chessboard, found_chessboard);

        if (found_chessboard != 0) {
            found_circle = cv::findCirclesGrid(working_frame, patternsize, corners_circle, CALIB_CB_SYMMETRIC_GRID);
            cv::drawChessboardCorners(working_frame, patternsize, corners_circle, found_circle);

        }

        cv::imshow("Working Frame", working_frame);


        //Determination de la carte
        Mat frame_gray; Mat frame_seuillee;
        cvtColor(frame, frame_gray, CV_BGR2GRAY);
        threshold(frame_gray, frame_seuillee, 130, 255, CV_THRESH_BINARY);

        
        //Affichage des droites centrales    
        if ((found_chessboard != 0) && (found_circle != 0) && (k_frame % 1 == 0)) {
            
            old_num_carte = num_carte;

            float distance_1 = sqrt(pow((corners_chessboard[0].x - corners_circle[0].x), 2)
                + pow((corners_chessboard[0].y - corners_circle[0].y), 2));
            
            float distance_2 = sqrt(pow((corners_chessboard[0].x - corners_circle[14].x), 2)
                + pow((corners_chessboard[0].y - corners_circle[14].y), 2));
            

            float diff = abs(distance_2 - distance_1);
            bool droit = 0;

            if (diff >= distance_1 / 4) {
                droit = 1;
            }
            
            k_frame++;
            if (droit == 1) {
                line(working_frame, corners_chessboard[10], corners_circle[0], Scalar(0, 0, 0), 3);
                line(working_frame, corners_chessboard[11], corners_circle[1], Scalar(0, 0, 0), 3);
                line(working_frame, corners_chessboard[12], corners_circle[2], Scalar(0, 0, 0), 3);
                line(working_frame, corners_chessboard[13], corners_circle[3], Scalar(0, 0, 0), 3);
                line(working_frame, corners_chessboard[14], corners_circle[4], Scalar(0, 0, 0), 3);

                p0 = Point((corners_chessboard[10].x + corners_circle[0].x) / 2, (corners_chessboard[10].y + corners_circle[0].y) / 2);
                p1 = Point((corners_chessboard[11].x + corners_circle[1].x) / 2, (corners_chessboard[11].y + corners_circle[1].y) / 2);
                p2 = Point((corners_chessboard[12].x + corners_circle[2].x) / 2, (corners_chessboard[12].y + corners_circle[2].y) / 2);
                p3 = Point((corners_chessboard[13].x + corners_circle[3].x) / 2, (corners_chessboard[13].y + corners_circle[3].y) / 2);
                p4 = Point((corners_chessboard[14].x + corners_circle[4].x) / 2, (corners_chessboard[14].y + corners_circle[4].y) / 2);

                circle(working_frame, p0, 3, Scalar(255, 0, 0), 5);
                circle(working_frame, p1, 3, Scalar(255, 30, 0), 5);
                circle(working_frame, p2, 3, Scalar(255, 60, 0), 5);
                circle(working_frame, p3, 3, Scalar(255, 90, 0), 5);
                circle(working_frame, p4, 3, Scalar(255, 150, 0), 5);

                cv::imshow("Working Frame", working_frame);

           

                //Detection carte 
                if (frame_seuillee.at<bool>(p0) == 0){
                    num_carte = 1;
                }
                else if (frame_seuillee.at<bool>(p1) == 0) {
                    num_carte = 2;
                }
                else if (frame_seuillee.at<bool>(p2) == 0) {
                    num_carte = 3;
                }
                else if (frame_seuillee.at<bool>(p3) == 0) {
                    num_carte = 4;
                }
                else if (frame_seuillee.at<bool>(p4) == 0) {
                    num_carte = 5;
                }
                            

            }
            else {
                line(working_frame, corners_chessboard[4], corners_circle[0], Scalar(0, 0, 0), 3);
                line(working_frame, corners_chessboard[3], corners_circle[1], Scalar(0, 0, 0), 3);
                line(working_frame, corners_chessboard[2], corners_circle[2], Scalar(0, 0, 0), 3);
                line(working_frame, corners_chessboard[1], corners_circle[3], Scalar(0, 0, 0), 3);
                line(working_frame, corners_chessboard[0], corners_circle[4], Scalar(0, 0, 0), 3);

                p0 = Point((corners_chessboard[4].x + corners_circle[0].x) / 2, (corners_chessboard[4].y + corners_circle[0].y) / 2);
                p1 = Point((corners_chessboard[3].x + corners_circle[1].x) / 2, (corners_chessboard[3].y + corners_circle[1].y) / 2);
                p2 = Point((corners_chessboard[2].x + corners_circle[2].x) / 2, (corners_chessboard[2].y + corners_circle[2].y) / 2);
                p3 = Point((corners_chessboard[1].x + corners_circle[3].x) / 2, (corners_chessboard[1].y + corners_circle[3].y) / 2);
                p4 = Point((corners_chessboard[0].x + corners_circle[4].x) / 2, (corners_chessboard[0].y + corners_circle[4].y) / 2);


                circle(working_frame, p0, 3, Scalar(255, 0, 0), 5);
                circle(working_frame, p1, 3, Scalar(255, 30, 0), 5);
                circle(working_frame, p2, 3, Scalar(255, 60, 0), 5);
                circle(working_frame, p3, 3, Scalar(255, 90, 0), 5);
                circle(working_frame, p4, 3, Scalar(255, 150, 0), 5);

                cv::imshow("Working Frame", working_frame);


                //Detection carte 
                if (frame_seuillee.at<bool>(p0) == 0) {
                    num_carte = 1;
                }
                else if (frame_seuillee.at<bool>(p1) == 0) {
                    num_carte = 2;
                }
                else if (frame_seuillee.at<bool>(p2) == 0) {
                    num_carte = 3;
                }
                else if (frame_seuillee.at<bool>(p3) == 0) {
                    num_carte = 4;
                }
                else if (frame_seuillee.at<bool>(p4) == 0) {
                    num_carte = 5;
                }

            }

            if (num_carte != old_num_carte) {
                std::cout << "     " << std::endl;
                std::cout << "============================" << std::endl;
                std::cout << "carte " << num_carte << std::endl;
                std::cout << "     " << std::endl;
            }

        }


        //Calcul du centre
        if ((found_chessboard != 0) && (found_circle != 0)) {
            Point2f pt_central = Point((corners_chessboard[7].x + corners_circle[7].x) / 2, (corners_chessboard[7].y + corners_circle[7].y) / 2);
            circle(working_frame, pt_central, 3, Scalar(255, 255, 255), 5);
            cv::imshow("Working Frame", working_frame);
        }

        //bouton quitter
        if (c == 'q') {
            break;
        }

    }//fin while


    return 0;
}


#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <stdio.h>
#include <opencv2/highgui.hpp>


using namespace cv;
using namespace std;


int main(int argc, char* argv[]) {

    //Récupération de la caméra
    cv::VideoCapture camera(0);

    if (!camera.isOpened()) {
        std::cout << "Camera could not be opened." << std::endl;
    }
    
    //Initialisatio des données liées à la caméra et à son affichage
    cv::Mat frame, working_frame, working_frame_old;
    cv::Size frame_size;
    cv::Size working_frame_size(640, 480);

    cv::namedWindow("Working Frame", CV_WINDOW_AUTOSIZE);

    //Paramètres liés à l'échiquier à détecter (à modifier en fonction de l'échiquier)
    int board_width = 5;
    int board_height = 3;
    int number_of_boards = board_width * board_height;
    cv::Size board_size(board_width, board_height);


    //Initialisation des vecteurs de coordonées 3D et 2D
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<cv::Point2f> corners;

    //Calcul coordonnees 3D des sommets
    vector<Point3f> obj;
    for (int j = 0; j < number_of_boards; j++)
        obj.push_back(Point3f(j / board_width, j % board_width, 0.0f));
    
    //Initialisation des paramètres intrasèques et extrasèques
    Mat intrinsic = Mat(3, 3, CV_32FC1);
    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;
    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;

    //Booléens
    bool calibrage = false;

    //Compteur
    int k_frame = 0;

    //Autre vecteur contenant des coordonees 2D (celui des droites affichées)
    vector<Point2f> img2_points;

    while (1) {
        char c = cv::waitKey(2);
        char d = cv::waitKey(2);
        char t = cv::waitKey(2);

        //affichage camera
        camera >> frame;
        if (frame.empty()) {
            continue;
        }

        //on trouve les coordonnees pixels des sommets
        frame_size = frame.size();
        cv::resize(frame, working_frame, working_frame_size);
        bool found = cv::findChessboardCorners(working_frame, board_size, corners);

        //bouton quitter
        if (c == 'q') {
            break;
        }

        k_frame++;

        //calibrage  
        if ((found != 0)&&(c == 'd')) {

            //std::cout << "calibrage done";

            //on recupere les coordonnes pixels et reelles des sommets
            image_points.clear();
            object_points.clear();
            image_points.push_back(corners);
            object_points.push_back(obj);

            calibrateCamera(object_points, image_points, frame.size(), intrinsic, distCoeffs, rvecs, tvecs);
      

            calibrage = true;
        }

        if ((found != 0) && (calibrage == true)) {

            //Crétation des points 3D
            vector<Point3f> obj2;
            Point3f p1 = Point3f(0.0, 1.0, 2.0);
            Point3f p2 = Point3f(0.0, 3.0, 2.0);
            Point3f p3 = Point3f(2.0, 3.0, 2.0);
            Point3f p4 = Point3f(2.0, 1.0, 2.0);

            //Ajout des points au vecteur objet
            obj2.push_back(p1);
            obj2.push_back(p2);
            obj2.push_back(p3);
            obj2.push_back(p4);

            //Projection des points 3D en coord 2D
            projectPoints(obj2, rvecs[0], tvecs[0], intrinsic, distCoeffs, img2_points);
            line(working_frame, img2_points[0], img2_points[1], Scalar(255, 255, 0), 5);
            line(working_frame, img2_points[1], img2_points[2], Scalar(255, 0, 255), 5);
            line(working_frame, img2_points[2], img2_points[3], Scalar(0, 255, 255), 5);
            line(working_frame, img2_points[3], img2_points[0], Scalar(0, 0, 255), 5);
            line(working_frame, img2_points[0], corners[1], Scalar(255, 255, 0), 5);
            line(working_frame, img2_points[1], corners[3], Scalar(255, 0, 255), 5);
            line(working_frame, img2_points[2], corners[13], Scalar(0, 255, 255), 5);
            line(working_frame, img2_points[3], corners[11], Scalar(0, 0, 255), 5);
        }

       

        //cv::drawChessboardCorners(working_frame, board_size, corners, found);
        cv::imshow("Working Frame", working_frame);


    }






    return 0;
}
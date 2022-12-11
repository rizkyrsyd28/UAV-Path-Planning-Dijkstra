#include "../avod_graph/ADT_Graph.hpp"

// KONSTRUKTOR 
void createGRAF(GRAF * G, int Row, int Col){
    ROW_EFF(*G) = Row;
    COL_EFF(*G) = Col;

    for (int i = 0; i < ROW_EFF(*G); i++){
        for (int j = 0; j < COL_EFF(*G); j++){
            ELMT(*G, i, j) = 0;
        }
    }
}

void createList(LIST * DIST, LIST * VIS, LIST * NODE, int CAP){
    BUFFER(*DIST) = (ListType*) malloc (CAP*sizeof(ListType));
    NEFF(*DIST) = CAP;

    BUFFER(*VIS) = (ListType*) malloc (CAP*sizeof(ListType));
    NEFF(*VIS) = CAP;

    BUFFER(*NODE) = (ListType*) malloc (CAP*sizeof(ListType));
    NEFF(*NODE) = CAP;

    for(int i = 0; i < CAP; i++){
        ElmtF(*DIST,i) = INT_MAX;

    }
    for(int j = 0; j < CAP; j++){
        ElmtUI(*VIS,j) = 0;
    }
    for(int k = 0; k < CAP; k++){
        ElmtI(*NODE,k) = -1;
    }
}

void singleList(LIST * L, int CAP){
    BUFFER(*L) = (ListType*) malloc (CAP*sizeof(ListType));
    NEFF(*L) = CAP;
}

// DISPLAY
void displayGRAF(GRAF G){
    for (int i = 0; i < ROW_EFF(G); i++){
        for (int j = 0; j < COL_EFF(G); j++){
            if (ELMT(G,i,j) != 0)
                printf("%.0f ",ELMT(G, i, j));
            else
                printf("NaN ");
        }
        printf("\n");
    }    
}

// IMG PROC
std::vector<std::vector<cv::Point>> detectPointer(cv::Mat img){
    cv::Mat hsv, th1, th2, th;

    cv::cvtColor(img, hsv , cv::COLOR_BGR2HSV);


    cv::inRange(hsv, cv::Scalar(0,50,50), cv::Scalar(15,255,255), th1);	// Red+
	cv::inRange(hsv, cv::Scalar(165,50,50), cv::Scalar(180,255,255), th2); // Red-
	th = th1 + th2;

	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10,10));
	cv::erode(th, th, kernel);
	cv::dilate(th, th, kernel);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(th, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
	return contours;
}

std::vector<std::vector<cv::Point>> detectStart(cv::Mat img){
    cv::Mat hsv, th;

    cv::cvtColor(img, hsv , cv::COLOR_BGR2HSV);


    cv::inRange(hsv, cv::Scalar(100,50,50), cv::Scalar(130,255,255), th);

	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10,10));
	cv::erode(th, th, kernel);
	cv::dilate(th, th, kernel);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(th, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
	return contours;
}

std::vector<std::vector<cv::Point>> detectFinal(cv::Mat img){
    cv::Mat hsv, th;

    cv::cvtColor(img, hsv , cv::COLOR_BGR2HSV);


    cv::inRange(hsv, cv::Scalar(45,50,50), cv::Scalar(80,255,255), th);

	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10,10));
	cv::erode(th, th, kernel);
	cv::dilate(th, th, kernel);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(th, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
	return contours;
}

void drawRedEdge(std::vector<std::vector<cv::Point>> contours, cv::Mat *raw){
    for (size_t i = 0; i < contours.size(); i++){
        cv::Moments M = cv::moments(contours[i], true);
		cv::Point centroid = cv::Point(int(M.m10/M.m00), int(M.m01/M.m00));
		
		cv::circle(*raw, centroid, 2, cv::Scalar(0,0,255), cv::FILLED);

	    cv::drawContours(*raw, contours, i, cv::Scalar(0,0,0), 2);
        // printf("%zu - <%d, %d>\n", i, centroid.x, centroid.y);
    }
}

unsigned int canMove(cv::Mat mat, cv::Point P1, cv::Point P2){
    int X0, Xt, Y0, Yt; 

    if (fabs(P1.x - P2.x) < 10){
        X0 = Xt = P1.x; 
        if (P1.y > P2.y){
            Yt = P1.y;
            Y0 = P2.y;
        }
        else{
            Yt = P2.y;
            Y0 = P1.y;
        }
    }
    else if (fabs(P1.y - P2.y) < 10){
        Y0 = Yt = P1.y;
        if (P1.x > P2.x){
            Xt = P1.x;
            X0 = P2.x;
        }
        else{
            Xt = P2.x;
            X0 = P1.x;
        }
    }
    for (int i = Y0; i <= Yt; i++){
        for (int j = X0; j <= Xt; j++){
            // printf("(%d, %d) = %d\n", i, j, mat.at<uchar>(i,j));
            if ((int)mat.at<uchar>(i,j) < 50){
                return 0;
            }
        }
    }
    return 1;
}
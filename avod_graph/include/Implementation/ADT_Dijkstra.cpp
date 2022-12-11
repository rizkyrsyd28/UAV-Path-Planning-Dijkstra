#include "../avod_graph/ADT_Dijkstra.hpp"
// #include "ADT_Graph.cpp"

int getMinimum(LIST Dist, LIST Tset){
    int index, min; 

    min = fabs(INT_MAX);

    // printf("[");
    // for (int i=0; i < NEFF(Dist); i++){
    //     printf("%.0f, ", ElmtF(Dist, i));
    // } printf("]\n");

    // printf("[");
    // for (int i=0; i < NEFF(Tset); i++){
    //     printf("%u, ", ElmtUI(Tset, i));
    // } printf("]\n");

    for (int i = 0; i < NEFF(Dist); i++){
        // printf("%f - %u\n", ElmtF(Dist,i), ElmtUI(Tset,i));
        if (ElmtUI(Tset,i) == 0 && ElmtF(Dist, i) < min){
            min = ElmtF(Dist,i);
            index = i;
        }
    }
    printf("%d\n", index);
    return index;
}
// Dijkstra Algorithm modified from https://www.educative.io/answers/how-to-implement-dijkstras-algorithm-in-cpp

LIST Dijkstra(GRAF G, int Src){
    LIST Dist, Tset, Node;

    createList(&Dist, &Tset, &Node, ROW_EFF(G));

    ElmtF(Dist,0) = 0; 

    for(int j = 0; j < ROW_EFF(G); j++){
        int m = getMinimum(Dist, Tset);
        ElmtUI(Tset, m) = 1;

        // printf(">>[");
        // for (int i=0; i < NEFF(Tset); i++){
        //     printf("%u, ", ElmtUI(Tset, i));
        // } printf("]\n");

        // printf("<<%d>>\n", j);
        for (int j = 0; j < ROW_EFF(G); j++){
            // printf("\t<<%d>> - ", j); printf(!ElmtUI(Tset,j) && (ELMT(G,m,j) != 0.0) && ElmtF(Dist,m) != INT_MAX && ElmtF(Dist, m) + ELMT(G,m,j) < ElmtF(Dist,j) ? "true\n" : "false\n");
            if (!ElmtUI(Tset,j) && (ELMT(G,m,j) != 0.0) && ElmtF(Dist,m) != INT_MAX && ElmtF(Dist, m) + ELMT(G,m,j) < ElmtF(Dist,j)){
                ElmtI(Node, j) = m;
                ElmtF(Dist, j) = ElmtF(Dist, m) + ELMT(G,m,j);
            }
        }
    }

    return Node;
}


std::vector<Coordinat> solutionNode(LIST RES, LIST POS){
    std::vector<Coordinat> node;
    Coordinat temp, offset;
    int currentNode = NEFF(RES)-1, counter = 1;
    
    temp.x = (float) ElmtPoint(POS,0).x;
    temp.y = (float) ElmtPoint(POS,0).y;

    offset = temp;

    temp.x = (float) ElmtPoint(POS,currentNode).x;
    temp.y = (float) ElmtPoint(POS,currentNode).y;

    temp.x = temp.x - offset.x;
    temp.y = temp.y - offset.y;

    node.push_back(temp);

    while (ElmtI(RES,currentNode) != -1){
        currentNode = ElmtI(RES, currentNode);

        temp.x = (float) ElmtPoint(POS, currentNode).x;
        temp.y = (float) ElmtPoint(POS, currentNode).y;

        temp.x = temp.x - offset.x;
        temp.y = temp.y - offset.y;

        node.insert(node.begin(), temp);
        counter++;
    }
    
    // for (int i = 0; i < counter; i++){
    //     printf("(%f, %f)", node[i].x, node[i].y);
    // }

    return node;
}

std::vector<Coordinat> Mapping(){
    cv::Mat img, raw, bnw;

    LIST POS, RES;
    GRAF G;

    raw = cv::imread("/home/rasyid/Downloads/path2.png");
    cv::resize(raw, raw, cv::Size(750,750));
    img = raw;

    cv::cvtColor(raw, bnw , cv::COLOR_BGR2GRAY);

	std::vector<std::vector<cv::Point>> contours, contours1, contours2;
	contours = detectPointer(raw);

    int cap = contours.size();

    singleList(&POS, cap+2);
    createGRAF(&G, cap+2, cap+2);

    drawRedEdge(contours, &img);

    for (size_t i = 0; i < contours.size(); i++){

        cv::Moments Mi = cv::moments(contours[i], true);
		cv::Point Ci = cv::Point(int(Mi.m10/Mi.m00), int(Mi.m01/Mi.m00));

        ElmtPoint(POS,(int)i+1) = Ci;

        for (size_t j = 0; j < contours.size(); j++){
            cv::Moments Mj = cv::moments(contours[j], true);
		    cv::Point Cj = cv::Point(int(Mj.m10/Mj.m00), int(Mj.m01/Mj.m00));
            

            if (contours[i] != contours[j] && canMove(bnw, Ci, Cj) && (fabs(Cj.x - Ci.x) < 60 || fabs(Cj.y - Ci.y) < 60)){
                cv::line(img, Ci, Cj, cv::Scalar(200,200,0), 2, cv::LINE_8);
                ELMT(G,i+1,j+1) = sqrt(pow(Ci.x-Cj.x, 2) + pow(Ci.y-Cj.y,2));
            }
        }
        std::string text = std::to_string(i+1); 
        cv::putText(img, text, Ci,cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255,255,255), 1, cv:: LINE_AA); 
    }


    contours1 = detectStart(raw);
    drawRedEdge(contours1, &raw);

    for (size_t i = 0; i < contours1.size(); i++){

        cv::Moments Mi = cv::moments(contours1[i], true);
		cv::Point Ci = cv::Point(int(Mi.m10/Mi.m00), int(Mi.m01/Mi.m00));
        // printf("%zu - <%d, %d>\n", i, Ci.y, Ci.x);
        ElmtPoint(POS,0) = Ci;

        for (size_t j = 0; j < contours.size(); j++){
            cv::Moments Mj = cv::moments(contours[j], true);
		    cv::Point Cj = cv::Point(int(Mj.m10/Mj.m00), int(Mj.m01/Mj.m00));
            

            if (contours[i] != contours[j] && canMove(bnw, Ci, Cj) && (fabs(Cj.x - Ci.x) < 60 || fabs(Cj.y - Ci.y) < 60)){
                cv::line(img, Ci, Cj, cv::Scalar(200,200,0), 2, cv::LINE_8);
                ELMT(G,i,j+1) = sqrt(pow(Ci.x-Cj.x, 2) + pow(Ci.y-Cj.y,2));
                ELMT(G,j+1,i) = sqrt(pow(Ci.x-Cj.x, 2) + pow(Ci.y-Cj.y,2));

            }
        }
        std::string text = std::to_string(i); 
        cv::putText(img, text, Ci,cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255,255,255), 1, cv:: LINE_AA); 
        
        // ElmtPoint(POS,(int)i) = Ci;
    }
    
    contours2 = detectFinal(raw);
    drawRedEdge(contours2, &raw);

    for (size_t i = 0; i < contours2.size(); i++){

        cv::Moments Mi = cv::moments(contours2[i], true);
		cv::Point Ci = cv::Point(int(Mi.m10/Mi.m00), int(Mi.m01/Mi.m00));
        // printf("%zu - <%d, %d>\n", i, Ci.x, Ci.y);
        ElmtPoint(POS,NEFF(POS)-1) = Ci;
        for (size_t j = 0; j < contours.size(); j++){
            cv::Moments Mj = cv::moments(contours[j], true);
		    cv::Point Cj = cv::Point(int(Mj.m10/Mj.m00), int(Mj.m01/Mj.m00));
            

            if (contours[i] != contours[j] && canMove(bnw, Ci, Cj) && (fabs(Cj.x - Ci.x) < 60 || fabs(Cj.y - Ci.y) < 60)){
                cv::line(img, Ci, Cj, cv::Scalar(200,200,0), 2, cv::LINE_8);
                ELMT(G,ROW_EFF(G)-1,j+1) = sqrt(pow(Ci.x-Cj.x, 2) + pow(Ci.y-Cj.y,2));
                ELMT(G,j+1,ROW_EFF(G)-1) = sqrt(pow(Ci.x-Cj.x, 2) + pow(Ci.y-Cj.y,2));

            }
        }
        std::string text = std::to_string(ROW_EFF(G)-1); 
        cv::putText(img, text, Ci,cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255,255,255), 1, cv:: LINE_AA); 
        
        // ElmtPoint(POS,(int)i) = Ci;
    }

    cv::circle(img, cv::Point(375, 80), 3, cv::Scalar(0,255,255), cv::FILLED);

    displayGRAF(G);

    RES = Dijkstra(G,0);



    // * WayPoint = solutionNode(RES, POS);

    return solutionNode(RES, POS);
    // cv::imshow("bnw", bnw);
}

#include <iostream>
#include <string.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <vector>

// DEFINE
#define ROW_CAP 100
#define COL_CAP 100

// STRUKTUR DATA
typedef union {float f; unsigned int ui; int in; cv::Point pt;} ListType;

typedef struct {
    ListType * BUFF;
    int N_Buff;
} LIST;

typedef struct {
    float GRAF[ROW_CAP][COL_CAP];
    int Col_Eff;
    int Row_Eff;
} GRAF;

typedef struct {
    float x;
    float y;
} Coordinat;


/* SELEKTOR */
#define ROW_EFF(G) (G).Row_Eff
#define COL_EFF(G) (G).Col_Eff
#define ELMT(G,i,j) (G).GRAF[(i)][(j)]

#define BUFFER(L) (L).BUFF

#define ElmtUndef(L,i) (L).BUFF[(i)]
#define ElmtF(L,i) (L).BUFF[(i)].f
#define ElmtUI(L,i) (L).BUFF[(i)].ui
#define ElmtI(L,i) (L).BUFF[(i)].in
#define ElmtPoint(L,i) (L).BUFF[(i)].pt

#define NEFF(L) (L).N_Buff

// KONSTRUKTOR 
void createGRAF(GRAF * G, int Row, int Col);

void createList(LIST * DIST, LIST * VIS, LIST * NODE, int CAP);

void singleList(LIST * L, int CAP);

// DISPLAY
void displayGRAF(GRAF G);

// IMG PROC
std::vector<std::vector<cv::Point>> detectPointer(cv::Mat img);

std::vector<std::vector<cv::Point>> detectStart(cv::Mat img);

std::vector<std::vector<cv::Point>> detectFinal(cv::Mat img);

void drawRedEdge(std::vector<std::vector<cv::Point>> contours, cv::Mat *raw);

unsigned int canMove(cv::Mat mat, cv::Point P1, cv::Point P2);
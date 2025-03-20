#include <windows.h>
#include <iostream>
#include <iomanip>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

#define CANVAS_WIDTH 600
#define CANVAS_HEIGHT 960
#define CLEARANCE 30 //3mm clearence from the pen touching the edge

#define IMAGE_PATH "C:\\Users\\dbintley\\OneDrive - University of Plymouth\\Year 2\\ROCO224\\Image Code\\img6.png"
#define THRESHOLD 127 //sets greyscale -> binary threshold

#define PEN_SIZE 22        //true pen size
#define PEN_PATH_SIZE 16   //size the pathing code thinks the pen is
#define FILL_OVERLAP 10    //how much the fill overlaps with the outline
#define SMALLEST_FEATURE 5 //smallest feature that wont be ignored
#define SMALLEST_PATH 5    //paths smaller than this will be ignored
#define SMALLEST_TRAVEL 20 //stops the pen from lifting for very small hops
#define PATH_SIMPLIFIFY 2.0//higher values will reduce path size at the expence of accuarcy
#define DRAW_SUBLINES true //try to draw things smaller than the pen



using namespace std;
using namespace cv;

Point neighbours[8] = {Point(-1,-1),Point(-1,0),Point(-1,1),Point(0,1),Point(1,1),Point(1,0),Point(1,-1),Point(0,-1)};

enum moveType{draw, travel};
struct moveInstruction
{
    Point to;
    moveType type = draw;
    int speed;
};

cv::Mat resizeWithPadding(const cv::Mat& inputImage, int targetWidth, int targetHeight, int padding = 0);
cv::Mat skeletonize(Mat img);
void drawMasks(Mat fillMask, Mat strokesMask, Mat outlineMask);
vector<vector<Point>> findPaths(Mat img);
vector<moveInstruction> joinPaths(vector<vector<Point>> pathList);
double distanceFromPointToLine(const cv::Point& pointA, const cv::Point& pointB, const cv::Point& pointC);

int main()
{

    //read image and threshold
    Mat canvas = imread(IMAGE_PATH, IMREAD_GRAYSCALE);
    canvas = resizeWithPadding(canvas, CANVAS_WIDTH, CANVAS_HEIGHT, CLEARANCE);
    inRange(canvas, 0, THRESHOLD, canvas);
//    cv::erode(canvas, canvas, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));
//    cv::dilate(canvas, canvas, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));


//    while(1)
//    {
//        imshow("canvas",canvas);
//        waitKey(10);
//    }

    //===================================================image segmentation===============================================================
    //classify fills
    Mat fillMask;
    cv::erode(canvas, fillMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(PEN_SIZE*2-FILL_OVERLAP, PEN_SIZE*2-FILL_OVERLAP)));
    for(int y=0; y<fillMask.size().height; ++y)
        for(int x=0; x<fillMask.size().width; ++x)
            if(fillMask.at<uchar>(y,x) && (x+y)%static_cast<int>(PEN_PATH_SIZE*M_SQRT2)!=0)
                fillMask.at<uchar>(y,x)=0;

    //classify outlines
    Mat outlineMask, outlineMaskInner;
    cv::erode(canvas, outlineMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(PEN_PATH_SIZE, PEN_PATH_SIZE)));
    cv::erode(outlineMask, outlineMaskInner, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    bitwise_not(outlineMaskInner,outlineMaskInner);
    bitwise_and(outlineMask, outlineMaskInner, outlineMask);
    outlineMask = skeletonize(outlineMask);

    //classify strokes
    Mat strokesMask, dilatedOutline, dilatedFill, strokeSkeleton;
    if(DRAW_SUBLINES)
    {
        cv::dilate(outlineMask, dilatedOutline, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(PEN_SIZE, PEN_SIZE)));
        bitwise_not(dilatedOutline,dilatedOutline);
        cv::dilate(fillMask, dilatedFill, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(PEN_SIZE, PEN_SIZE)));
        bitwise_not(dilatedFill,dilatedFill);
        bitwise_and(canvas, dilatedFill, strokesMask);
        bitwise_and(strokesMask, dilatedOutline, strokesMask);
        cv::erode(strokesMask, strokesMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(SMALLEST_FEATURE, SMALLEST_FEATURE)));
        strokesMask = skeletonize(strokesMask);
    }
    else
    {
        strokesMask = Mat(canvas.size(), CV_8U, Scalar(0));
    }


    cout<<"Classifying regions. press SPACE to continue..."<<endl;
    while(waitKey(10)!=' ')
        drawMasks(fillMask, strokesMask, outlineMask);

    //======================================================Finding Paths=================================================================
//    vector<vector<Point>> fillPaths, outlinePaths, strokePaths;
//    fillPaths = findPaths(fillMask);
//    outlinePaths = findPaths(outlineMask);
//    strokePaths = findPaths(strokesMask);
//    cout<<"Number of fill paths: "   <<fillPaths.size()<<endl;
//    cout<<"Number of outline paths: "<<outlinePaths.size()<<endl;
//    cout<<"Number of stroke paths: " <<strokePaths.size()<<endl;

//    //join paths based on whats closest
//    vector<moveInstruction> fillPath = joinPaths(fillPaths);
//    vector<moveInstruction> outlinePath = joinPaths(outlinePaths);
//    vector<moveInstruction> strokePath = joinPaths(strokePaths);
//    vector<moveInstruction> drawPath;
//    drawPath.insert(drawPath.end(), fillPath.begin(), fillPath.end());
//    drawPath.insert(drawPath.end(), strokePath.begin(), strokePath.end());
//    drawPath.insert(drawPath.end(), outlinePath.begin(), outlinePath.end());

    Mat allMasks;
    bitwise_or(fillMask, strokesMask, allMasks);
    bitwise_or(allMasks, outlineMask, allMasks);
    vector<vector<Point>> allPaths = findPaths(allMasks);
    vector<moveInstruction> drawPath = joinPaths(allPaths);

    //filter small travels
    int travelsDeleted=0;
    for(int n=1; n<drawPath.size(); ++n)
    {
        if(drawPath[n].type==travel)
        {
            int dist = (drawPath[n-1].to.x-drawPath[n].to.x)*(drawPath[n-1].to.x-drawPath[n].to.x) + (drawPath[n-1].to.y-drawPath[n].to.y)*(drawPath[n-1].to.y-drawPath[n].to.y);
            if(dist<(SMALLEST_TRAVEL*SMALLEST_TRAVEL))
            {
                drawPath.erase(drawPath.begin()+n);
                --n;
                ++travelsDeleted;
            }
        }
    }
    cout<<"Small Travels Deleted: "<<travelsDeleted<<endl;

    //filter draws with the same position as a neighbour
    int zeroLengthDrawsDeleted=0;
    for(int n=1; n<drawPath.size(); ++n)
    {
        if(drawPath[n].type==draw)
        {
            if(drawPath[n].to == drawPath[n-1].to)
            {
                drawPath.erase(drawPath.begin()+n);
                --n;
                ++zeroLengthDrawsDeleted;
            }
        }
    }
    cout<<"Zero Length Draws Deleted: "<<zeroLengthDrawsDeleted<<endl;

    //simplify path
    cout<<"simplifying path"<<endl;
    cout<<"Path nodes: "<<drawPath.size()<<endl;
    bool endLoop=false;
    while(!endLoop)
    {
        endLoop=true;
        for(int n=1; n<(drawPath.size()-1); ++n)
        {
            //skip these as to not delete a travel instruction
            if(drawPath[n].type==travel || drawPath[n+1].type==travel)
                continue;

            //find how far the middle point is off the line of the other two points
            double error = distanceFromPointToLine(drawPath[n-1].to, drawPath[n+1].to, drawPath[n].to);

            //if the error is low enough, the middle point can be deleted
            if(error<PATH_SIMPLIFIFY)
            {
                drawPath.erase(drawPath.begin()+n);
                endLoop=false;
            }

        }
        cout<<"Path nodes: "<<drawPath.size()<<endl;
    }


    //draw path
    cout<<"Final Path. press SPACE to continue..."<<endl;
    Mat frame(Size(CANVAS_WIDTH, CANVAS_HEIGHT), CV_8UC3, Scalar(0,0,0));
    Point penPos(0,0);
    for(auto i : drawPath)
    {
        if(i.type==draw)
            line(frame, penPos, i.to, Scalar(255,255,255), PEN_SIZE);

        penPos=i.to;
    }

    while(waitKey(10)!=' ')
        imshow("View", frame);

    cout<<"\t{"+to_string(drawPath.front().to.x*10)+","+to_string(drawPath.front().to.y*10)+",0},"<<endl;
    int count = 0;
    for(auto i:drawPath)
    {
        count++;
        if(i.type==draw)
        {
            cout<<"\t{"+to_string(i.to.x*10)+","+to_string(i.to.y*10)+",90},"<<endl;
        }
        else
        {
            cout<<"\t{"+to_string(i.to.x*10)+","+to_string(i.to.y*10)+",0},"<<endl;
            cout<<"\t{"+to_string(i.to.x*10)+","+to_string(i.to.y*10)+",0},"<<endl;
            cout<<"\t{"+to_string(i.to.x*10)+","+to_string(i.to.y*10)+",90},"<<endl;
            count += 2;
        }
    }

    cout<<"\t{"+to_string(drawPath.back().to.x*10)+","+to_string(drawPath.back().to.y*10)+",0},"<<endl;
    cout<<"\tThe number of elements = "+to_string(count+2)<<endl;
    return 0;
}

double distanceFromPointToLine(const cv::Point& pointA, const cv::Point& pointB, const cv::Point& pointC) {
    int x1 = pointA.x, y1 = pointA.y;
    int x2 = pointB.x, y2 = pointB.y;
    int x0 = pointC.x, y0 = pointC.y;

    double numerator = std::abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1);
    double denominator = std::sqrt(std::pow(y2 - y1, 2) + std::pow(x2 - x1, 2));

    if (denominator == 0) {
        // The points A and B are the same, so just return the distance between A (or B) and C
        return std::sqrt(std::pow(x0 - x1, 2) + std::pow(y0 - y1, 2));
    }

    return numerator / denominator;
}

vector<moveInstruction> joinPaths(vector<vector<Point>> pathList)
{
    //if there are no paths, return an empty vector
    vector<moveInstruction> path;
    if(pathList.empty())
        return path;

    //add move instruction to the first path, then add the path
    path.push_back(moveInstruction{pathList.front().front(), moveType::travel,-1});
    for(auto p:pathList.front())
        path.push_back(moveInstruction{p, moveType::draw,-1});
    pathList.erase(pathList.begin());

    //while there are still paths, keep connecting paths
    while(pathList.size()>0)
    {
        //seach all path beginings and ends to find the closest
        Point lastPathEnd = path.back().to;
        bool reverseNeeded=false;
        int smallestDistance=100000000;
        int closestPathindex=0;
        for(int n=0; n<pathList.size(); ++n)
        {
            //check front
            Point pathStart = pathList[n].front();
            int dist = (pathStart.x-lastPathEnd.x)*(pathStart.x-lastPathEnd.x) + (pathStart.y-lastPathEnd.y)*(pathStart.y-lastPathEnd.y);
            if(dist<smallestDistance)
            {
                smallestDistance=dist;
                reverseNeeded=false;
                closestPathindex=n;
            }

            //check back
            Point pathEnd = pathList[n].back();
            dist = (pathEnd.x-lastPathEnd.x)*(pathEnd.x-lastPathEnd.x) + (pathEnd.y-lastPathEnd.y)*(pathEnd.y-lastPathEnd.y);
            if(dist<smallestDistance)
            {
                smallestDistance=dist;
                reverseNeeded=true;
                closestPathindex=n;
            }
        }

        //add this closest path to the full path
        if(reverseNeeded)
            reverse(pathList[closestPathindex].begin(), pathList[closestPathindex].end());

        path.push_back(moveInstruction{pathList[closestPathindex].front(), moveType::travel,-1});
        for(auto p:pathList[closestPathindex])
            path.push_back(moveInstruction{p, moveType::draw,-1});
        pathList.erase(pathList.begin()+closestPathindex);
    }
    return path;
}

vector<vector<Point>> findPaths(Mat img)
{
    cout<<"Tracing Paths..."<<endl;
    vector<vector<Point>> paths;

    while(1)
    {


        //find a path start closest to the last endPoint
        Point newPathStart(-1,-1);
        for(int y=1; y<(img.size().height-1); ++y)
        {
            for(int x=1; x<(img.size().width-1); ++x)
            {
                //find a path point
                if(img.at<uchar>(y,x))
                {
                    //if there is only 1 neighbour, this is an endpoint
                    int neighbourCount=0;
                    for(auto n : neighbours)
                        if(img.at<uchar>(Point(x,y)+n))
                            neighbourCount++;

                    //if point is an endpoint, check if its the closest
                    if(neighbourCount<=1)
                    {
                        newPathStart=Point(x,y);
                        break;
                    }
                }
            }

            //if a path start has been found, exit
            if(newPathStart!=Point(-1,-1))
                break;
        }

        //if a path start hasnt been found, look for any pixel, as there may be loops
        if(newPathStart==Point(-1,-1))
            for(int y=1; y<(img.size().height-1); ++y)
                for(int x=1; x<(img.size().width-1); ++x)
                    if(img.at<uchar>(y,x))
                        newPathStart=Point(x,y);

        //if still no path start can be found, there are no paths left
        if(newPathStart==Point(-1,-1))
            break;

        //step along the path untill an end point is found
        int dir=-1;
        vector<Point> newPath;
        newPath.push_back(newPathStart);
        img.at<uchar>(newPathStart)=0;

        while(1)
        {
            Point nextPos(-1,-1);

            //if there isnt a known direction... find one
            if(dir==-1)
            {
                for(int n=0; n<8; ++n)
                {
                    if(img.at<uchar>(newPath.back()+neighbours[n]))
                    {
                        dir=n;
                        nextPos = newPath.back()+neighbours[n];
                    }
                }
            }
            //otherwise, try to follow the same direction
            else
            {
                //try stright forward
                if(img.at<uchar>(newPath.back()+neighbours[dir]))
                {
                    nextPos = newPath.back()+neighbours[dir];
                }
                //try slightly right
                else if(img.at<uchar>(newPath.back()+neighbours[(8+dir+1)%8]))
                {
                    nextPos = newPath.back()+neighbours[(8+dir+1)%8];
                    dir=(8+dir+1)%8;
                }
                //try slightly left
                else if(img.at<uchar>(newPath.back()+neighbours[(8+dir-1)%8]))
                {
                    nextPos = newPath.back()+neighbours[(8+dir-1)%8];
                    dir=(8+dir-1)%8;
                }
                //try hard right
                else if(img.at<uchar>(newPath.back()+neighbours[(8+dir+2)%8]))
                {
                    nextPos = newPath.back()+neighbours[(8+dir+2)%8];
                    dir=(8+dir+2)%8;
                }
                //try hard left
                else if(img.at<uchar>(newPath.back()+neighbours[(8+dir-2)%8]))
                {
                    nextPos = newPath.back()+neighbours[(8+dir-2)%8];
                    dir=(8+dir-2)%8;
                }
            }

            //if no next point was found, the path is over
            if(nextPos==Point(-1,-1))
                break;

            //otherwise, set this point to 0 so its not added again, and add it to the path
            img.at<uchar>(nextPos)=0;
            newPath.push_back(nextPos);
        }
        //add new path to the list
        paths.push_back(newPath);
    }

    //smallest path filtering
    for(int n=0; n<paths.size(); ++n)
    {
        if(paths[n].size()<SMALLEST_PATH)
        {
            paths.erase(paths.begin() + n);
            --n;
        }
    }

    return paths;
}


void drawMasks(Mat fillMask, Mat strokesMask, Mat outlineMask)
{
    Mat fillDilated, strokeDilated, outlineDilated;
    cv::dilate(fillMask, fillDilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(PEN_SIZE, PEN_SIZE)));
    cv::dilate(strokesMask, strokeDilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(PEN_SIZE, PEN_SIZE)));
    cv::dilate(outlineMask, outlineDilated, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(PEN_SIZE, PEN_SIZE)));

    Mat frame(Size(CANVAS_WIDTH, CANVAS_HEIGHT), CV_8UC3, Scalar(0,0,0));

    frame.setTo(Scalar(100,100,100), fillDilated);
    frame.setTo(Scalar(100,100,100), strokeDilated);
    frame.setTo(Scalar(100,100,100), outlineDilated);

    frame.setTo(Scalar(255,0,0), fillMask);
    frame.setTo(Scalar(0,255,0), strokesMask);
    frame.setTo(Scalar(0,0,255), outlineMask);
    imshow("View", frame);

}


cv::Mat skeletonize(cv::Mat img) {


    int dir = 0;
    bool done;
    int dirCompleted=0;
    do
    {


        //add pixels that are part of a line to the skeleton
        Mat newImg;
        img.copyTo(newImg);
        done=true;
        for(int y=1; y<(img.size().height-1); ++y)
        {
            for(int x=1; x<(img.size().width-1); ++x)
            {
                if(img.at<uchar>(y,x))
                {
                    if(dir==0 && img.at<uchar>(y,x-1)==0 && img.at<uchar>(y-1,x-1)==0 && img.at<uchar>(y+1,x-1)==0 && img.at<uchar>(y,x+1)!=0 && (img.at<uchar>(y-1,x) || img.at<uchar>(y+1,x)))
                    {
                        newImg.at<uchar>(y,x) = 0;
                        done=false;
                    }
                    else if(dir==1 && img.at<uchar>(y+1,x)==0 && img.at<uchar>(y+1,x-1)==0 && img.at<uchar>(y+1,x+1)==0 && img.at<uchar>(y-1,x)!=0 && (img.at<uchar>(y,x-1) || img.at<uchar>(y,x+1)))
                    {
                        newImg.at<uchar>(y,x) = 0;
                        done=false;
                    }
                    else if(dir==2 && img.at<uchar>(y,x+1)==0 && img.at<uchar>(y-1,x+1)==0 && img.at<uchar>(y+1,x+1)==0 && img.at<uchar>(y,x-1)!=0 && (img.at<uchar>(y-1,x) || img.at<uchar>(y+1,x)))
                    {
                        newImg.at<uchar>(y,x) = 0;
                        done=false;
                    }
                    else if(dir==3 && img.at<uchar>(y-1,x)==0 && img.at<uchar>(y-1,x-1)==0 && img.at<uchar>(y-1,x+1)==0 && img.at<uchar>(y+1,x)!=0 && (img.at<uchar>(y,x-1) || img.at<uchar>(y,x+1)))
                    {
                        newImg.at<uchar>(y,x) = 0;
                        done=false;
                    }
                }
            }
        }

        newImg.copyTo(img);

        //change direction
        ++dir;
        if(dir>3)
            dir=0;

        if(done)
            ++dirCompleted;
        else
            dirCompleted=0;


    }while(dirCompleted<4);

    return img;
}

cv::Mat resizeWithPadding(const cv::Mat& inputImage, int targetWidth, int targetHeight, int padding) {
    // Subtract padding from the target dimensions
    int effectiveWidth = targetWidth - 2 * padding;
    int effectiveHeight = targetHeight - 2 * padding;

    // Compute aspect ratios
    double originalAspect = (double)inputImage.cols / inputImage.rows;
    double targetAspect = (double)effectiveWidth / effectiveHeight;

    int newWidth, newHeight;

    // Determine new dimensions
    if (originalAspect > targetAspect) {
        newWidth = effectiveWidth;
        newHeight = (int)(effectiveWidth / originalAspect);
    } else {
        newHeight = effectiveHeight;
        newWidth = (int)(effectiveHeight * originalAspect);
    }

    // Resize the input image
    cv::Mat resizedImage;
    cv::resize(inputImage, resizedImage, cv::Size(newWidth, newHeight));

    // Compute top-left point to place the resized image on the center of blank canvas
    int x = (targetWidth - newWidth) / 2;
    int y = (targetHeight - newHeight) / 2;

    // Create a blank white image
    cv::Mat outputImage(targetHeight, targetWidth, inputImage.type(), cv::Scalar(255, 255, 255));

    // Copy the resized image to the center of the blank canvas
    resizedImage.copyTo(outputImage(cv::Rect(x, y, newWidth, newHeight)));

    return outputImage;
}

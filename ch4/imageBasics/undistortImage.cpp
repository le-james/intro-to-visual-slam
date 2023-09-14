#include <opencv2/opencv.hpp>
#include <string>

std::string distortedImg = "../../distorted.png";

int main(int argc, char **argv)
{
    // rad-tan distortion model - get from camera calibration
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.7618711e-05;
    // intrinsics of a camera - get from camera calibration
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    // store the image in an opencv matrix
    cv::Mat image = cv::imread(distortedImg,0);

    // get the type of the image
    // will return a 0 which is CV_8UC1
    std::cout << "image type: " << image.type() << std::endl;

    // get the rows and cols of the distorted image
    int rows = image.rows, cols = image.cols;

    // setup a matrix to store the undistorted image
    // all pixels are initially set to 0
    cv::Mat undistortedImg = cv::Mat(rows,cols,CV_8UC1);


    for(int v = 0; v < rows; v++)
    {
        for(int u = 0; u < cols; u++)
        {
            // convert from pixel plane to image plane
            double x = (u - cx)/fx, y = (v - cy)/fy;

            double r = sqrt(x*x + y*y);

            // get the location of the undistorted pixel in the image plane
            double x_distorted = x*(1 + k1*r*r + k2*r*r*r*r) + 2*p1*x*y + p2*(r*r + 2*x*x);
            double y_distorted = y*(1 + k1*r*r + k2*r*r*r*r) + p1*(r*r + 2*y*y) + 2*p2*x*y;

            // convert the pixel location back to the pixel plane
            double u_distorted = fx * x_distorted + cx;
            double v_distorted = fy * y_distorted + cy;


            if(u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows)
            {
                // at<>() gets a reference to the array element
                // need to look at the documentation to get the correct template type
                // for CV_8UC1 have to use uchar
                undistortedImg.at<uchar>(v,u) = image.at<uchar>((int)v_distorted, (int)u_distorted);
            } else {
                // set pixel to black if out of bounds
                undistortedImg.at<uchar>(v,u) = 255;
            }
        }
    }

    // undistortedImg(cv::Rect(0,0,100,100)).setTo(255);

    cv::imshow("distorted image", image);
    cv::imshow("undistorted image", undistortedImg);

    cv::waitKey(0);
    cv::destroyAllWindows();
}

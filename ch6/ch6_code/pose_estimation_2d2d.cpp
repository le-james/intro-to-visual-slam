#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

// has functions to compute the matrices
#include <opencv2/calib3d/calib3d.hpp>


// declare functions

// 
void find_feature_matches(
    const cv::Mat &img_1,
    const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches);

void pose_estimation_2d2d(
    std::vector<cv::KeyPoint> keypoints_1,
    std::vector<cv::KeyPoint> keypoints_2,
    std::vector<cv::DMatch> matches,
    cv::Mat &R,
    cv::Mat &t);

cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);



int main(int argc, char **argv)
{
    // precheck to ensure images are included into the program
    if(argc != 3)
    {
        std::cout << "ERR: Need to include img1 and img2";
        return 1;
    }

    // load the images
    cv::Mat img_1 = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], cv::IMREAD_COLOR);

    // make sure that the arguments are true otherwise return error
        // a pointer is true if it is not a nullptr
    assert(img_1.data && img_2.data && "Custom Err Msg: Could not load the two images!");


    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> matches;

    // match the features
        // outputs matches
    find_feature_matches(img_1,img_2,keypoints_1,keypoints_2,matches);
    std::cout << "In total we get: " << matches.size() << " set of feature points" << std::endl;

    // estimate the motion between two frames
        // outputs R and t
    cv::Mat R, t;
    // the function that computes t, normalizes it because of scale ambiguity
    pose_estimation_2d2d(keypoints_1,keypoints_2,matches,R,t);

    // check E = t^R
    // convert t to a skew-symmetric matrix
    cv::Mat t_x = (cv::Mat_<double>(3,3) << 0, -t.at<double>(2,0), t.at<double>(1,0),
                                            t.at<double>(2,0), 0, -t.at<double>(0,0),
                                            -t.at<double>(1,0), t.at<double>(0,0), 0);
    std::cout << "t^R: \n" << t_x*R << std::endl;

    // check epipolar constraints
    // camera intrinsics 
    cv::Mat K = (cv::Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for(cv::DMatch m:matches)
    {
        // keypoint pixel position
        cv::Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt,K);
        cv::Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt,K);

        // store pixel position in vector form
        cv::Mat y1 = (cv::Mat_<double>(3,1) << pt1.x,pt1.y,1);
        cv::Mat y2 = (cv::Mat_<double>(3,1) << pt2.x,pt2.y,1);

        // epipolar constraint - should equal to zero
        cv::Mat d = y2.t() * t_x * R *y1;

        std::cout << "Epipolar constraint: " << d << std::endl;
    }

    std::size_t m = matches.size();

    std::cout << "type: " << m << std::endl;

}

// find and match the orb features
    // this is basically the code from orb_cv.cpp
void find_feature_matches(
    const cv::Mat &img_1, 
    const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1, 
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches)
{
    
    cv::Mat descriptor_1, descriptor_2;

    // cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    // or - since ORB inherits from Featuredetector which is typedef of Feature2d
    cv::Ptr<cv::ORB> detector = cv::ORB::create();

    // cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::ORB> descriptor = cv::ORB::create();    // DOES THIS WORK?

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    // Step 1: Detect Oriented FAST corner position - find keypoints in the two images
    detector->detect(img_1,keypoints_1);
    detector->detect(img_2,keypoints_2);

    // Step 2: Calculate the BRIEF descriptor based on the corner position
    descriptor->compute(img_1,keypoints_1,descriptor_1);
    descriptor->compute(img_2,keypoints_2,descriptor_2);

    // Step 3: Match the BRIEF descriptors in the two images, using the Hamming distance
    std::vector<cv::DMatch> match;

    // store the matches in the vector match
    // computes and stores the distance between every descriptor in match
        // then we have to filter out the best on our own below
    matcher->match(descriptor_1,descriptor_2,match);

    // Step 4: Match point pair filtering
    double min_dist = 10000, max_dist = 0;

    // Find the minimum and maximum distances between all matches, that is, 
        // the distance between the most similar and least similar two sets of points  
    for(int i = 0; i < descriptor_1.rows; i++)
    {
        double dist = match[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }

    printf("Max dist: %f \n", max_dist);
    printf("Mix dist: %f \n", min_dist);

    // filter match to get the best matches
    // When the distance between descriptors is greater than twice the minimum distance, 
        // it is considered that the match is wrong. But sometimes the minimum distance will be very small, 
        // set an experience value of 30 as the lower limit. 
    for(int i = 0; i < descriptor_1.rows; i++)
    {
        if(match[i].distance < std::max(2*min_dist,30.0))
        {
            matches.push_back(match[i]);
        }
    }
}


// estimate the rotation matrix and translation from the essential matrix
void pose_estimation_2d2d(
    std::vector<cv::KeyPoint> keypoints_1,
    std::vector<cv::KeyPoint> keypoints_2,
    std::vector<cv::DMatch> matches,
    cv::Mat &R,
    cv::Mat &t)
{
    // camera intrinsics 
    cv::Mat K = (cv::Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    // keypoints.pt is of type Point2f
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;

    // extract the image plane coordinates of the keypoint
    // .size() returns a type of std::size_t - unsigned integer (not negative)
    for(int i = 0; i < matches.size(); i ++)
    {
        // find the keypoints in keypoints_1 and keypoints_2 that correspond to each other
            // queryIdx is where the keypoint is in keypoints_1 and trainIdx is where it is in keypoints_2
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    // compute the fundamental matrix - changed CV_FM_8POINT to FM_8POINT
    cv::Mat fundamental_matrix;
    fundamental_matrix = cv::findFundamentalMat(points1,points2,cv::FM_8POINT);
    std::cout << "The fundamental matrix is: \n" << fundamental_matrix << std::endl;

    // compute the essential matrix
    // camera principle point
    cv::Point2d principle_point(325.1,249.7);
    // camera focal length
    double focal_length = 521;
    cv::Mat essential_matrix = cv::findEssentialMat(points1,points2,focal_length,principle_point);
    std::cout << "The essential matrix is: \n" << essential_matrix << std::endl;

    // compute the homography matrix
        // the scene is not planar so this is not useful here
    cv::Mat homography_matrix;
    homography_matrix = cv::findHomography(points1,points2,cv::RANSAC,3);
    std::cout << "The homography matrix is: \n" << homography_matrix << std::endl;

    // compute the rotation matrix and translation from img_1 to img_2
    cv::recoverPose(essential_matrix,points1,points2,R,t,focal_length,principle_point);
    std::cout << "R is: \n" << R << std::endl;
    std::cout << "t is: \n" << t << std::endl;
}


// conver from pixel plane to normalized camera plane
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K)
{ 
    return cv::Point2d( 
        (p.x - K.at<double>(0,2)) / K.at<double>(0,0),      // X/Z
        (p.y - K.at<double>(1,2)) / K.at<double>(1,1));     // Y/Z
} 

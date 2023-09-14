#include <iostream>
#include <opencv2/opencv.hpp>

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

void triangulation(
    const std::vector<cv::KeyPoint> &Keypoint_1,
    const std::vector<cv::KeyPoint> &Keypoint_2,
    const std::vector<cv::DMatch> &matches,
    const cv::Mat &R,
    const cv::Mat &t,
    std::vector<cv::Point3d> &points);

inline cv::Scalar get_color(float depth)
{
    float up_thres = 50, low_thres = 10, thres_range = up_thres - low_thres;

    if(depth > up_thres) depth = up_thres;
    if(depth < low_thres) depth = low_thres;

    return cv::Scalar(255*depth/thres_range, 255*(1-depth/thres_range));
}


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

    // triangulation
    std::vector<cv::Point3d> points;
    // 
    triangulation(keypoints_1, keypoints_2, matches, R, t, points);


    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    cv::Mat img1_plot = img_1.clone();
    cv::Mat img2_plot = img_2.clone();

    for (int i = 0; i < matches.size(); i++)
    {
        // first image
        float depth1 = points[i].z;

        std::cout << "depth: " << depth1 << std::endl;

        // this isn't even used
        // cv::Point2d pt1_cam = pixel2cam(keypoints_1[matches[i].queryIdx].pt, K);

        // plot coloured dots based on the distance
        cv::circle(img1_plot, keypoints_1[matches[i].queryIdx].pt, 4, get_color(depth1), 2);


        // second image
        // convert points into the second camera frame
        cv::Mat pt2_trans = R * (cv::Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;

        float depth2 = pt2_trans.at<double>(2, 0);

        // plot coloured dots based on the distance
        cv::circle(img2_plot, keypoints_2[matches[i].trainIdx].pt, 4, get_color(depth2), 2);
    }

    cv::imshow("img 1", img1_plot);
    cv::imshow("img 2", img2_plot);
    cv::waitKey();
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
    // triangulation needs a float to work
    return cv::Point2f( 
        (p.x - K.at<double>(0,2)) / K.at<double>(0,0), 
        (p.y - K.at<double>(1,2)) / K.at<double>(1,1)); 
} 

// get depth of the pixel
void triangulation(
    const std::vector<cv::KeyPoint> &keypoint_1,
    const std::vector<cv::KeyPoint> &keypoint_2,
    const std::vector<cv::DMatch> &matches,
    const cv::Mat &R,
    const cv::Mat &t,
    std::vector<cv::Point3d> &points)
{
    // extrinsic matrix of camera 1
    cv::Mat T1 = (cv::Mat_<float>(3,4) << 
        1,0,0,0, 
        0,1,0,0, 
        0,0,1,0);

    // extrinsic matrix of camera 2
    cv::Mat T2 = (cv::Mat_<float>(3,4) << 
        R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0), 
        R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0), 
        R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0));

    // intrinsic camera matrix
    cv::Mat K = (cv::Mat_<double>(3,3) << 520.9,0,325.1,0,521.0,249.7,0,0,1);

    // extract good matched keypoints
    std::vector<cv::Point2f> pts_1, pts_2; 

    for(cv::DMatch m:matches)
    {
        // Convert pixel coordinates to normalized camera coordinates 
        pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt,K)); 
        pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt,K)); 
    }

    // 4xN (numbers of feature points) as per triangulatePoints()
    cv::Mat pts_4d;

    cv::triangulatePoints(T1,T2,pts_1,pts_2,pts_4d);

    // Convert to non-homogeneous coordinates (cartesian form)
    for(int i =  0; i < pts_4d.cols; i++)
    { 
        cv::Mat x = pts_4d.col(i); 

        // convert to cartesian by dividing the 4x1 vector by the last element
        x /= x.at<float>(3,0);

        cv::Point3d p(x.at<float>(0,0), x.at<float>(1,0), x.at<float>(2,0));

        points.push_back(p); 
    } 
}







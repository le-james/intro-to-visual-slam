#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>


int main(int argc, char **argv)
{

    // exit program if img1 and img2 are not included
    if(argc != 3)
    {
        std::cout << "ERR: Need to include img1 and img2";
        return 1;
    }

    // read images
    // the second argument used in the example is removed from opencv
    cv::Mat img_1 = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], cv::IMREAD_COLOR);

    // initialization
    std::vector<cv::KeyPoint> keypoints1, keypoints2;

    cv::Mat descriptors_1, descriptors_2;

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();

    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");


    // get orb features - fast and brief
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // detect oriented fast - find keypoints in the images
    detector->detect(img_1, keypoints1);
    detector->detect(img_2, keypoints2);

    // compute brief descriptor
    descriptor->compute(img_1, keypoints1, descriptors_1);
    descriptor->compute(img_2, keypoints2, descriptors_2);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);

    std::cout << "Extract ORB features time: " << time_used.count() << " seconds" << std::endl;


    // draw the matched points
    cv::Mat outimg1;
    cv::drawKeypoints(img_1, keypoints1, outimg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("ORB features", outimg1);


    // use hamming distance to match the features
    // vector of DMatch objects
    std::vector<cv::DMatch> matches;

    t1 = std::chrono::steady_clock::now();
    // match the descriptors in img1 and img2
    matcher->match(descriptors_1, descriptors_2, matches);
    t2 = std::chrono::steady_clock::now();

    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);

    std::cout << "Match ORB features time" << time_used.count() << " seconds" << std::endl;


    // sort and remove outliers

    // min and max distance - returns pointers to the min and max values
    auto min_max = std::minmax_element(
        matches.begin(), 
        matches.end(), 
        [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });

    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    printf("Min distance: %f \n", min_dist);
    printf("Max distance: %f \n", max_dist);

    // remove bad matching using the minimum distance
    std::vector<cv::DMatch> good_matches;
    for(int i = 0; i < descriptors_1.rows; i++)
    {
        // sometimes min_dist is too small so the minimum is then set to 30
        if(matches[i].distance < std::max(2*min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
        }
    }


    // draw the results
    cv::Mat img_match;
    cv::Mat img_goodmatch;

    cv::drawMatches(img_1, keypoints1, img_2, keypoints2, matches, img_match);
    cv::drawMatches(img_1, keypoints2, img_2, keypoints2, good_matches, img_goodmatch);

    imshow("All matches", img_match);
    imshow("Good matches", img_goodmatch);

    cv::waitKey(0);

}

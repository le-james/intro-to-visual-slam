#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


// int main(int argc, char **argv)
// or
int main(int argc, char *argv[])
{

    // checking opencv version
    // std::cout << "opencv ver: " << CV_VERSION << std::endl;
    // std::cout << "opencv ver: " << CV_MAJOR_VERSION << std::endl;
    // std::cout << "opencv ver: " << CV_MINOR_VERSION << std::endl;
 
    // read the image from the command line argument
    cv::Mat image;
    image = cv::imread(argv[1]);

    // check if the image is loaded
    if(image.data == nullptr)
    {
        std::cerr << "file" << argv[1] << " does not exist" << std::endl;
    }


    // display the image
    // cv::imshow("spacex image", image);
    // cv::waitKey(0);
    // cv::destroyAllWindows();
    // or
    // cv::destroyWindow("spacex image");


    // info of the image
    std::cout << "image cols: " << image.cols << ", image rows: " << image.rows 
        << ", channels: " << image.channels() << std::endl;


    // get type of image
    if(image.type() == CV_8UC3)
    {
        std::cout << "image type is CV_8UC3 which is: " << image.type() << std::endl;
    }


    // loop through all the image pixels

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    for(size_t y = 0; y < image.rows; y++)
    {
        // get the row to loop through
        unsigned char* row_ptr = image.ptr<unsigned char>(y);

        // loop through the row
        for(size_t x = 0; x < image.cols; x++)
        { 
            // get all the channels of pixel x
            unsigned char* data_ptr = &row_ptr[x*image.channels()];

            // loop through each channel of pixel x
            for(int c = 0; c != image.channels(); c++)
            {
                // get the channel data
                unsigned char data = data_ptr[c];
            }  
        }
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    // time to loop through the image pixels
    std::chrono::duration<double> time_used = 
        std::chrono::duration_cast< std::chrono::duration<double> >(t2-t1);
    std::cout << "time used: " << time_used.count() << " seconds" << std::endl;


    // copying a cv::Mat type

    // creates a reference not a new copy of image
    cv::Mat another_image = image;

    // change pixels of the image black
    another_image(cv::Rect(400,200,100,100)).setTo(0);

    // another_image changes image so another_image is a reference of image only
    cv::imshow("changed image", image);
    cv::waitKey(0);


    // copy image
    cv::Mat clone_image = image.clone();

    clone_image(cv::Rect(0,0,100,100)).setTo(0);

    cv::imshow("original image", image);
    cv::imshow("cloned image", clone_image);

    cv::waitKey(0);

}


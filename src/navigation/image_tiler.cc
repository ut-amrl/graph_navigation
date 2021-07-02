#include "image_tiler.h"
#include <iostream>

using namespace cv;
using namespace std;

ImageCells::ImageCells( int rows, int cols, int width, int height)
{
    image = Mat::zeros( rows * height, cols * width, CV_8UC3);
    m_width = width;
    m_height = height;
    m_cols = cols;
    m_rows = rows;
}

void ImageCells::setCell( int col, int row, Mat img )
{
    if((img.cols == m_width) & (img.rows == m_height))
    {
        Mat roi = image( Rect(col * m_width, row * m_height, m_width, m_height) );
        img.copyTo(roi);
    } else {
        std::cerr << "Got image of size " << img.cols << ", " << img.rows << ". Expected size " << m_width << ", " << m_height << std::endl;
        exit(1);
    }
}

Mat ImageCells::getCell( int col, int row )
{
    Mat roi = image( Rect(col * m_width, row * m_height, m_width, m_height) );
    return roi.clone();
}

void ImageCells::setImage( Mat img )
{
    if((img.cols >= image.cols) & (img.rows >= image.rows))
    img.copyTo(image);
}

#ifndef REDIS_COMMUNICATION
#define REDIS_COMMUNICATION

#include <hiredis/hiredis.h>
#include <sw/redis++/redis++.h>

#include <iostream>
#include <stdlib.h>
#include <chrono>
#include <numeric>

#include <string>
#include <vector>
#include <optional>
#include <opencv2/opencv.hpp>

#include "itkImage.h"
#include "itkVector.h"


namespace ProstmorphBridge 
{    

using ImagePixelType = unsigned char;
using ImageSliceType = itk::Image<ImagePixelType, 2>;
using ImageType = itk::Image<ImagePixelType, 3>;
using FieldPixelType = float;  // TODO change vector pixel type
using FieldImageType = itk::Image<itk::Vector<FieldPixelType, 3>, 3>;

class Connector
{

private:
    std::string _outgoing_channel, _incoming_channel;
    // MR image tags
    std::string _mr_img_encoding_key = "mr_image";
    std::string _mr_seg_encoding_key = "mr_seg";
    std::string _mr_size_key = "mr_size";
    std::string _mr_origin_key = "mr_origin";
    std::string _mr_spacing_key = "mr_spacing";
    std::string _mr_direction_key = "mr_direction";
    // US image tags
    std::string _us_img_encoding_key = "us_image";
    std::string _us_seg_encoding_key = "us_seg";
    std::string _us_size_key = "us_size";
    std::string _us_origin_key = "us_origin";
    std::string _us_spacing_key = "us_spacing";
    std::string _us_direction_key = "us_direction";
    // deformation field tags
    std::string _deformation_field_encoding_key = "field";
    std::string _deformation_field_size_key = "field_size";

    // deformation encoding
    std::vector<std::string> _deformation_encodings;
    std::vector<std::string> _deformation_field_size;

    std::unique_ptr<sw::redis::Redis> _redis_client;
    std::unique_ptr<sw::redis::Subscriber> _sub_receive_deformation;

public:
    Connector() = delete;
    Connector(const Connector &b) = delete;
    Connector(std::string redis_ip, std::string outgoing_channel, std::string incoming_channel);
    ~Connector(){};

    void 
    request_deformation(
        const ImageType::Pointer& mr_image, const ImageType::Pointer& us_image, 
        const ImageType::Pointer& mr_segmentation, const ImageType::Pointer& us_segmentation, 
        FieldImageType::Pointer& deformation_field
    );

    const ImageType::Pointer
    rasify_image(const ImageType::Pointer& image);

    void 
    encode_image(const ImageType::Pointer& image, std::vector<std::string>& image_encodings);

    void 
    decode_field(const std::vector<std::string>& field_encodings, FieldImageType::Pointer& field);
};

}

#endif
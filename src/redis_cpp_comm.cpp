#include "redis_cpp_comm.hpp"
#include <unistd.h>
#include <cmath>
#include <iostream>

#include "itkImage.h"
#include "itkVectorImage.h"
#include "itkOpenCVImageBridge.h"
#include "itkExtractImageFilter.h"
#include "itkComposeImageFilter.h"
#include "itkPasteImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkOrientImageFilter.h"


using namespace ProstmorphBridge;

Connector::Connector(
    std::string redis_ip, 
    std::string outgoing_channel, 
    std::string incoming_channel
):
    _outgoing_channel{outgoing_channel},
    _incoming_channel{incoming_channel}
{
    sw::redis::ConnectionOptions connection_options;
    connection_options.host = "127.0.0.1";
    connection_options.port = 6379; // 6379 default
    // connection_options.password = "auth"; // no password default
    // connection_options.db = 0;  // 0th database default
    connection_options.socket_timeout = std::chrono::milliseconds(10000); // 0ms default, never timeout

    sw::redis::ConnectionPoolOptions pool_options;
    pool_options.size = 1;
    // pool_options.wait_timeout = std::chrono::milliseconds(100);  // 0ms default, never timeout
    // pool_options.connection_lifetime = std::chrono::minutes(10);  //  0ms default, forever lifetime
    
    // Create an Redis object, which is MOVABLE but NOT copyable.
    this->_redis_client = std::make_unique<sw::redis::Redis>(connection_options, pool_options);
    this->_sub_receive_deformation = std::make_unique<sw::redis::Subscriber>(this->_redis_client->subscriber());  // socket_timeout = 10000 ms, same as Redis object

    // Set callback function
    this->_sub_receive_deformation->on_message([this](std::string channel, std::string msg) {
        // since the subscriber only listenes to this->_incoming_channel, there's no need to check for the right channel
        // check result message from prostmorph
        if (msg == "done")
        {
            // get the image
            std::cout << "deformation done" << std::endl;
            // const int deformation_field_slices = std::stoi(this->_redis_client->get(this->_deformation_field_slices_key).value());
            this->_deformation_field_size.clear();
            this->_redis_client->lrange(this->_deformation_field_size_key, 0, -1, std::back_inserter(this->_deformation_field_size));
            this->_deformation_encodings.clear();
            this->_redis_client->lrange(this->_deformation_field_encoding_key, 0, -1, std::back_inserter(this->_deformation_encodings));

        }
        else if (msg == "fail")
        {
            std::cout << "deformation fail" << std::endl;
            this->_deformation_encodings.clear();
        }
        else
        {
            std::cerr << "Unknown result message: " << msg << std::endl;
            this->_deformation_encodings.clear();
        }
    });
    this->_sub_receive_deformation->on_meta([this](sw::redis::Subscriber::MsgType type, sw::redis::OptionalString channel, long long num) {
        switch (type)
        {
            case sw::redis::Subscriber::MsgType::SUBSCRIBE:
                std::cout << "subscription done" << std::endl;
                std::cout << "Bridge initialized on incoming channel: " << channel.value() << "\n" << std::endl;
                break;
            default:
                assert(false && "This should never happen");
        }
    });
    this->_sub_receive_deformation->subscribe(this->_incoming_channel);

    // check subscription
    try {
        std::cout << "Subscription... ";
        this->_sub_receive_deformation->consume();
    }
    catch (const sw::redis::TimeoutError& tme) {
        std::cout << "error" << std::endl;
        std::cerr << "Timeout, quitting..." << std::endl;
        throw tme;
    }
};


void Connector::request_deformation(
    const ImageType::Pointer& mr_image, 
    const ImageType::Pointer& us_image,
    const ImageType::Pointer& mr_segmentation,
    const ImageType::Pointer& us_segmentation,
    FieldImageType::Pointer& deformation_field
) {
    std::chrono::steady_clock::time_point start, end;
    long time;

    // set same orientation
    start = std::chrono::steady_clock::now();
    const ImageType::Pointer mr_img_ras = mr_image; //rasify_image(mr_image);
    const ImageType::Pointer us_img_ras = us_image; //rasify_image(us_image);
    const ImageType::Pointer mr_seg_ras = mr_segmentation; //rasify_image(mr_segmentation);
    const ImageType::Pointer us_seg_ras = us_segmentation; //rasify_image(us_segmentation);
    end = std::chrono::steady_clock::now();
    time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "RASified images in " << time << "ms" << std::endl;

    // set dimensions and image encodings
    ImageType::SizeType mr_size = mr_img_ras->GetLargestPossibleRegion().GetSize();
    ImageType::PointType mr_origin = mr_img_ras->GetOrigin();
    ImageType::SpacingType mr_spacing = mr_img_ras->GetSpacing();
    auto mr_direction = mr_img_ras->GetDirection().GetVnlMatrix();
    ImageType::SizeType us_size = us_img_ras->GetLargestPossibleRegion().GetSize();
    ImageType::PointType us_origin = us_img_ras->GetOrigin();
    ImageType::SpacingType us_spacing = us_img_ras->GetSpacing();
    auto us_direction = us_img_ras->GetDirection().GetVnlMatrix();
    
    std::vector<std::string> mr_img_encodings, us_img_encodings, mr_seg_encodings, us_seg_encodings;
    start = std::chrono::steady_clock::now();
    this->encode_image(mr_img_ras, mr_img_encodings);
    this->encode_image(us_img_ras, us_img_encodings);
    this->encode_image(mr_seg_ras, mr_seg_encodings);
    this->encode_image(us_seg_ras, us_seg_encodings);
    end = std::chrono::steady_clock::now();
    time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Encoded images in " << time << "ms" << std::endl;

    // send everything
    start = std::chrono::steady_clock::now();
    auto pipe_replies = this->_redis_client->pipeline()
        // MR image
        .del(this->_mr_size_key)
        .rpush(this->_mr_size_key, mr_size.begin(), mr_size.end())
        .del(this->_mr_origin_key)
        .rpush(this->_mr_origin_key, mr_origin.begin(), mr_origin.end())
        .del(this->_mr_spacing_key)
        .rpush(this->_mr_spacing_key, mr_spacing.begin(), mr_spacing.end())
        .del(this->_mr_direction_key)
        .rpush(this->_mr_direction_key, mr_direction.begin(), mr_direction.end())
        .del(this->_mr_img_encoding_key)
        .rpush(this->_mr_img_encoding_key, mr_img_encodings.begin(), mr_img_encodings.end())
        .del(this->_mr_seg_encoding_key)
        .rpush(this->_mr_seg_encoding_key, mr_seg_encodings.begin(), mr_seg_encodings.end())
        // US image
        .del(this->_us_size_key)
        .rpush(this->_us_size_key, us_size.begin(), us_size.end())
        .del(this->_us_origin_key)
        .rpush(this->_us_origin_key, us_origin.begin(), us_origin.end())
        .del(this->_us_spacing_key)
        .rpush(this->_us_spacing_key, us_spacing.begin(), us_spacing.end())
        .del(this->_us_direction_key)
        .rpush(this->_us_direction_key, us_direction.begin(), us_direction.end())
        .del(this->_us_img_encoding_key)
        .rpush(this->_us_img_encoding_key, us_img_encodings.begin(), us_img_encodings.end())
        .del(this->_us_seg_encoding_key)
        .rpush(this->_us_seg_encoding_key, us_seg_encodings.begin(), us_seg_encodings.end())
        .exec();

    // ping Python side about the new data
    this->_redis_client->publish(this->_outgoing_channel, "compute_deformation");
    end = std::chrono::steady_clock::now();
    time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Images sent in " << time << "ms" << std::endl;

    // wait until response message
    while(true)
    {
        try 
        {
            std::cout << "Waiting for the deformation field... ";
            auto start = std::chrono::steady_clock::now();
            this->_sub_receive_deformation->consume();
            auto end = std::chrono::steady_clock::now();
            long rtt = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "Images received in " << rtt << "ms" << std::endl;
            break;
        }
        catch (const sw::redis::TimeoutError& tme) 
        {
            std::cout << "Timeout waiting response message, retrying..." << std::endl;
        }
        catch (const sw::redis::Error& e) 
        {
            std::cerr << "Exception in redis msg consume: " << e.what() << std::endl;
            break;
        }
    }
    
    // decode deformation encoding
    if (!this->_deformation_encodings.empty())
    {
        std::cout << "Deconding... " << std::flush;
        auto start = std::chrono::steady_clock::now();
        this->decode_field(this->_deformation_encodings, deformation_field);
        auto end = std::chrono::steady_clock::now();
        long rtt = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "decoding done in " << rtt << "ms" << std::endl;
    }
    else
    {
        std::cerr << "Deformation failed" << std::endl;
        deformation_field = nullptr;
    }
};


const ImageType::Pointer Connector::rasify_image(
    const ImageType::Pointer& image
) {
    // force RAS orientation for consistency
    using OrienterType = itk::OrientImageFilter<ImageType,ImageType>;
    OrienterType::Pointer orienter = OrienterType::New();
    orienter->UseImageDirectionOn();
    orienter->SetDesiredCoordinateOrientation(itk::SpatialOrientationEnums::ValidCoordinateOrientations::ITK_COORDINATE_ORIENTATION_RSA);
    orienter->SetInput(image);
    orienter->Update();
    return orienter->GetOutput();
}


inline int argmin(
    const ImageType::SizeType size
) {
    int arg = 0;
    if (size[1] < size[arg])
        arg = 1;
    if (size[2] < size[arg])
        arg = 2;
    return arg;
}


void Connector::encode_image(
    const ImageType::Pointer& image, 
    std::vector<std::string>& image_encodings
) {
    image_encodings.clear();

    // set up the extraction region
    ImageType::RegionType region = image->GetLargestPossibleRegion();
    ImageType::IndexType start = region.GetIndex();
    ImageType::SizeType size = region.GetSize();

    // size = [W, H, D], we extract along the last direction
    const int extract_direction = argmin(size);
    const int n_slices = size[extract_direction];
    size[extract_direction] = 0;

    // prepare the slice pasteFilter
    ImageType::RegionType sliceRegion;
    sliceRegion.SetSize(size);

    using ExtractFilterType = itk::ExtractImageFilter<ImageType, ImageSliceType>;
    auto extractFilter = ExtractFilterType::New();
    extractFilter->SetDirectionCollapseToIdentity();
    extractFilter->SetInput(image);

    // declare collector variables
    cv::Mat slice;
    ImageSliceType::Pointer img;
    std::vector<unsigned char> image_encoding_buffer;
    for (int i = 0; i < n_slices; ++i)
    // in memoriam of (int i = n_slices * flipZ; i * (1 - 2 * flipZ) < n_slices * (!flipZ); i += (1 * !flipZ) - (1 * flipZ))
    {
        start[extract_direction] = i;
        sliceRegion.SetIndex(start);
        extractFilter->SetExtractionRegion(sliceRegion);
        extractFilter->Update();  // actually extract the pixel values from the image
        
        img = extractFilter->GetOutput();
        // apparently, the bridge uses (w, h) = size[0], size[1], so we also transpose the result
        slice = itk::OpenCVImageBridge::ITKImageToCVMat<ImageSliceType>(img).t();

        cv::imencode(".png", slice, image_encoding_buffer);
        image_encodings.push_back({image_encoding_buffer.begin(), image_encoding_buffer.end()});
    }
};


void Connector::decode_field(
    const std::vector<std::string>& field_encodings, 
    FieldImageType::Pointer& field
) {
    using FixedFieldImageType = itk::Image<itk::FixedArray<FieldPixelType, 3>, 3>;
    using RGBCompressedSliceType = itk::Image<itk::RGBPixel<FieldPixelType>, 2>;
    FixedFieldImageType::Pointer vectorField = FixedFieldImageType::New();
    FixedFieldImageType::IndexType sliceStart = {0, 0, 0};
    const FixedFieldImageType::SizeType fieldShape = {
        std::stoul(this->_deformation_field_size[0]),
        std::stoul(this->_deformation_field_size[1]),
        std::stoul(this->_deformation_field_size[2])
    };
    const FixedFieldImageType::RegionType vectorRegion(sliceStart, fieldShape);
    vectorField->SetRegions(vectorRegion); 
    vectorField->Allocate();
    
    auto pasteFilter = itk::PasteImageFilter<FixedFieldImageType, RGBCompressedSliceType>::New();
    pasteFilter->SetDestinationImage(vectorField);
    if (pasteFilter->CanRunInPlace())
        pasteFilter->InPlaceOn();

    const int extract_direction = 2;
    std::vector<char> image_encoding_buffer;
    cv::Mat cvSlice;
    RGBCompressedSliceType::Pointer fieldSlice;
    const RGBCompressedSliceType::RegionType sliceRegion({0, 0}, {fieldShape[0], fieldShape[1]});
    pasteFilter->SetSourceRegion(sliceRegion);
    for (int i = 0; i < field_encodings.size(); ++i) 
    {
        sliceStart[extract_direction] = i;
        image_encoding_buffer.assign(field_encodings[i].begin(), field_encodings[i].end());  // imdecode needs a vector<char>
        cv::imdecode(image_encoding_buffer, cv::IMREAD_UNCHANGED, &cvSlice);  // 2D image with 3 channels

        // auto pixel = cvSlice.at<cv::Vec3f>(20, 20);
        // std::cout << "(20, 20): [" << int(pixel[0]) << ", " << int(pixel[1]) << ", " << int(pixel[2]) << "]" << std::endl;

        fieldSlice = itk::OpenCVImageBridge::CVMatToITKImage<RGBCompressedSliceType>(cvSlice);  // 2D image with RGBPixel

        pasteFilter->SetSourceImage(fieldSlice);
        pasteFilter->SetDestinationIndex(sliceStart);
        pasteFilter->Update();
    }

    auto castFilter = itk::CastImageFilter<FixedFieldImageType, FieldImageType>::New();
    castFilter->SetInput(pasteFilter->GetOutput());
    
    field = castFilter->GetOutput();
};


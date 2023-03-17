#include "redis_cpp_comm.hpp"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkNrrdImageIOFactory.h"

#include <opencv2/opencv.hpp>
#include <unistd.h>

using namespace ProstmorphBridge;

int main(int argc, char** argv) 
{
    // https://itk.org/Wiki/ITK/FAQ#NoFactoryException
    itk::NrrdImageIOFactory::RegisterOneFactory();

    std::string redis_ip = "127.0.0.1";
    std::string outgoing_channel = "ProstmorphBridge/compute_deformation/command";
    std::string incoming_channel = "ProstmorphBridge/compute_deformation/results";

    auto bridge = Connector(redis_ip, outgoing_channel, incoming_channel);

    std::string mr_img_filepath, us_img_filepath, mr_seg_filepath, us_seg_filepath;
    ImageType::Pointer mr_img, us_img, mr_seg, us_seg;
    FieldImageType::Pointer def_field;
    long rtt;
    std::vector<long int> times;
    int N = 3;
    for (int i = 0; i < N; ++i) 
    {
        mr_img_filepath = "/mnt/r/DATASET_PROSTATE/mri/paziente1/Dato1/MRI1_1.3.6.1.4.1.14519.5.2.1.266717969984343981963002258381778490221.nrrd";
        mr_seg_filepath = "/mnt/r/DATASET_PROSTATE/mri/paziente1/Dato1/Prostate1_1.3.6.1.4.1.14519.5.2.1.266717969984343981963002258381778490221.nrrd";
        us_img_filepath = "/mnt/r/DATASET_PROSTATE/us/paziente1/Dato1/US_1.3.6.1.4.1.14519.5.2.1.140367896789002601449386011052978380612.nrrd";
        us_seg_filepath = "/mnt/r/DATASET_PROSTATE/us/paziente1/Dato1/Prostate_1.3.6.1.4.1.14519.5.2.1.140367896789002601449386011052978380612.nrrd";

        mr_img = itk::ReadImage<ImageType>(mr_img_filepath);
        mr_seg = itk::ReadImage<ImageType>(mr_seg_filepath);
        us_img = itk::ReadImage<ImageType>(us_img_filepath);
        us_seg = itk::ReadImage<ImageType>(us_seg_filepath);

        // send images over the bridge and get back the deformation
        auto start = std::chrono::steady_clock::now();
        bridge.request_deformation(mr_img, us_img, mr_seg, us_seg, def_field);
        auto end = std::chrono::steady_clock::now();

        // std::cout << deformation_field->GetLargestPossibleRegion().GetSize() << " x " << deformation_field->GetVectorLength() << std::endl;

        // store RTT
        rtt = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Done in " << rtt << "\n" <<std::endl;
        times.push_back(rtt);
    }

    long int mean = std::reduce(times.begin(), times.end()) / N;
    std::cout << "Mean RTT: " << std::to_string(mean) << std::endl;

    return 0;
}

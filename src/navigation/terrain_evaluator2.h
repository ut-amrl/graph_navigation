#include "terrain_evaluator.h"

namespace motion_primitives {

class CustomTerrainEvaluator : public TerrainEvaluator {
 public:
 torch::Tensor context_tensor_;
  CustomTerrainEvaluator() : TerrainEvaluator() {
    // todo: store the context as private variable
    torch::jit::script::Module tensors = torch::jit::load("../terrain_models/context.pt");
    context_tensor_ = tensors.run_method("return_tensor").toTensor();
  }



// latest bev image 749 1476
// latest bev image channels 3
// Image min: 0
// Image max: 255

  cv::Mat1f GetScalarCostImage(const cv::Mat3b& bev_image) override {
    // print here
    // std::cout << "Custom GetScalarCostImage" << std::endl;
    cv::Mat3b latest_bev_image = image.clone();
    // print the shape
    std::cout << "latest bev image " << latest_bev_image.rows << " " << latest_bev_image.cols << std::endl;
    int rows = latest_bev_image.rows;
    int cols = latest_bev_image.cols;
    // print the number of channels
    std::cout << "latest bev image channels " << latest_bev_image.channels() << std::endl;

    // // for debugging
    // cv::Mat1f scalar_cost_map(rows, cols);
    // // make the left half of the image black and the right half white
    // for (int i = 0; i < scalar_cost_map.rows; i++) {
    //   for (int j = 0; j < scalar_cost_map.cols; j++) {
    //     if (j < scalar_cost_map.cols / 2) {
    //       scalar_cost_map.at<float>(i, j) = 0;
    //     } else {
    //       scalar_cost_map.at<float>(i, j) = 1;
    //     }
    //   }
    // }

    // // write the latest_bev_image to a file
    // cv::imwrite("latest_bev_image.png", latest_bev_image);
    // // write the scalar_cost_map to a file
    // cv::imwrite("scalar_cost_map.png", scalar_cost_map*255);
    // return scalar_cost_map;


    // resize the image
    cv::resize(latest_bev_image, latest_bev_image, cv::Size(256, 128));

    // swap the channels bgr to rgb
    cv::cvtColor(latest_bev_image, latest_bev_image, cv::COLOR_BGR2RGB);
    auto img_tensor = torch::from_blob(latest_bev_image.data, {1, latest_bev_image.rows, latest_bev_image.cols, 3}, torch::kByte);
    img_tensor = img_tensor.permute({0, 3, 1, 2});
    // convert to float between 0 and 1
    img_tensor = img_tensor.to(torch::kFloat32);
    img_tensor = img_tensor / 255.0;

    // print the context shape
    // std::cout << "Context shape: ";
    // for (auto& size : context_tensor_.sizes()) {
    // std::cout << size << " ";
    // }
    // std::cout << std::endl;
    

    // print the shape
    // std::cout << "Image shape: ";
    // for (auto& size : img_tensor.sizes()) {
    // std::cout << size << " ";
    // }

    // print the min and max
    std::cout << std::endl;
    std::cout << "Image min: " << img_tensor.min().item<float>() << std::endl;
    std::cout << "Image max: " << img_tensor.max().item<float>() << std::endl;


    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(context_tensor_.to(torch_device_));
    inputs.push_back(img_tensor.to(torch_device_));

    torch::NoGradGuard no_grad;

    // Run the model with the inputs
    auto output = cost_model_.forward(inputs).toTensor();

    // apply sigmoid to the output
    output = torch::sigmoid(output).to(torch::kCPU);

    
    // Print the shape of the output
    std::cout << "Output shape: ";
    for (auto& size : output.sizes()) {
    std::cout << size << " ";
    }
    std::cout << std::endl;

    // Print the min and max of the output
    std::cout << "Output min: " << output.min().item<float>() << std::endl;
    std::cout << "Output max: " << output.max().item<float>() << std::endl;

    // Convert the output to a cv::Mat
    cv::Mat1f scalar_cost_map(output.size(2), output.size(3));
    std::memcpy(scalar_cost_map.data, output.data_ptr(), output.numel() * sizeof(float));

    // resize the scalar_cost_map to the original size
    cv::resize(scalar_cost_map, scalar_cost_map, cv::Size(cols, rows));
    // print the new size
    // std::cout << "Scalar cost map size: " << scalar_cost_map.rows << " " << scalar_cost_map.cols << std::endl;

    // todo: out-of-view cost



    // // write the latest_bev_image to a file
    // cv::imwrite("latest_bev_image.png", latest_bev_image);
    // // write the scalar_cost_map to a file
    // cv::imwrite("scalar_cost_map.png", scalar_cost_map*255);

    // return the output
    return scalar_cost_map;

    
  }
};

}  // namespace motion_primitives
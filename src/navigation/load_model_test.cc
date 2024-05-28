#include <torch/script.h> // One-stop header.
#include <opencv2/opencv.hpp>

#include <iostream>
#include <memory>

int main(int argc, const char* argv[]) {
    // load an image
    cv::Mat3b img = cv::imread("13_140.png", cv::IMREAD_COLOR);
    std::cout << "Image rows: " << img.rows << std::endl;
    std::cout << "Image cols: " << img.cols << std::endl;
    // resize width to 256 and height to 128
    cv::resize(img, img, cv::Size(256, 128));
    // print the rows and cols
    std::cout << "Image rows: " << img.rows << std::endl;
    std::cout << "Image cols: " << img.cols << std::endl;
    // bgr to rgb
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    // transpose to 3x128x256
    // img = img.t();
    // reshape to 1x3x128x256
    auto img_tensor = torch::from_blob(img.data, {1, img.rows, img.cols, 3}, torch::kByte);
    img_tensor = img_tensor.permute({0, 3, 1, 2});
    // convert to float between 0 and 1
    img_tensor = img_tensor.to(torch::kFloat32);
    img_tensor = img_tensor / 255.0;

    // print the shape
    std::cout << "Image shape: ";
    for (auto& size : img_tensor.sizes()) {
    std::cout << size << " ";
    }

    // print the min and max
    std::cout << std::endl;
    std::cout << "Image min: " << img_tensor.min().item<float>() << std::endl;
    std::cout << "Image max: " << img_tensor.max().item<float>() << std::endl;

        

  if (argc != 2) {
    std::cerr << "usage: example-app <path-to-exported-script-module>\n";
    return -1;
  }


  torch::jit::script::Module module;
  try {
    // Deserialize the ScriptModule from a file using torch::jit::load().
    module = torch::jit::load(argv[1]);

    std::cout << "Model loaded successfully!" << std::endl;
    torch::NoGradGuard no_grad;

    // Create example_bev tensor
    // auto example_bev = torch::randn({1, 3, 128, 256});

    // Create example_context tensor
    auto example_context = torch::randn({1, 9, 128, 64});

    std::cout << "Example tensors created!" << std::endl;

    // Create a vector of inputs
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(example_context);
    inputs.push_back(img_tensor);

    // auto i = torch::jit::IValue(std::make_tuple(example_context, example_bev));
    // inputs.push_back(i);

    std::cout << "forward inference started!" << std::endl;

    // Get all the methods of the module
    auto methods = module.get_methods();

    // Print all the methods
    for (const auto& method : methods) {
        std::cout << "Method: " << method.name() << std::endl;
    }
    // Run the model with the inputs
    auto output = module.forward(inputs).toTensor();

    // apply sigmoid to the output
    output = torch::sigmoid(output);

    // Print the shape of the output
    std::cout << "Output shape: ";
    for (auto& size : output.sizes()) {
    std::cout << size << " ";
    }
    std::cout << std::endl;

    // Print the min and max of the output
    std::cout << "Output min: " << output.min().item<float>() << std::endl;
    std::cout << "Output max: " << output.max().item<float>() << std::endl;

    // write the output to a grayscale image
    cv::Mat1f output_mat(output.size(2), output.size(3), output.data_ptr<float>());
    // save to disk
    cv::imwrite("output.png", output_mat * 255);
    



  }
  catch (const c10::Error& e) {
    std::cerr << "an error occurred\n";
    // print the error
    std::cerr << e.what();
    return -1;
  }

  std::cout << "ok\n";
}
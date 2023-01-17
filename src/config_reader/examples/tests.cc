// Copyright 2020 Kyle Vedder (kvedder@seas.upenn.edu)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// ========================================================================
#include <csignal>
#include <iostream>

#include "config_reader/config_reader.h"

void Check(const bool statement) {
  if (!statement) {
    exit(1);
  }
}

int MyFunction() {
  CONFIG_INT(twelve, "twelve");
  config_reader::ConfigReader reader({"test_config2.lua"});
  return CONFIG_twelve;
}

int main() {
  CONFIG_INT(seven, "seven");
  CONFIG_STRING(str, "str");
  CONFIG_FLOAT(seven_point_five, "seven_point_five");
  CONFIG_INTLIST(int_list, "int_list");
  CONFIG_DOUBLELIST(double_list, "double_list");
  CONFIG_BOOLLIST(bool_list, "bool_list");
  CONFIG_VECTOR2F(sample_vector2f, "sample_vector2f");
  CONFIG_VECTOR2FLIST(sample_vector2f_list, "sample_vector2f_list");
  CONFIG_VECTOR2FLIST(wrapped_sample_vector2f_list, "wrapper.another.sample_vector2f_list");
  config_reader::ConfigReader reader({"test_config.lua"});

  Check(CONFIG_int_list.size() == 16);
  int sum_i = 0;
  for (const int& i : CONFIG_int_list) {
    sum_i += i;
  }
  Check(sum_i == 224);

  Check(CONFIG_double_list.size() == 2);
  double sum_d = 0;
  for (const double& i : CONFIG_double_list) {
    sum_d += i;
  }
  Check(sum_d == 4.554);

  Check(CONFIG_bool_list.size() == 2);
  Check(CONFIG_bool_list[0] == true);
  Check(CONFIG_bool_list[1] == false);

  Check(CONFIG_sample_vector2f == Eigen::Vector2f(1.2, 3.4));

  Check(CONFIG_sample_vector2f_list.size() == 2);
  Check(CONFIG_sample_vector2f_list[0] == Eigen::Vector2f(1.2, 3.4));
  Check(CONFIG_sample_vector2f_list[1] == Eigen::Vector2f(5.6, 7.8));

  Check(CONFIG_wrapped_sample_vector2f_list.size() == 2);
  Check(CONFIG_wrapped_sample_vector2f_list[0] == Eigen::Vector2f(9.1, 2.3));
  Check(CONFIG_wrapped_sample_vector2f_list[1] == Eigen::Vector2f(4.5, 6.7));


  Check(CONFIG_seven == 7);
  Check(CONFIG_str == "str");
  Check(std::abs(CONFIG_seven_point_five - 7.5) < 0.0001f);
  Check(MyFunction() == 12);
  Check(CONFIG_seven == 7);
  Check(CONFIG_str == "str");
  Check(std::abs(CONFIG_seven_point_five - 7.5) < 0.0001f);
  std::cout << "All tests passed!\n";
  return 0;
}

// Copyright 2019 Kyle Vedder (kvedder@seas.upenn.edu), 2018 Ishan Khatri
// (ikhatri@umass.edu)
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
#ifndef CONFIGREADER_TYPES_CONFIG_NUMERIC_H_
#define CONFIGREADER_TYPES_CONFIG_NUMERIC_H_

#include "config_reader/types/type_interface.h"

namespace config_reader {
namespace config_types {

#define NUMERIC_CLASS(ClassName, EnumName, CPPType)                     \
  class ClassName : public TypeInterface {                              \
   public:                                                              \
    ClassName(const std::string& key)                                   \
        : TypeInterface(key, Type::EnumName),                           \
          upper_bound_(std::numeric_limits<CPPType>::max()),            \
          lower_bound_(std::numeric_limits<CPPType>::lowest()),         \
          val_(0) {}                                                    \
                                                                        \
    ClassName(const std::string& key, const int& upper_bound,           \
              const int& lower_bound)                                   \
        : TypeInterface(key, Type::EnumName),                           \
          upper_bound_(upper_bound),                                    \
          lower_bound_(lower_bound),                                    \
          val_(0) {                                                     \
      if (upper_bound_ < lower_bound_) {                                \
        std::cerr << #ClassName << " upperbound " << upper_bound_       \
                  << " below lowerbound " << lower_bound_ << std::endl; \
      }                                                                 \
    }                                                                   \
                                                                        \
    ClassName() = delete;                                               \
    ~ClassName() = default;                                             \
                                                                        \
    void SetValue(LuaScript* lua_script) override {                     \
      const CPPType value = lua_script->GetVariable<CPPType>(key_);     \
      if (value < lower_bound_ || value > upper_bound_) {               \
        std::cerr << #ClassName << " Value " << value                   \
                  << " outside bounds; upperbound " << upper_bound_     \
                  << " below lowerbound " << lower_bound_ << std::endl; \
        return;                                                         \
      }                                                                 \
      val_ = value;                                                     \
    }                                                                   \
                                                                        \
    const CPPType& GetValue() { return this->val_; }                    \
                                                                        \
    static Type GetEnumType() { return Type::EnumName; }                \
                                                                        \
   private:                                                             \
    CPPType upper_bound_;                                               \
    CPPType lower_bound_;                                               \
    CPPType val_;                                                       \
  };

NUMERIC_CLASS(ConfigInt, CINT, int);
NUMERIC_CLASS(ConfigUnsignedInt, CUINT, unsigned int);
NUMERIC_CLASS(ConfigFloat, CFLOAT, float);
NUMERIC_CLASS(ConfigDouble, CDOUBLE, double);

}  // namespace config_types
}  // namespace config_reader
#endif  // CONFIGREADER_TYPES_CONFIG_FLOAT_H_
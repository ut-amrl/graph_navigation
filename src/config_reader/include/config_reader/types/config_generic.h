// Copyright 2019 - 2020 Kyle Vedder (kvedder@seas.upenn.edu),
// 2018 Ishan Khatri (ikhatri@umass.edu)
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
#ifndef CONFIGREADER_TYPES_CONFIG_GENERIC_H_
#define CONFIGREADER_TYPES_CONFIG_GENERIC_H_

#include "config_reader/types/type_interface.h"

#include <eigen3/Eigen/Core>

#define GENERIC_CLASS(ClassName, EnumName, CPPType, DefaultValue)   \
  namespace config_reader {                                         \
  namespace config_types {                                          \
  class ClassName : public TypeInterface {                          \
   public:                                                          \
    ClassName(const std::string& key)                               \
        : TypeInterface(key, Type::EnumName), val_(DefaultValue) {} \
                                                                    \
    ClassName() = delete;                                           \
    ~ClassName() = default;                                         \
                                                                    \
    void SetValue(LuaScript* lua_script) override {                 \
      const auto result =                                           \
          lua_script->GetVariable<CPPType>(key_, var_locations_);   \
      if (!result.first) {                                          \
        return;                                                     \
      }                                                             \
      val_ = result.second;                                         \
    }                                                               \
                                                                    \
    const CPPType& GetValue() { return this->val_; }                \
                                                                    \
    static Type GetEnumType() { return Type::EnumName; }            \
                                                                    \
    static CPPType GetDefaultValue() { return DefaultValue; }       \
                                                                    \
   private:                                                         \
    CPPType val_;                                                   \
  };                                                                \
  }                                                                 \
  template <>                                                       \
  inline CPPType GetDefaultValue<CPPType>() {                       \
    return DefaultValue;                                            \
  }                                                                 \
  }

GENERIC_CLASS(ConfigString, CSTRING, std::string, "");
GENERIC_CLASS(ConfigBool, CBOOL, bool, false);
GENERIC_CLASS(ConfigIntList, CINTLIST, std::vector<int>, {});
GENERIC_CLASS(ConfigUnsignedIntList, CUINTLIST, std::vector<unsigned int>, {});
GENERIC_CLASS(ConfigFloatList, CFLOATLIST, std::vector<float>, {});
GENERIC_CLASS(ConfigDoubleList, CDOUBLELIST, std::vector<double>, {});
GENERIC_CLASS(ConfigStringList, CSTRINGLIST, std::vector<std::string>, {});
GENERIC_CLASS(ConfigBoolList, CBOOLLIST, std::vector<bool>, {});
GENERIC_CLASS(ConfigVector2f, CVECTOR2F, Eigen::Vector2f,
              Eigen::Vector2f::Zero());
GENERIC_CLASS(ConfigVector3f, CVECTOR3F, Eigen::Vector3f,
              Eigen::Vector3f::Zero());
GENERIC_CLASS(ConfigVector2fList, CVECTOR2FLIST, std::vector<Eigen::Vector2f>,
              {});
GENERIC_CLASS(ConfigVector3fList, CVECTOR3FLIST, std::vector<Eigen::Vector3f>,
              {});

#endif  // CONFIGREADER_TYPES_CONFIG_FLOAT_H_
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
#ifndef CONFIGREADER_TYPES_CONFIG_GENERIC_H_
#define CONFIGREADER_TYPES_CONFIG_GENERIC_H_

#include "config_reader/types/type_interface.h"

namespace config_reader {
namespace config_types {

#define GENERIC_CLASS(ClassName, EnumName, CPPType, DefaultValue)   \
  class ClassName : public TypeInterface {                          \
   public:                                                          \
    ClassName(const std::string& key)                               \
        : TypeInterface(key, Type::EnumName), val_(DefaultValue) {} \
                                                                    \
    ClassName() = delete;                                           \
    ~ClassName() = default;                                         \
                                                                    \
    void SetValue(LuaScript* lua_script) override {                 \
      val_ = lua_script->GetVariable<CPPType>(key_);                \
    }                                                               \
                                                                    \
    const CPPType& GetValue() { return this->val_; }                \
                                                                    \
    static Type GetEnumType() { return Type::EnumName; }            \
                                                                    \
   private:                                                         \
    CPPType val_;                                                   \
  };

GENERIC_CLASS(ConfigString, CSTRING, std::string, "");
GENERIC_CLASS(ConfigBool, CBOOL, bool, false);
}  // namespace config_types
}  // namespace config_reader
#endif  // CONFIGREADER_TYPES_CONFIG_FLOAT_H_
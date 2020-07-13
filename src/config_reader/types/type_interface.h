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
#ifndef CONFIGREADER_TYPES_TYPE_INTERFACE_H_
#define CONFIGREADER_TYPES_TYPE_INTERFACE_H_

#include <iostream>
#include <string>

#include "config_reader/lua_script.h"

namespace config_reader {
namespace config_types {

enum Type {
  CNULL,
  CINT,
  CUINT,
  CDOUBLE,
  CFLOAT,
  CSTRING,
  CBOOL,
};

class TypeInterface {
 public:
  TypeInterface() = delete;
  TypeInterface(const std::string& key, const Type& type)
      : key_(key), type_(type) {}
  virtual ~TypeInterface() {}
  std::string GetKey() const { return key_; };
  Type GetType() const { return type_; };
  virtual void SetValue(LuaScript* lua_script) = 0;

 protected:
  std::string key_;
  Type type_;
};
}  // namespace config_types
}  // namespace config_reader
#endif  // CONFIGREADER_TYPES_TYPE_INTERFACE_H_
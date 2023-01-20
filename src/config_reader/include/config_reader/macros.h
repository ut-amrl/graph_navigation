// Copyright 2019 - 2020 Kyle Vedder (kvedder@seas.upenn.edu)
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

#ifndef CONFIGREADER_MACROS_H_
#define CONFIGREADER_MACROS_H_

#include <atomic>
#include <memory>
#include <unordered_map>
#include <vector>

#include "config_reader/types/config_generic.h"
#include "config_reader/types/config_numeric.h"
#include "config_reader/types/type_interface.h"

namespace config_reader {

// Unfortunately these layered macros are needed to get __LINE__ to appear
// properly.
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define LOCATION (__FILE__ ":" TOSTRING(__LINE__))

#define MAKE_NAME(name) CONFIG_##name

#define MAKE_MACRO(name, key, cpptype, configtype)                         \
  static const cpptype& MAKE_NAME(name) =                                  \
      ::config_reader::InitVar<cpptype,                                    \
                               ::config_reader::config_types::configtype>( \
          key, LOCATION)

// Define macros for creating new config vars
// clang-format off
#define CONFIG_INT(name, key) MAKE_MACRO(name, key, int, ConfigInt)
#define CONFIG_UINT(name, key) MAKE_MACRO(name, key, unsigned int, ConfigUnsignedInt)
#define CONFIG_DOUBLE(name, key) MAKE_MACRO(name, key, double, ConfigDouble)
#define CONFIG_FLOAT(name, key) MAKE_MACRO(name, key, float, ConfigFloat)
#define CONFIG_STRING(name, key) MAKE_MACRO(name, key, std::string, ConfigString)
#define CONFIG_BOOL(name, key)  MAKE_MACRO(name, key, bool, ConfigBool)
#define CONFIG_INTLIST(name, key)  MAKE_MACRO(name, key, std::vector<int>, ConfigIntList)
#define CONFIG_UINTLIST(name, key) MAKE_MACRO(name, key, std::vector<unsigned int>, ConfigUnsignedIntList)
#define CONFIG_FLOATLIST(name, key) MAKE_MACRO(name, key, std::vector<float>, ConfigFloatList)
#define CONFIG_DOUBLELIST(name, key) MAKE_MACRO(name, key, std::vector<double>, ConfigDoubleList)
#define CONFIG_STRINGLIST(name, key) MAKE_MACRO(name, key, std::vector<std::string>, ConfigStringList)
#define CONFIG_BOOLLIST(name, key) MAKE_MACRO(name, key, std::vector<bool>, ConfigBoolList)
#define CONFIG_VECTOR2F(name, key) MAKE_MACRO(name, key, Eigen::Vector2f, ConfigVector2f)
#define CONFIG_VECTOR3F(name, key) MAKE_MACRO(name, key, Eigen::Vector3f, ConfigVector3f)
#define CONFIG_VECTOR2FLIST(name, key) MAKE_MACRO(name, key, std::vector<Eigen::Vector2f>, ConfigVector2fList)
#define CONFIG_VECTOR3FLIST(name, key) MAKE_MACRO(name, key, std::vector<Eigen::Vector3f>, ConfigVector3fList)
// clang-format on

class MapSingleton {
  static constexpr int kNumMapBuckets = 1000000;

 public:
  using KeyLookupMap =
      std::unordered_map<std::string,
                         std::unique_ptr<config_types::TypeInterface>>;

  static KeyLookupMap& Singleton() {
    static KeyLookupMap config(kNumMapBuckets);
    return config;
  }

  static std::atomic_bool* NewKeyAdded() {
    static std::atomic_bool new_key_added(false);
    return &new_key_added;
  }

  static std::atomic_bool* ConfigInitialized() {
    static std::atomic_bool config_initialized(false);
    return &config_initialized;
  }
};

template <typename CPPType, typename ConfigType>
const CPPType& InitVar(const std::string& key,
                       const std::string& var_location) {
  auto& map = MapSingleton::Singleton();
  auto find_res = map.find(key);
  if (find_res != map.end()) {
    config_types::TypeInterface* ti = find_res->second.get();
    if (ti->GetType() != ConfigType::GetEnumType()) {
      std::cerr << "Mismatch of types for key " << key
                << ". Existing type: " << ti->GetType()
                << ", requested type: " << ConfigType::GetEnumType()
                << std::endl;
      exit(1);
    }
    ti->AddVarLocation(var_location);
    return static_cast<ConfigType*>(ti)->GetValue();
  }
  auto insert_res = map.insert(std::make_pair(
      key, std::unique_ptr<config_types::TypeInterface>(new ConfigType(key))));
  if (!insert_res.second) {
    std::cerr << "Creation of " << key << " failed!" << std::endl;
    exit(1);
  }
  *MapSingleton::NewKeyAdded() = true;
  config_types::TypeInterface* ti = insert_res.first->second.get();
  ti->AddVarLocation(var_location);
  return static_cast<ConfigType*>(ti)->GetValue();
}
}  // namespace config_reader

#endif  // CONFIGREADER_MACROS_H_

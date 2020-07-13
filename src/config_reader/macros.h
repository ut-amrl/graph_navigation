// Copyright 2019 Kyle Vedder (kvedder@seas.upenn.edu)
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
#define MAKE_NAME(name) CONFIG_ ## name

// Define macros for creating new config vars

#define CONFIG_INT(name, key)                                                  \
  const int& MAKE_NAME(name) =                                                 \
      ::config_reader::InitVar<int, ::config_reader::config_types::ConfigInt>( \
          key)
#define CONFIG_UINT(name, key)                                    \
  const unsigned int& MAKE_NAME(name) = ::config_reader::InitVar< \
      unsigned int, ::config_reader::config_types::ConfigUnsignedInt>(key)
#define CONFIG_DOUBLE(name, key)                            \
  const double& MAKE_NAME(name) = ::config_reader::InitVar< \
      double, ::config_reader::config_types::ConfigDouble>(key)
#define CONFIG_FLOAT(name, key)                            \
  const float& MAKE_NAME(name) = ::config_reader::InitVar< \
      float, ::config_reader::config_types::ConfigFloat>(key)
#define CONFIG_STRING(name, key)                                 \
  const std::string& MAKE_NAME(name) = ::config_reader::InitVar< \
      std::string, ::config_reader::config_types::ConfigString>(key)
#define CONFIG_BOOL(name, key)       \
  const bool& MAKE_NAME(name) =      \
      ::config_reader::InitVar<bool, \
                               ::config_reader::config_types::ConfigBool>(key)

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
};

template <typename CPPType, typename ConfigType>
const CPPType& InitVar(const std::string& key) {
  auto& map = MapSingleton::Singleton();
  auto find_res = map.find(key);
  if (find_res != map.end()) {
    config_types::TypeInterface* ti = find_res->second.get();
    if (ti->GetType() != ConfigType::GetEnumType()) {
      std::cerr << "Mismatch of types for key " << key
                << ". Existing type: " << ti->GetType()
                << ", requested type: " << ConfigType::GetEnumType()
                << std::endl;
      exit(0);
    }
    return static_cast<ConfigType*>(ti)->GetValue();
  }
  auto insert_res = map.insert(
      {key, std::unique_ptr<config_types::TypeInterface>(new ConfigType(key))});
  if (!insert_res.second) {
    std::cerr << "Creation of " << key << " failed!" << std::endl;
    exit(0);
  }
  *MapSingleton::NewKeyAdded() = true;
  config_types::TypeInterface* ti = insert_res.first->second.get();
  return static_cast<ConfigType*>(ti)->GetValue();
}
}  // namespace config_reader

#endif  // CONFIGREADER_MACROS_H_

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
#ifndef CONFIGREADER_LUA_SCRIPT_H_
#define CONFIGREADER_LUA_SCRIPT_H_

#include <iostream>
#include <string>
#include <vector>

extern "C" {
#include "lua5.1/lauxlib.h"
#include "lua5.1/lua.h"
#include "lua5.1/lualib.h"
}

namespace config_reader {
class LuaScript {
  lua_State* lua_state_;

  void ResetStack() { lua_pop(lua_state_, lua_gettop(lua_state_)); }

  void Error(const std::string& variable_name,
             const std::string& reason) const {
    std::cerr << "Error: can't get [" << variable_name << "]. " << reason
              << std::endl;
  }

  bool LoadStackLocation(const std::string& variable_name) {
    int level = 0;
    std::string var = "";
    for (size_t i = 0; i < variable_name.size(); i++) {
      if (variable_name.at(i) == '.') {
        if (level == 0) {
          lua_getglobal(lua_state_, var.c_str());
        } else {
          lua_getfield(lua_state_, -1, var.c_str());
        }

        if (lua_isnil(lua_state_, -1)) {
          Error(variable_name, var + " is not defined");
          return false;
        } else {
          var = "";
          level++;
        }
      } else {
        var += variable_name.at(i);
      }
    }
    if (level == 0) {
      lua_getglobal(lua_state_, var.c_str());
    } else {
      lua_getfield(lua_state_, -1, var.c_str());
    }
    if (lua_isnil(lua_state_, -1)) {
      Error(variable_name, var + " is not defined");
      return false;
    }

    return true;
  }

  // Generic get
  template <typename T>
  T Get(__attribute__((unused)) const std::string& variable_name) {
    return 0;
  }

  template <typename T>
  T GetDefault() {
    return 0;
  }

  void CleanupLuaState() {
    if (lua_state_) {
      lua_close(lua_state_);
      lua_state_ = nullptr;
    }
  }

 public:
  LuaScript() : lua_state_(nullptr) {}

  explicit LuaScript(const std::vector<std::string>& files)
      : lua_state_(luaL_newstate()) {
    luaL_openlibs(lua_state_);
    for (const std::string& filename : files) {
      if (luaL_loadfile(lua_state_, filename.c_str()) ||
          lua_pcall(lua_state_, 0, 0, 0)) {
        std::cout << "Error: failed to load (" << filename << ")" << std::endl;
        std::cout << "Error Message: " << lua_tostring(lua_state_, -1)
                  << std::endl;
        CleanupLuaState();
        break;
      }
    }
  }

  ~LuaScript() { CleanupLuaState(); }

  template <typename T>
  T GetVariable(const std::string& variable_name) {
    if (lua_state_ == nullptr) {
      Error(variable_name, "Script is not loaded");
      return GetDefault<T>();
    }

    if (!LoadStackLocation(variable_name)) {
      Error(variable_name, "Cannot load stack location");
      ResetStack();
      return GetDefault<T>();
    }

    const T result = Get<T>(variable_name);
    ResetStack();
    return result;
  }
};

template <>
inline bool LuaScript::GetDefault<bool>() {
  return false;
}

template <>
inline bool LuaScript::Get<bool>(const std::string& variable_name) {
  if (!lua_isboolean(lua_state_, -1)) {
    Error(variable_name, "Not a boolean");
    return GetDefault<bool>();
  }
  return static_cast<bool>(lua_toboolean(lua_state_, -1));
}

template <>
inline float LuaScript::Get<float>(const std::string& variable_name) {
  if (!lua_isnumber(lua_state_, -1)) {
    Error(variable_name, "Not a number");
    return GetDefault<float>();
  }
  return static_cast<float>(lua_tonumber(lua_state_, -1));
}

template <>
inline int LuaScript::Get<int>(const std::string& variable_name) {
  if (!lua_isnumber(lua_state_, -1)) {
    Error(variable_name, "Not a number");
    return GetDefault<int>();
  }
  return static_cast<int>(lua_tonumber(lua_state_, -1));
}

template <>
inline unsigned int LuaScript::Get<unsigned int>(
    const std::string& variable_name) {
  if (!lua_isnumber(lua_state_, -1)) {
    Error(variable_name, "Not a number");
    return GetDefault<unsigned int>();
  }
  return static_cast<unsigned int>(lua_tonumber(lua_state_, -1));
}

template <>
inline double LuaScript::Get<double>(const std::string& variable_name) {
  if (!lua_isnumber(lua_state_, -1)) {
    Error(variable_name, "Not a number");
    return GetDefault<double>();
  }
  return static_cast<double>(lua_tonumber(lua_state_, -1));
}

template <>
inline std::string LuaScript::GetDefault<std::string>() {
  return "null";
}

template <>
inline std::string LuaScript::Get<std::string>(
    const std::string& variable_name) {
  if (!lua_isstring(lua_state_, -1)) {
    Error(variable_name, "Not a string");
    return GetDefault<std::string>();
  }
  return std::string(lua_tostring(lua_state_, -1));
}

}  // namespace config_reader

#endif  // CONFIGREADER_LUA_SCRIPT_H_

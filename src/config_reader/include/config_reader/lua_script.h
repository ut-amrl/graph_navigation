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
#ifndef CONFIGREADER_LUA_SCRIPT_H_
#define CONFIGREADER_LUA_SCRIPT_H_

#include <eigen3/Eigen/Core>
#include <iostream>
#include <string>
#include <vector>

extern "C" {
#include "lua5.1/lauxlib.h"
#include "lua5.1/lua.h"
#include "lua5.1/lualib.h"
}

static constexpr bool kDisableTopLevelMissingError = true;

namespace config_reader {

namespace util {
inline void StackDump(lua_State* L) {
  int top = lua_gettop(L);
  for (int i = 1; i <= top; i++) { /* repeat for each level */
    int t = lua_type(L, i);
    switch (t) {
      case LUA_TSTRING: /* strings */
        printf("`%s'", lua_tostring(L, i));
        break;

      case LUA_TBOOLEAN: /* booleans */
        printf(lua_toboolean(L, i) ? "true" : "false");
        break;

      case LUA_TNUMBER: /* numbers */
        printf("%g", lua_tonumber(L, i));
        break;

      default: /* other values */
        printf("%s", lua_typename(L, t));
        break;
    }
    printf(" <-> "); /* put a separator */
  }
  printf("\n"); /* end the listing */
}
}  // namespace util

template <typename T>
inline T GetDefaultValue();

class LuaScript {
  lua_State* lua_state_;

  void ResetStack() { lua_pop(lua_state_, lua_gettop(lua_state_)); }

  void Error(const std::string& variable_name, const std::string& reason,
             const std::vector<std::string>& var_locations) const {
    for (const auto& l : var_locations) {
      std::cerr << l << ": Can't get [" << variable_name << "]. " << reason
                << std::endl;
    }
  }

  void Error(const std::string& variable_name,
             const std::string& reason) const {
    std::cerr << "Error: can't get [" << variable_name << "]. " << reason
              << std::endl;
  }

  bool LoadStackLocation(const std::string& variable_name,
                         const std::vector<std::string>& var_locations) {
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
          if (!kDisableTopLevelMissingError || level > 0) {
            Error(variable_name, var + " is not defined", var_locations);
          }
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
      if (!kDisableTopLevelMissingError || level > 0) {
        Error(variable_name, var + " is not defined", var_locations);
      }
      return false;
    }

    return true;
  }

  template <typename T>
  T Get(const std::string& variable_name);

  template <typename T>
  T GetDefault() {
    return GetDefaultValue<T>();
  };

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
  std::pair<bool, T> GetVariable(
      const std::string& variable_name,
      const std::vector<std::string>& var_locations) {
    if (lua_state_ == nullptr) {
      Error(variable_name, "Script is not loaded", var_locations);
      return {false, GetDefault<T>()};
    }

    if (!LoadStackLocation(variable_name, var_locations)) {
      ResetStack();
      return {false, GetDefault<T>()};
    }

    const T result = Get<T>(variable_name);
    ResetStack();
    return {true, result};
  }
};

#define GET_NUMBER(Type)                                               \
  template <>                                                          \
  inline Type LuaScript::Get<Type>(const std::string& variable_name) { \
    if (!lua_isnumber(lua_state_, -1)) {                               \
      Error(variable_name, "Not a number");                            \
      return GetDefault<Type>();                                       \
    }                                                                  \
    return static_cast<Type>(lua_tonumber(lua_state_, -1));            \
  }

#define GET_NUMBER_LIST(Type)                                              \
  template <>                                                              \
  inline std::vector<Type> LuaScript::Get<std::vector<Type>>(              \
      const std::string& variable_name) {                                  \
    if (!lua_istable(lua_state_, -1)) {                                    \
      Error(variable_name, "Not a std::vector<Type>");                     \
      return GetDefault<std::vector<Type>>();                              \
    }                                                                      \
    const int table_length = static_cast<int>(lua_objlen(lua_state_, -1)); \
    std::vector<Type> data;                                                \
    for (int i = 1; i <= table_length; ++i) {                              \
      lua_pushinteger(lua_state_, i);                                      \
      lua_gettable(lua_state_, -2);                                        \
      if (!lua_isnumber(lua_state_, -1)) {                                 \
        Error(variable_name, "Element not a Type");                        \
        return GetDefault<std::vector<Type>>();                            \
      }                                                                    \
      data.push_back(static_cast<Type>(lua_tonumber(lua_state_, -1)));     \
      lua_pop(lua_state_, 1);                                              \
    }                                                                      \
    return data;                                                           \
  }

#define GET_OBJECT_LIST(Type)                                              \
  template <>                                                              \
  inline std::vector<Type> LuaScript::Get<std::vector<Type>>(              \
      const std::string& variable_name) {                                  \
    if (!lua_istable(lua_state_, -1)) {                                    \
      Error(variable_name, "Not a std::vector<Type>");                     \
      return GetDefault<std::vector<Type>>();                              \
    }                                                                      \
    const int table_length = static_cast<int>(lua_objlen(lua_state_, -1)); \
    std::vector<Type> data;                                                \
    for (int i = 1; i <= table_length; ++i) {                              \
      lua_pushinteger(lua_state_, i);                                      \
      lua_gettable(lua_state_, -2);                                        \
      data.push_back(Get<Type>(variable_name + " element"));               \
      lua_pop(lua_state_, 1);                                              \
    }                                                                      \
    return data;                                                           \
  }

GET_NUMBER(float);

GET_NUMBER(int);

GET_NUMBER(unsigned int);

GET_NUMBER(double);

template <>
inline bool LuaScript::Get<bool>(const std::string& variable_name) {
  if (!lua_isboolean(lua_state_, -1)) {
    Error(variable_name, "Not a boolean");
    return GetDefault<bool>();
  }
  return static_cast<bool>(lua_toboolean(lua_state_, -1));
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

GET_NUMBER_LIST(int);

GET_NUMBER_LIST(unsigned int);

GET_NUMBER_LIST(float);

GET_NUMBER_LIST(double);

template <>
inline std::vector<std::string> LuaScript::Get<std::vector<std::string>>(
    const std::string& variable_name) {
  if (!lua_istable(lua_state_, -1)) {
    Error(variable_name, "Not a std::vector<std::string>");
    return GetDefault<std::vector<std::string>>();
  }
  const int table_length = static_cast<int>(lua_objlen(lua_state_, -1));
  std::vector<std::string> data;
  for (int i = 1; i <= table_length; ++i) {
    lua_pushinteger(lua_state_, i);
    lua_gettable(lua_state_, -2);
    if (!lua_isstring(lua_state_, -1)) {
      Error(variable_name, "Element not a string");
      return GetDefault<std::vector<std::string>>();
    }
    data.push_back(std::string(lua_tostring(lua_state_, -1)));
    lua_pop(lua_state_, 1);
  }
  return data;
}

template <>
inline std::vector<bool> LuaScript::Get<std::vector<bool>>(
    const std::string& variable_name) {
  if (!lua_istable(lua_state_, -1)) {
    Error(variable_name, "Not a std::vector<bool>");
    return GetDefault<std::vector<bool>>();
  }
  const int table_length = static_cast<int>(lua_objlen(lua_state_, -1));
  std::vector<bool> data;
  for (int i = 1; i <= table_length; ++i) {
    lua_pushinteger(lua_state_, i);
    lua_gettable(lua_state_, -2);
    if (!lua_isboolean(lua_state_, -1)) {
      Error(variable_name, "Element not a bool");
      return GetDefault<std::vector<bool>>();
    }
    data.push_back(lua_toboolean(lua_state_, -1));
    lua_pop(lua_state_, 1);
  }
  return data;
}

template <>
inline Eigen::Vector2f LuaScript::Get<Eigen::Vector2f>(
    const std::string& variable_name) {
  if (!lua_istable(lua_state_, -1)) {
    Error(variable_name, "Not a std::vector<bool>");
    return GetDefault<Eigen::Vector2f>();
  }
  const int table_length = static_cast<int>(lua_objlen(lua_state_, -1));
  if (table_length != 2) {
    Error(variable_name, "Wrong number of entries for Vector2f (" +
                             std::to_string(table_length) + ")");
    return GetDefault<Eigen::Vector2f>();
  }
  Eigen::Vector2f data = Eigen::Vector2f::Zero();
  for (int i = 1; i <= 2; ++i) {
    lua_pushinteger(lua_state_, i);
    lua_gettable(lua_state_, -2);
    if (!lua_isnumber(lua_state_, -1)) {
      Error(variable_name, "Element not a number");
      return GetDefault<Eigen::Vector2f>();
    }
    data(i - 1) = static_cast<float>(lua_tonumber(lua_state_, -1));
    lua_pop(lua_state_, 1);
  }
  return data;
}

template <>
inline Eigen::Vector3f LuaScript::Get<Eigen::Vector3f>(
    const std::string& variable_name) {
  if (!lua_istable(lua_state_, -1)) {
    Error(variable_name, "Not a std::vector<bool>");
    return GetDefault<Eigen::Vector3f>();
  }
  const int table_length = static_cast<int>(lua_objlen(lua_state_, -1));
  if (table_length != 3) {
    Error(variable_name, "Wrong number of entries for Vector3f (" +
                             std::to_string(table_length) + ")");
    return GetDefault<Eigen::Vector3f>();
  }
  Eigen::Vector3f data = Eigen::Vector3f::Zero();
  for (int i = 1; i <= 3; ++i) {
    lua_pushinteger(lua_state_, i);
    lua_gettable(lua_state_, -2);
    if (!lua_isnumber(lua_state_, -1)) {
      Error(variable_name, "Element not a number");
      return GetDefault<Eigen::Vector3f>();
    }
    data(i - 1) = static_cast<float>(lua_tonumber(lua_state_, -1));
    lua_pop(lua_state_, 1);
  }
  return data;
}

GET_OBJECT_LIST(Eigen::Vector2f);

GET_OBJECT_LIST(Eigen::Vector3f);

}  // namespace config_reader

#endif  // CONFIGREADER_LUA_SCRIPT_H_

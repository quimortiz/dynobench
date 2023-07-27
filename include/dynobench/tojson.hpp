// ORIGINAL CODE:
// https://github.com/mircodezorzi/tojson

#pragma once

#include "nlohmann/json.hpp"
#include <iostream>
#include <yaml-cpp/yaml.h>

#include <fstream>

/* Adding declarations to make it compatible with gcc 4.7 and greater */
#if __GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__ > 40700
#endif

#if __has_cpp_attribute(nodiscard)
#define TOJSON_NODISCARD [[nodiscard]]
#else
#define TOJSON_NODISCARD
#endif

namespace tojson {
namespace detail {

/// \todo refactor and pass nlohmann::json down by reference instead of
/// returning it

inline nlohmann::json parse_scalar(const YAML::Node &node) {
  int i;
  double d;
  bool b;
  std::string s;

  if (YAML::convert<int>::decode(node, i))
    return i;
  if (YAML::convert<double>::decode(node, d))
    return d;
  if (YAML::convert<bool>::decode(node, b))
    return b;
  if (YAML::convert<std::string>::decode(node, s))
    return s;

  return nullptr;
}

/// \todo refactor and pass nlohmann::json down by reference instead of
/// returning it
inline nlohmann::json yaml2json(const YAML::Node &root) {
  nlohmann::json j{};

  switch (root.Type()) {
  case YAML::NodeType::Null:
    break;
  case YAML::NodeType::Scalar:
    return parse_scalar(root);
  case YAML::NodeType::Sequence:
    for (auto &&node : root)
      j.emplace_back(yaml2json(node));
    break;
  case YAML::NodeType::Map:
    for (auto &&it : root)
      j[it.first.as<std::string>()] = yaml2json(it.second);
    break;
  default:
    break;
  }
  return j;
}

/// \todo handle @text entries better

inline void __toyaml(const nlohmann::json &j, YAML::Emitter &e,
                     bool is_sequence) {
  for (auto &[key, val] : j.items()) {
    if (!is_sequence)
      e << YAML::Key << key;
    e << YAML::Value;
    if (val.is_object()) {
      e << YAML::BeginMap;
      __toyaml(val, e, false);
      e << YAML::EndMap;
    } else if (val.is_array()) {
      e << YAML::Flow << YAML::BeginSeq;
      __toyaml(val, e, true);
      e << YAML::EndSeq;
    } else {
      if (val.is_primitive()) {
        if (val.is_number_float()) {
          e << val.get<double>();
        } else if (val.is_number_integer()) {
          e << val.get<int>();
        } else if (val.is_number_unsigned()) {
          e << val.get<unsigned>();
        } else if (val.is_boolean()) {
          e << val.get<bool>();
        } else if (val.is_string()) {
          e << val.get<std::string>();
        }
      }
    }
    std::cout << "key: " << key << ", value:" << val << '\n';
  }

  // for (auto it = j.begin(); it != j.end(); ++it) {
  //   std::cout << it->dump() << std::endl;
  //   if (it->is_object()) {
  //     std::cout << "is_object" << std::endl;
  //     std::cout << it.key() << std::endl;
  //     e << YAML::Key << it.key() << YAML::Value << YAML::BeginMap;
  //     toyaml(*it, e);
  //     e << YAML::EndMap;
  //   } else if (it->is_array()) {
  //     std::cout << "is_array" << std::endl;
  //     std::cout << it.key() << std::endl;
  //     e << YAML::Key << it.key() << YAML::Value << YAML::Flow <<
  //     YAML::BeginSeq; toyaml(it.value(), e); e << YAML::EndSeq;
  //   } else {
  //     std::cout << "no object, no array" << std::endl;
  //     if (it->is_primitive()) {
  //       std::cout << "is_primitive" << std::endl;
  //       if (it->is_number_float()) {
  //         e << it.value().get<double>();
  //       }
  //     } else {
  //       std::cout << "not primitves" << std::endl;
  //       e << YAML::Key << it.key() << YAML::Value;
  //       if (it->is_string()) {
  //         e << it.value().get<std::string>();
  //       } else {
  //         e << it.value().get<double>();
  //       }
  //     }
  //
  //     // if (it->is_()) {
  //     //   e << it.value().get<double>();
  //     // }
  //
  //     // std::cout << "dumping " << e << it->dump();
  //     // if (false && it.key() == "@text") {
  //     //   e << YAML::Value << it.value().get<std::string>();
  //     // } else {
  //     //   e << YAML::Key << it.key() << YAML::Value;
  //     //   if (it->type() == nlohmann::json::value_t::string)
  //     //     e << it.value().get<std::string>();
  //     //   else
  //     //     e << it->dump();
  //     // }
  //   }
  // }
}

inline void toyaml(const nlohmann::json &j, YAML::Emitter &e) {
  if (j.is_object())
    __toyaml(j, e, false);
  else if (j.is_array())
    __toyaml(j, e, true);
  else
    __toyaml(j, e, false);
}

// Forward declaration required here for circular dipedency.

inline std::string repr(const nlohmann::json &j) {
  if (j.is_number())
    return std::to_string(j.get<int>());
  if (j.is_boolean())
    return j.get<bool>() ? "true" : "false";
  if (j.is_number_float())
    return std::to_string(j.get<double>());
  if (j.is_string())
    return j.get<std::string>();
  throw std::runtime_error("invalid type");
  return "";
}

/// \todo handle @text entries better

/// \todo handle @text entries better

} // namespace detail

/// \brief Convert XML string to JSON.

/// \brief Convert YAML string to JSON.
TOJSON_NODISCARD inline nlohmann::json yaml2json(const std::string &str) {
  YAML::Node root = YAML::Load(str);
  return detail::yaml2json(root);
}

/// \brief Load a YAML file to JSON.
TOJSON_NODISCARD inline nlohmann::json loadyaml(const std::string &filepath) {
  YAML::Node root = YAML::LoadFile(filepath);
  return detail::yaml2json(root);
}

namespace emitters {

/// \brief Generate string representation of json as an YAML document.
TOJSON_NODISCARD inline std::string toyaml(const nlohmann::json &j) {
  YAML::Emitter e;
  e << YAML::BeginDoc;
  if (j.is_object()) {
    e << YAML::BeginMap;
    detail::toyaml(j, e);
    e << YAML::EndMap;
  } else if (j.is_array()) {
    e << YAML::BeginSeq;
    detail::toyaml(j, e);
    e << YAML::EndSeq;
  }
  e << YAML::EndDoc;
  return e.c_str();
}

} // namespace emitters
} // namespace tojson

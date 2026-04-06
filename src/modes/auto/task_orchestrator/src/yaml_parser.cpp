#include "task_orchestrator/task_catalog.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "auto_library/context.hpp"
#include "auto_library/method.hpp"
#include "auto_library/step.hpp"

namespace task_orchestrator::detail {
namespace {

using core::Binding;
using core::ContextKey;
using core::makeTaskPlan;
using core::addStep;
using core::persistKey;
using core::Step;
using core::taskKey;
using core::Value;

std::string toLowerAscii(std::string value) {
  for (char &ch : value) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  return value;
}

bool parseTaskIdName(const std::string &name, TaskId &out) {
  for (std::size_t i = 0; i < kTaskCount; ++i) {
    if (name == kTaskNames[i]) {
      out = static_cast<TaskId>(i);
      return true;
    }
  }
  return false;
}

int requireInt(const YAML::Node &node, const std::string &field) {
  if (!node || !node.IsScalar()) {
    throw std::runtime_error("missing or invalid int field: " + field);
  }
  return node.as<int>();
}

std::string requireString(const YAML::Node &node, const std::string &field) {
  if (!node || !node.IsScalar()) {
    throw std::runtime_error("missing or invalid string field: " + field);
  }
  return node.as<std::string>();
}

bool parseIntegerScalar(const std::string &text, int64_t &out) {
  if (text.empty()) {
    return false;
  }
  errno = 0;
  char *end = nullptr;
  const long long value = std::strtoll(text.c_str(), &end, 10);
  if (errno != 0 || end != text.c_str() + text.size()) {
    return false;
  }
  out = static_cast<int64_t>(value);
  return true;
}

bool parseDoubleScalar(const std::string &text, double &out) {
  if (text.empty()) {
    return false;
  }
  errno = 0;
  char *end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (errno != 0 || end != text.c_str() + text.size()) {
    return false;
  }
  out = value;
  return true;
}

Value parseInferredScalarValue(const YAML::Node &node) {
  const std::string text = node.as<std::string>();
  const std::string lower = toLowerAscii(text);
  if (lower == "true") {
    return true;
  }
  if (lower == "false") {
    return false;
  }

  int64_t integer_value = 0;
  if (parseIntegerScalar(text, integer_value)) {
    return integer_value;
  }

  double double_value = 0.0;
  if (parseDoubleScalar(text, double_value)) {
    return double_value;
  }

  return text;
}

std::vector<double> parseDoubleArray(const YAML::Node &node, const std::string &field) {
  if (!node || !node.IsSequence()) {
    throw std::runtime_error("expected double array for: " + field);
  }

  std::vector<double> values;
  values.reserve(node.size());
  for (const auto &item : node) {
    values.push_back(item.as<double>());
  }
  return values;
}

DoubleArray resolveDoubleArrayPreset(const TaskCatalog &catalog,
                                     const std::string &preset_name) {
  if (const auto it = catalog.double_arrays.find(preset_name);
      it != catalog.double_arrays.end()) {
    return it->second;
  }

  throw std::runtime_error("unknown double array preset: " + preset_name);
}

std::array<double, 6> parseDoubleTableRow(const YAML::Node &node, const std::string &field) {
  const auto row = parseDoubleArray(node, field);
  if (row.size() != 6) {
    throw std::runtime_error("joints table row must have 6 elements: " + field);
  }
  return {row[0], row[1], row[2], row[3], row[4], row[5]};
}

ContextKey parseContextKey(const YAML::Node &node, const std::string &field) {
  if (!node) {
    throw std::runtime_error("missing context key field: " + field);
  }

  if (node.IsScalar()) {
    return taskKey(node.as<std::string>());
  }

  if (!node.IsMap()) {
    throw std::runtime_error("invalid context key field: " + field);
  }

  const std::string name = requireString(node["name"], field + ".name");
  const std::string scope = node["scope"] ? node["scope"].as<std::string>() : "task";
  if (scope == "task") {
    return taskKey(name);
  }
  if (scope == "persist") {
    return persistKey(name);
  }

  throw std::runtime_error("unsupported context scope: " + scope);
}

Value parseTypedValue(const YAML::Node &node, const std::string &field,
                      const TaskCatalog &catalog) {
  const std::string type = requireString(node["type"], field + ".type");

  if (type == "string") {
    return requireString(node["value"], field + ".value");
  }
  if (type == "bool") {
    if (!node["value"]) {
      throw std::runtime_error("missing bool value for: " + field);
    }
    return node["value"].as<bool>();
  }
  if (type == "int") {
    return static_cast<int64_t>(requireInt(node["value"], field + ".value"));
  }
  if (type == "double") {
    if (!node["value"] || !node["value"].IsScalar()) {
      throw std::runtime_error("missing double value for: " + field);
    }
    return node["value"].as<double>();
  }
  if (type == "double_array") {
    if (node["preset"]) {
      const std::string preset_name = node["preset"].as<std::string>();
      return resolveDoubleArrayPreset(catalog, preset_name);
    }
    return parseDoubleArray(node["value"], field + ".value");
  }

  throw std::runtime_error("unsupported param type: " + type);
}

Value parseParamValue(const YAML::Node &node, const std::string &field,
                      const TaskCatalog &catalog) {
  if (node.IsScalar()) {
    return parseInferredScalarValue(node);
  }
  if (node.IsSequence()) {
    return parseDoubleArray(node, field);
  }
  if (node.IsMap()) {
    return parseTypedValue(node, field, catalog);
  }

  throw std::runtime_error("unsupported param node for: " + field);
}

bool containsName(const std::vector<std::string> &names, const std::string &name) {
  return std::find(names.begin(), names.end(), name) != names.end();
}

std::vector<std::string> collectProvidedParams(const Step &step) {
  std::vector<std::string> params;
  params.reserve(step.command.params.size() + step.bindings.size());
  for (const auto &entry : step.command.params) {
    params.push_back(entry.first);
  }
  for (const auto &binding : step.bindings) {
    if (!containsName(params, binding.to_param)) {
      params.push_back(binding.to_param);
    }
  }
  return params;
}

void validateStepKindSpec(const Step &step,
                          const core::KindSpecMap &kind_specs,
                            const std::string &field_prefix) {
  const auto spec_it = kind_specs.find(step.command.kind);
  if (spec_it == kind_specs.end()) {
    throw std::runtime_error("unknown kind: " + step.command.kind +
                             " at " + field_prefix + ".kind");
  }
  const core::KindSpec &spec = spec_it->second;

  const std::vector<std::string> provided_params = collectProvidedParams(step);
  for (const auto &name : provided_params) {
    if (!containsName(spec.allowed_params, name)) {
      throw std::runtime_error("unexpected param for " + step.command.kind + ": " + name);
    }
  }

  for (const auto &name : spec.required_params) {
    if (!containsName(provided_params, name)) {
      throw std::runtime_error("missing required param for " + step.command.kind + ": " + name);
    }
  }

  std::vector<std::string> declared_outputs;
  declared_outputs.reserve(step.outputs.size());
  for (const auto &output : step.outputs) {
    declared_outputs.push_back(output.name);
  }

  for (const auto &name : declared_outputs) {
    if (!containsName(spec.required_outputs, name)) {
      throw std::runtime_error("unexpected output for " + step.command.kind + ": " + name);
    }
  }

  for (const auto &name : spec.required_outputs) {
    if (!containsName(declared_outputs, name)) {
      throw std::runtime_error("missing required output for " + step.command.kind + ": " + name);
    }
  }
}

void parsePresetDoubleArrays(const YAML::Node &node, TaskCatalog &catalog) {
  if (!node) {
    return;
  }
  if (!node.IsMap()) {
    throw std::runtime_error("presets.double_arrays must be a map");
  }

  for (const auto &entry : node) {
    const std::string name = entry.first.as<std::string>();
    catalog.double_arrays[name] =
        parseDoubleArray(entry.second, "presets.double_arrays." + name);
  }
}

void parsePresetDoubleTables(const YAML::Node &node, TaskCatalog &catalog) {
  if (!node) {
    return;
  }
  if (!node.IsMap()) {
    throw std::runtime_error("presets.double_tables must be a map");
  }

  for (const auto &entry : node) {
    const std::string name = entry.first.as<std::string>();
    const YAML::Node table_node = entry.second;
    if (!table_node.IsSequence()) {
      throw std::runtime_error("double table must be a sequence: " + name);
    }

    auto table = std::make_shared<DoubleTable>();
    table->reserve(table_node.size());
    for (std::size_t i = 0; i < table_node.size(); ++i) {
      table->push_back(parseDoubleTableRow(
          table_node[i], "presets.double_tables." + name + "[" + std::to_string(i) + "]"));
    }
    catalog.double_tables[name] = std::move(table);
  }
}

Binding parseBinding(const YAML::Node &node, const TaskCatalog &catalog,
                     const std::string &field_prefix) {
  if (!node || !node.IsMap()) {
    throw std::runtime_error("binding must be a map: " + field_prefix);
  }

  Binding binding{};
  binding.from = parseContextKey(node["from"], field_prefix + ".from");
  binding.to_param = requireString(node["to_param"], field_prefix + ".to_param");

  const std::string op = node["op"] ? node["op"].as<std::string>() : "direct";
  if (op == "direct") {
    binding.op = core::BindingOp::Direct;
    return binding;
  }

  if (op == "index_to_joints_table") {
    const std::string table_name = requireString(node["table"], field_prefix + ".table");
    const auto it = catalog.double_tables.find(table_name);
    if (it == catalog.double_tables.end() || !it->second) {
      throw std::runtime_error("unknown joints table preset: " + table_name);
    }
    binding.op = core::BindingOp::IndexToJointsTable;
    binding.joints_table = it->second->data();
    binding.joints_table_size = it->second->size();
    return binding;
  }

  throw std::runtime_error("unsupported binding op: " + op);
}

Step parseStep(const YAML::Node &node, const TaskCatalog &catalog,
               const core::KindSpecMap &kind_specs,
               const std::string &task_name, std::size_t step_index) {
  if (!node || !node.IsMap()) {
    throw std::runtime_error("step must be a map: " + task_name);
  }

  const std::string step_prefix =
      task_name + ".steps[" + std::to_string(step_index) + "]";

  Step step{};
  step.id = requireString(node["id"], step_prefix + ".id");
  step.label = node["label"] ? node["label"].as<std::string>() : step.id;
  step.command.kind = requireString(node["kind"], step_prefix + ".kind");

  if (node["timeout_ms"]) {
    step.timeout_ms = requireInt(node["timeout_ms"], step_prefix + ".timeout_ms");
  }
  if (node["post_delay_ms"]) {
    step.post_delay_ms = requireInt(node["post_delay_ms"], step_prefix + ".post_delay_ms");
  }
  if (node["max_retries"]) {
    step.max_retries = requireInt(node["max_retries"], step_prefix + ".max_retries");
  } else if (node["retries"]) {
    step.max_retries = requireInt(node["retries"], step_prefix + ".retries");
  }

  if (const YAML::Node inputs = node["inputs"]) {
    if (!inputs.IsSequence()) {
      throw std::runtime_error("inputs must be a sequence: " + step.id);
    }
    for (std::size_t i = 0; i < inputs.size(); ++i) {
      step.inputs.push_back(
          parseContextKey(inputs[i], step.id + ".inputs[" + std::to_string(i) + "]"));
    }
  }

  if (const YAML::Node outputs = node["outputs"]) {
    if (!outputs.IsSequence()) {
      throw std::runtime_error("outputs must be a sequence: " + step.id);
    }
    for (std::size_t i = 0; i < outputs.size(); ++i) {
      step.outputs.push_back(
          parseContextKey(outputs[i], step.id + ".outputs[" + std::to_string(i) + "]"));
    }
  }

  if (const YAML::Node params = node["params"]) {
    if (!params.IsMap()) {
      throw std::runtime_error("params must be a map: " + step.id);
    }
    for (const auto &entry : params) {
      const std::string param_name = entry.first.as<std::string>();
      step.command.params[param_name] =
          parseParamValue(entry.second, step.id + ".params." + param_name, catalog);
    }
  }

  if (const YAML::Node bindings = node["bindings"]) {
    if (!bindings.IsSequence()) {
      throw std::runtime_error("bindings must be a sequence: " + step.id);
    }
    for (std::size_t i = 0; i < bindings.size(); ++i) {
      step.bindings.push_back(
          parseBinding(bindings[i], catalog, step.id + ".bindings[" + std::to_string(i) + "]"));
    }
  }

  validateStepKindSpec(step, kind_specs, step_prefix);

  return step;
}

void parseTasks(const YAML::Node &node,
                TaskCatalog &catalog,
                const core::KindSpecMap &kind_specs) {
  if (!node || !node.IsMap()) {
    throw std::runtime_error("tasks root must be a map");
  }

  for (const auto &entry : node) {
    const std::string task_name = entry.first.as<std::string>();
    TaskId task_id = TaskId::IDLE;
    if (!parseTaskIdName(task_name, task_id)) {
      throw std::runtime_error("unknown task id name in YAML: " + task_name);
    }
    if (task_id == TaskId::IDLE) {
      throw std::runtime_error("IDLE task should not be declared in YAML");
    }
    if (catalog.plans[static_cast<std::size_t>(task_id)].has_value()) {
      throw std::runtime_error("duplicate task declaration in YAML: " + task_name);
    }

    const YAML::Node task_node = entry.second;
    const YAML::Node steps_node = task_node["steps"];
    if (!steps_node || !steps_node.IsSequence()) {
      throw std::runtime_error("task steps must be a sequence: " + task_name);
    }

    core::TaskPlan plan = makeTaskPlan(static_cast<core::TaskId>(task_id));
    for (std::size_t i = 0; i < steps_node.size(); ++i) {
      addStep(plan, parseStep(steps_node[i], catalog, kind_specs, task_name, i));
    }

    catalog.plans[static_cast<std::size_t>(task_id)] = std::move(plan);
  }
}

YAML::Node extractTasksNode(const YAML::Node &root) {
  if (!root || !root.IsMap()) {
    throw std::runtime_error("task yaml root must be a map");
  }

  if (const YAML::Node tasks = root["tasks"]) {
    return tasks;
  }

  YAML::Node tasks_node(YAML::NodeType::Map);
  for (const auto &entry : root) {
    const std::string key = entry.first.as<std::string>();
    if (key == "version" || key == "presets" || key == "tasks") {
      continue;
    }
    tasks_node[key] = entry.second;
  }

  if (!tasks_node || tasks_node.size() == 0) {
    throw std::runtime_error("task yaml does not contain any task definitions");
  }
  return tasks_node;
}

} // namespace

TaskCatalog parseTaskCatalogNodes(const YAML::Node &presets_root,
                                  const std::vector<YAML::Node> &task_roots,
                                  const core::KindSpecMap &kind_specs,
                                  std::string source_path) {
  TaskCatalog catalog{};
  catalog.source_path = source_path;

  try {
    if (presets_root["version"] && presets_root["version"].as<int>() != 1) {
      throw std::runtime_error("unsupported task plan version");
    }

    parsePresetDoubleArrays(presets_root["double_arrays"], catalog);
    parsePresetDoubleTables(presets_root["double_tables"], catalog);

    for (const auto &task_root : task_roots) {
      if (task_root["version"] && task_root["version"].as<int>() != 1) {
        throw std::runtime_error("unsupported task plan version");
      }
      parseTasks(extractTasksNode(task_root), catalog, kind_specs);
    }

    catalog.loaded = true;
  } catch (const std::exception &ex) {
    catalog.loaded = false;
    catalog.error = ex.what();
  }

  return catalog;
}

TaskCatalog parseTaskCatalogFile(const std::string &source_path,
                                 const core::KindSpecMap &kind_specs) {
  const YAML::Node root = YAML::LoadFile(source_path);
  return parseTaskCatalogNodes(root["presets"], {root}, kind_specs, source_path);
}

} // namespace task_orchestrator::detail

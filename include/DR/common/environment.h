/*
 Tools that access information from the host system via environment variables & other read-only queries to the
 host filesystem.
 */
#pragma once

#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <experimental/filesystem>  // requires linking to stdc++fs
#include <string>

#include <DR/common/exception.h>

namespace DR {

inline std::string GetModelDirectoryFromEnvironment() {
  // Get the absolute model path from an environment variable.
  const char* absolute_model_path_env_var = std::getenv("DR_ABSOLUTE_MODEL_PATH");
  DR_DEMAND(absolute_model_path_env_var != nullptr);
  std::string absolute_model_path = std::string(absolute_model_path_env_var);

  // Add a trailing slash if necessary.
  if (absolute_model_path.back() != '/') absolute_model_path += '/';
  if (!is_directory(std::experimental::filesystem::path(absolute_model_path))) {
    std::runtime_error("the environment variable DR_ABSOLUTE_MODEL_PATH does not name an existing directory: " +
                       absolute_model_path);
  }

  return absolute_model_path;
}

}  // namespace DR

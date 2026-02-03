/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gflags/gflags.h"

// Define shared flags here (exactly once)
// These flags are used across multiple executables in cartographer_ros

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics.");

// Common flags shared between node_main, offline_node, etc.
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched.");

DEFINE_string(configuration_basename, "",
              "Basename of the configuration file.");

DEFINE_string(configuration_basenames, "",
              "Comma-separated list of configuration basenames.");

DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load.");

DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen trajectories.");

DEFINE_bool(start_trajectory_with_default_topics, true,
            "Enable to immediately start the first trajectory.");

DEFINE_string(save_state_filename, "",
              "If non-empty, serialize state and write it to disk.");

DEFINE_string(save_map_filename, "",
              "If non-empty, serialize map and write it to disk.");

// Offline node specific flags
DEFINE_string(bag_filenames, "",
              "Comma-separated list of bags to process.");

DEFINE_string(urdf_filenames, "",
              "Comma-separated list of URDF files.");

DEFINE_bool(use_bag_transforms, true,
            "Whether to read, use and republish transforms from bags.");

DEFINE_bool(keep_running, false,
            "Keep running after all messages processed.");

DEFINE_double(skip_seconds, 0,
              "Seconds to skip from bag beginning.");

// gRPC specific flags
DEFINE_string(server_address, "localhost:50051",
              "gRPC server address.");

DEFINE_bool(upload_load_state_file, false,
            "Upload .pbstream file from local path.");

DEFINE_string(client_id, "",
              "Cartographer client ID for gRPC connection.");


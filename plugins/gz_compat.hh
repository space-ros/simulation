/*
 * Copyright (C) 2026 Alexey Simonov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SIMULATION_GZ_COMPAT_HH_
#define SIMULATION_GZ_COMPAT_HH_

// The plugins are written against the Gazebo Harmonic (gz-sim 8) API. Ignition
// Fortress (gz-sim 6), used on ROS 2 Humble, ships redirect headers under gz/
// that provide the gz::sim, gz::rendering, gz::sensors, ... namespaces, but
// not the GZ_ / gz prefixed macros. Map those onto their Ignition names when
// they are missing, so one set of sources builds on both.

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#ifndef GZ_ADD_PLUGIN
#define GZ_ADD_PLUGIN IGNITION_ADD_PLUGIN
#endif

#ifndef GZ_ADD_PLUGIN_ALIAS
#define GZ_ADD_PLUGIN_ALIAS IGNITION_ADD_PLUGIN_ALIAS
#endif

#ifndef GZ_PROFILE
#define GZ_PROFILE IGN_PROFILE
#endif

#ifndef GZ_ASSERT
#define GZ_ASSERT IGN_ASSERT
#endif

#ifndef gzerr
#define gzerr ignerr
#endif

#ifndef gzwarn
#define gzwarn ignwarn
#endif

#ifndef gzmsg
#define gzmsg ignmsg
#endif

#ifndef gzdbg
#define gzdbg igndbg
#endif

#endif  // SIMULATION_GZ_COMPAT_HH_

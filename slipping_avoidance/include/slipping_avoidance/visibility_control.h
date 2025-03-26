// Copyleft (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Subhas Das, Denis Stogl
 */

#ifndef SLIPPING_AVOIDANCE__VISIBILITY_CONTROL_H_
#define SLIPPING_AVOIDANCE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define SLIPPING_AVOIDANCE_EXPORT __attribute__((dllexport))
#define SLIPPING_AVOIDANCE_IMPORT __attribute__((dllimport))
#else
#define SLIPPING_AVOIDANCE_EXPORT __declspec(dllexport)
#define SLIPPING_AVOIDANCE_IMPORT __declspec(dllimport)
#endif
#ifdef SLIPPING_AVOIDANCE_BUILDING_DLL
#define SLIPPING_AVOIDANCE_PUBLIC SLIPPING_AVOIDANCE_EXPORT
#else
#define SLIPPING_AVOIDANCE_PUBLIC SLIPPING_AVOIDANCE_IMPORT
#endif
#define SLIPPING_AVOIDANCE_PUBLIC_TYPE SLIPPING_AVOIDANCE_PUBLIC
#define SLIPPING_AVOIDANCE_LOCAL
#else
#define SLIPPING_AVOIDANCE_EXPORT __attribute__((visibility("default")))
#define SLIPPING_AVOIDANCE_IMPORT
#if __GNUC__ >= 4
#define SLIPPING_AVOIDANCE_PUBLIC __attribute__((visibility("default")))
#define SLIPPING_AVOIDANCE_LOCAL __attribute__((visibility("hidden")))
#else
#define SLIPPING_AVOIDANCE_PUBLIC
#define SLIPPING_AVOIDANCE_LOCAL
#endif
#define SLIPPING_AVOIDANCE_PUBLIC_TYPE
#endif

#endif  // SLIPPING_AVOIDANCE__VISIBILITY_CONTROL_H_

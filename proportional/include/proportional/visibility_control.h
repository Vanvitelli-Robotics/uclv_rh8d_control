// Copyleft (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef PROPORTIONAL__VISIBILITY_CONTROL_H_
#define PROPORTIONAL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PROPORTIONAL__VISIBILITY_EXPORT __attribute__((dllexport))
#define PROPORTIONAL__VISIBILITY_IMPORT __attribute__((dllimport))
#else
#define PROPORTIONAL__VISIBILITY_EXPORT __declspec(dllexport)
#define PROPORTIONAL__VISIBILITY_IMPORT __declspec(dllimport)
#endif
#ifdef PROPORTIONAL__VISIBILITY_BUILDING_DLL
#define PROPORTIONAL__VISIBILITY_PUBLIC PROPORTIONAL__VISIBILITY_EXPORT
#else
#define PROPORTIONAL__VISIBILITY_PUBLIC PROPORTIONAL__VISIBILITY_IMPORT
#endif
#define PROPORTIONAL__VISIBILITY_PUBLIC_TYPE PROPORTIONAL__VISIBILITY_PUBLIC
#define PROPORTIONAL__VISIBILITY_LOCAL
#else
#define PROPORTIONAL__VISIBILITY_EXPORT __attribute__((visibility("default")))
#define PROPORTIONAL__VISIBILITY_IMPORT
#if __GNUC__ >= 4
#define PROPORTIONAL__VISIBILITY_PUBLIC __attribute__((visibility("default")))
#define PROPORTIONAL__VISIBILITY_LOCAL __attribute__((visibility("hidden")))
#else
#define PROPORTIONAL__VISIBILITY_PUBLIC
#define PROPORTIONAL__VISIBILITY_LOCAL
#endif
#define PROPORTIONAL__VISIBILITY_PUBLIC_TYPE
#endif

#endif  // PROPORTIONAL__VISIBILITY_CONTROL_H_

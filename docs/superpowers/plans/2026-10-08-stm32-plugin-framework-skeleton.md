# STM32 C++ Plugin Framework Skeleton Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a C++17 plugin-graph skeleton that builds on STM32 + FreeRTOS, enters through CubeMX C user-code bridges, runs a 1 kHz observation-only executor, and leaves all existing motor/output behavior unchanged.

**Architecture:** CubeMX-generated C remains C. New business/framework files live outside generated directories and expose only a small `extern "C"` bridge. A fixed-capacity plugin graph is composed once at startup, validated and frozen, then called by a FreeRTOS task at 1 kHz. The first version has no chassis/electrical output nodes; it proves compilation, graph lifecycle, frame timing, debug snapshot, and performance counters without changing vehicle behavior.

**Tech Stack:** ARMClang V6.24/Keil, STM32H7 HAL, CMSIS-RTOS v1/FreeRTOS, C++17, no exceptions, no RTTI, fixed-capacity storage.

---

### Task 1: Add shared build configuration and C++ source roots

**Files:**
- Create: `Config/robot_build_config.h`
- Create: `ApplicationCpp/README.md`
- Modify: `CMakeLists.txt`
- Modify: `MDK-ARM/Sentry_H7_Upper_CMake.uvprojx`

- [ ] **Step 1: Write the configuration contract first**

Create `Config/robot_build_config.h` with include guards and only compile-time options needed by the skeleton:

```cpp
#ifndef ROBOT_BUILD_CONFIG_H
#define ROBOT_BUILD_CONFIG_H

#define ROBOT_PLATFORM_STM32 1
#define ROBOT_HAS_PLUGIN_GRAPH 1
#define ROBOT_USE_PLUGIN_GRAPH 0
#define ROBOT_BASE_RATE_HZ 1000U
#define ROBOT_MAX_PLUGIN_NODES 32U
#define ROBOT_MAX_PLUGIN_EDGES 64U
#define ROBOT_ENABLE_PERF_COUNTERS 1
#define ROBOT_ENABLE_PERF_TRACE 0
#define ROBOT_ENABLE_DEBUG_SNAPSHOT 1

#endif
```

`ROBOT_USE_PLUGIN_GRAPH` must remain `0` for this phase so no legacy output path changes.

- [ ] **Step 2: Add the user C++ source root documentation**

Create `ApplicationCpp/README.md` stating that this directory is user-owned C++ code, never CubeMX-generated, and that files under `Core/`, `Drivers/`, `USB_DEVICE/`, and `Middlewares/` remain C/CubeMX-managed.

- [ ] **Step 3: Make CMake discover C++ without changing CubeMX sources**

Modify the project settings as follows:

```cmake
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
enable_language(C CXX ASM)

file(GLOB_RECURSE USER_CPP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/ApplicationCpp/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Platform/*.cpp"
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
    ${USER_CPP_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Config
    ${CMAKE_CURRENT_SOURCE_DIR}/ApplicationCpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Platform
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti>
)
```

Do not change CubeMX subdirectory source lists or generated C options.

- [ ] **Step 4: Add the initial C++ files to the Keil project**

Add the new `.cpp`/`.h` paths to the user-owned group in `MDK-ARM/Sentry_H7_Upper_CMake.uvprojx`. Keep all CubeMX files as `.c`. Add C++ compiler controls for the new C++ files only: C++17, no exceptions, no RTTI, and the `Config`, `ApplicationCpp`, and `Platform` include paths.

- [ ] **Step 5: Verify the source-root change before framework code**

Run:

```powershell
cmake -S . -B .codex-build -G Ninja
```

Expected: configuration succeeds and the generated compile database contains C++ language settings for files under `ApplicationCpp` or `Platform` once those files exist. Do not claim an ARM build here; the user performs the Keil build.

- [ ] **Step 6: Commit the source-root change**

```powershell
git add Config ApplicationCpp CMakeLists.txt MDK-ARM/Sentry_H7_Upper_CMake.uvprojx
git commit -m "build: add STM32 C++ application roots"
```

---

### Task 2: Implement the fixed-capacity plugin node and graph core

**Files:**
- Create: `ApplicationCpp/Framework/PluginTypes.hpp`
- Create: `ApplicationCpp/Framework/PluginNode.hpp`
- Create: `ApplicationCpp/Framework/PluginGraph.hpp`
- Create: `ApplicationCpp/Framework/PluginGraph.cpp`
- Create: `ApplicationCpp/Framework/FrameContext.hpp`

- [ ] **Step 1: Define compile-time-friendly status and lifecycle types**

Provide `PluginStatus`, `ProcessResult`, `PluginState`, `PluginHealth`, `PluginId`, and `FrameContext`. No exceptions, RTTI, STL allocation, or FreeRTOS includes are allowed in these files.

- [ ] **Step 2: Define the single node interface**

`PluginNode` must be the same interface for every level. It exposes:

```cpp
virtual void compose(PluginGraph& graph) = 0;
virtual PluginStatus configure() = 0;
virtual PluginStatus start() = 0;
virtual ProcessResult process(FrameContext& context) = 0;
virtual void stop() = 0;
virtual PluginHealth health() const = 0;
```

Every node owns no dynamic memory. An empty internal graph is allowed; a node may register fixed child members during `compose()`.

- [ ] **Step 3: Define fixed graph storage**

`PluginGraph` stores `PluginNode* nodes_[ROBOT_MAX_PLUGIN_NODES]` and fixed edge records. It provides `add()`, `addDependency(before, after)`, `validate()`, `compile()`, `startAll()`, `processFrame()`, and `stopAll()`.

For this phase, implement dependency edges and deterministic topological ordering. Reject duplicate nodes, duplicate edges, self-edges, capacity overflow, and cycles with status codes.

- [ ] **Step 4: Implement graph freeze semantics**

`compose()`/`addDependency()` are legal only before `compile()`. After `compile()`, all mutation attempts return a failure status. `processFrame()` executes the compiled order only; it never sorts the graph.

- [ ] **Step 5: Add compile-time smoke assertions**

Use `static_assert` for configured capacities and `sizeof` checks that protect the fixed-storage contract. Add a small framework self-check node only if needed to exercise lifecycle calls; it must not touch hardware.

- [ ] **Step 6: Commit framework core**

```powershell
git add ApplicationCpp/Framework
git commit -m "feat: add fixed-capacity plugin graph core"
```

---

### Task 3: Add STM32 clock, performance, and debug services

**Files:**
- Create: `ApplicationCpp/Platform/IClock.hpp`
- Create: `ApplicationCpp/Platform/IProfiler.hpp`
- Create: `ApplicationCpp/Platform/DebugSnapshot.hpp`
- Create: `Platform/STM32/Stm32Clock.cpp`
- Create: `Platform/STM32/Stm32Profiler.cpp`
- Create: `Platform/STM32/DebugSnapshot.cpp`

- [ ] **Step 1: Define the platform-neutral service contracts**

`IClock` returns a monotonic microsecond/tick value. `IProfiler` records node start/end and frame timing. No Linux implementation is added in this phase.

- [ ] **Step 2: Implement STM32 clock backend**

Use the existing `bsp_dwt` interface where possible; do not reconfigure CubeMX clocks. If DWT is not initialized, return a safe monotonic fallback based on `HAL_GetTick()` and mark reduced-resolution status.

- [ ] **Step 3: Implement fixed debug snapshot**

Create one exported `volatile` `RobotDebugSnapshot` with frame counter, graph state, last process result, last frame duration, max frame duration, and fault flags. Keep it in user C++ code so Keil can watch it.

- [ ] **Step 4: Add automatic executor instrumentation hooks**

The graph executor records frame start/end and per-node duration when `ROBOT_ENABLE_PERF_COUNTERS` is enabled. Full trace storage remains disabled by default.

- [ ] **Step 5: Commit services**

```powershell
git add ApplicationCpp/Platform Platform/STM32
git commit -m "feat: add STM32 clock profiler and debug snapshot"
```

---

### Task 4: Add the C bridge and observation-only FreeRTOS executor

**Files:**
- Create: `ApplicationCpp/app_cpp_entry.h`
- Create: `ApplicationCpp/app_cpp_entry.cpp`
- Create: `ApplicationCpp/RobotApplication.hpp`
- Create: `ApplicationCpp/RobotApplication.cpp`
- Create: `ApplicationCpp/ControlExecutorTask.cpp`
- Modify only USER CODE regions: `Core/Src/main.c`, `Core/Src/freertos.c`

- [ ] **Step 1: Define the C ABI bridge**

`app_cpp_entry.h` must be C-compatible:

```c
#ifdef __cplusplus
extern "C" {
#endif

void AppCpp_Initialize(void);
void AppCpp_ControlTask(void const* argument);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 2: Implement static application ownership**

`app_cpp_entry.cpp` owns one static `RobotApplication` instance. `AppCpp_Initialize()` composes, validates, compiles, freezes, and starts the graph. On failure it updates the debug snapshot and leaves the executor disabled; it must not change motor outputs.

- [ ] **Step 3: Implement the 1 kHz task wrapper**

`ControlExecutorTask.cpp` is the only new file allowed to include CMSIS-RTOS/FreeRTOS headers. It uses `vTaskDelayUntil` or the project’s CMSIS-compatible timing primitive to call `RobotApplication::processFrame()` every 1 ms. The task wrapper owns scheduling; nodes remain platform-independent.

- [ ] **Step 4: Add CubeMX-safe bridge calls**

Add only `USER CODE` additions to `main.c` for `AppCpp_Initialize()` after HAL/peripheral initialization and before `MX_FREERTOS_Init()`. Add the executor task declaration/creation in the `USER CODE` sections of `freertos.c`. Do not rename or convert either generated file to C++.

- [ ] **Step 5: Keep the graph observation-only**

The first `RobotApplication` graph contains a heartbeat/diagnostic node only. It must not call CAN transmit, motor functions, or modify legacy globals.

- [ ] **Step 6: Commit bridge and executor**

```powershell
git add ApplicationCpp Core/Src/main.c Core/Src/freertos.c
git commit -m "feat: start STM32 plugin executor without changing outputs"
```

---

### Task 5: Static verification and user handoff

**Files:**
- Modify: `doc/插件化架构设计讨论-01.md`
- Modify: `ApplicationCpp/README.md`

- [ ] **Step 1: Run repository static checks**

```powershell
git diff --check
git status --short
```

Verify that no CubeMX generated file outside explicit `USER CODE` regions changed and no generated build output is newly staged.

- [ ] **Step 2: Verify graph invariants by inspection**

Confirm that `PluginGraph::compile()` rejects cycles and that `processFrame()` never mutates graph storage or performs dynamic allocation.

- [ ] **Step 3: Document the user’s STM32 test procedure**

Record the expected Keil observations:

```text
ROBOT_USE_PLUGIN_GRAPH remains 0
legacy motor output remains active
RobotDebugSnapshot.frame_id increments at 1 kHz
last_frame_duration_us remains within the configured budget
plugin graph state is Running
no new CAN transmit path is active
```

- [ ] **Step 4: Handoff without claiming an ARM build**

Report changed files, static checks, and the exact Keil/real-vehicle checks the user must perform. Do not claim build success or vehicle safety until the user supplies the result.

---

## Scope boundaries for this phase

- No chassis algorithm migration.
- No CAN/motor output migration.
- No Linux backend.
- No PC simulation or host test runner.
- No CubeMX configuration changes.
- No runtime graph mutation.
- No dynamic memory in framework or plugin code.

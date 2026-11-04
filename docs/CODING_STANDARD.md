# Code organization and readability

Alibaba P3C is a Java-oriented guideline, not an official Python/C++ compliance
checker. This repository applies its transferable practices: meaningful naming,
single responsibility, explicit constants/configuration, layered modules, comments
for intent and coordinate conventions, bounded resources, useful errors, and tests
at behavior boundaries. It does not claim Alibaba certification or that vendored
third-party code has been rewritten to Java naming conventions.

The engineering mapping is:

| Guideline concern | Repository rule |
|---|---|
| Naming | Python snake_case/functions, PascalCase/classes, UPPER_CASE constants |
| Layering | ROS drivers, transport, capture, numerical processing, orchestration, docs |
| OOP | Stateful process/receiver ownership in classes; pure numerical transforms remain functions |
| Resources | Context managers; bounded buffers; explicit close/stop; PID ownership validation |
| Exceptions | Fail with context; skip disconnected inputs independently; no silent successful result |
| Configuration | Repository-relative paths, environment overrides, explicit units/frame direction |
| Reproducibility | Pinned direct dependencies, source manifest, input hashes and quality reports |
| Formatting | UTF-8/LF, EditorConfig, Ruff formatter for Python, ROS-compatible C++ style |
| Verification | Syntax/XML/map audits, fatal lint, geometry tests, process lifecycle tests, ROS build |

RoboSense rs_driver and the serial library retain their upstream notices and source
layout. Large historical research modules are preserved with their original function
boundaries to avoid changing the numerical algorithms during packaging. They are not
all refactored into classes; algorithmic changes require new data validation.
Run scripts/verify.sh after modifications. All new entry points must support
repository relocation and must avoid starting chassis-control output.

Reference: [Alibaba P3C](https://github.com/alibaba/p3c).

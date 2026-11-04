# Environment manifest

The target is Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12 / C++17.
Ubuntu 22.04 / ROS 2 Humble / Python 3.10 is a compatibility target.
macOS Python 3.11 can execute offline solvers; it cannot execute ROS hardware capture.

- ../requirement.txt: pinned direct Python dependencies.
- apt.txt: native development/runtime packages; install.sh adds distro-specific ROS packages.
- ../install.sh: one entry for native setup; --python-only requires no sudo.
- ../scripts/build.sh: native colcon driver compilation.
- ../docs/VALIDATION.md: actual observed test results and pending checks.

ROS apt-source package information:
https://github.com/ros-infrastructure/ros-apt-source

The installer resolves the official apt-source release, while numerical-library
versions remain pinned. Ubuntu security and ROS apt patch versions follow the
machine's repositories. This is a reproducible source/requirements recipe, not an
immutable operating-system image. Record pip freeze, dpkg-query and compiler
versions when qualifying a new deployment.

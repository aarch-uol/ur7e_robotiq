#!/usr/bin/env bash
# What the image's dependencies are made from: the docker/ files, the workspace script, the patches and every
# package.xml (the declared dependencies). Everything else (launch files, yaml, URDF, Python, the C++ of
# topic_sensor_bridge) is built from the source at stack start (the overlay, see README "Docker"), so changing
# it needs no new image.
#
#   docker/deps_hash.sh           # prints one hash of those files' contents; build.sh stores it in the image
#                                 # (label ur7e_robotiq.deps) and the launcher rebuilds only when it differs
#   docker/deps_hash.sh --files   # lists the files, one path per line (relative to the repo)
set -eo pipefail
cd "$(dirname "$0")/.."
files() {
  ls docker/* scripts/setup_workspace.sh patches/*.patch */package.xml 2>/dev/null | LC_ALL=C sort
}
if [ "$1" = --files ]; then files; exit 0; fi
files | while IFS= read -r f; do printf '%s  %s\n' "$(sha256sum < "$f" | cut -d' ' -f1)" "$f"; done \
  | sha256sum | cut -c1-16

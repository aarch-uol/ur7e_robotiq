#!/usr/bin/env bash
# Builds the Docker image "ur7e_robotiq:jazzy" from this repo.
set -e
cd "$(dirname "$0")/.."
docker build -f docker/Dockerfile -t ur7e_robotiq:jazzy .

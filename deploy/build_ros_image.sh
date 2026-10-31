#!/usr/bin/env bash
# Run on MSI. Builds the ROS image used by the episode recorder and the EE bridge.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/ros"
docker build -t "${1:-xrcam-ros:humble}" .

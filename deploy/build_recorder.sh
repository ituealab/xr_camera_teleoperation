#!/usr/bin/env bash
# Run on MSI. Builds the episode recorder image.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/recorder"
docker build -t "${1:-xrcam-recorder:humble}" .

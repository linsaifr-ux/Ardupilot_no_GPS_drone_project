#!/bin/bash
# Run Isaac Sim headless test using the isaac_sim_test conda environment
OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test python "$(dirname "$0")/test_isaac.py" "$@"

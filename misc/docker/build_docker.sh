#!/bin/bash

# You should run this script from the misc/docker/bases directory! So not ./misc/docker/bases/build_docker_cuda.sh!
# You should not have to modify this script. You only need to edit the .params file.

# Define color codes for better output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

# Exit immediately if a command exits with a non-zero status or an unset variable is used.
set -euo pipefail

# --- Configuration ---
PARAMS_FILE=".params"
EXAMPLE_FILE=".params.example"

# --- Global Variables ---
BUILD_SOURCE="registry" # Default build source: "registry" or "local"

# --- Functions ---

# Function to check for required commands
check_dependencies() {
    # 1. Check for Docker (Critical)
    command -v docker >/dev/null 2>&1 || { printf "${RED}Error: 'docker' command not found. Please install Docker.${NC}\n"; exit 1; }

    # 2. Check for VSCode CLI (Optional)
    command -v code >/dev/null 2>&1 || { printf "${YELLOW}Warning: 'code' command (VSCode CLI) not found. VSCODE_COMMIT_HASH will be empty.${NC}\n"; }

    ################## TODO: comment these out if CUDA is needed ##################

    # 3. Check for NVIDIA Driver (Critical)
    if ! command -v nvidia-smi >/dev/null 2>&1; then
        printf "${RED}Error: 'nvidia-smi' not found. Please install NVIDIA drivers.${NC}\n"
        exit 1
    elif ! nvidia-smi >/dev/null 2>&1; then
        printf "${RED}Error: 'nvidia-smi' failed to execute. The driver may be misconfigured or the kernel module is not loaded.${NC}\n"
        exit 1
    fi

    # 4. Check for NVIDIA Container Toolkit (Critical)
    # We check for 'nvidia-ctk' (modern) or the legacy 'nvidia-docker' just in case.
    if ! command -v nvidia-ctk >/dev/null 2>&1 && ! command -v nvidia-docker >/dev/null 2>&1; then
        printf "${RED}Error: NVIDIA Container Toolkit not found. Please install 'nvidia-container-toolkit'.${NC}\n"
        exit 1
    fi
}

# Function to load and validate parameters
load_parameters() {
    #printf "${YELLOW}Loading parameters from '%s'...${NC}\n" "$PARAMS_FILE"

    if ! [ -f "$PARAMS_FILE" ]; then
        printf "${RED}Error: '%s' file not found. Creating from example '%s'.${NC}\n" "$PARAMS_FILE" "$EXAMPLE_FILE"
        cp "$EXAMPLE_FILE" "$PARAMS_FILE"
        read -p "Do you want to continue using the default .params file? (y/n): " choice
        case "$choice" in
            y|Y ) printf "Continuing with the example parameters...\n";;
            * ) printf "Please edit the '%s' file before running again.\n" "$PARAMS_FILE"; exit 1;;
        esac
    fi

    # Source the parameters file
    # Using 'set -a' to export variables sourced from the file, though not strictly necessary for this script
    # but good practice if child processes might need them.
    # shellcheck disable=SC1090
    source "$PARAMS_FILE"

    # Validate essential parameters after sourcing
    if [ -z "${container_name:-}" ]; then
        printf "${RED}Error: container_name is not set in '%s'.${NC}\n" "$PARAMS_FILE"
        exit 1
    fi
    if [ -z "${image_name:-}" ]; then
        printf "${RED}Error: image_name is not set in '%s'.${NC}\n" "$PARAMS_FILE"
        exit 1
    fi
    if [ -z "${image_tag:-}" ]; then
        printf "${RED}Error: image_tag is not set in '%s'.${NC}\n" "$PARAMS_FILE"
        exit 1
    fi
    if [ -z "${my_packages_folder:-}" ]; then
        printf "${RED}Error: my_packages_folder is not set in '%s'.${NC}\n" "$PARAMS_FILE"
        exit 1
    fi
    if  ! [ -d "$my_packages_folder" ]; then
        printf "${RED}Error: my_packages_folder directory does not exist as specified in '%s'.${NC}\n" "$PARAMS_FILE"
        exit 1
    fi
    if [ -z "${vscode_folder:-}" ]; then
        printf "${RED}Error: vscode_folder is not set in '%s'.${NC}\n" "$PARAMS_FILE"
        exit 1
    fi
    if  ! [ -d "$vscode_folder" ]; then
        printf "${RED}Error: vscode_folder directory does not exist as specified in '%s'.${NC}\n" "$PARAMS_FILE"
        exit 1
    fi
        if [ -z "${data_folder:-}" ]; then
        printf "${RED}Error: data_folder is not set in '%s'.${NC}\n" "$PARAMS_FILE"
        exit 1
    fi
    if  ! [ -d "$data_folder" ]; then
        printf "${RED}Error: data_folder directory does not exist as specified in '%s'.${NC}\n" "$PARAMS_FILE"
        exit 1
    fi
    if [ -z "${COLCON_BUILD_TYPE:-}" ]; then
        printf "${RED}Error: COLCON_BUILD_TYPE is not set in '%s'.${NC}\n" "$PARAMS_FILE"
        exit 1
    fi
}

# Function to get VSCode commit hash
get_vscode_commit_hash() {
    if command -v code >/dev/null 2>&1; then
        VSCODE_COMMIT_HASH=$(code --version | sed -n '2p')
        printf "VSCode Commit Hash: ${GREEN}%s${NC}\n\n" "$VSCODE_COMMIT_HASH"
    else
        VSCODE_COMMIT_HASH=""
        printf "${YELLOW}VSCode CLI not found. VSCode Commit Hash will be empty.${NC}\n\n"
    fi
}

# Function to check and set git shared repository settings
check_git_settings() {
    # Check if the git folder exists in the relative path
    if [[ -e "../../.git" ]]; then
        # Check if core.sharedRepository is NOT set to true
        if [[ "$(git config core.sharedRepository)" != "true" ]]; then
            # Use printf with colors if available, or just echo
            printf "${YELLOW}The repository is not shared (git config core.sharedRepository).${NC}\n"
            echo "If you will use git at any point, you should make it shared."
            echo "If not, file modifications from inside the container may break git permissions."
            echo "Would you like to make the repo shared?"

            # specific prompt string for the select command
            PS3="Select an option (1 or 2): "

            select yn in "Yes" "No"; do
                case $yn in
                    Yes )
                        echo "Changing repo to shared..."
                        git config core.sharedRepository true
                        echo "Done."
                        break;;
                    No )
                        echo "Leaving it untouched."
                        break;;
                esac
            done
        fi
    fi
}

################################################
# --- Main Script Execution ---
################################################

check_dependencies
load_parameters
get_vscode_commit_hash
check_git_settings

# Get the host's group ID
HOST_USER_GROUP_ARG=$(id -g "$USER")
printf "Host User Group ID: ${GREEN}%s${NC}\n" "$HOST_USER_GROUP_ARG"

git -C "$(git rev-parse --show-toplevel)" submodule update --init --recursive

docker build \
    --file Dockerfile\
    --tag $image_name:$image_tag \
    --build-arg HOST_USER_GROUP_ARG=$HOST_USER_GROUP_ARG\
    --build-arg COLCON_BUILD_TYPE=$COLCON_BUILD_TYPE\
    --build-arg VSCODE_COMMIT_HASH=$VSCODE_COMMIT_HASH\
    ./../..\
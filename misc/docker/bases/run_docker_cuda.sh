#!/bin/bash

# You should run this script from the misc/docker/bases directory! So not ./misc/docker/bases/run_docker_cuda.sh!
# You should not have to modify this script. You only need to edit the .params file.

# Define color codes for better output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

# Exit immediately if a command exits with a non-zero status or an unset variable is used.
set -euo pipefail

# --- Configuration ---
PARAMS_FILE="./.params"
EXAMPLE_FILE="./.params.example"

# --- Functions ---

# Function to check for required commands
check_dependencies() {
    # 1. Check for Docker (Critical)
    command -v docker >/dev/null 2>&1 || { printf "${RED}Error: 'docker' command not found. Please install Docker.${NC}\n"; exit 1; }

    # 2. Check for VSCode CLI (Optional)
    command -v code >/dev/null 2>&1 || { printf "${YELLOW}Warning: 'code' command (VSCode CLI) not found. VSCODE_COMMIT_HASH will be empty.${NC}\n"; }

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

    #printf "${GREEN}Parameters loaded and validated.${NC}\n\n"
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
check_git_settings

#add all the local processes to xhost, so the container reaches the window manager
xhost + local:

# Ensure permissions
setfacl -PRdm u::rwx,g::rwx,o::r "${my_packages_folder}" || { printf "${RED}Error: Failed to set ACLs on %s. Ensure 'acl' package is installed and filesystem supports ACLs.${NC}\n" "$my_packages_folder"; exit 1; }
printf "${GREEN}Shared repository permissions ensured for %s.${NC}\n\n" "$my_packages_folder"
setfacl -PRdm u::rwx,g::rwx,o::r "${data_folder}" || { printf "${RED}Error: Failed to set ACLs on %s. Ensure 'acl' package is installed and filesystem supports ACLs.${NC}\n" "$data_folder"; exit 1; }
printf "${GREEN}Shared repository permissions ensured for %s.${NC}\n\n" "$data_folder"

#check if container exists --> then we only restart it
if [[ $( docker ps -a -f name=test_spot_cuda_base | wc -l ) -eq 2 ]];
then
    echo "Container already exists. Do you want to restart it or remove it?"
    select yn in "Restart" "Remove"; do
        case $yn in
            Restart )
                echo "Restarting it... If it was started without USB, it will be restarted without USB.";
                docker restart test_spot_cuda_base;
                break;;
            Remove )
                echo "Stopping it and deleting it... You should simply run this script again to start it.";
                docker stop test_spot_cuda_base;
                docker rm test_spot_cuda_base;
                break;;
        esac
    done
else
    echo "Container does not exist. Creating it."
    docker run \
        --env DISPLAY=${DISPLAY} \
        --volume /tmp/.X11-unix:/tmp/.X11-unix \
        --volume ${my_packages_folder}:/home/appuser/ros2_ws/src/my_packages \
        --volume ${vscode_folder}:/home/appuser/ros2_ws/.vscode \
        --network host \
        --interactive \
        --tty \
        --detach \
        --gpus all \
        --runtime=nvidia \
        --env NVIDIA_VISIBLE_DEVICES=all \
        --env NVIDIA_DRIVER_CAPABILITIES=all \
        --privileged \
        --name test_spot_cuda_base \
        spot-ros2:cuda
    docker exec -u appuser test_spot_cuda_base bash -c 'python3 /home/appuser/ros2_ws/test_required_packages.py'
fi
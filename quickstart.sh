#! /usr/bin/env bash
set -o errexit
set -o nounset

# Get repository path
REPO="$(builtin cd "$(dirname "$(readlink -fm "${0}")")" > /dev/null && pwd)"

# Get solutions argument
if [[ $# > 0 ]]
then
	SOL=$1
else
	SOL="base"
fi

# Set files names
EXPERIMENT="diffusion_10"
CONTROLLER="footbot_diffusion"

# Set path
DIR="${REPO}/solutions/${SOL}"
A_IN="${REPO}/solutions/${SOL}/${EXPERIMENT}.argos" 
A_OU="${REPO}/experiments/${EXPERIMENT}.argos"
H_IN="${REPO}/solutions/${SOL}/${CONTROLLER}.h"
H_OU="${REPO}/controllers/${CONTROLLER}/${CONTROLLER}.h"
C_IN="${REPO}/solutions/${SOL}/${CONTROLLER}.cpp"
C_OU="${REPO}/controllers/${CONTROLLER}/${CONTROLLER}.cpp"

# Check if file exist
if [ ! -d "$DIR" ]; then
    echo "$DIR does not exist."
	exit 1
elif [[ ! -f "${A_IN}" ]]; then
    echo "${A_IN} does not exists."
	exit 1
elif [[ ! -f "${H_IN}" ]]; then
    echo "${H_IN} does not exists."
	exit 1
elif [[ ! -f "${C_IN}" ]]; then
    echo "${C_IN} does not exists."
	exit 1
fi

# Symlink selected solution
ln -fs "${A_IN}" "${A_OU}"
ln -fs "${C_IN}" "${C_OU}"
ln -fs "${H_IN}" "${H_OU}"

# Build
mkdir -p "${REPO}/build"
cd "${REPO}/build"
cmake -DCMAKE_BUILD_TYPE=Release .. 
make

# Run
cd "${REPO}"
argos3 -c "experiments/${EXPERIMENT}.argos" &

exit 0
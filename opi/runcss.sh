#!/bin/bash

# Generated run script for cs-studio.
PREFIX=${CONVERTER_ROOT}
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
TOP=${SCRIPT_DIR}/..

if [[ -z $1 ]]; then
    echo "No OPI specified to launch!"
    exit 1
fi

# pop the opifile off the head of the argument list
opifile="$1"
shift

# css.sh
CSS_RUN_SCRIPT=$(configure-ioc s -p CSS-gui)

if [ -e ${TOP}/configure/VERSION ] ; then
    version="$(cat ${TOP}/configure/VERSION)"
else
    version="dev"
fi
project=Libera_${version}
module=Libera
launch_opi=/${project}/${module}/$opifile

links="${SCRIPT_DIR}=${project}/${module},\
${PREFIX}/dls_sw/prod/R3.14.12.3/support/TimingTemplates/6-10/TimingTemplatesApp/opi/opi=/${project}/TimingTemplates,\
${PREFIX}/dls_sw/prod/R3.14.12.3/support/4chTimer/3-2/4chTimerApp/opi/opi=/${project}/4chTimer,\
${PREFIX}/dls_sw/prod/R3.14.12.3/support/diagOpi/2-60/.=/${project}/diagOpi"

$CSS_RUN_SCRIPT -o ${launch_opi} -s -l "$links" "$@"

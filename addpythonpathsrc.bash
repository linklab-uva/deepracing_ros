#!/bin/bash
pushd . > /dev/null
SCRIPT_PATH="${BASH_SOURCE[0]}";
if ([ -h "${SCRIPT_PATH}" ]) then
  while([ -h "${SCRIPT_PATH}" ]) do cd `dirname "$SCRIPT_PATH"`; SCRIPT_PATH=`readlink "${SCRIPT_PATH}"`; done
fi
cd `dirname ${SCRIPT_PATH}` > /dev/null
SCRIPT_PATH=`pwd`;
popd  > /dev/null

extrapythonpaths=$SCRIPT_PATH/deepracing_rclpy
echo "Adding $extrapythonpaths to PYTHONPATH"

if [[ -z "${PYTHONPATH}" ]]; then
  export PYTHONPATH=${extrapythonpaths}
else
  export PYTHONPATH=${extrapythonpaths}:${PYTHONPATH}
fi

mapsdir=$SCRIPT_PATH/deepracing_launch/maps
if [[ -z "${F1_MAP_DIRS}" ]]; then
  export F1_MAP_DIRS=${mapsdir}
else
  export F1_MAP_DIRS=${mapsdir}:${F1_MAP_DIRS}
fi
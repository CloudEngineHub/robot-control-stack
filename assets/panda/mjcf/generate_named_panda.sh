#!/bin/sh
script_dir=$(dirname $(realpath $0))
id=${1}
dst_dir=${2}

sed -E -e "s/name=\"([^\"]*)/name=\"\1_${id}/" \
       -e "s/joint=\"([^\"]*)/joint=\"\1_${id}/" \
       -e "s/joint1=\"([^\"]*)/joint1=\"\1_${id}/" \
       -e "s/joint2=\"([^\"]*)/joint2=\"\1_${id}/" \
       -e "s/tendon=\"([^\"]*)/tendon=\"\1_${id}/" \
       -e "s/body1=\"([^\"]*)/body1=\"\1_${id}/" \
       -e "s/body2=\"([^\"]*)/body2=\"\1_${id}/" \
       "${script_dir}/panda_unnamed.xml" > "$(realpath ${dst_dir})/panda_${id}.xml"

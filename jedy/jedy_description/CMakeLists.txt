cmake_minimum_required(VERSION 3.16)
project(jedy_description)

find_package(ament_cmake REQUIRED)

# The visual meshes come from the CAD model (scikit-robot-models'
# jedy_draco_description: rebuilt from the CAD, Draco-compressed GLB), turned
# into DAE at build time with scikit-robot's convert-urdf-mesh, because Gazebo
# and RViz cannot read Draco meshes. jedy_gz.xacro addresses them as
# package://jedy_description/meshes/cad/<link>.dae, placed where the CAD model
# puts them (the arms and head share their frames with the CAD model; the
# body is offset by base_link's different height, the wheel axles were moved
# to the CAD's). Collision geometry stays in the xacro and meshes/gz.
#
# The bundle is pinned to a commit and its MD5, and scikit-robot and DracoPy to
# versions, so the meshes do not change unless this file does.
set(JEDY_CAD_BUNDLE_URL
  "https://github.com/iory/scikit-robot-models/raw/1dd9412bad655cb3bd45c46c71d1ec1db110ce6f/jedy_draco_description.tar.gz")
set(JEDY_CAD_BUNDLE_MD5 "eaaa5dc19a12ab01e3144a1964184ce9")
set(JEDY_CAD_TOOLS --with scikit-robot==0.4.15 --with DracoPy==2.2.0)
# Every link with a visual: base, 2 x 9 arm links, 2 head links, 4 wheels.
set(JEDY_CAD_VISUALS 25)

find_program(UV_EXECUTABLE uv)
if(NOT UV_EXECUTABLE)
  message(FATAL_ERROR
    "jedy_description: `uv` not found on PATH. It converts the CAD meshes "
    "(see docs: curl -LsSf https://astral.sh/uv/install.sh | sh).")
endif()

set(_cad "${CMAKE_CURRENT_BINARY_DIR}/cad")
set(_cad_bundle "${_cad}/jedy_draco_description.tar.gz")
set(_cad_meshes "${_cad}/jedy_draco_description/meshes")
set(_cad_stamp "${_cad}/converted.stamp")
# Downloaded at configure time, once (checked by MD5).
if(NOT EXISTS "${_cad_bundle}")
  file(DOWNLOAD "${JEDY_CAD_BUNDLE_URL}" "${_cad_bundle}"
       EXPECTED_HASH MD5=${JEDY_CAD_BUNDLE_MD5} STATUS _status TLS_VERIFY ON)
  list(GET _status 0 _code)
  if(NOT _code EQUAL 0)
    file(REMOVE "${_cad_bundle}")
    message(FATAL_ERROR "jedy_description: could not download ${JEDY_CAD_BUNDLE_URL}: ${_status}")
  endif()
endif()
# Converted at build time, and again whenever the bundle or the finishing
# script changes (a configure-time step would keep stale meshes after an edit).
# convert-urdf-mesh resolves package://jedy_draco_description through
# ROS_PACKAGE_PATH.
add_custom_command(
  OUTPUT "${_cad_stamp}"
  COMMAND ${CMAKE_COMMAND} -E rm -rf "${_cad}/jedy_draco_description"
  COMMAND ${CMAKE_COMMAND} -E tar xzf "${_cad_bundle}"
  COMMAND ${CMAKE_COMMAND} -E env ROS_PACKAGE_PATH=${_cad}
          ${UV_EXECUTABLE} run --no-project ${JEDY_CAD_TOOLS}
          convert-urdf-mesh ${_cad}/jedy_draco_description/urdf/jedy.urdf
          --format dae -o ${_cad}/jedy_dae.urdf
  COMMAND ${UV_EXECUTABLE} run --no-project ${JEDY_CAD_TOOLS} --with pycollada==0.9.2
          python ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_cad_meshes.py
          ${_cad_meshes} ${JEDY_CAD_VISUALS}
  COMMAND ${CMAKE_COMMAND} -E touch "${_cad_stamp}"
  DEPENDS "${_cad_bundle}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_cad_meshes.py"
  WORKING_DIRECTORY "${_cad}"
  COMMENT "jedy_description: converting the CAD meshes to DAE"
  VERBATIM)
add_custom_target(jedy_cad_meshes ALL DEPENDS "${_cad_stamp}")
install(DIRECTORY "${_cad_meshes}/" DESTINATION share/${PROJECT_NAME}/meshes/cad
        FILES_MATCHING PATTERN "*.dae")

# urdf/ and meshes/ must land under share/jedy_description/ with exactly these
# names: the URDFs address their own meshes as
# package://jedy_description/meshes/..., and resource_retriever resolves that to
# <prefix>/share/jedy_description/meshes/... Renaming either directory here
# breaks every mesh silently -- the model still loads, it just has no geometry.
install(
  DIRECTORY urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

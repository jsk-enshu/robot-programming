cmake_minimum_required(VERSION 3.18)
project(room13bc_description)
find_package(ament_cmake REQUIRED)

# The URDFs and meshes come from scikit-robot-models, the same bundle that
# scikit-robot (skrobot.data.room13bc_urdfpath) and scikit-robot-web load, so
# there is one copy of the room. Fetched once at build time and checked by
# hash: a changed file fails the build instead of silently changing the room.
set(ROOM13BC_URL
  "https://github.com/iory/scikit-robot-models/raw/main/room13bc_description.tar.gz"
  CACHE STRING "room13bc_description.tar.gz to download")
set(ROOM13BC_SHA256
  "50bd4683ba1270c2cc7afe326155e7437d3dd1ad71b5f5cee158099ec44e68c9")
set(_archive "${CMAKE_CURRENT_BINARY_DIR}/room13bc_description.tar.gz")
set(_bundle "${CMAKE_CURRENT_BINARY_DIR}/bundle")
if(NOT EXISTS "${_bundle}/room13bc_description/urdf/room13bc_scene.urdf")
  message(STATUS "[room13bc_description] downloading ${ROOM13BC_URL}")
  file(DOWNLOAD "${ROOM13BC_URL}" "${_archive}"
       EXPECTED_HASH SHA256=${ROOM13BC_SHA256}
       STATUS _status TLS_VERIFY ON)
  list(GET _status 0 _code)
  if(NOT _code EQUAL 0)
    file(REMOVE "${_archive}")
    message(FATAL_ERROR
      "[room13bc_description] could not download ${ROOM13BC_URL}: ${_status}")
  endif()
  file(REMOVE_RECURSE "${_bundle}")
  file(ARCHIVE_EXTRACT INPUT "${_archive}" DESTINATION "${_bundle}")
  file(REMOVE "${_archive}")
endif()

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY "${_bundle}/room13bc_description/urdf"
                  "${_bundle}/room13bc_description/meshes"
        DESTINATION share/${PROJECT_NAME})
ament_package()

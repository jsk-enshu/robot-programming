cmake_minimum_required(VERSION 3.16)
project(jedy_vision)

find_package(ament_cmake REQUIRED)
find_package(ament_uv_virtualenv REQUIRED)
# Same interpreter as jedy_skrobot's venv (see its CMakeLists.txt).
find_package(Python3 REQUIRED COMPONENTS Interpreter)

ament_generate_virtualenv(
  PYTHON ${Python3_EXECUTABLE}
  OVERRIDES overrides.txt
)

find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

ament_uv_install_python_scripts(
  PROGRAMS
    scripts/detect_people.py
    scripts/follow_person.py
    scripts/look_at_person.py
    scripts/mimic_arms.py
    scripts/tag_pose.py
    scripts/approach_tag.py
    scripts/touch_tag.py
    scripts/grasp_tag.py
    scripts/find_object.py
    scripts/wave_back.py
    scripts/voice_demo.py
    scripts/demos.py
    scripts/object_detection_node.py
    scripts/gesture_node.py
  STRIP_PY_EXTENSION
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Model weights, fetched once at build time and checked by hash, so a class
# of students does not each download them on first run (and a changed file
# fails the build instead of silently changing the demo).
set(_MODELS_DIR "${CMAKE_CURRENT_BINARY_DIR}/models")
function(jedy_vision_model name url sha256)
  set(_dst "${_MODELS_DIR}/${name}")
  if(NOT EXISTS "${_dst}")
    message(STATUS "[jedy_vision] downloading ${name}")
    file(DOWNLOAD "${url}" "${_dst}" EXPECTED_HASH SHA256=${sha256}
         STATUS _status TLS_VERIFY ON)
    list(GET _status 0 _code)
    if(NOT _code EQUAL 0)
      file(REMOVE "${_dst}")
      message(FATAL_ERROR "[jedy_vision] could not download ${url}: ${_status}")
    endif()
  endif()
endfunction()
jedy_vision_model(yolo11n.pt
  "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n.pt"
  0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1)
jedy_vision_model(yolo11n-pose.pt
  "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n-pose.pt"
  869e83fcdffdc7371fa4e34cd8e51c838cc729571d1635e5141e3075e9319dc0)
# The object demos (find_object) use the small model: on Gazebo's renders
# yolo11n saw a cup at 0.21, yolo11s at 0.78 (145 ms on a laptop CPU).
jedy_vision_model(yolo11s.pt
  "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11s.pt"
  85a76fe86dd8afe384648546b56a7a78580c7cb7b404fc595f97969322d502d5)
jedy_vision_model(pose_landmarker_lite.task
  "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task"
  59929e1d1ee95287735ddd833b19cf4ac46d29bc7afddbbf6753c459690d574a)
# Objects named by words (object_detection_node): YOLOE and the text encoder
# it was trained with. Measured on a 640x428 photo of three people at a table
# with bottles and cups (i7-1165G7): 26s found bottle 0.79 / cup 0.69,
# 11s bottle 0.55 / cup 0.64, at 156 vs 134 ms; 26s's pair is also the smaller
# download (0.29 vs 0.63 GB).
jedy_vision_model(yoloe-26s-seg.pt
  "https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26s-seg.pt"
  48f24206bc8680d60cbbfa296b0140da849669b9515058b72f5a945142df0654)
jedy_vision_model(mobileclip2_b.ts
  "https://github.com/ultralytics/assets/releases/download/v8.4.0/mobileclip2_b.ts"
  35d7f213e4d75f38514e4656ad3cb91158bd33e3805d8ac349f23b186f66982f)
# Hand gestures (gesture_node).
jedy_vision_model(gesture_recognizer.task
  "https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task"
  97952348cf6a6a4915c2ea1496b4b37ebabc50cbbf80571435643c455f2b0482)
# Faces and hands (FaceEstimator, HandEstimator; detect_people --face
# --hands). The head camera sees a face at 1.5-2.5 m as ~30 px, too small for
# the face models on the whole image: FaceEstimator crops the head found by
# Pose and upscales it (Gazebo's person: face found 0.87-0.89 from 85-115 px
# crops, none from the whole image).
jedy_vision_model(blaze_face_short_range.tflite
  "https://storage.googleapis.com/mediapipe-models/face_detector/blaze_face_short_range/float16/1/blaze_face_short_range.tflite"
  b4578f35940bf5a1a655214a1cce5cab13eba73c1297cd78e1a04c2380b0152f)
jedy_vision_model(face_landmarker.task
  "https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task"
  64184e229b263107bc2b804c6625db1341ff2bb731874b0bcc2fe6544e0bc9ff)
jedy_vision_model(hand_landmarker.task
  "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
  fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1)
install(DIRECTORY "${_MODELS_DIR}/" DESTINATION share/${PROJECT_NAME}/models)

ament_package()

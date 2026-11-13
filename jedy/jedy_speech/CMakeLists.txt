cmake_minimum_required(VERSION 3.16)
project(jedy_speech)

find_package(ament_cmake REQUIRED)
find_package(ament_uv_virtualenv REQUIRED)
# Same interpreter as jedy_skrobot's venv (see its CMakeLists.txt).
find_package(Python3 REQUIRED COMPONENTS Interpreter)

ament_generate_virtualenv(PYTHON ${Python3_EXECUTABLE})

find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

ament_uv_install_python_scripts(
  PROGRAMS
    scripts/speaker.py
    scripts/listener.py
    scripts/say.py
    scripts/voice_command.py
    scripts/setup_voicevox.py
  STRIP_PY_EXTENSION
)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

# Model files, fetched once at build time and checked by hash (nothing is
# downloaded at run time): the Open JTalk dictionary, and Whisper "base"
# (multilingual, 145 MB) at a pinned Hugging Face revision.
set(_MODELS_DIR "${CMAKE_CURRENT_BINARY_DIR}/models")
function(jedy_speech_fetch dst url sha256)
  if(NOT EXISTS "${dst}")
    message(STATUS "[jedy_speech] downloading ${url}")
    file(DOWNLOAD "${url}" "${dst}" EXPECTED_HASH SHA256=${sha256}
         STATUS _status TLS_VERIFY ON)
    list(GET _status 0 _code)
    if(NOT _code EQUAL 0)
      file(REMOVE "${dst}")
      message(FATAL_ERROR "[jedy_speech] could not download ${url}: ${_status}")
    endif()
  endif()
endfunction()

jedy_speech_fetch("${_MODELS_DIR}/open_jtalk_dic_utf_8-1.11.tar.gz"
  "https://github.com/r9y9/open_jtalk/releases/download/v1.11.1/open_jtalk_dic_utf_8-1.11.tar.gz"
  fe6ba0e43542cef98339abdffd903e062008ea170b04e7e2a35da805902f382a)
if(NOT EXISTS "${_MODELS_DIR}/open_jtalk_dic_utf_8-1.11/sys.dic")
  file(ARCHIVE_EXTRACT INPUT "${_MODELS_DIR}/open_jtalk_dic_utf_8-1.11.tar.gz"
       DESTINATION "${_MODELS_DIR}")
endif()

set(_WHISPER_URL "https://huggingface.co/Systran/faster-whisper-base/resolve/ebe41f70d5b6dfa9166e2c581c45c9c0cfc57b66")
set(_WHISPER_DIR "${_MODELS_DIR}/faster-whisper-base")
jedy_speech_fetch("${_WHISPER_DIR}/config.json" "${_WHISPER_URL}/config.json"
  56a6d8110d311f19c8f0471e562832c7527f146b567275bfca59fcf7c184da9a)
jedy_speech_fetch("${_WHISPER_DIR}/model.bin" "${_WHISPER_URL}/model.bin"
  d01c3014881c9c6f3133c182f3d2887eb6ca1c789a7538c5c007196857a0a6a9)
jedy_speech_fetch("${_WHISPER_DIR}/tokenizer.json" "${_WHISPER_URL}/tokenizer.json"
  fb7b63191e9bb045082c79fd742a3106a12c99513ab30df4a0d47fa6cb6fd0ab)
jedy_speech_fetch("${_WHISPER_DIR}/vocabulary.txt" "${_WHISPER_URL}/vocabulary.txt"
  34ce3fe1c5041027b3f8d42912270993f986dbc4bb34cf27f951e34a1e453913)

install(DIRECTORY "${_MODELS_DIR}/open_jtalk_dic_utf_8-1.11" "${_WHISPER_DIR}"
        DESTINATION share/${PROJECT_NAME}/models)

ament_package()

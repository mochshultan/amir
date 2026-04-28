cmake_minimum_required(VERSION 3.8)
project(waveshare_modbus_io_interfaces)

# (opsional) flag kompilasi kalau nanti ada kode C/C++
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies utama untuk paket interface
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Daftar berkas IDL (.srv) yang akan digenerate
set(IDL_FILES
  "srv/SetSingle.srv"
  "srv/SetMask.srv"
)

# Generate interface untuk semua bahasa (termasuk Python)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${IDL_FILES}
)

# Ekspor runtime rosidl untuk konsumen (node Python/C++)
ament_export_dependencies(rosidl_default_runtime)

# (opsional) linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Lewati cek copyright/format untuk sementara
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

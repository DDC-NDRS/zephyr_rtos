# SPDX-License-Identifier: Apache-2.0
# Copyright (c) Zephyr Project Contributors

if(NOT CONFIG_USERSPACE)
  zephyr_sources(sem.c)
endif()

zephyr_sources(
  thread_entry.c
)

#CUSTOM@NDRS kernel code always be speed optimization !!! (same as kernel/CMakeLists.txt)
set_source_files_properties(
  sem.c
  thread_entry.c
  PROPERTIES
  COMPILE_FLAGS ${COMPILER_OPTIMIZE_FOR_SPEED_FLAG}
  )

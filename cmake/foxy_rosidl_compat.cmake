# Loaded externally with CMAKE_PROJECT_INCLUDE_BEFORE on Foxy only.
# Keep the official driver source unchanged. Foxy's generated typesupport
# library already has the same target name used by the later helper API.
if(NOT COMMAND rosidl_get_typesupport_target)
  function(rosidl_get_typesupport_target output_variable interface_target typesupport_name)
    set(_flapping_typesupport_target "${interface_target}__${typesupport_name}")
    if(NOT TARGET "${_flapping_typesupport_target}")
      message(FATAL_ERROR "Missing Foxy typesupport target ${_flapping_typesupport_target}")
    endif()
    set(${output_variable} "${_flapping_typesupport_target}" PARENT_SCOPE)
  endfunction()
endif()

include_guard(GLOBAL)

set(_ORVD_DEPENDENCY_SOURCE_SCALAR_FIELDS
    KEY
    NAME
    VERSION
    ARCHIVE
    SOURCE_DIRECTORY
    SOURCE_URL)
set(_ORVD_DEPENDENCY_SOURCE_DECLARATION_FIELDS
    ${_ORVD_DEPENDENCY_SOURCE_SCALAR_FIELDS}
    LICENSE_PATH)
set(_ORVD_DEPENDENCY_SOURCE_EXPORT_FIELDS
    ${_ORVD_DEPENDENCY_SOURCE_SCALAR_FIELDS}
    LICENSE_PATHS)
set(_ORVD_REQUIRED_DEPENDENCY_KEYS
    eigen
    fmt
    nlohmann_json
    sundials)

function(_orvd_require_plain_dependency_value field value dependency_key)
    string(STRIP "${value}" stripped_value)
    if(stripped_value STREQUAL "")
        message(FATAL_ERROR
            "dependency '${dependency_key}' field ${field} must be a non-empty string")
    endif()
    if("${value}" MATCHES "[;\r\n]")
        message(FATAL_ERROR
            "dependency '${dependency_key}' field ${field} must not contain "
            "a semicolon or line break")
    endif()
endfunction()

function(_orvd_require_relative_posix_path field value dependency_key)
    _orvd_require_plain_dependency_value("${field}" "${value}" "${dependency_key}")
    if(IS_ABSOLUTE "${value}" OR "${value}" MATCHES "\\\\")
        message(FATAL_ERROR
            "dependency '${dependency_key}' field ${field} must be a relative POSIX path")
    endif()
    string(REPLACE "/" ";" path_parts "${value}")
    foreach(path_part IN LISTS path_parts)
        if(path_part STREQUAL "" OR path_part STREQUAL "." OR
           path_part STREQUAL "..")
            message(FATAL_ERROR
                "dependency '${dependency_key}' field ${field} contains an "
                "empty, current-directory or parent-directory path component")
        endif()
    endforeach()
endfunction()

function(orvd_begin_dependency_sources)
    set_property(GLOBAL PROPERTY ORVD_DEPENDENCY_SOURCES_BEGUN TRUE)
    set_property(GLOBAL PROPERTY ORVD_DEPENDENCY_SOURCES_FINALIZED FALSE)
    set_property(GLOBAL PROPERTY ORVD_DEPENDENCY_SOURCE_KEYS "")
    set_property(GLOBAL PROPERTY ORVD_DEPENDENCY_SOURCE_ARCHIVES "")
endfunction()

function(orvd_declare_dependency)
    get_property(sources_begun GLOBAL PROPERTY ORVD_DEPENDENCY_SOURCES_BEGUN)
    if(NOT sources_begun)
        message(FATAL_ERROR
            "orvd_begin_dependency_sources() must precede every dependency declaration")
    endif()
    get_property(sources_finalized GLOBAL PROPERTY ORVD_DEPENDENCY_SOURCES_FINALIZED)
    if(sources_finalized)
        message(FATAL_ERROR
            "dependency declarations are not allowed after finalization")
    endif()
    if(ARGC EQUAL 0)
        message(FATAL_ERROR "an empty dependency declaration is not allowed")
    endif()

    foreach(field IN LISTS _ORVD_DEPENDENCY_SOURCE_SCALAR_FIELDS)
        set("field_seen_${field}" FALSE)
    endforeach()
    set(current_field "")
    set(current_field_values "")
    set(license_paths "")
    set(license_path_fields_begun FALSE)
    set(declaration_tokens ${ARGV})
    list(APPEND declaration_tokens "__ORVD_DEPENDENCY_DECLARATION_END__")
    foreach(token IN LISTS declaration_tokens)
        list(FIND _ORVD_DEPENDENCY_SOURCE_DECLARATION_FIELDS
             "${token}" field_position)
        if(NOT field_position EQUAL -1 OR
           token STREQUAL "__ORVD_DEPENDENCY_DECLARATION_END__")
            if(NOT current_field STREQUAL "")
                list(LENGTH current_field_values current_field_value_count)
                if(NOT current_field_value_count EQUAL 1)
                    message(FATAL_ERROR
                        "dependency field ${current_field} must have exactly "
                        "one scalar value")
                endif()
                list(GET current_field_values 0 current_field_value)
                if(current_field STREQUAL "LICENSE_PATH")
                    list(APPEND license_paths "${current_field_value}")
                else()
                    set("field_value_${current_field}" "${current_field_value}")
                endif()
            endif()
            if(token STREQUAL "__ORVD_DEPENDENCY_DECLARATION_END__")
                break()
            endif()

            if(token STREQUAL "LICENSE_PATH")
                set(license_path_fields_begun TRUE)
            else()
                if(license_path_fields_begun)
                    message(FATAL_ERROR
                        "LICENSE_PATH must be the final field family in a "
                        "dependency declaration")
                endif()
                if(field_seen_${token})
                    message(FATAL_ERROR
                        "dependency declaration repeats field ${token}")
                endif()
                set("field_seen_${token}" TRUE)
            endif()
            set(current_field "${token}")
            set(current_field_values "")
        elseif(current_field STREQUAL "")
            message(FATAL_ERROR
                "dependency declaration begins with unknown field or value '${token}'")
        else()
            list(APPEND current_field_values "${token}")
        endif()
    endforeach()

    foreach(field IN LISTS _ORVD_DEPENDENCY_SOURCE_SCALAR_FIELDS)
        if(NOT field_seen_${field})
            message(FATAL_ERROR "dependency declaration is missing field ${field}")
        endif()
    endforeach()
    list(LENGTH license_paths license_path_count)
    if(license_path_count LESS 1)
        message(FATAL_ERROR
            "dependency declaration must contain at least one LICENSE_PATH field")
    endif()

    set(dependency_key "${field_value_KEY}")
    _orvd_require_plain_dependency_value("KEY" "${dependency_key}" "<unresolved>")
    list(FIND _ORVD_REQUIRED_DEPENDENCY_KEYS "${dependency_key}" key_position)
    if(key_position EQUAL -1)
        message(FATAL_ERROR "unknown dependency key '${dependency_key}'")
    endif()
    get_property(declared_keys GLOBAL PROPERTY ORVD_DEPENDENCY_SOURCE_KEYS)
    list(FIND declared_keys "${dependency_key}" duplicate_key_position)
    if(NOT duplicate_key_position EQUAL -1)
        message(FATAL_ERROR "dependency key '${dependency_key}' is declared twice")
    endif()

    foreach(field IN ITEMS NAME VERSION ARCHIVE SOURCE_DIRECTORY SOURCE_URL)
        _orvd_require_plain_dependency_value(
            "${field}" "${field_value_${field}}" "${dependency_key}")
    endforeach()
    if("${field_value_VERSION}" MATCHES "[/\\\\]")
        message(FATAL_ERROR
            "dependency '${dependency_key}' VERSION must not contain a path separator")
    endif()
    _orvd_require_relative_posix_path(
        "ARCHIVE" "${field_value_ARCHIVE}" "${dependency_key}")
    get_filename_component(archive_basename "${field_value_ARCHIVE}" NAME)
    if(NOT archive_basename STREQUAL field_value_ARCHIVE)
        message(FATAL_ERROR
            "dependency '${dependency_key}' ARCHIVE must be a single file name")
    endif()
    _orvd_require_relative_posix_path(
        "SOURCE_DIRECTORY" "${field_value_SOURCE_DIRECTORY}" "${dependency_key}")

    get_property(declared_archives GLOBAL PROPERTY ORVD_DEPENDENCY_SOURCE_ARCHIVES)
    list(FIND declared_archives "${field_value_ARCHIVE}" duplicate_archive_position)
    if(NOT duplicate_archive_position EQUAL -1)
        message(FATAL_ERROR
            "dependency archive '${field_value_ARCHIVE}' is declared more than once")
    endif()

    set(license_output_names "")
    set(validated_license_paths "")
    foreach(license_path IN LISTS license_paths)
        _orvd_require_relative_posix_path(
            "LICENSE_PATH" "${license_path}" "${dependency_key}")
        list(FIND validated_license_paths "${license_path}" duplicate_license_position)
        if(NOT duplicate_license_position EQUAL -1)
            message(FATAL_ERROR
                "dependency '${dependency_key}' repeats licence path '${license_path}'")
        endif()
        get_filename_component(license_output_name "${license_path}" NAME)
        list(FIND license_output_names "${license_output_name}"
             duplicate_output_position)
        if(NOT duplicate_output_position EQUAL -1)
            message(FATAL_ERROR
                "dependency '${dependency_key}' licence paths collide at output "
                "file '${license_output_name}'")
        endif()
        list(APPEND validated_license_paths "${license_path}")
        list(APPEND license_output_names "${license_output_name}")
    endforeach()

    list(APPEND declared_keys "${dependency_key}")
    list(APPEND declared_archives "${field_value_ARCHIVE}")
    set_property(GLOBAL PROPERTY ORVD_DEPENDENCY_SOURCE_KEYS "${declared_keys}")
    set_property(GLOBAL PROPERTY ORVD_DEPENDENCY_SOURCE_ARCHIVES
                 "${declared_archives}")
    string(TOUPPER "${dependency_key}" upper_key)
    foreach(field IN LISTS _ORVD_DEPENDENCY_SOURCE_SCALAR_FIELDS)
        set_property(GLOBAL PROPERTY
            "ORVD_DEPENDENCY_SOURCE_${upper_key}_${field}"
            "${field_value_${field}}")
    endforeach()
    set_property(GLOBAL PROPERTY
        "ORVD_DEPENDENCY_SOURCE_${upper_key}_LICENSE_PATHS"
        "${validated_license_paths}")
endfunction()

function(orvd_finalize_dependency_sources)
    get_property(sources_begun GLOBAL PROPERTY ORVD_DEPENDENCY_SOURCES_BEGUN)
    if(NOT sources_begun)
        message(FATAL_ERROR
            "orvd_begin_dependency_sources() must precede finalization")
    endif()
    get_property(declared_keys GLOBAL PROPERTY ORVD_DEPENDENCY_SOURCE_KEYS)
    set(sorted_declared_keys ${declared_keys})
    set(sorted_required_keys ${_ORVD_REQUIRED_DEPENDENCY_KEYS})
    list(SORT sorted_declared_keys)
    list(SORT sorted_required_keys)
    if(NOT sorted_declared_keys STREQUAL sorted_required_keys)
        message(FATAL_ERROR
            "dependency declarations must contain exactly eigen, fmt, "
            "nlohmann_json and sundials; found '${declared_keys}'")
    endif()
    set_property(GLOBAL PROPERTY ORVD_DEPENDENCY_SOURCES_FINALIZED TRUE)

    set(ORVD_DEPENDENCY_KEYS "${declared_keys}" PARENT_SCOPE)
    foreach(dependency_key IN LISTS declared_keys)
        string(TOUPPER "${dependency_key}" upper_key)
        foreach(field IN LISTS _ORVD_DEPENDENCY_SOURCE_EXPORT_FIELDS)
            get_property(field_value GLOBAL PROPERTY
                "ORVD_DEPENDENCY_SOURCE_${upper_key}_${field}")
            set("ORVD_${upper_key}_${field}" "${field_value}" PARENT_SCOPE)
        endforeach()
    endforeach()
endfunction()

function(orvd_export_dependency_sources output_directory)
    get_property(sources_finalized GLOBAL PROPERTY ORVD_DEPENDENCY_SOURCES_FINALIZED)
    if(NOT sources_finalized)
        message(FATAL_ERROR
            "dependency sources must be finalized before they are exported")
    endif()
    if(EXISTS "${output_directory}")
        message(FATAL_ERROR
            "refusing to replace dependency export directory '${output_directory}'")
    endif()
    file(MAKE_DIRECTORY "${output_directory}")
    get_property(declared_keys GLOBAL PROPERTY ORVD_DEPENDENCY_SOURCE_KEYS)
    string(JOIN "\n" key_lines ${declared_keys})
    file(WRITE "${output_directory}/keys.txt" "${key_lines}\n")
    foreach(dependency_key IN LISTS declared_keys)
        set(record_directory "${output_directory}/${dependency_key}")
        file(MAKE_DIRECTORY "${record_directory}")
        string(TOUPPER "${dependency_key}" upper_key)
        foreach(field IN LISTS _ORVD_DEPENDENCY_SOURCE_SCALAR_FIELDS)
            get_property(field_value GLOBAL PROPERTY
                "ORVD_DEPENDENCY_SOURCE_${upper_key}_${field}")
            string(TOLOWER "${field}" lower_field)
            file(WRITE "${record_directory}/${lower_field}.txt" "${field_value}")
        endforeach()
        get_property(license_paths GLOBAL PROPERTY
            "ORVD_DEPENDENCY_SOURCE_${upper_key}_LICENSE_PATHS")
        string(JOIN "\n" license_lines ${license_paths})
        file(WRITE "${record_directory}/license_paths.txt" "${license_lines}\n")
    endforeach()
    file(WRITE "${output_directory}/COMPLETE" "dependency sources exported\n")
endfunction()

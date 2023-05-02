## `(<str:<string>> <search:<string>>)-><bool>`
##
## Returns true if "str" starts with the string "search"
##
## **Examples**
##  string_starts_with("substring" "sub") # => true
##  string_starts_with("substring" "ub") # => false
##
##
function(string_starts_with str search out_state)
    string(FIND "${str}" "${search}" out)
    if("${out}" EQUAL 0)
        set(${out_state} true PARENT_SCOPE)
    else()
        set(${out_state} false PARENT_SCOPE)
    endif()
endfunction()
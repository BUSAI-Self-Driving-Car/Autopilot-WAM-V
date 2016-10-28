# Build our NUClear module
NUCLEAR_MODULE(LIBRARIES openblas lapack)
add_definitions(-DINT_LONG_INT -DOS_LIN)



#if RUN_TYPE == RUN_APP

#include "main/main_run.cpp"

#elif RUN_TYPE == RUN_DEBUG

#include "main/main_debug.cpp"

#elif RUN_TYPE == RUN_TEST

#include "main/main_test.cpp"

#endif






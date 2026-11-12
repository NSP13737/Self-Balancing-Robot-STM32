//implement custom debug functions
#ifndef DEBUGMODE
#define DEBUGMODE 0
#endif
#if DEBUGMODE
#define DEBUG(...) printf(__VA_ARGS__)
#else
#define DEBUG(...) ((void)0)
#endif
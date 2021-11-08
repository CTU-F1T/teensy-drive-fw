#ifndef _TEENSY_DRIVE_DEBUG_H
#define _TEENSY_DRIVE_DEBUG_H

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG != 1
#define debug(code) ((void) 0)
#else
#define debug(code) (code)
#endif

#endif // _TEENSY_DRIVE_DEBUG_H

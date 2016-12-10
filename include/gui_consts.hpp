#ifndef _GUI_CONSTS_H
#define _GUI_CONSTS_H

/**
 * gui_consts.hpp defines all of the constants needed by
 * the program.
 */

/**
 * Define all of the hotkeys for applying
 * operations to the chosen image in the gui.
 */
#define WAIT_TIME   10
#define ESCAPE      27
#define SPACE       ' '
#define COMPRESS    'c'

/**
 * 8-bit image, range: 0-255
 */
#ifndef PIXEL_MAX
#define PIXEL_MAX 255
#endif

// Enable logging here
#ifndef DEBUG_COND
#define DEBUG_COND 0
#endif

// Define PI here
#ifndef MATH_PI
#define MATH_PI 3.14159265
#endif

// Define macro for logging
#ifndef DEBUG
#define DEBUG(x) if(DEBUG_COND) { std::clog << x << std::endl; }
#endif

// Define macro for error handling
#ifndef ERROR_EXIT
#define ERROR_EXIT(x) do { std::cerr << x << std::endl; exit(1); } while (0)
#endif

#ifndef ERROR_CONT
#define ERROR_CONT(x) do { std::cerr << x << std::endl; } while (0)
#endif

#endif

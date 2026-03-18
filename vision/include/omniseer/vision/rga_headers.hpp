#pragma once

#include <cstddef>

#if defined(__has_include)
#  if __has_include(<rga/im2d.h>)
#    include <rga/im2d.h>
#  elif __has_include(<im2d.h>)
#    include <im2d.h>
#  else
#    error "Rockchip RGA headers not found"
#  endif
#else
#  include <im2d.h>
#endif

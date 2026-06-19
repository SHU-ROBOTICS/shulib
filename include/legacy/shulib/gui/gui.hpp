#pragma once

#include "liblvgl/lvgl.h"

namespace shulib {

class Gui {
public:
  Gui();

  /**
   * @brief Initialize the GUI
   *
   */
  void init();

  void create_main_screen();
  
  void create_styles();

  lv_obj_t *main_screen;
  lv_style_t red_style;
  lv_style_t blue_style;
};

} // namespace shulib
#include "main.h"
#include "s.hpp"


#include "liblvgl/lvgl.h"


namespace sec{


int auton;
const char *btnmMap[] = {"Blue WP (+)","Blue Elims (+)","Blue (-)","\n","Red WP (+)","Red Elims (+)","Red (-)","\n","None","Skills",""}; // up to 10 autons

static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        uint32_t id = lv_btnmatrix_get_selected_btn(obj);
        const char * txt = lv_btnmatrix_get_btn_text(obj, id);

        auton = id;

        lv_btnmatrix_set_selected_btn(obj, id);

    }
}

void init(int hue, int default_auton, const char **autons){
   auton = 0;

    lv_obj_t * btnm1 = lv_btnmatrix_create(lv_scr_act());
    lv_btnmatrix_set_map(btnm1, btnmMap);

    lv_obj_set_size(btnm1, 480, 272 - 60);

    for (int i = 0; i < 8; i++) {
        lv_btnmatrix_set_btn_width(btnm1, i, 20);    
    }

    lv_obj_align(btnm1, LV_ALIGN_CENTER, 0, 30); // Adjust to leave space for the label
    lv_obj_add_event_cb(btnm1, event_handler, LV_EVENT_ALL, NULL);
}
}

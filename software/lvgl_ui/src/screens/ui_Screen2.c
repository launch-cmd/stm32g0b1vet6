// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen2_screen_init(void)
{
    ui_Screen2 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen2, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_bg_color(ui_Screen2, lv_color_hex(0x85888C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen2, 100, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tempChart = lv_chart_create(ui_Screen2);
    lv_obj_set_width(ui_tempChart, 195);
    lv_obj_set_height(ui_tempChart, 80);
    lv_obj_set_x(ui_tempChart, -5);
    lv_obj_set_y(ui_tempChart, 0);
    lv_obj_set_align(ui_tempChart, LV_ALIGN_RIGHT_MID);
    lv_chart_set_type(ui_tempChart, LV_CHART_TYPE_LINE);
    lv_chart_set_range(ui_tempChart, LV_CHART_AXIS_PRIMARY_Y, 200, 300);
    lv_chart_set_range(ui_tempChart, LV_CHART_AXIS_SECONDARY_Y, 0, 0);
    lv_chart_set_div_line_count(ui_tempChart, 5, 10);
    lv_chart_set_axis_tick(ui_tempChart, LV_CHART_AXIS_PRIMARY_X, 10, 5, 10, 1, true, 50);
    lv_chart_set_axis_tick(ui_tempChart, LV_CHART_AXIS_PRIMARY_Y, 10, 5, 3, 5, true, 50);
    lv_chart_set_axis_tick(ui_tempChart, LV_CHART_AXIS_SECONDARY_Y, 0, 0, 0, 0, false, 25);

    lv_obj_set_style_line_color(ui_tempChart, lv_color_hex(0xFFFFFF), LV_PART_TICKS | LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(ui_tempChart, 255, LV_PART_TICKS | LV_STATE_DEFAULT);
    lv_obj_set_style_line_width(ui_tempChart, 3, LV_PART_TICKS | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_tempChart, lv_color_hex(0xFFFFFF), LV_PART_TICKS | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_tempChart, 255, LV_PART_TICKS | LV_STATE_DEFAULT);

    ui_tempLabel2 = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_tempLabel2, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_tempLabel2, LV_SIZE_CONTENT); /// 1
    lv_obj_set_x(ui_tempLabel2, 0);
    lv_obj_set_y(ui_tempLabel2, -2);
    lv_obj_set_align(ui_tempLabel2, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_tempLabel2, "Temperatuur");
    lv_obj_set_style_text_color(ui_tempLabel2, lv_color_hex(0xFF4C39), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_tempLabel2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_tempLabel2, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
}
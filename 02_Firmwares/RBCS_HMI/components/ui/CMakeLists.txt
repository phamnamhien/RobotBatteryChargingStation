# ESP-IDF Component CMakeLists.txt
# Auto-generated from SquareLine Studio export

idf_component_register(
    SRCS 
        "screens/ui_Screen1.c"
        "screens/ui_Screen2.c"
        "screens/ui_Screen3.c"
        "screens/ui_Screen4.c"
        "ui.c"
        "components/ui_comp_hook.c"
        "ui_helpers.c"
        "images/ui_img_scrsplash_background_png.c"
        "images/ui_img_scrmain_batteryempty_png.c"
        "images/ui_img_scrsettingicon_png.c"
    INCLUDE_DIRS 
        "."
        "screens"
        "components"
        "images"
    REQUIRES 
        lvgl
)

# Add compile options if needed
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-variable
    -Wno-unused-function
)

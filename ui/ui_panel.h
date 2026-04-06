#ifndef UI_PANEL_H
#define UI_PANEL_H

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    FILE *backend;
    int enabled;
    int refresh_div;
    const char *csv_path;
    FILE *log_file;
    int refresh_count;
    int last_refresh_step;
    int fail_step;
    int init_ok;
} UiPanel;

void ui_panel_init(UiPanel *ui,
                   const char *csv_path,
                   int refresh_div);

void ui_panel_refresh(UiPanel *ui,
                      int step);

void ui_panel_close(UiPanel *ui);

#ifdef __cplusplus
}
#endif

#endif /* UI_PANEL_H */

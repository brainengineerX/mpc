#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <string.h>

#include "ui_panel.h"

/* 写 UI 调试日志（若日志文件不可用则静默） */
static void ui_log(UiPanel *ui, const char *msg)
{
    if (ui->log_file != 0) {
        fprintf(ui->log_file, "%s\n", msg);
        fflush(ui->log_file);
    }
}

void ui_panel_init(UiPanel *ui,
                   const char *csv_path,
                   int refresh_div)
{
    ui->backend = 0;
    ui->enabled = 0;
    ui->refresh_div = (refresh_div > 0) ? refresh_div : 20;
    ui->csv_path = csv_path;
    ui->refresh_count = 0;
    ui->last_refresh_step = -1;
    ui->fail_step = -1;
    ui->init_ok = 0;
    ui->log_file = fopen("build/ui_debug.txt", "w");

    ui_log(ui, "[UI] init start");

    signal(SIGPIPE, SIG_IGN);
    ui_log(ui, "[UI] SIGPIPE ignored");

    /* 把 gnuplot 后端错误单独重定向到文件，便于定位 GUI 打不开的原因 */
    ui->backend = popen("gnuplot -persist 2> build/ui_backend_err.log", "w");
    if (ui->backend == 0) {
        char buf[256];
        snprintf(buf, sizeof(buf), "[UI] popen failed errno=%d (%s)", errno, strerror(errno));
        ui_log(ui, buf);
        return;
    }

    ui->enabled = 1;
    ui->init_ok = 1;
    ui_log(ui, "[UI] popen success, backend enabled");

    fprintf(ui->backend, "set datafile separator ','\n");
    fprintf(ui->backend, "set grid\n");
    fprintf(ui->backend, "set key outside\n");
    fprintf(ui->backend, "set term qt size 1280,860\n");
    if (fflush(ui->backend) != 0) {
        char buf[256];
        snprintf(buf, sizeof(buf), "[UI] init handshake flush failed errno=%d (%s)", errno, strerror(errno));
        ui_log(ui, buf);
        pclose(ui->backend);
        ui->backend = 0;
        ui->enabled = 0;
        ui->init_ok = 0;
    } else {
        ui_log(ui, "[UI] init handshake flush success");
    }
}

void ui_panel_refresh(UiPanel *ui,
                      int step)
{
    if (!ui->enabled || ui->backend == 0) {
        return;
    }
    if ((step % ui->refresh_div) != 0) {
        return;
    }
    ui->refresh_count++;
    ui->last_refresh_step = step;

    fprintf(ui->backend, "set multiplot layout 2,2 title 'MPC Realtime Wave'\n");

    fprintf(ui->backend, "set title 'Speed'\n");
    fprintf(ui->backend, "set xlabel 'time(s)'\n");
    fprintf(ui->backend, "set ylabel 'rad/s'\n");
    fprintf(ui->backend,
            "plot '%s' every ::1 using 2:8 with lines title 'omega_m', \\\n"
            "     '%s' every ::1 using 2:9 with lines title 'omega_ref'\n",
            ui->csv_path, ui->csv_path);

    fprintf(ui->backend, "set title 'Position'\n");
    fprintf(ui->backend, "set xlabel 'time(s)'\n");
    fprintf(ui->backend, "set ylabel 'rad'\n");
    fprintf(ui->backend,
            "plot '%s' every ::1 using 2:13 with lines title 'theta_ref', \\\n"
            "     '%s' every ::1 using 2:14 with lines title 'theta_mech', \\\n"
            "     '%s' every ::1 using 2:15 with lines title 'e_pos'\n",
            ui->csv_path, ui->csv_path, ui->csv_path);

    fprintf(ui->backend, "set title 'Current'\n");
    fprintf(ui->backend, "set xlabel 'time(s)'\n");
    fprintf(ui->backend, "set ylabel 'A'\n");
    fprintf(ui->backend,
            "plot '%s' every ::1 using 2:11 with lines title 'iq_ref', \\\n"
            "     '%s' every ::1 using 2:10 with lines title 'id_ref', \\\n"
            "     '%s' every ::1 using 2:18 with lines title 'i_alpha', \\\n"
            "     '%s' every ::1 using 2:19 with lines title 'i_beta'\n",
            ui->csv_path, ui->csv_path, ui->csv_path, ui->csv_path);

    fprintf(ui->backend, "set title 'Cost/Param'\n");
    fprintf(ui->backend, "set xlabel 'time(s)'\n");
    fprintf(ui->backend, "set ylabel '-'\n");
    fprintf(ui->backend,
            "plot '%s' every ::1 using 2:21 with lines title 'MPC J', \\\n"
            "     '%s' every ::1 using 2:7 with lines title 'Rs', \\\n"
            "     '%s' every ::1 using 2:6 with lines title 'Vdc'\n",
            ui->csv_path, ui->csv_path, ui->csv_path);

    fprintf(ui->backend, "unset multiplot\n");
    if (fflush(ui->backend) != 0) {
        char buf[256];
        ui->fail_step = step;
        snprintf(buf, sizeof(buf), "[UI] refresh flush failed at step=%d errno=%d (%s)",
                 step, errno, strerror(errno));
        ui_log(ui, buf);
        pclose(ui->backend);
        ui->backend = 0;
        ui->enabled = 0;
    } else if ((ui->refresh_count % 10) == 0) {
        char buf[128];
        snprintf(buf, sizeof(buf), "[UI] refresh ok count=%d last_step=%d",
                 ui->refresh_count, step);
        ui_log(ui, buf);
    }
}

void ui_panel_close(UiPanel *ui)
{
    char buf[256];
    snprintf(buf, sizeof(buf),
             "[UI] close: init_ok=%d enabled=%d refresh_count=%d last_step=%d fail_step=%d",
             ui->init_ok, ui->enabled, ui->refresh_count, ui->last_refresh_step, ui->fail_step);
    ui_log(ui, buf);

    if (ui->backend != 0) {
        fflush(ui->backend);
        pclose(ui->backend);
        ui->backend = 0;
    }
    ui->enabled = 0;
    if (ui->log_file != 0) {
        fclose(ui->log_file);
        ui->log_file = 0;
    }
}

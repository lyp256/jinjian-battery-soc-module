#include "bms_interface.h"

#include <string.h>

static bms_driver_t s_active;
static bool s_has_driver;

void bms_manager_set_driver(const bms_driver_t *drv)
{
    if (s_has_driver && s_active.deinit) {
        s_active.deinit();
    }
    memset(&s_active, 0, sizeof(s_active));
    s_has_driver = false;
    if (drv != NULL) {
        s_active = *drv;
        if (s_active.init && s_active.init(s_active.config) != 0) {
            memset(&s_active, 0, sizeof(s_active));
            return;
        }
        s_has_driver = true;
    }
}

const char *bms_manager_name(void)
{
    return s_has_driver && s_active.name ? s_active.name : "none";
}

bool bms_manager_get_snapshot(bms_snapshot_t *out)
{
    if (!s_has_driver || !s_active.get_snapshot) {
        return false;
    }
    return s_active.get_snapshot(out);
}

void bms_manager_set_charge_time_min(uint16_t minutes)
{
    if (s_has_driver && s_active.set_charge_time_min) {
        s_active.set_charge_time_min(minutes);
    }
}

void bms_manager_set_charge_target_soc(uint16_t soc)
{
    if (s_has_driver && s_active.set_charge_target_soc) {
        s_active.set_charge_target_soc(soc);
    }
}

void bms_manager_end_fast_charge(void)
{
    if (s_has_driver && s_active.end_fast_charge) {
        s_active.end_fast_charge();
    }
}

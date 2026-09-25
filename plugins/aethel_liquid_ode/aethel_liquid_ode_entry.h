#pragma once

extern bool aethel_liquid_ode_init(const char *plugin_path);
extern void aethel_liquid_ode_deinit(void);
extern const void *aethel_liquid_ode_get_factory(const char *factory_id);

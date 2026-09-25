#include <clap/clap.h>
#include "aethel_liquid_ode_entry.h"

extern "C" {
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
#endif

const CLAP_EXPORT struct clap_plugin_entry clap_entry = {
    CLAP_VERSION,
    aethel_liquid_ode_init,
    aethel_liquid_ode_deinit,
    aethel_liquid_ode_get_factory
};

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
}

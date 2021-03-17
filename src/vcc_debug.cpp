#include "config.h"
#include "vcc_debug.h"

INITIALIZE_EASYLOGGINGPP

struct InitEeasyloggingPP {
  InitEeasyloggingPP() {
    el::Configurations conf(LOG_CONF);
    el::Loggers::reconfigureLogger("default", conf);
    el::Loggers::reconfigureAllLoggers(conf);
    srand(time(nullptr));
  }
} __init_elpp__;
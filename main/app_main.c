/*
 * Dispatch only. The selected sample is the only one compiled in; see
 * main/Kconfig and main/CMakeLists.txt.
 */
#include "sdkconfig.h"

void sample_run(void);

void app_main(void)
{
    sample_run();
}

#ifndef __CONTROL_TASK
#define __CONTROL_TASK

#include "cmsis_os.h"
#include "can_protocol.h"
#include "module.h"

void StartControlTask(void const * argument);

#endif

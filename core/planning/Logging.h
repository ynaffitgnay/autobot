#pragma once
#include <memory/TextLogger.h>
#define plog(level, fstring, ...) \
  if(tlogger_) \
    tlogger_->logFromPlanning(level, fstring, ##__VA_ARGS__)

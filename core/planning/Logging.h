#pragma once
#include <memory/TextLogger.h>
#define log(level, fstring, ...) \
  if(tlogger_) \
    tlogger_->logFromPlanning(level, fstring, ##__VA_ARGS__)

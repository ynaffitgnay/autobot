from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import pose
import head
import commands
import cfgstiff
from task import Task

class Playing(Task):
  def run(self):
    commands.setHeadPan(-3.0,1.5)
    self.finish()

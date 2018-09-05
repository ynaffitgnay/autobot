from __future__ import print_function
from __future__ import division
from __future__ import absolute_import


from task import Task
import memory
import core


class Playing(Task):
  """Main behavior task."""

  def run(self):
    print('My head yaw value is %f!' % core.joint_values[core.HeadYaw])

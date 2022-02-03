# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from field import Field
from controller import Supervisor
import numpy as np

# start the webots supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())

field = Field("kid")
children = supervisor.getRoot().getField('children')
children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "kid" }}')
children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.3 0 0.450 rotation 0 0 1 0 controller "capture_image" controllerArgs "x-0.30_y-0.00_the_0.00.jpg"}}')
player = supervisor.getFromDef('PLAYER')
player_translation = supervisor.getFromDef('PLAYER').getField('translation')
player_rotation = supervisor.getFromDef('PLAYER').getField('rotation')
player_controller = supervisor.getFromDef('PLAYER').getField('controller')

try:
    for x in np.arange(-4.3, 4.3, 0.2):
        for y in np.arange(-3, 3, 0.2):
            for the in np.arange(-1.57, 1.57, 3.14/10):
                count = 0
                player.remove()
                filename = "x"+format(x,"+.2f")+"_y"+format(y,"+.2f")+"_the"+format(the,"+.3f")+".jpg"
                children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation {x} {y} 0.450 rotation 0 0 1 {the} controller "capture_image" controllerArgs "{filename}"}}')
                player = supervisor.getFromDef('PLAYER')
                while supervisor.step(time_step) != -1:
                    count += 1
                    if count > 10:
                        break
except Exception:
    error(f"Unexpected exception in main referee loop: {traceback.format_exc()}", fatal=True)

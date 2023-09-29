import numpy as np
from controller import Supervisor

class Player():
    def __init__(self, name = None, supervisor = None):
        super().__init__()
        self.name = name
        self.supervisor = supervisor
        self.player = None
        self.emitter = None
        self.waiting_time = 0

    def reset(self, pos = [0.0, 0.0, 0.0]):
        children = self.supervisor.getRoot().getField('children')
        if self.player != None:
            self.player.remove()
        if "blue" in self.name:
            ch = int(self.name[-1])
            children.importMFNodeFromString(-1, f'DEF {self.name} GankenKun_simple {{translation {pos[0]} {pos[1]} 0.450 rotation 0 0 1 {pos[2]} jerseyTexture "textures/GankenKun_{self.name}.png" jerseyColor 0, 0, 1 channel {ch} controller "GankenKun_soccer"}}')
        else:
            ch = int(self.name[-1])+3
            children.importMFNodeFromString(-1, f'DEF {self.name} GankenKun_simple {{translation {pos[0]} {pos[1]} 0.450 rotation 0 0 1 {pos[2]} jerseyTexture "textures/GankenKun_{self.name}.png" jerseyColor 1, 0, 0 channel {ch} controller "GankenKun_soccer"}}')
        self.pos = pos
        self.emitter = self.supervisor.getDevice(f'{self.name}_emitter')
        self.player = self.supervisor.getFromDef(f'{self.name}')
        self.player_pos = self.player.getField('translation')
        self.player_rot = self.player.getField('rotation')

        self.alive = True
        self.score = 0
        self.action = 0
        self.is_fall = False
        self.waiting_time = 1

    def send(self, message):
        if self.waiting_time > 0:
            self.waiting_time -= 1
            return
        if "kick" in message.decode('utf-8'):
            self.waiting_time = 4
        if self.emitter != None:
            self.emitter.send(message)

    def update(self):
        #self.action = action
        x, y, _ = self.player_pos.getSFVec3f()
        yaw, pitch, roll = self.rotation_to_euler(self.player_rot.getSFRotation())
        self.pos = [x, y, yaw]
        if abs(pitch) > 1.0 or abs(roll) > 1.0:
            self.is_fall = True
    
    def is_done(self):
        return not self.alive
    
    def rotation_to_euler(self, rotation):
        x, y, z, angle = rotation
        c = np.cos(angle)
        s = np.sin(angle)
        t = 1 - c
        R = np.array([
            [t*x*x + c, t*x*y - z*s, t*x*z + y*s],
            [t*x*y + z*s, t*y*y + c, t*y*z - x*s],
            [t*x*z - y*s, t*y*z + x*s, t*z*z + c]
        ])
        yaw = np.arctan2(R[1, 0], R[0, 0])
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        roll = np.arctan2(R[2, 1], R[2, 2])
        return yaw, pitch, roll

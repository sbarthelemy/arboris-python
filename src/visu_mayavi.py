# coding=utf-8
"""
Visualization of a simulation
"""
import numpy as np
import visu
from enthought.mayavi import mlab


class World(visu.World):
    """ A drawable version of rigidmotion.World
    """
    def add_body(self, added_body):
        self.bodies.append(Body(added_body))
        
        
class Body(visu.Body):
    """ A drawable version of rigidmotion.Body
    """
    def draw_body(self):
        if self._body.pose != None:
            self.frames.append(self.draw_frame(self._body.pose, self._body.frames[0].name));
            for f in self._body.frames[1:]:
                nf = self.draw_frame(f.pose, f.name, parent=self.frames[0])
                self.frames.append(nf);
                self.links.append(self.draw_link(self.frames[0],  nf))

    def draw_frame(self, pose=np.eye(4), label=None, parent=None):
        """Draw the arrows and label of a frame.
        """
        if parent == None:
            pos = pose
        else:
            pos = np.dot(parent['pose'], pose)
        f = {}
        f['pose'] = pose    #attention ici, c'est la pose dans le frame
        f['vx'] = mlab.quiver3d([pos[0,3]],[pos[1,3]],[pos[2,3]],[pos[0,0]],[pos[1,0]],[pos[2,0]], color=(1,0,0))
        f['vy'] = mlab.quiver3d([pos[0,3]],[pos[1,3]],[pos[2,3]],[pos[0,1]],[pos[1,1]],[pos[2,1]], color=(0,1,0))
        f['vz'] = mlab.quiver3d([pos[0,3]],[pos[1,3]],[pos[2,3]],[pos[0,2]],[pos[1,2]],[pos[2,2]], color=(0,0,1))
        f['parent'] = parent
        f['label'] = label
        return f
    
    def draw_link(self, start_frame, end_frame, color=(1,1,1)):
        link = {}
        link['start_frame'] = start_frame
        link['end_frame'] = end_frame
        if start_frame['parent'] == None:
            start = start_frame['pose']
        else:
            start = np.dot(start_frame['parent']['pose'], start_frame['pose'])
        if end_frame['parent'] == None:
            end = end_frame['pose']
        else:
            end = np.dot(end_frame['parent']['pose'], end_frame['pose'])
        x = (start[0,3], end[0,3])
        y = (start[1,3], end[1,3])
        z = (start[2,3], end[2,3])
        link['points'] = mlab.plot3d(x, y, z)
        return link
        
    def update(self):
        self.frames[0]['pose'] = self._body.pose
        for f in self.frames:
            if f['parent'] == None:
                pos = self._body.pose
            else:
                pos = np.dot(f['parent']['pose'], f['pose'])
            origin = ([[pos[0,3],pos[1,3],pos[2,3]]])
            f['vx'].mlab_source.points = origin
            f['vx'].mlab_source.vectors = ([[pos[0,0],pos[1,0],pos[2,0]]])
            f['vy'].mlab_source.points = origin
            f['vy'].mlab_source.vectors = ([[pos[0,1],pos[1,1],pos[2,1]]])
            f['vz'].mlab_source.points = origin
            f['vz'].mlab_source.vectors = ([[pos[0,2],pos[1,2],pos[2,2]]])
        for l in self.links:
            if l['start_frame']['parent'] == None:
                start = l['start_frame']['pose']
            else:
                start = np.dot(l['start_frame']['parent']['pose'], l['start_frame']['pose'])
            if l['end_frame']['parent'] == None:
                end = l['end_frame']['pose']
            else:
                end = np.dot(l['end_frame']['parent']['pose'], l['end_frame']['pose'])
            x = ([[start[0,3], end[0,3]]])
            y = ([[start[1,3], end[1,3]]])
            z = ([[start[2,3], end[2,3]]])
            l['points'].mlab_source.x = x
            l['points'].mlab_source.y = y
            l['points'].mlab_source.z = z
            
        
    def draw_shape():
        pass
    
    
if __name__=='__main__':
    # testing!
    from worldfactory import triplehinge
    w = triplehinge()
    w.geometric()

    import visu_mayavi as vmaya
    vw = vmaya.World(w)
    for t in range(100):
        w.joints[0].gpos=[t/20.]
        w.joints[1].gpos=[t/20.]
        w.geometric()
        vw.update()

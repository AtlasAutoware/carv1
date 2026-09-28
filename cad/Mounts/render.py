"""Quick shaded PNG of one or more CadQuery solids, no GPU needed.

    render([(solid, (r, g, b)), ...], 'file.png', elev=28, azim=-55)
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def _tris(shape, tol=0.12):
    verts, tris = shape.tessellate(tol, 0.2)
    v = np.array([(p.x, p.y, p.z) for p in verts])
    return v, np.array(tris)


def render(parts, path, elev=28, azim=-55, title=None, views=None):
    views = views or [(elev, azim)]
    fig = plt.figure(figsize=(6 * len(views), 5.5), dpi=140)
    light = np.array([0.4, -0.6, 0.7]); light /= np.linalg.norm(light)
    allv = []
    for i, (el, az) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection='3d')
        # One collection for everything: matplotlib depth-sorts faces within a collection
        # only, so separate collections per part draw in insertion order and a plate
        # ends up painted over the parts sitting on it.
        allP, allC = [], []
        for obj, col in parts:
            shape = obj.val() if hasattr(obj, 'val') else obj
            v, t = _tris(shape)
            allv.append(v)
            P = v[t]
            n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
            n /= (np.linalg.norm(n, axis=1, keepdims=True) + 1e-9)
            shade = 0.45 + 0.55 * np.clip(n @ light, 0, 1)
            allP.append(P); allC.append(np.clip(np.array(col)[None, :] * shade[:, None], 0, 1))
        pc = Poly3DCollection(np.vstack(allP), facecolors=np.vstack(allC), edgecolors='none')
        ax.add_collection3d(pc)
        V = np.vstack(allv)
        c = (V.min(0) + V.max(0)) / 2; r = (V.max(0) - V.min(0)).max() / 2
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(elev=el, azim=az); ax.set_axis_off()
    if title: fig.suptitle(title, fontsize=11)
    fig.tight_layout(); fig.savefig(path, bbox_inches='tight'); plt.close(fig)
    return path

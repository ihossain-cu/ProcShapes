import sys
import argparse
from tqdm import tqdm
import numpy as np
import bpy
import bmesh
import mathutils
import trimesh
import open3d as o3d
from paramvectordef import get_param_vec_def

def normalize_mesh(mesh: trimesh.Trimesh):
    t = np.sum(mesh.bounding_box.bounds, axis=0) / 2
    mesh.apply_translation(-t)
    s = np.max(mesh.extents)
    mesh.apply_scale(1 / s)


class Procedure:
    def __init__(self, category: str):
        self.category = category
        self.paramvecdef = get_param_vec_def(category)
        self.procmap = {
            'bed': self.create_bed,
            'chair': self.create_chair,
            'storage': self.create_storage,
            'table': self.create_table
        }

    def _get_transform(self, loc, scl):
        mat_loc = mathutils.Matrix.Translation(loc)
        mat_scx = mathutils.Matrix.Scale(scl[0], 4, mathutils.Vector((1, 0, 0)))
        mat_scy = mathutils.Matrix.Scale(scl[1], 4, mathutils.Vector((0, 1, 0)))
        mat_scz = mathutils.Matrix.Scale(scl[2], 4, mathutils.Vector((0, 0, 1)))
        mat = mat_loc @ mat_scx @ mat_scy @ mat_scz
        return mat

    def _cube(self, loc=(0.0, 0.0, 0.0), scl=(1.0, 1.0, 1.0)):
        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=1.0, matrix=self._get_transform(loc, scl))
        return bm

    def _cylinder(self, loc=(0.0, 0.0, 0.0), scl=(1.0, 1.0, 1.0), seg=30):
        bm = bmesh.new()
        bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=seg, radius1=0.5, radius2=0.5, depth=1.0, matrix=self._get_transform(loc, scl))
        return bm

    def _merge_bmeshes(self, bmeshes):
        bm_final = bmesh.new()
        for bm in bmeshes:
            offset = len(bm_final.verts)
            for v in bm.verts:
                bm_final.verts.new(v.co)
            bm_final.verts.index_update()
            bm_final.verts.ensure_lookup_table()
            if bm.faces:
                for face in bm.faces:
                    new_face = bm_final.faces.new(tuple(bm_final.verts[i.index + offset] for i in face.verts))
                bm_final.faces.index_update()
            if bm.edges:
                for edge in bm.edges:
                    try:
                        bm_final.edges.new(tuple(bm_final.verts[i.index + offset] for i in edge.verts))
                    except ValueError:
                        pass
                bm_final.edges.index_update()
        bm_final.normal_update()
        return bm_final

    def _bmesh_to_trimesh(self, bm):
        verts = np.array([list(v.co) for v in bm.verts])
        verts = np.array([1, 1, -1]) * verts[:, [0, 2, 1]]
        triangles = bm.calc_loop_triangles()
        faces = np.array([[l.vert.index for l in triangle] for triangle in triangles])
        colors = np.zeros((faces.shape[0], 4))
        tm = trimesh.Trimesh(vertices=verts, faces=faces, face_colors=colors, smooth=False)
        return tm
        
    def remap_input(self, x, min, max, isratio=False):
        if not isratio:
            return min + (max - min) * x
        else:
            if x < 0.5:
                return min + 2 * (1 - min) * x
            else:
                return 1 + 2 * (max - 1) * (x - 0.5)

    def create_bed(self, paramvector):
        bmeshes = []
        width = self.remap_input(paramvector[0], 0.5, 1.0)
        leg_height = self.remap_input(paramvector[1], 0.2, 0.4)
        headboard_height = self.remap_input(paramvector[2], 0.0, 0.45)
        frontboard_height = self.remap_input(paramvector[3], 0.0, 0.45)
        mattress_height = self.remap_input(paramvector[4], 0.1, 0.2)
        leg_type = paramvector[5]
        w, l_h, hb_h, fb_h, m_h, l_tp = width, leg_height, headboard_height, frontboard_height, mattress_height, leg_type

        d = 1.0
        s_h = 0.15
        th = 0.05
        h = l_h + s_h + max(m_h, max(hb_h, fb_h))

        # create legs
        z = (-h + l_h) / 2
        xvals = [(w - th) / 2, -(w - th) / 2]
        yvals = [(d - th) / 2, -(d - th) / 2]
        if l_tp == 'basic':
            for i, x in enumerate(xvals):
                for j, y in enumerate(yvals):
                    bm = self._cube(loc=(x, y, z), scl=(th, th, l_h))
                    bmeshes.append(bm)
        else:
            for i, y in enumerate(yvals):
                bm = self._cube(loc=(0, y, z), scl=(w, th, l_h))
                bmeshes.append(bm)
            if l_tp == 'box':
                for i, x in enumerate(xvals):
                    bm = self._cube(loc=(x, 0, z), scl=(th, d, l_h))
                    bmeshes.append(bm)

        # create bed platform
        bm = self._cube(loc=(0, 0, -h / 2 + l_h + s_h / 2), scl=(w, d, s_h))
        bmeshes.append(bm)

        # create headboard
        if hb_h > 0.00001:
            bm = self._cube(loc=(0, yvals[0], -h / 2 + l_h + s_h + hb_h / 2), scl=(w, th, hb_h))
            bmeshes.append(bm)

        # create frontboard
        if fb_h > 0.00001:
            bm = self._cube(loc=(0, yvals[1], -h / 2 + l_h + s_h + fb_h / 2), scl=(w, th, fb_h))
            bmeshes.append(bm)

        # create mattress
        bm = self._cube(loc=(0, 0, -h / 2 + l_h + s_h + m_h / 2), scl=(w - 2 * th, d - 2 * th, m_h))
        bev_edges = [x for x in bm.edges]
        bmesh.ops.bevel(bm, geom=bev_edges, offset=0.02, segments=5, profile=0.5, affect='EDGES', clamp_overlap=False)
        bmeshes.append(bm)

        mesh = self._bmesh_to_trimesh(self._merge_bmeshes(bmeshes))
        normalize_mesh(mesh)
        return mesh

    def create_chair(self, paramvector):
        bmeshes = []
        whratio = self.remap_input(paramvector[0], 0.5, 0.8, True)
        depth = self.remap_input(paramvector[1], 0.5, 0.7)
        leg_height = self.remap_input(paramvector[2], 0.3, 0.5)
        leg_type = paramvector[3]
        arm_type = paramvector[4]
        back_type = paramvector[5]

        wh_r, d, l_h, l_t, a_t, b_t = whratio, depth, leg_height, leg_type, arm_type, back_type

        if wh_r < 1.0:
            w, h = wh_r, 1.0
        else:
            w, h = 1.0, 1.0 / wh_r
        l_h = h * l_h
        s_h = h * 0.1
        b_h = h - l_h - s_h
        a_h = b_h * 0.3
        th = 0.05

        # create legs
        if l_t == 'pedestal':
            bm = self._cylinder(loc=(0, 0, -h / 2 + 0.0125), scl=(0.7, 0.7, 0.025))
            bmeshes.append(bm)
            bm = self._cylinder(loc=(0, 0, -h / 2 + l_h / 2), scl=(2 * th, 2 * th, l_h))
            bmeshes.append(bm)
        elif l_t == 'split' or l_t == 'rocker':
            xvals = [(w - th) / 2, -(w - th) / 2]
            y = 0 if l_t == 'split' else -d / 2 + th / 2
            for i, x in enumerate(xvals):
                bm = self._cube(loc=(x, y, -h / 2 + l_h / 2), scl=(th, th, l_h))
                bmeshes.append(bm)
                bm = self._cube(loc=(x, 0, -h / 2 + th / 2), scl=(th, d, th))
                bmeshes.append(bm)
            if l_t == 'rocker':
                bm = self._cube(loc=(0, d / 2 - th / 2, -h / 2 + th / 2), scl=(w, th, th))
                bmeshes.append(bm)
        else:
            xvals = [(w - th) / 2, -(w - th) / 2]
            yvals = [(d - th) / 2, -(d - th) / 2]
            for i, x in enumerate(xvals):
                if l_t == 'support':
                    bm = self._cube(loc=(x, 0, -h / 2 + l_h / 2), scl=(th, d, th))
                    bmeshes.append(bm)
                for j, y in enumerate(yvals):
                    bm = self._cube(loc=(x, y, -h / 2 + l_h / 2), scl=(th, th, l_h))
                    bmeshes.append(bm)

        # create seat
        bm = self._cube(loc=(0, 0, -h / 2 + l_h + s_h / 2), scl=(w, d, s_h))
        bmeshes.append(bm)

        # create arm
        if a_t == 'office':
            xvals = [w / 2 - th, -w / 2 + th]
            for i, x in enumerate(xvals):
                bm = self._cube(loc=(x, 0, -h / 2 + l_h + s_h + a_h / 2), scl=(th, th, a_h))
                bmeshes.append(bm)
                bm = self._cube(loc=(x, 0, -h / 2 + l_h + s_h + a_h - th / 2), scl=(2 * th, 2 * d / 3, th))
                bev_edges = [x for x in bm.edges]
                bmesh.ops.bevel(bm, geom=bev_edges, offset=0.008, segments=5, profile=0.5, affect='EDGES', clamp_overlap=False)
                bmeshes.append(bm)
        elif a_t == 'solid':
            xvals = [(w - th) / 2, -(w - th) / 2]
            for i, x in enumerate(xvals):
                bm = self._cube(loc=(x, 0, -h / 2 + l_h + s_h + a_h / 2), scl=(th, d, a_h))
                bmeshes.append(bm)
        elif a_t == 'basic':
            xvals = [(w - th) / 2, -(w - th) / 2]
            for i, x in enumerate(xvals):
                bm = self._cube(loc=(x, -d / 2 + th / 2, -h / 2 + l_h + s_h + a_h / 2), scl=(th, th, a_h))
                bmeshes.append(bm)
                bm = self._cube(loc=(x, 0, -h / 2 + l_h + s_h + a_h - th / 2), scl=(th, d, th))
                bmeshes.append(bm)

        # create back
        if b_t == 'hbar' or b_t == 'vbar':
            xvals = [w / 2 - th, -w / 2 + th]
            for i, x in enumerate(xvals):
                bm = self._cube(loc=(x, d / 2 - th / 2, h / 2 - b_h / 2), scl=(2 * th, th, b_h))
                bmeshes.append(bm)
            bm = self._cube(loc=(0, d / 2 - th / 2, h / 2 - th), scl=(w, th, 2 * th))
            bmeshes.append(bm)
            if b_t == 'hbar':
                zvals = [h / 2 - b_h / 2 - th, h / 2 - b_h / 2 + th, h / 2 - b_h / 2 - 3 * th]
                for i, z in enumerate(zvals):
                    bm = self._cube(loc=(0, d / 2 - th / 2, z), scl=(w, th, th))
                    bmeshes.append(bm)
            else:
                xvals = [(w - th * 2) / 4, 0, -(w - th * 2) / 4]
                for i, x in enumerate(xvals):
                    bm = self._cube(loc=(x, d / 2 - th / 2, h / 2 - b_h / 2), scl=(th, th, b_h))
                    bmeshes.append(bm)
        else:
            bm = self._cube(loc=(0, d / 2 - th / 2, h / 2 - b_h / 2), scl=(w, th, b_h))
            bmeshes.append(bm)

        mesh = self._bmesh_to_trimesh(self._merge_bmeshes(bmeshes))
        normalize_mesh(mesh)
        return mesh

    def create_storage(self, paramvector):
        bmeshes = []
        whratio = self.remap_input(paramvector[0], 0.2, 5.0, True)
        depth = self.remap_input(paramvector[1], 0.1, 0.3)
        leg_height = self.remap_input(paramvector[2], 0.0, 0.2)
        num_rows = paramvector[3]
        num_columns = paramvector[4]
        fill_back = paramvector[5]
        fill_sides = paramvector[6]
        fill_columns = paramvector[7]
        wh_r, d, l_h, n_r, n_c, f_b, f_s, f_c = whratio, depth, leg_height, num_rows, num_columns, fill_back, fill_sides, fill_columns

        if wh_r < 1.0:
            w, h = wh_r, 1.0
        else:
            w, h = 1.0, 1.0 / wh_r

        l_h = h * l_h
        c_h = h - l_h
        th = 0.01

        # create legs
        if l_h > 0.001:
            xvals = [(-w + th) / 2, -(-w + th) / 2]
            yvals = [(-d + th) / 2, -(-d + th) / 2]

            for i, x in enumerate(xvals):
                for j, y in enumerate(yvals):
                    bm = self._cube(loc=(x, y, -h / 2 + l_h / 2), scl=(th, th, l_h))
                    bmeshes.append(bm)

        # create back
        if f_b:
            bm = self._cube(loc=(0, d / 2 - th / 2, -h / 2 + l_h + c_h / 2), scl=(w, th, c_h))
            bmeshes.append(bm)

        # create rows
        zvals = np.linspace(start=-h / 2 + l_h + th / 2, stop=h / 2 - th / 2, num=n_r + 1, endpoint=True)
        for i, z in enumerate(zvals):
            bm = self._cube(loc=(0, 0, z), scl=(w, d, th))
            bmeshes.append(bm)

        # create sides
        xxvals = np.linspace(start=-(w - th) / 2, stop=(w - th) / 2, num=n_c + 1, endpoint=True)
        xvals = [xxvals[0], xxvals[-1]]
        for i, x in enumerate(xvals):
            if not f_s:
                bm = self._cube(loc=(x, -d / 2 + th / 2, -h / 2 + l_h + c_h / 2), scl=(th, th, c_h))
                bmeshes.append(bm)
                if not f_b:
                    bm = self._cube(loc=(x, d / 2 - th / 2, -h / 2 + l_h + c_h / 2), scl=(th, th, c_h))
                    bmeshes.append(bm)
            else:
                bm = self._cube(loc=(x, 0, -h / 2 + l_h + c_h / 2), scl=(th, d, c_h))
                bmeshes.append(bm)

        # create partitions
        if n_c > 1:
            xvals = xxvals[1:-1]
            for i, x in enumerate(xvals):
                if not f_c:
                    bm = self._cube(loc=(x, -d / 2 + th / 2, -h / 2 + l_h + c_h / 2), scl=(th, th, c_h))
                    bmeshes.append(bm)
                    if not f_b:
                        bm = self._cube(loc=(x, d / 2 - th / 2, -h / 2 + l_h + c_h / 2), scl=(th, th, c_h))
                        bmeshes.append(bm)
                else:
                    bm = self._cube(loc=(x, 0, -h / 2 + l_h + c_h / 2), scl=(th, d, c_h))
                    bmeshes.append(bm)

        mesh = self._bmesh_to_trimesh(self._merge_bmeshes(bmeshes))
        normalize_mesh(mesh)
        return mesh

    def create_table(self, paramvector):
        bmeshes = []
        whratio = self.remap_input(paramvector[0], 1.0, 4.0, True)
        depth = self.remap_input(paramvector[1], 0.4, 1.0)
        top_thickness = self.remap_input(paramvector[2], 0.03, 0.06)
        leg_thickness = self.remap_input(paramvector[3], 0.07, 0.12)
        basictop = paramvector[4]
        leg_type = paramvector[5]
        wh_r, d, t_th, l_th, bt, l_tp = whratio, depth, top_thickness, leg_thickness, basictop, leg_type

        if wh_r < 1.0:
            w, h = wh_r, 1.0
        else:
            w, h = 1.0, 1.0 / wh_r
        l_h = h - t_th

        # create legs
        a = 1.0 if bt else 1.41
        xvals = [(w - a * l_th) / (2 * a), -(w - a * l_th) / (2 * a)]
        yvals = [(d - a * l_th) / (2 * a), -(d - a * l_th) / (2 * a)]
        if l_tp == 'pedestal':
            bm = self._cylinder(loc=(0, 0, -h / 2 + 0.0125), scl=(0.7, 0.7, 0.025))
            bmeshes.append(bm)
            bm = self._cylinder(loc=(0, 0, -h / 2 + l_h / 2), scl=(l_th, l_th, l_h))
            bmeshes.append(bm)
        elif l_tp == 'bracket' or l_tp == 'split':
            # create split legs
            y = 0 if l_tp == 'split' else yvals[0]
            for i, x in enumerate(xvals):
                bm = self._cube(loc=(x, y, -h / 2 + l_h / 2), scl=(l_th, l_th, l_h))
                bmeshes.append(bm)
                bm = self._cube(loc=(x, 0, -h / 2 + l_th / 2), scl=(l_th, d, l_th))
                bmeshes.append(bm)
        else:
            if l_tp == 'solid':
                # create solid legs
                for i, x in enumerate(xvals):
                    bm = self._cube(loc=(x, 0, -h / 2 + l_h / 2), scl=(l_th, d / a, l_h))
                    bmeshes.append(bm)
            else:
                # create four legs
                for i, x in enumerate(xvals):
                    for j, y in enumerate(yvals):
                        bm = self._cube(loc=(x, y, -h / 2 + l_h / 2), scl=(l_th, l_th, l_h))
                        bmeshes.append(bm)
                # create support between legs
                if l_tp == 'support':
                    for i, x in enumerate(xvals):
                        bm = self._cube(loc=(x, 0, -h / 2 + l_h / 2), scl=(l_th, d / a - l_th * 2, l_th))
                        bmeshes.append(bm)

        # create table-top
        top_loc, top_scl = (0, 0, h / 2 - t_th / 2), (w, d, t_th)
        if bt:
            bm = self._cube(loc=top_loc, scl=top_scl)
            bmeshes.append(bm)

        else:
            bm = self._cylinder(loc=top_loc, scl=top_scl)
            bmeshes.append(bm)

        mesh = self._bmesh_to_trimesh(self._merge_bmeshes(bmeshes))
        normalize_mesh(mesh)
        return mesh

    def get_shape(self, paramvector) -> trimesh.Trimesh:
        return self.procmap[self.category](paramvector)


def unit_test(args):
    num_samples = args.num_samples
    category = 'storage'
    proc = Procedure(category)
    '''
    Parameter vectors can be manually defined, such as
    vectors = [
        [1.0, 1.0, 1.0, 5, 5, False, False, False]
    ]
    or we can randomly sample them.
    '''
    vectors = proc.paramvecdef.get_random_vectors(num_samples)
    #vectors = [[1.0, 0.0, 1.0, 4, 4, False, False, True]]
    print(vectors)
    print(proc.paramvecdef.decode(proc.paramvecdef.encode(vectors)))
    meshes = []
    for vector in tqdm(vectors, file=sys.stdout, desc='Generating procedural shapes'):
        mesh = proc.get_shape(vector)
        meshes.append(mesh)
    omeshes = []
    for mesh in meshes:
        omesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(np.array(mesh.vertices, dtype=np.float64)),
            o3d.utility.Vector3iVector(np.array(mesh.faces, np.int32))
        )
        omesh.compute_vertex_normals()
        omeshes.append(omesh)
    for omesh in omeshes:
        o3d.visualization.draw_geometries([omesh])
    print()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_samples", type=int, default=5, help="Number of examples")
    parsed_args = parser.parse_args()
    unit_test(parsed_args)


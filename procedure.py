import sys
import argparse
from tqdm import tqdm
import numpy as np
import math
import bpy
import bmesh
import mathutils
import trimesh
import open3d as o3d
from paramvectordef import ParamVectorDef

def get_transform(loc, scl):
    mat_loc = mathutils.Matrix.Translation(loc)
    mat_scx = mathutils.Matrix.Scale(scl[0], 4, mathutils.Vector((1, 0, 0)))
    mat_scy = mathutils.Matrix.Scale(scl[1], 4, mathutils.Vector((0, 1, 0)))
    mat_scz = mathutils.Matrix.Scale(scl[2], 4, mathutils.Vector((0, 0, 1)))
    mat = mat_loc @ mat_scx @ mat_scy @ mat_scz
    return mat

def cube(loc=(0.0, 0.0, 0.0), scl=(1.0, 1.0, 1.0)):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0, matrix=get_transform(loc, scl))
    return bm

def cylinder(loc=(0.0, 0.0, 0.0), scl=(1.0, 1.0, 1.0), seg=30):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=seg, radius1=0.5, radius2=0.5, depth=1.0, matrix=get_transform(loc, scl))
    return bm

def merge_bmeshes(bmeshes):
    bm_final = bmesh.new()
    for bm in bmeshes:
        offset = len(bm_final.verts)
        for v in bm.verts:
            bm_final.verts.new(v.co)
        bm_final.verts.index_update()
        bm_final.verts.ensure_lookup_table()
        if bm.faces:
            for face in bm.faces:
                bm_final.faces.new(tuple(bm_final.verts[i.index+offset] for i in face.verts))
            bm_final.faces.index_update()
        if bm.edges:
            for edge in bm.edges:
                try:
                    bm_final.edges.new(tuple(bm_final.verts[i.index+offset] for i in edge.verts))
                except ValueError:
                    pass
            bm_final.edges.index_update()
    bm_final.normal_update()
    return bm_final

def bmesh_to_trimesh(bm):
    verts = np.array([list(v.co) for v in bm.verts])
    verts = np.array([1, 1, -1]) * verts[:, [0, 2, 1]]
    faces = np.array([[l.vert.index for l in loop] for loop in bm.calc_loop_triangles()])
    tm = trimesh.Trimesh(vertices=verts, faces=faces)
    return tm

def normalize_mesh(mesh: trimesh.Trimesh):
    t = np.sum(mesh.bounding_box.bounds, axis=0) / 2
    mesh.apply_translation(-t)
    s = np.max(mesh.extents)
    mesh.apply_scale(1 / s)

class Procedure:
    def __init__(self, shape):
        self.shape = shape
        self.paramvecdef = ParamVectorDef(shape)
        self.procmap = {
            'bed': self.create_bed,
            'chair': self.create_chair,
            'shelf': self.create_shelf,
            'table': self.create_table
        }
        bpy.ops.wm.read_factory_settings(use_empty=True)
        bpy.ops.mesh.primitive_plane_add(size=1.0)
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.subdivide(number_cuts=20)
        bpy.ops.mesh.extrude_region_move(MESH_OT_extrude_region={"mirror": False},
                                         TRANSFORM_OT_translate={"value": (0, 0, 0.05)})
        bpy.ops.object.modifier_add(type='CLOTH')
        bpy.context.object.modifiers["Cloth"].settings.use_pressure = True
        bpy.context.object.modifiers["Cloth"].settings.uniform_pressure_force = 1.0
        bpy.context.object.modifiers["Cloth"].settings.effector_weights.gravity = 0
        bpy.context.object.modifiers["Cloth"].collision_settings.use_self_collision = True
        bpy.ops.object.mode_set(mode="OBJECT")
        obj = bpy.context.active_object
        for frame in range(1, 10):
            bpy.context.scene.frame_current = frame
            dg = bpy.context.evaluated_depsgraph_get()
            obj = obj.evaluated_get(dg)
        self.pillow = bmesh.new()
        self.pillow.from_mesh(obj.data)
        bmesh.ops.scale(self.pillow, vec=mathutils.Vector((0.24, 0.12, 0.2)), verts=self.pillow.verts)

    def create_bed(self, paramvector):
        bmeshes = []
        whratio = paramvector[0]
        leg_height = paramvector[1]
        headboard_height = paramvector[2]
        frontboard_height = paramvector[3]
        mattress_height = paramvector[4]
        leg_type = paramvector[5]
        num_pillows = paramvector[6]
        wh_r, l_h, hb_h, fb_h, m_h, l_tp,  n_p = whratio, leg_height, headboard_height, frontboard_height, mattress_height, leg_type, num_pillows

        if wh_r < 1.0:
            w, h = wh_r, 1.0
        else:
            w, h = 1.0, 1.0 / wh_r

        l_h = h * l_h
        hb_h = h * hb_h
        fb_h = h * fb_h
        m_h = h * m_h

        # create legs
        thickness = h * 0.05
        x = w / 2.0 - thickness / 2.0
        y = 0.5 - thickness / 2.0
        z = (-h + l_h) / 2.0
        if l_tp == 'basic':
            leg_locs, leg_scls = [(x, y, z), (x, -y, z), (-x, y, z), (-x, -y, z)], [(thickness, thickness, l_h) for i in range(4)]
        else:
            leg_locs, leg_scls = [(0.0, y, z), (0.0, -y, z)], [(w, thickness, l_h) for i in range(2)]
            if l_tp == 'box':
                leg_locs.extend([(x, 0.0, z), (-x, 0.0, z)])
                leg_scls.extend([(thickness, 1.0 - thickness * 2.0, l_h) for i in range(2)])
        for leg_loc, leg_scl in zip(leg_locs, leg_scls):
            bmeshes.append(cube(loc=leg_loc, scl=leg_scl))
        # create bed platform
        mid_loc, mid_scl = (0.0, 0.0, (-h / 2.0 + l_h + thickness / 2.0)), (w, 1.0, thickness)
        bmeshes.append(cube(loc=mid_loc, scl=mid_scl))
        # create headboard
        z = -h / 2.0 + l_h + thickness + hb_h / 2.0
        bmeshes.append(cube(loc=(0.0, y, z), scl=(w, thickness, hb_h)))
        # create frontboard
        z = -h / 2.0 + l_h + thickness + fb_h / 2.0
        bmeshes.append(cube(loc=(0.0, -y, z), scl=(w, thickness, fb_h)))
        # create mattress
        z = -h / 2.0 + l_h + thickness + m_h / 2.0
        bm = cube(loc=(0.0, 0.0, z), scl=(w - thickness, 1.0 - thickness * 2, m_h))
        bev_edges = [x for x in bm.edges]
        bmesh.ops.bevel(bm, geom=bev_edges, offset=0.02, segments=5, profile=0.5, affect='EDGES', clamp_overlap=False)
        bmeshes.append(bm)
        # add pillows
        x = (w - thickness * 2.0) / 4.0
        z = -h / 2.0 + l_h + thickness + m_h + 0.02
        if n_p == 1:
            pil_locs = [(0.0, 0.4 - thickness, z)]
        elif n_p == 2:
            pil_locs = [(x, 0.4 - thickness, z), (-x, 0.4 - thickness, z)]
        else:
            pil_locs = []
        for i, pil_loc in enumerate(pil_locs):
            bm = self.pillow.copy()
            bmesh.ops.transform(bm, matrix=get_transform(pil_loc, (1, 1, 1)), verts=bm.verts)
            bmeshes.append(bm)

        mesh = bmesh_to_trimesh(merge_bmeshes(bmeshes))
        normalize_mesh(mesh)
        return mesh

    def create_chair(self, paramvector):
        bmeshes = []
        whratio = paramvector[0]
        depth = paramvector[1]
        leg_height = paramvector[2]
        leg_type = paramvector[3]
        arm_type = paramvector[4]
        back_type = paramvector[5]

        wh_r, d, l_h, l_t, a_t, b_t = whratio, depth, leg_height, leg_type, arm_type, back_type

        if wh_r < 1.0:
            w, h = wh_r, 1.0
        else:
            w, h = 1.0, 1.0 / wh_r

        l_h = h * l_h
        th = 0.02

        # create arm
        if a_t == 'office':
            x = (w - 3 * th) / 2.0
            for i in [x, -x]:
                bmeshes.append(cube(loc=(i, 0, -h / 2.0 + l_h + h * 0.15), scl=(th, th, h * 0.3)))
                bm = cube(loc=(i, 0, -h / 2.0 + l_h + h * 0.3 + th / 2.0), scl=(3 * th, d * 0.4, th))
                bev_edges = [x for x in bm.edges]
                bmesh.ops.bevel(bm, geom=bev_edges, offset=0.01, segments=5, profile=0.5, affect='EDGES', clamp_overlap=False)
                bmeshes.append(bm)
            w = w - th * 4.0
        elif a_t == 'solid':
            x = (w - th) / 2.0
            for i in [-1, 1]:
                bmeshes.append(cube(loc=(i * x, 0, -h / 2.0 + l_h + h * 0.2), scl=(th, d * 0.8, h * 0.2)))
        elif a_t == 'basic':
            x = (w - th) / 2.0
            for i in [x, -x]:
                bmeshes.append(cube(loc=(i, (th - d * 0.8) / 2.0, -h / 2.0 + l_h + h * 0.2), scl=(th, th, h * 0.2)))
                bmeshes.append(cube(loc=(i, 0.0, -h / 2.0 + l_h + h * 0.3 + th / 2.0), scl=(th, d * 0.8, th)))

        # create seat
        bm = cube(loc=(0.0, 0.0, -h / 2.0 + l_h + h * 0.05), scl=(w, d, h * 0.1))
        bm.edges.ensure_lookup_table()
        bev_edges = [bm.edges[i] for i in [11]]
        bmesh.ops.bevel(bm, geom=bev_edges, offset=0.02, segments=5, profile=0.5, affect='EDGES', clamp_overlap=False)
        bmeshes.append(bm)
        # create back
        if b_t == 'office':
            bm = cube(loc=(0.0, d / 2.0 - th / 2.0, -h / 2.0 + l_h + h * 0.2), scl=(w, th, h * 0.2))
            bm.faces.ensure_lookup_table()
            bmesh.ops.translate(bm, vec=(0.0, -d * 0.1, 0.0), verts=bm.faces[5].verts)
            ret = bmesh.ops.extrude_face_region(bm, geom=[bm.faces[5]])
            bmesh.ops.translate(bm, vec=(0.0, d * 0.1, h - l_h - h * 0.3), verts=[v for v in ret["geom"] if isinstance(v, bmesh.types.BMVert)])
            bm.faces.ensure_lookup_table()
            bmesh.ops.delete(bm, geom=[bm.faces[5]], context='FACES')
            bm.edges.ensure_lookup_table()
            bev_edges = [bm.edges[i] for i in [5, 11, 12, 13, 14, 15]]
            bmesh.ops.bevel(bm, geom=bev_edges, offset=0.01, segments=5, profile=0.5, affect='EDGES', clamp_overlap=False)
            bmeshes.append(bm)
        elif b_t == 'hbar' or b_t == 'vbar':
            x = w / 2.0 - th / 2.0
            for i in [x, -x]:
                bmeshes.append(cube(loc=(i, d / 2.0 - th / 2.0, l_h / 2.0), scl=(th, th, (h - l_h - h * 0.2))))
            bm = cube(loc=(0, d / 2.0 - th / 2.0, (h - h * 0.1) / 2.0), scl=(w, th, h * 0.1))
            bm.edges.ensure_lookup_table()
            bev_edges = [bm.edges[i] for i in [2, 5, 8, 11]]
            bmesh.ops.bevel(bm, geom=bev_edges, offset=0.01, segments=5, profile=0.5, affect='EDGES', clamp_overlap=False)
            bmeshes.append(bm)
            for i in [-1, 0, 1]:
                if b_t == 'hbar':
                    bmeshes.append(cube(loc=(0, d / 2.0 - th / 2.0, l_h / 2.0 + i * (h - l_h - h * 0.2) / 4.0), scl=(w - th * 2.0, th, h * 0.05)))
                else:
                    bmeshes.append(cube(loc=(i * (w - th * 2.0) / 4.0, d / 2.0 - th / 2.0, l_h / 2.0), scl=(w * 0.1, th, (h - l_h - h * 0.2))))
        else:
            bm = cube(loc=(0.0, d / 2.0 - th / 2.0, (l_h + h * 0.1) / 2.0), scl=(w, th, (h - l_h - h * 0.1)))
            bm.edges.ensure_lookup_table()
            bev_edges = [bm.edges[i] for i in [2, 5, 8, 11]]
            bmesh.ops.bevel(bm, geom=bev_edges, offset=0.01, segments=5, profile=0.5, affect='EDGES', clamp_overlap=False)
            bmeshes.append(bm)
        # create legs
        if l_t == 'office':
            bm = cylinder(loc=(0.0, 0.0, -h / 2.0 + 0.1), scl=(0.05, 0.05, 0.05), seg=5)
            bm.faces.ensure_lookup_table()
            ext_faces = [bm.faces[i] for i in range(len(bm.faces)) if i not in [3, 6]]
            centers = []
            for face in ext_faces:
                ret = bmesh.ops.extrude_face_region(bm, geom=[face])
                translate_verts = [v for v in ret['geom'] if isinstance(v, bmesh.types.BMVert)]
                bmesh.ops.translate(bm, vec=- 0.3 * face.normal, verts=translate_verts)
                center = [f for f in ret['geom'] if isinstance(f, bmesh.types.BMFace)][0].calc_center_median()
                centers.append(center)
                for v in translate_verts:
                    v.co = center + 0.5 * (v.co - center)
                    v.co[2] -= 0.0375
            bmeshes.append(bm)
            for c in centers:
                bm = cylinder(scl=(0.05, 0.05, 0.05))
                bmesh.ops.rotate(bm, cent=mathutils.Vector((0, 0, 0)), matrix=mathutils.Matrix.Rotation(math.pi / 2.0, 4, 'Y'), verts=bm.verts)
                bmesh.ops.translate(bm, vec=(c[0], c[1], -h / 2.0 + 0.025), verts=bm.verts)
                bmeshes.append(bm)
            bmeshes.append(cylinder(loc=(0.0, 0.0, -h / 2.0 + 0.125 + (l_h - 0.125) / 2.0), scl=(0.05, 0.05, (l_h - 0.125))))
        elif l_t == 'round':
            bmeshes.append(cylinder(loc=(0.0, 0.0, -h / 2.0 + 0.0125), scl=(0.4, 0.4, 0.025)))
            bmeshes.append(cylinder(loc=(0.0, 0.0, -h / 2.0 + (l_h + 0.025) / 2.0), scl=(0.05, 0.05, l_h - 0.025)))
        else:
            x, y = w / 2.0 - th / 2.0, d / 2.0 - th / 2.0
            for i in [x, -x]:
                for j in [y, -y]:
                    bmeshes.append(cube(loc=(i, j, -h / 2.0 + l_h / 2.0), scl=(th, th, l_h)))
            if l_t == 'support':
                for i in [x, -x]:
                    bmeshes.append(cube(loc=(i, 0, -h / 2.0 + l_h / 2.0), scl=(th, d - th * 2.0, th)))

        mesh = bmesh_to_trimesh(merge_bmeshes(bmeshes))
        normalize_mesh(mesh)
        return mesh

    def create_shelf(self, paramvector):
        bmeshes = []
        whratio = paramvector[0]
        depth = paramvector[1]
        leg_height = paramvector[2]
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
        th = 0.01
        c_h = (h - l_h - th * (n_r + 1)) / n_r
        c_w = (w - th * (n_c + 1)) / n_c
        z = -h / 2.0 + l_h + th / 2.0
        if f_b:
            c_d = d - th
            y = -th / 2.0
            bmeshes.append(cube(loc=(0, (d - th) / 2.0, l_h / 2.0), scl=(w, th, h - l_h)))
        else:
            c_d = d
            y = 0
        for i in range(n_r + 1):
            bmeshes.append(cube(loc=(0, y, z), scl=(w, c_d, th)))
            if i < n_r:
                x = -w / 2.0 + th / 2.0
                for j in range(n_c + 1):
                    fill = f_s if j == 0 or j == n_c else f_c
                    if not fill:
                        bmeshes.append(cube(loc=(x, -(d - th) / 2.0, z + (th + c_h) / 2.0), scl=(th, th, c_h)))
                        if not f_b:
                            bmeshes.append(cube(loc=(x, (d - th) / 2.0, z + (th + c_h) / 2.0), scl=(th, th, c_h)))
                    else:
                        bmeshes.append(cube(loc=(x, y, z + (th + c_h) / 2.0), scl=(th, c_d, c_h)))
                    x += th + c_w
            z += th + c_h
        x = (-w + th) / 2.0
        y = (-d + th) / 2.0
        for i in [x, -x]:
            for j in [y, -y]:
                bmeshes.append(cube(loc=(i, j, (-h + l_h) / 2.0), scl=(th, th, l_h)))

        mesh = bmesh_to_trimesh(merge_bmeshes(bmeshes))
        normalize_mesh(mesh)
        return mesh

    def create_table(self, paramvector):
        bmeshes = []
        whratio = paramvector[0]
        depth = paramvector[1]
        top_thickness = paramvector[2]
        leg_thickness = paramvector[3]
        roundtop = paramvector[4]
        leg_type = paramvector[5]
        wh_r, d, t_th, l_th, rt, l_tp = whratio, depth, top_thickness, leg_thickness, roundtop, leg_type

        if wh_r < 1.0:
            w, h = wh_r, 1.0
        else:
            w, h = 1.0, 1.0 / wh_r

        t_th = h * t_th
        l_th = d * leg_thickness

        # create table-top
        top_loc, top_scl = (0.0, 0.0, (h - t_th) / 2.0), (w, d, t_th)
        if rt:
            bmeshes.append(cylinder(loc=top_loc, scl=top_scl))
        else:
            bmeshes.append(cube(loc=top_loc, scl=top_scl))

        if l_tp == 'round' or l_tp == 'square':
            bmeshes.append(cube(loc=(0.0, 0.0, (h * 0.01 - t_th) / 2.0), scl=(l_th, l_th, h - t_th - h * 0.02)))
            if l_tp == 'round':
                # create rounded leg
                bmeshes.append(cylinder(loc=(0.0, 0.0, -h / 2.0 + h * 0.01), scl=(0.5, 0.5, h * 0.02)))
            else:
                # create square leg
                bmeshes.append(cube(loc=(0.0, 0.0, -h / 2.0 + h * 0.01), scl=(0.5, 0.5, h * 0.02)))
        elif l_tp == 'split':
            # create split legs
            x = (w - l_th) / 2.0
            for i in [x, -x]:
                bmeshes.append(cube(loc=(i, 0.0, (l_th - t_th) / 2.0), scl=(l_th, l_th, h - t_th - l_th)))
                bmeshes.append(cube(loc=(i, 0.0, (l_th - h) / 2.0), scl=(l_th, d, l_th)))
        else:
            a = 1.41 if rt else 1.0
            x = (w - a * l_th) / (2 * a)
            y = (d - a * l_th) / (2 * a)
            if l_tp == 'solid':
                # create solid legs
                for i in [x, -x]:
                    bmeshes.append(cube(loc=(i, 0, -t_th / 2.0), scl=(l_th, d / a, h - t_th)))
            else:
                # create four legs
                for i in [x, -x]:
                    for j in [y, -y]:
                        bmeshes.append(cube(loc=(i, j, -t_th / 2.0), scl=(l_th, l_th, h - t_th)))
                # create support between legs
                if l_tp == 'support':
                    bmeshes.append(cube(loc=(x, 0.0, -h / 3.0), scl=(l_th, d / a - l_th * 2.0, l_th)))
                    bmeshes.append(cube(loc=(-x, 0.0, -h / 3.0), scl=(l_th, d / a - l_th * 2.0, l_th)))
                    bmeshes.append(cube(loc=(0.0, 0.0, -h / 3.0), scl=((w - a * l_th * 2.0) / a, l_th, l_th)))

        mesh = bmesh_to_trimesh(merge_bmeshes(bmeshes))
        normalize_mesh(mesh)
        return mesh

    def get_shape(self, paramvector):
        return self.procmap[self.shape](paramvector)


def unit_test(args):
    num_samples = args.num_samples
    proc = Procedure('chair')
    '''
    Parameter vectors can be manually defined, such as
    vectors = [
        [0.6, 0.4, 0.05, 0.05, False, 'basic'],
        [0.6, 0.4, 0.05, 0.05, True, 'basic'],
        [0.6, 0.4, 0.05, 0.05, True, 'support'],
        [0.6, 0.4, 0.05, 0.05, True, 'round'],
        [0.6, 0.4, 0.05, 0.05, False, 'split']
    ]
    or we can randomly sample them.
    '''
    vectors = proc.paramvecdef.get_random_vectors(num_samples)
    meshes = []
    for vector in tqdm(vectors, file=sys.stdout, desc='Generating procedural shapes'):
        meshes.append(proc.get_shape(vector))
    omeshes = []
    for mesh in meshes:
        omesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(mesh.vertices), o3d.utility.Vector3iVector(mesh.faces))
        omesh.compute_vertex_normals()
        omeshes.append(omesh)
    for omesh in omeshes:
        o3d.visualization.draw_geometries([omesh])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_samples", type=int, default=5, help="Number of examples")
    parsed_args = parser.parse_args()
    unit_test(parsed_args)


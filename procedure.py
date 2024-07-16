import sys
import argparse
from tqdm import tqdm
import numpy as np
import math
import bpy
import bmesh
import open3d as o3d
from mathutils import Vector
from paramvectordef import ParamVectorDef


def cube(location=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=location, scale=scale)


def cylinder(location=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)):
    bpy.ops.mesh.primitive_cylinder_add(radius=0.5, depth=1.0, location=location, scale=scale)


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
        bmesh.ops.scale(self.pillow, vec=Vector((0.24, 0.12, 0.2)), verts=self.pillow.verts)

    def create_bed(self, paramvector):
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
            cube(location=leg_loc, scale=leg_scl)
        # create bed platform
        mid_loc, mid_scl = (0.0, 0.0, (-h / 2.0 + l_h + thickness / 2.0)), (w, 1.0, thickness)
        cube(location=mid_loc, scale=mid_scl)
        # create headboard
        z = -h / 2.0 + l_h + thickness + hb_h / 2.0
        cube(location=(0.0, y, z), scale=(w, thickness, hb_h))
        # create frontboard
        z = -h / 2.0 + l_h + thickness + fb_h / 2.0
        cube(location=(0.0, -y, z), scale=(w, thickness, fb_h))
        # create mattress
        z = -h / 2.0 + l_h + thickness + m_h / 2.0
        cube(location=(0.0, 0.0, z), scale=(w - thickness, 1.0 - thickness * 2, m_h))
        bpy.ops.object.mode_set(mode='EDIT')
        me = bpy.context.active_object.data
        bm = bmesh.from_edit_mesh(me)
        bev_edges = [x for x in bm.edges]
        bmesh.ops.bevel(bm, geom=bev_edges, offset=0.02, segments=5, profile=0.5, affect='EDGES', clamp_overlap=False)
        bmesh.update_edit_mesh(me)
        bpy.ops.object.mode_set(mode='OBJECT')
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
            pillow_bm = self.pillow.copy()
            pil_msh = bpy.data.meshes.new('Pillow{}'.format(i+1))
            pil_obj = bpy.data.objects.new('Pillow{}'.format(i+1), pil_msh)
            pillow_bm.to_mesh(pil_obj.data)
            pillow_bm.free()
            bpy.context.collection.objects.link(pil_obj)
            pil_obj.location = Vector(pil_loc)

    def create_chair(self, paramvector):
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

        if a_t == 'office':
            x = (w - 3 * th) / 2.0
            for i in [x, -x]:
                cube(location=(i, 0, -h / 2.0 + l_h + h * 0.15), scale=(th, th, h * 0.3))
                cube(location=(i, 0, -h / 2.0 + l_h + h * 0.3 + th / 2.0), scale=(3 * th, d * 0.4, th))
                bpy.ops.object.mode_set(mode='EDIT')
                me = bpy.context.active_object.data
                bm = bmesh.from_edit_mesh(me)
                bm.edges.ensure_lookup_table()
                bev_edges = [bm.edges[i] for i in [11]]
                bmesh.ops.bevel(bm, geom=[edge for edge in bm.edges], offset=0.01, segments=5, profile=0.5,
                                affect='EDGES', clamp_overlap=False)
                bmesh.update_edit_mesh(me)
                bpy.ops.object.mode_set(mode='OBJECT')
            w = w - th * 4.0
        elif a_t == 'solid':
            x = (w - th) / 2.0
            for i in [-1, 1]:
                cube(location=(i * x, 0, -h / 2.0 + l_h + h * 0.2), scale=(th, d * 0.8, h * 0.2))
        elif a_t == 'basic':
            x = (w - th) / 2.0
            for i in [x, -x]:
                cube(location=(i, (th - d * 0.8) / 2.0, -h / 2.0 + l_h + h * 0.2),
                     scale=(th, th, h * 0.2))
                cube(location=(i, 0.0, -h / 2.0 + l_h + h * 0.3 + th / 2.0), scale=(th, d * 0.8, th))

        # create seat
        cube(location=(0.0, 0.0, -h / 2.0 + l_h + h * 0.05), scale=(w, d, h * 0.1))
        bpy.ops.object.mode_set(mode='EDIT')
        me = bpy.context.active_object.data
        bm = bmesh.from_edit_mesh(me)
        bm.edges.ensure_lookup_table()
        bev_edges = [bm.edges[i] for i in [11]]
        bmesh.ops.bevel(bm, geom=bev_edges, offset=0.02, segments=5, profile=0.5, affect='EDGES', clamp_overlap=False)
        bmesh.update_edit_mesh(me)
        bpy.ops.object.mode_set(mode='OBJECT')
        # create back
        if b_t == 'office':
            cube(location=(0.0, d / 2.0 - th / 2.0, -h / 2.0 + l_h + h * 0.2), scale=(w, th, h * 0.2))
            bpy.ops.object.mode_set(mode='EDIT')
            me = bpy.context.active_object.data
            bm = bmesh.from_edit_mesh(me)
            bm.faces.ensure_lookup_table()
            bmesh.ops.translate(bm, vec=(0.0, -d * 0.1, 0.0), verts=bm.faces[5].verts)
            ret = bmesh.ops.extrude_face_region(bm, geom=[bm.faces[5]])
            bmesh.ops.translate(bm, vec=(0.0, d * 0.1, h - l_h - h * 0.3),
                                verts=[v for v in ret["geom"] if isinstance(v, bmesh.types.BMVert)])
            bm.faces.ensure_lookup_table()
            bmesh.ops.delete(bm, geom=[bm.faces[5]], context='FACES')
            bm.edges.ensure_lookup_table()
            bev_edges = [bm.edges[i] for i in [5, 11, 12, 13, 14, 15]]
            bmesh.ops.bevel(bm, geom=bev_edges, offset=0.01, segments=5, profile=0.5, affect='EDGES',
                            clamp_overlap=False)
            bmesh.update_edit_mesh(me)
            bpy.ops.object.mode_set(mode='OBJECT')
        elif b_t == 'hbar' or b_t == 'vbar':
            x = w / 2.0 - th / 2.0
            for i in [x, -x]:
                cube(location=(i, d / 2.0 - th / 2.0, l_h / 2.0), scale=(th, th, (h - l_h - h * 0.2)))
            cube(location=(0, d / 2.0 - th / 2.0, (h - h * 0.1) / 2.0), scale=(w, th, h * 0.1))
            bpy.ops.object.mode_set(mode='EDIT')
            me = bpy.context.active_object.data
            bm = bmesh.from_edit_mesh(me)
            bm.edges.ensure_lookup_table()
            bev_edges = [bm.edges[i] for i in [2, 5, 8, 11]]
            bmesh.ops.bevel(bm, geom=bev_edges, offset=0.01, segments=5, profile=0.5, affect='EDGES',
                            clamp_overlap=False)
            bmesh.update_edit_mesh(me)
            bpy.ops.object.mode_set(mode='OBJECT')
            for i in [-1, 0, 1]:
                if b_t == 'hbar':
                    cube(location=(0, d / 2.0 - th / 2.0, l_h / 2.0 + i * (h - l_h - h * 0.2) / 4.0),
                         scale=(w - th * 2.0, th, h * 0.05))
                else:
                    cube(location=(i * (w - th * 2.0) / 4.0, d / 2.0 - th / 2.0, l_h / 2.0),
                         scale=(w * 0.1, th, (h - l_h - h * 0.2)))

        else:
            cube(location=(0.0, d / 2.0 - th / 2.0, (l_h + h * 0.1) / 2.0), scale=(w, th, (h - l_h - h * 0.1)))
            bpy.ops.object.mode_set(mode='EDIT')
            me = bpy.context.active_object.data
            bm = bmesh.from_edit_mesh(me)
            bm.edges.ensure_lookup_table()
            bev_edges = [bm.edges[i] for i in [2, 5, 8, 11]]
            bmesh.ops.bevel(bm, geom=bev_edges, offset=0.01, segments=5, profile=0.5, affect='EDGES',
                            clamp_overlap=False)
            bmesh.update_edit_mesh(me)
            bpy.ops.object.mode_set(mode='OBJECT')
        # create legs
        if l_t == 'office':
            bpy.ops.mesh.primitive_cylinder_add(vertices=5, radius=0.05, depth=0.05,
                                                location=(0.0, 0.0, -h / 2.0 + 0.1), scale=(1.0, 1.0, 1.0))
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_mode(type="FACE")
            obj = bpy.context.active_object
            me = obj.data
            bm = bmesh.from_edit_mesh(me)
            bm.faces.ensure_lookup_table()
            bm.faces[3].select = False
            bm.faces[6].select = False
            centers = []
            bpy.ops.mesh.extrude_faces_move(MESH_OT_extrude_faces_indiv={"mirror": False},
                                            TRANSFORM_OT_shrink_fatten={"value": 0.3})
            for face in bm.faces:
                if face.select:
                    center = face.calc_center_median()
                    centers.append(center)
                    for v in face.verts:
                        v.co = center + 0.5 * (v.co - center)
                        v.co[2] -= 0.0375
            bmesh.update_edit_mesh(me)
            bpy.ops.object.mode_set(mode='OBJECT')
            for c in centers:
                cylinder(location=(c[0], c[1], -h / 2.0 + 0.025), scale=(0.05, 0.05, 0.05))
                bpy.context.active_object.rotation_euler[0] = math.radians(90)
            cylinder(location=(0.0, 0.0, -h / 2.0 + 0.125 + (l_h - 0.125) / 2.0), scale=(0.05, 0.05, (l_h - 0.125)))
        elif l_t == 'round':
            cylinder(location=(0.0, 0.0, -h / 2.0 + 0.0125), scale=(0.4, 0.4, 0.025))
            cylinder(location=(0.0, 0.0, -h / 2.0 + (l_h + 0.025) / 2.0), scale=(0.05, 0.05, l_h - 0.025))
        else:
            x, y = w / 2.0 - th / 2.0, d / 2.0 - th / 2.0
            for i in [x, -x]:
                for j in [y, -y]:
                    cube(location=(i, j, -h / 2.0 + l_h / 2.0), scale=(th, th, l_h))
            if l_t == 'support':
                for i in [x, -x]:
                    cube(location=(i, 0, -h / 2.0 + l_h / 2.0), scale=(th, d - th * 2.0, th))

    def create_shelf(self, paramvector):
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
            cube(location=(0, (d - th) / 2.0, l_h / 2.0), scale=(w, th, h - l_h))
        else:
            c_d = d
            y = 0
        for i in range(n_r + 1):
            cube(location=(0, y, z), scale=(w, c_d, th))
            if i < n_r:
                x = -w / 2.0 + th / 2.0
                for j in range(n_c + 1):
                    fill = f_s if j == 0 or j == n_c else f_c
                    if not fill:
                        cube(location=(x, -(d - th) / 2.0, z + (th + c_h) / 2.0), scale=(th, th, c_h))
                        if not f_b:
                            cube(location=(x, (d - th) / 2.0, z + (th + c_h) / 2.0), scale=(th, th, c_h))
                    else:
                        cube(location=(x, y, z + (th + c_h) / 2.0), scale=(th, c_d, c_h))
                    x += th + c_w
            z += th + c_h
        x = (-w + th) / 2.0
        y = (-d + th) / 2.0
        for i in [x, -x]:
            for j in [y, -y]:
                cube(location=(i, j, (-h + l_h) / 2.0), scale=(th, th, l_h))

    def create_table(self, paramvector):
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
        l_th = min(w, d) * leg_thickness

        # create table-top
        top_loc, top_scl = (0.0, 0.0, (h - t_th) / 2.0), (w, d, t_th)
        if rt:
            cylinder(location=top_loc, scale=top_scl)
        else:
            cube(location=top_loc, scale=top_scl)

        if l_tp == 'round':
            # create rounded leg
            cylinder(location=(0.0, 0.0, (h * 0.01 - t_th) / 2.0), scale=(l_th, l_th, h - t_th - h * 0.02))
            cylinder(location=(0.0, 0.0, -h / 2.0 + h * 0.01), scale=(0.5, 0.5, h * 0.02))
        elif l_tp == 'split':
            # create split legs
            x = (w - l_th) / 2.0
            for i in [x, -x]:
                cube(location=(i, 0.0, (l_th - t_th) / 2.0), scale=(l_th, l_th, h - t_th - l_th))
                cube(location=(i, 0.0, (l_th - h) / 2.0), scale=(l_th, d, l_th))
        else:
            # create four legs
            a = 1.41 if rt else 1.0
            x = (w - a * l_th) / (2 * a)
            y = (d - a * l_th) / (2 * a)
            leg_locs = [(x, y, 0.0), (x, -y, 0.0), (-x, y, 0.0), (-x, -y, 0.0)]
            for i in [x, -x]:
                for j in [y, -y]:
                    cube(location=(i, j, -t_th / 2.0), scale=(l_th, l_th, h - t_th))
            # create support between legs
            if l_tp == 'support':
                cube(location=(x, 0.0, -h / 3.0), scale=(l_th, d / a - l_th * 2.0, l_th))
                cube(location=(-x, 0.0, -h / 3.0), scale=(l_th, d / a - l_th * 2.0, l_th))
                cube(location=(0.0, 0.0, -h / 3.0), scale=((w - a * l_th * 2.0) / a, l_th, l_th))

    def get_shapes(self, paramvectors):
        verts, faces = [], []
        for vector in tqdm(paramvectors, file=sys.stdout, desc='Generating procedural shapes'):
            bpy.ops.wm.read_factory_settings(use_empty=True)
            self.procmap[self.shape](vector)
            bpy.ops.object.select_all(action='DESELECT')
            bpy.ops.object.select_by_type(type='MESH')
            bpy.ops.object.join()
            obj = bpy.context.selected_objects[0].to_mesh()
            verts.append(np.array([list(x.co) for x in obj.vertices]))
            faces.append(np.array([list(x.vertices) for x in obj.loop_triangles]))
        return verts, faces


def unit_test(args):
    num_samples = args.num_samples
    proc = Procedure('bed')
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
    verts, faces = proc.get_shapes(vectors)
    meshes = []
    for v, f in zip(verts, faces):
        mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(v), o3d.utility.Vector3iVector(f))
        mesh.compute_vertex_normals()
        R = mesh.get_rotation_matrix_from_xyz((-np.pi / 2, 0, 0))
        mesh.rotate(R, center=(0, 0, 0))
        meshes.append(mesh)
    for mesh in meshes:
        o3d.visualization.draw_geometries([mesh])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_samples", type=int, default=5, help="Number of examples")
    parsed_args = parser.parse_args()
    unit_test(parsed_args)

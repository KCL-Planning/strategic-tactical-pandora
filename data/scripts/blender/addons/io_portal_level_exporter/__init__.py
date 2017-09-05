bl_info = {
    "name":         "Portal Level Format",
    "author":       "Bram Ridder",
    "blender":      (2,6,9),
    "version":      (0,0,1),
    "location":     "File > Import-Export",
    "description":  "Region and Portal data format",
    "category":     "Import-Export"
}

import bpy;
import struct;
import math;

from bpy_extras.io_utils import ExportHelper

def dot(p1, p2):
    return sum(a * b for a,b in zip(p1,p2))

def dist3d_segment_to_segment(p1_begin, p1_end, p2_begin, p2_end):
    u = [p1_end[i] - p1_begin[i] for i in range(0, 3)]# [a - b for a,b in zip(p1_end, p1_begin)]
    v = [p2_end[i] - p2_begin[i] for i in range(0, 3)]#[a - b for a,b in zip(p2_end, p2_begin)]
    w = [p1_begin[i] - p2_begin[i] for i in range(0, 3)]# [a - b for a,b in zip(p1_begin, p2_begin)]
    a = dot(u,u)
    b = dot(u,v)
    c = dot(v,v)
    d = dot(u,w)
    e = dot(v,w)
    D = a*c - b*b
    sc = 0
    sN = 0
    sD = D
    tc = 0
    tN = 0
    tD = D
    if (D < 0.01):
        sN = 0.0
        sD = 1.0
        tN = e
        tD = c
    else:
        sN = (b*e - c*d)
        tN = (a*e - b*d)
        if sN < 0.0:
        	sN = 0.0
        	tN = e
        	tD = c
        elif sN > sD:
        	sN = sD
        	tN = e + b
        	tD = c
    if (tN < 0.0):
        tN = 0.0
        if -d < 0.0:
            sN = 0.0
        elif -d > a:
            sN = sD;
        else:
            sN = -d;
            sD = a;
    elif tN > tD:
        tN = tD;
        if ((-d + b) < 0.0):
        	sN = 0;
        elif ((-d + b) > a):
        	sN = sD;
        else:
        	sN = (-d +  b);
        	sD = a;
    # finally do the division to get sc and tc
    if abs(sN) < 0.01:
        sc = 0.0
    else:
        sc = sN / sD
    if abs(tN) < 0.01:
        tc = 0.0
    else:
        tc = tN / tD
    #sc = (abs(sN) < 0.01f ? 0.0 : sN / sD)
    #tc = (abs(tN) < 0.01f ? 0.0 : tN / tD)
    dP = [w[i] + (sc * u[i]) - (tc * v[i]) for i in range(0, 3)]
    #glm::vec3   dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)
    #return sqrt(glm::dot(dP,dP));   // return the closest distance
    #return glm::length(dP);
    return math.sqrt(sum(a*a for a in dP))

class Section:
    triangle_list = None;
    portals = None;
    collisions = None;
    id = -1;
    ob = None;
    vertex_group = None;

    def __init__(self, id, ob, polygons, vertex_group):
        self.id = id
        self.ob = ob
        self.triangle_list = []
        self.portals = []
        self.collisions = []
        self.vertex_group = vertex_group

        uv_layer = ob.data.uv_layers.active

        # Loop through all of the faces defined in the mesh
        for face in polygons:
            # Store access to the vertices
            face_verts = face.vertices;

            # If there are 3 vertices the face is a triangle else the face is a quad
            if len(face_verts) == 3:
                # Create a new triangle wrapper for
                #new_tri = TriangleWrapper((face_verts[0], face_verts[1], face_verts[2]), (uv_layer.data[face.loop_start].uv, uv_layer.data[face.loop_start + 1].uv, uv_layer.data[face.loop_start + 2].uv), (face.normal[0], face.normal[1], face.normal[2]));
                new_tri = TriangleWrapper((ob.data.loops[face.loop_start].vertex_index, ob.data.loops[face.loop_start + 1].vertex_index, ob.data.loops[face.loop_start + 2].vertex_index), (uv_layer.data[face.loop_start].uv, uv_layer.data[face.loop_start + 1].uv, uv_layer.data[face.loop_start + 2].uv), (face.normal[0], face.normal[1], face.normal[2]));
                self.triangle_list.append(new_tri);
            else:
                #new_tri_1 = TriangleWrapper((face_verts[0], face_verts[1], face_verts[2]), (uv_layer.data[face.loop_start].uv, uv_layer.data[face.loop_start + 1].uv, uv_layer.data[face.loop_start + 2].uv), (face.normal[0], face.normal[1], face.normal[2]));
                #new_tri_2 = TriangleWrapper((face_verts[0], face_verts[2], face_verts[3]), (uv_layer.data[face.loop_start].uv, uv_layer.data[face.loop_start + 2].uv, uv_layer.data[face.loop_start + 3].uv), (face.normal[0], face.normal[1], face.normal[2]));
                new_tri_1 = TriangleWrapper((ob.data.loops[face.loop_start].vertex_index, ob.data.loops[face.loop_start + 1].vertex_index, ob.data.loops[face.loop_start + 2].vertex_index), (uv_layer.data[face.loop_start].uv, uv_layer.data[face.loop_start + 1].uv, uv_layer.data[face.loop_start + 2].uv), (face.normal[0], face.normal[1], face.normal[2]));
                new_tri_2 = TriangleWrapper((ob.data.loops[face.loop_start].vertex_index, ob.data.loops[face.loop_start + 2].vertex_index, ob.data.loops[face.loop_start + 3].vertex_index), (uv_layer.data[face.loop_start].uv, uv_layer.data[face.loop_start + 2].uv, uv_layer.data[face.loop_start + 3].uv), (face.normal[0], face.normal[1], face.normal[2]));
                self.triangle_list.append(new_tri_1);
                self.triangle_list.append(new_tri_2);

    def addPortal(self, p):
        self.portals.append(p)

    def addCollision(self, c):
        self.collisions.append(c)

class Portal:
    sector_1 = None;
    sector_2 = None;
    vertices = None;

    def __init__(self, s1, s2, vertices):
        self.sector_1 = s1
        self.sector_2 = s2
        self.vertices = []
        self.vertices = list(vertices.values())

class TriangleWrapper(object):
    __slots__ = "vertex_indices", "uv_coords", "normal";

    def __init__(self, vertex_index=(0,0,0), uv_coords=((0,0),(0,0),(0,0)), normal=(0,0,0)):
        self.vertex_indices = vertex_index;
        self.uv_coords = uv_coords;
        self.normal = (round(normal[0], 2), round(normal[1], 2), round(normal[2], 2));

class Exporter(bpy.types.Operator, ExportHelper):
    bl_idname       = "export_portal_level.plf";
    bl_label        = "Exporter for Portal Level Format";
    bl_options      = {'PRESET'};

    filename_ext    = ".plf";

    def execute(self, context):
        # Ensure Blender is currently in OBJECT mode to allow data access.
        bpy.ops.object.mode_set(mode='OBJECT');

        # Set the default return state to FINISHED
        result = {'FINISHED'};

        # Open the file for writing
        file = open(self.filepath, 'w');

        # Build a hashmap between vertex indeces and portals that connect them.
        portal_objects = []
        collision_objects = []
        sections = []
        level_mesh = None;
        section_maps = {};

        # Check that the currently selected object contains mesh data for exporting
        for ob in bpy.data.objects:

            #ob = context.object;
            if not ob or ob.type != 'MESH':
                file.write("Skip " + ob.name + "\n")
                continue
                #raise NameError("Cannot export: object %s is not a mesh" % ob);

            if ob.name.lower().find("portal") != -1:
                file.write("# Portal! " + ob.name + "\n")

                # Check which sections this portal is part of.
                portal_objects.append(ob)
                continue
            elif ob.name.lower().find("collision") != -1:
                file.write("# Collision! " + ob.name + "\n")
                collision_objects.append(ob)
                continue
            else:
                file.write("# Mesh? " + ob.name + "\n")
                
            if level_mesh != None:
                file.write("Multiple meshes, found! Aborting!")
                break
            level_mesh = ob

            # Transform the polygons to the world space.
            ws_mesh = level_mesh.to_mesh(bpy.context.scene, True, 'PREVIEW')
            ws_mesh.transform(level_mesh.matrix_world)

            # Try to find all the triangles and to which group they belong.
            for group in ob.vertex_groups:
                vlist = [];
                #for p in ob.data.polygons:
                for p in ws_mesh.polygons:
                    all_vertices_part_of_group = True
                    for v in p.vertices:
                        part_of_same_group = False
                        # Find the same vertex index in the data field so we can recover the vertex group id.
                        for v2 in ob.data.vertices:
                            if v2.index == v:
                                for g in v2.groups:
                                    if g.group == group.index:
                                        part_of_same_group = True
                                        break
                            if part_of_same_group:
                                break
                        if not part_of_same_group:
                            all_vertices_part_of_group = False
                            break
                    if all_vertices_part_of_group:
                        vlist.append(p)

                section = Section(len(sections), ob, vlist, group)
                sections.append(section)
                section_maps[group.name] = section

#                file.write("Vertex group: " + str(index.index) + " (" + index.name + ") (" + str(len(vlist)) + "/" + str(len(section.triangle_list)) + ") - " + str(id(section.triangle_list)) + "\n")

#        for section in sections:
#            file.write("Vertex group: " + str(len(section.triangle_list)) + ")\n")

#        file.write("#" + str(len(collision_objects)))

        # Construct the portals and link them to the sections.
        portals = []
        for portal_object in portal_objects:
            if ("link0" not in portal_object):
                file.write("The portal: " + portal_object.name + " has no 'link0' attribute.\n")
                print("The portal: " + portal_object.name + " has no 'link0' attribute.")
                s1 = None
            else:
                if portal_object["link0"] not in section_maps:
                    file.write("The portal: " + portal_object.name + " links to the unknown section: " + portal_object["link0"] + ".\n")
                    print("The portal: " + portal_object.name + " links to the unknown section: " + portal_object["link0"] + ".")
                else:
                    s1 = section_maps[portal_object["link0"]]
                    
            if ("link1" not in portal_object):
                file.write("The portal: " + portal_object.name + " has no 'link1' attribute.\n")
                print("The portal: " + portal_object.name + " has no 'link1' attribute.")
                s2 = None
            else:
                if portal_object["link1"] not in section_maps:
                    file.write("The portal: " + portal_object.name + " links to the unknown section: " + portal_object["link1"] + ".\n")
                    print("The portal: " + portal_object.name + " links to the unknown section: " + portal_object["link1"] + ".")
                else:
                    s2 = section_maps[portal_object["link1"]]
#            s1 = None
#            s2 = None

#            for section in sections:

                # This should not happen.
#                if len(section.triangle_list) == 0:
#                    continue

#                section_matches = True
#                for v_i in range(0, len(portal_object.data.vertices)):
#                    found_matching_vertex = False

#                    for t in section.triangle_list:

                        # Because the sections are convex, we can simply check the angle between the line segments of the
                        # triangles and compare those with the points of the portal.
#                        for i in range(0, 3):
#                            if dist3d_segment_to_segment(portal_object.data.vertices[v_i].co, portal_object.data.vertices[(v_i + 1) % len(portal_object.data.vertices)].co, section.ob.data.vertices[t.vertex_indices[i]].co, section.ob.data.vertices[t.vertex_indices[(i + 1) % 3]].co) < 0.01:
#                                found_matching_vertex = True
#                                break
#                        if found_matching_vertex:
#                            break
#                    if not found_matching_vertex:
#                        section_matches = False
#                        break

#                if not section_matches:
#                    continue

#                if s1 == None:
#                    s1 = section
#                elif s2 == None:
#                    s2 = section
#                    break

            if (s1 == None and s2 != None):
                file.write("? -> " + str(s2.id))
            if (s1 != None and s2 == None):
                file.write(str(s1.id) + " -> ?")
            if (s2 != None and s1 == None):
                file.write(str(s2.id) + " -> ?")
            if s1 == None or s2 == None:
                continue

            p_collision_mesh = portal_object.to_mesh(bpy.context.scene, True, 'PREVIEW')
            p_collision_mesh.transform(portal_object.matrix_world)

            #portal = Portal(s1, s2, portal_object.data.vertices)
            portal = Portal(s1, s2, p_collision_mesh.vertices)
            s1.addPortal(portal)
            s2.addPortal(portal)
            portals.append(portal)

        # Construct the collision detection cubes.
        for collision_object in collision_objects:

            # Transform the collision objects to world space.
            ws_collision_mesh = collision_object.to_mesh(bpy.context.scene, True, 'PREVIEW')
            ws_collision_mesh.transform(collision_object.matrix_world)
            
            for i in range(0,100):
                if ("link" + str(i) in collision_object):
                    if collision_object["link" + str(i)] not in section_maps:
                        file.write("A collision object: " + collision_object.name + " is linked to " + collision_object["link" + str(i)] + ", but this section does not exists!\n");
                        print("A collision object: " + collision_object.name + " is linked to " + collision_object["link" + str(i)] + ", but this section does not exists!")
                    else:
                        section_maps[collision_object["link" + str(i)]].addCollision(ws_collision_mesh)

#            for section in sections:
#                is_in_section = False

#                for t in section.triangle_list:
#                    for i in range(0, 3):
#                        for j in range(0, len(ws_collision_mesh.vertices)):
#                            if dist3d_segment_to_segment(collision_object.data.vertices[j].co, collision_object.data.vertices[(j + 1) % len(collision_object.data.vertices)].co, section.ob.data.vertices[t.vertex_indices[i]].co, section.ob.data.vertices[t.vertex_indices[(i + 1) % 3]].co) < 0.1:
#                                is_in_section = True
#                                section.addCollision(ws_collision_mesh)
#                                break
#                        if is_in_section:
#                            break
#                    if is_in_section:
#                        break

        # Time to write this information to a file :).

        # Global information that is shared by all sections. We assume there is only one mesh and one uv layer.
        # Verteces and UV mappings.
        for t in level_mesh.data.uv_textures.active.data:
            if t.image != None:
                file.write("tex " + t.image.filepath + "\n")
                break
        #for vert in level_mesh.data.vertices:
        for vert in ws_mesh.vertices:
            file.write("v " + repr(round(vert.co[0], 2)) + " " + repr(round(vert.co[2], 2)) + " " + repr(round(vert.co[1], 2)) + "\n")

        for section in sections:
            # Section ID.
            file.write("s " + str(section.id) + "\n")
            # Vertice indices.
            for v in section.triangle_list:
                file.write("i " + str(v.vertex_indices[0]) + " " + str(v.vertex_indices[2]) + " " + str(v.vertex_indices[1]) + "\n")
                file.write("n " + str(v.normal[0]) + " " + str(v.normal[2]) + " " + str(v.normal[1]) + "\n")
                # UV mappings.
                #for i in range(len(v.vertex_indices)):
                #    file.write("u " + str(v.uv_coords[i][0]) + " " + str(v.uv_coords[i][1]) + "\n")
                file.write("u " + str(v.uv_coords[0][0]) + " " + str(v.uv_coords[0][1]) + "\n")
                file.write("u " + str(v.uv_coords[2][0]) + " " + str(v.uv_coords[2][1]) + "\n")
                file.write("u " + str(v.uv_coords[1][0]) + " " + str(v.uv_coords[1][1]) + "\n")
            # Collision information.
            for c in section.collisions:
            #    file.write("c " + str(len(c.data.vertices)) + "\n")
            #    for v in c.data.vertices:
            #        file.write("cv " + repr(round(v.co[0], 2)) + " " + repr(round(v.co[2], 2)) + " " + repr(round(v.co[1], 2)) + "\n");
                file.write("c " + str(len(c.vertices)) + "\n")
                for v in c.vertices:
                    file.write("cv " + repr(round(v.co[0], 2)) + " " + repr(round(v.co[2], 2)) + " " + repr(round(v.co[1], 2)) + "\n");


        # Next are the portals.
        for portal in portals:
            file.write("p " + str(portal.sector_1.id) + " " + str(portal.sector_2.id) + "\n")
            for v in portal.vertices:
                file.write("pv " + repr(round(v.co[0], 2)) + " " + repr(round(v.co[2], 2)) + " " + repr(round(v.co[1], 2)) + "\n");
            file.write("\n")

        # Close the file
        file.close();

        return result;

def menu_func(self, context):
    self.layout.operator(Exporter.bl_idname, text="Exporter for Portal Level Format(.plf)");

def register():
    bpy.utils.register_module(__name__);
    bpy.types.INFO_MT_file_export.append(menu_func);

def unregister():
    bpy.utils.unregister_module(__name__);
    bpy.types.INFO_MT_file_export.remove(menu_func);

if __name__ == "__main__":
    register()

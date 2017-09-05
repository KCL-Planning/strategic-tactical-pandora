'''
Created on 04/12/2013

@author: Bram Ridder
'''

import bpy;
import struct;
from bpy_extras.io_utils import ExportHelper

class FileHeader:
    sizeInBytes = 12;
    numVertices = 0;
    numIndices  = 0;

    def __init__(self, numVertices, numIndices):
        self.numVertices = numVertices;
        self.numIndices  = numIndices;

    def write(self, file):
        file.write(struct.pack('hhh', self.sizeInBytes, self.numVertices, self.numIndices));

class FileBody:
    mesh      = None;
    tri_list  = [];

    def __init__(self, mesh):
        self.mesh = mesh;

    def write(self, file):
        for vert in self.mesh.vertices:
            file.write(struct.pack('fff', vert.co[0], vert.co[1], vert.co[2]));

        for tri in self.tri_list:
            file.write(struct.pack('hhh', tri.vertex_indices[0], tri.vertex_indices[1], tri.vertex_indices[2]));

class TriangleWrapper(object):
    __slots__ = "vertex_indices", "offset";

    def __init__(self, vertex_index=(0,0,0)):
        self.vertex_indices = vertex_index;

class Exporter(bpy.types.Operator, ExportHelper):
    bl_idname       = "export_portal_level.plf";
    bl_label        = "Exporter for Portal Level Format";
    bl_options      = {'PRESET'};

    filename_ext    = ".plf";

    # This method will be used to extract triangle vertex and index data from 3D Objects in the Blender scene.
    # This method can export objects built from either triangle or quad primitives.
    def extract_triangles(self, mesh):
        # Create an empty array to store out triangles.
        triangle_list = [];

        # Loop through all of the faces defined in the mesh
        for face in enumerate(mesh.polygons):
            # Store access to the vertices
            face_verts = face.vertices;
            # If there are 3 vertices the face is a triangle else the face is a quad
            if len(face_verts) == 3:
                # Create a new triangle wrapper for
                new_tri = TriangleWrapper((face_verts[0], face_verts[1], face_verts[2]));
                triangle_list.append(new_tri);
            elif len(face_verts) == 4:
                new_tri_1 = TriangleWrapper((face_verts[0], face_verts[1], face_verts[2]));
                new_tri_2 = TriangleWrapper((face_verts[0], face_verts[2], face_verts[3]));
                triangle_list.append(new_tri_1);
                triangle_list.append(new_tri_2);
            else:
                print("Unsupported, face; Skipping it...")

        return triangle_list;

    def execute(self, context):
        # Ensure Blender is currently in OBJECT mode to allow data access.
        bpy.ops.object.mode_set(mode='OBJECT');

        # Set the default return state to FINISHED
        result = {'FINISHED'};

        # Check that the currently selected object contains mesh data for exporting
        ob = context.object;
        if not ob or ob.type != 'MESH':
            raise NameError("Cannot export: object %s is not a mesh" % ob);

        # Create a file body object for storing the data to be written to file.
        fileBody = FileBody(ob.data);
        fileBody.tri_list = self.extract_triangles(fileBody.mesh);

        # Create a file header object with data stored in the body section
        fileHeader = FileHeader(
            len(fileBody.mesh.vertices),
            len(fileBody.tri_list) * 3);

        # Open the file for writing
        file = open(self.filepath, 'bw');
        # Write the file data
        fileHeader.write(file);
        fileBody.write(file);
        # Close the file
        file.close();

        return result;

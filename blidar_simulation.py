# Version 4 - Noise, FOV, Stop Scan, Parent Translation

import bpy
import bmesh
import numpy as np
import random
import csv
import mathutils
import math

bl_info = {
    "name": "BLIDAR 2D",
    "blender": (2, 80, 0),
    "category": "Object",
    "description": "Assign a mesh as a 2D LIDAR, customize parameters, and scan pointclouds."
}

# Global variable to store the timer
blidar_timer = None
scanning_active = False  # This will track whether scanning is active or not

class BLIDAR_OT_AssignLidar(bpy.types.Operator):
    bl_idname = "blidar.assign_lidar"
    bl_label = "Assign LIDAR"
    bl_description = "Assign selected mesh or empty as LIDAR"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = context.active_object
        if obj is None or obj.type not in {'MESH', 'EMPTY'}:
            self.report({'WARNING'}, "Please select a mesh or empty object.")
            return {'CANCELLED'}
        context.scene.blidar_object = obj
        self.report({'INFO'}, f"Assigned {obj.name} as LIDAR.")
        return {'FINISHED'}

class BLIDAR_OT_StartScan(bpy.types.Operator):
    bl_idname = "blidar.start_scan"
    bl_label = "Start Scanning"
    bl_description = "Start scanning pointcloud data"
    bl_options = {'REGISTER', 'UNDO'}

    _timer = None  # Use local variable for the modal's timer

    def modal(self, context, event):
        global scanning_active  # Use global variable to track active scanning
        if event.type == 'TIMER':
            if scanning_active:
                self.scan_frame(context)
            else:
                self.cancel(context)  # Stop scanning and cancel modal operation
                return {'CANCELLED'}
        return {'PASS_THROUGH'}

    def execute(self, context):
        global blidar_timer, scanning_active  # Use global variable for timer and scanning status
        # Check if there is already a point cloud in the scene
        if "BLIDAR_PointCloud" in bpy.data.objects:
            self.report({'WARNING'}, "Point cloud already exists. Please remove it before starting a new scan.")
            return {'CANCELLED'}
        
        wm = context.window_manager
        frequency = context.scene.blidar_frequency
        blidar_timer = wm.event_timer_add(1.0 / frequency, window=context.window)  # Assign to global variable
        scanning_active = True  # Set scanning to active
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        global blidar_timer, scanning_active  # Use the global variable for timer and scanning status
        wm = context.window_manager
        if blidar_timer:
            wm.event_timer_remove(blidar_timer)  # Remove the timer when the operator is cancelled
            blidar_timer = None
        scanning_active = False  # Set scanning to inactive

    def scan_frame(self, context):
        obj = context.scene.blidar_object
        if obj is None:
            return

        range_val = context.scene.blidar_range
        resolution = context.scene.blidar_resolution
        fov = context.scene.blidar_fov
        noise_x = context.scene.blidar_noise_x
        noise_y = context.scene.blidar_noise_y
        noise_z = context.scene.blidar_noise_z
        depsgraph = context.evaluated_depsgraph_get()

        # Compute half FOV in radians
        half_fov_radians = math.radians(fov / 2.0)

        # Get the LIDAR's forward direction (-Y axis in Blender)
        forward_direction = obj.matrix_world.to_quaternion() @ mathutils.Vector((0, -1, 0))
        
        # Get the world position of the LIDAR object (handles parent transforms)
        origin = obj.matrix_world.translation

        pointcloud = []
        bm = bmesh.new()
        if obj.type == 'MESH' and obj.data:
            bm.from_mesh(obj.data)

        for i in range(resolution):
            angle = (-half_fov_radians) + (i / (resolution - 1)) * math.radians(fov)

            direction = forward_direction.copy()
            direction.rotate(mathutils.Euler((0, 0, angle), 'XYZ'))

            ray_max = range_val + random.uniform(-max(noise_x, noise_y, noise_z), max(noise_x, noise_y, noise_z))

            result = context.scene.ray_cast(depsgraph, origin, direction, distance=ray_max)
            hit, loc, normal, face_index = result[0], result[1], result[2], result[3]
            if hit:
                # Apply noise to the hit location
                loc.x += random.uniform(-noise_x, noise_x)
                loc.y += random.uniform(-noise_y, noise_y)
                loc.z += random.uniform(-noise_z, noise_z)
                # Apply noise to the hit location
                loc.x += random.uniform(-noise_x, noise_x)
                loc.y += random.uniform(-noise_y, noise_y)
                loc.z += random.uniform(-noise_z, noise_z)
                pointcloud.append((loc.x, loc.y, loc.z))
            elif obj.type == 'EMPTY':
                # For EMPTY, simulate a point at max range in the given direction
                loc = origin + direction * ray_max
                loc.x += random.uniform(-noise_x, noise_x)
                loc.y += random.uniform(-noise_y, noise_y)
                loc.z += random.uniform(-noise_z, noise_z)
                pointcloud.append((loc.x, loc.y, loc.z))

        context.scene.blidar_pointcloud.clear()
        for point in pointcloud:
            item = context.scene.blidar_pointcloud.add()
            item.x, item.y, item.z = point

        # Create point cloud visualization in the viewport
        self.create_pointcloud_visual(context, pointcloud)

    def create_pointcloud_visual(self, context, pointcloud):
        # Remove previous point cloud mesh if it exists
        if "BLIDAR_PointCloud" in bpy.data.objects:
            bpy.data.objects.remove(bpy.data.objects["BLIDAR_PointCloud"], do_unlink=True)

        # Create new point cloud mesh
        mesh = bpy.data.meshes.new(name="BLIDAR_PointCloud")
        obj = bpy.data.objects.new("BLIDAR_PointCloud", mesh)
        context.collection.objects.link(obj)

        # # Set the point cloud as a child of the LIDAR object
        # obj.parent = context.scene.blidar_object

        # Create vertices from pointcloud data
        vertices = [mathutils.Vector((point[0], point[1], point[2])) for point in pointcloud]
        edges = []
        faces = []

        mesh.from_pydata(vertices, edges, faces)
        mesh.update()

class BLIDAR_OT_StopScan(bpy.types.Operator):
    bl_idname = "blidar.stop_scan"
    bl_label = "Stop Scanning"
    bl_description = "Stop the scanning and remove the pointcloud data"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        global scanning_active  # Use the global variable for scanning status
        # Stop the scan by setting the control flag to False
        scanning_active = False

        # Remove point cloud if exists
        if "BLIDAR_PointCloud" in bpy.data.objects:
            bpy.data.objects.remove(bpy.data.objects["BLIDAR_PointCloud"], do_unlink=True)

        self.report({'INFO'}, "Scanning stopped and point cloud removed.")
        return {'FINISHED'}

class BLIDAR_OT_ExportCSV(bpy.types.Operator):
    bl_idname = "blidar.export_csv"
    bl_label = "Export CSV"
    bl_description = "Export pointcloud data to CSV"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        pointcloud = context.scene.blidar_pointcloud
        if not pointcloud:
            self.report({'WARNING'}, "No pointcloud data to export.")
            return {'CANCELLED'}

        filepath = bpy.path.abspath('//blidar_pointcloud.csv')
        with open(filepath, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['X', 'Y', 'Z'])
            for point in pointcloud:
                writer.writerow([point.x, point.y, point.z])
        self.report({'INFO'}, f"Pointcloud data exported to {filepath}")
        return {'FINISHED'}

class BLIDAR_PT_Panel(bpy.types.Panel):
    bl_idname = "BLIDAR_PT_Panel"
    bl_label = "BLIDAR 2D"
    bl_category = "BLIDAR"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'

    def draw(self, context):
        layout = self.layout
        scene = context.scene

        layout.operator("blidar.assign_lidar")
        layout.prop(scene, "blidar_range")
        layout.prop(scene, "blidar_resolution")
        layout.prop(scene, "blidar_fov")
        layout.prop(scene, "blidar_noise_x")
        layout.prop(scene, "blidar_noise_y")
        layout.prop(scene, "blidar_noise_z")
        layout.prop(scene, "blidar_frequency")
        layout.operator("blidar.start_scan")
        layout.operator("blidar.stop_scan")
        layout.operator("blidar.export_csv")

# Property registration
classes = [
    BLIDAR_OT_AssignLidar,
    BLIDAR_OT_StartScan,
    BLIDAR_OT_StopScan,
    BLIDAR_OT_ExportCSV,
    BLIDAR_PT_Panel,
]

class BLIDAR_Point(bpy.types.PropertyGroup):
    x: bpy.props.FloatProperty()
    y: bpy.props.FloatProperty()
    z: bpy.props.FloatProperty()

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.utils.register_class(BLIDAR_Point)
    bpy.types.Scene.blidar_object = bpy.props.PointerProperty(name="LIDAR Object", type=bpy.types.Object)
    bpy.types.Scene.blidar_range = bpy.props.FloatProperty(name="Range", default=10.0, min=0.0)
    bpy.types.Scene.blidar_resolution = bpy.props.IntProperty(name="Resolution", default=360, min=1)
    bpy.types.Scene.blidar_fov = bpy.props.FloatProperty(name="Field of View (degrees)", default=360.0, min=1.0, max=360.0)
    bpy.types.Scene.blidar_noise_x = bpy.props.FloatProperty(name="Noise X", default=0.1, min=0.0)
    bpy.types.Scene.blidar_noise_y = bpy.props.FloatProperty(name="Noise Y", default=0.1, min=0.0)
    bpy.types.Scene.blidar_noise_z = bpy.props.FloatProperty(name="Noise Z", default=0.1, min=0.0)
    bpy.types.Scene.blidar_frequency = bpy.props.FloatProperty(name="Frequency", default=15.0, min=0.1)
    bpy.types.Scene.blidar_pointcloud = bpy.props.CollectionProperty(type=BLIDAR_Point)

def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)
    bpy.utils.unregister_class(BLIDAR_Point)
    del bpy.types.Scene.blidar_object
    del bpy.types.Scene.blidar_range
    del bpy.types.Scene.blidar_resolution
    del bpy.types.Scene.blidar_fov
    del bpy.types.Scene.blidar_noise_x
    del bpy.types.Scene.blidar_noise_y
    del bpy.types.Scene.blidar_noise_z
    del bpy.types.Scene.blidar_frequency
    del bpy.types.Scene.blidar_pointcloud

if __name__ == "__main__":
    register()

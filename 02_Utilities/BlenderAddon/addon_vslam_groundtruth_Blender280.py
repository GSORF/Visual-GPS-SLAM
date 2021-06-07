bl_info = {
    "name": "B-SLAM-SIM",
    "author": "Adam Kalisz",
    "version": (0, 1),
    "blender": (2, 80, 0),
    "location": "View3D > Add > Mesh > New Object",
    "description": "Helpful tools for creating synthetic ground truth data in order to test Visual SLAM algorithms",
    "warning": "",
    "wiki_url": "https://master.kalisz.co",
    "category": "Import-Export",
    }

"""
REFERENCES USED:
    Lane widths are 3.0 - 3.3 meters for low speed traffic
    https://www.driverknowledgetests.com/resources/road-widths/
    (virtual lane width was 0.3 m on Feburary, 1st - fixed on Feburary, 7th)
    
    GoPro Hero 4 Black Focal Length: Wide 17.2mm
    https://gopro.com/help/articles/Question_Answer/HERO4-Field-of-View-FOV-Information
"""


import bpy
import os
import math
from bpy.types import Operator
from bpy.types import Panel
from bpy_extras.io_utils import ExportHelper
from mathutils import Vector, Euler, Quaternion, Matrix
import numpy as np


def VSLAMMappingFromBlender2DSO(translation, quaternion):
    '''
    Mapping from Blender to DSO world space:
    
    Blender:
        Up = Y-Axis
        Right = X-Axis
        Forward = -Z-Axis
        
    DSO:
        Up = -Y-Axis
        Right = X-Axis
        Forward = Z-Axis
    
    x =  x
    y = -y
    z = -z
    
    Matrix:
         1   0   0
         0  -1   0
         0   0  -1
    '''
    
    # Create space transformation matrix
    Blender2DSO = Euler([math.pi/2.0, 0.0, 0.0], 'XYZ').to_matrix().to_4x4()
    
    # Create matrix from translation and quaternion
    mat_rot = quaternion.to_matrix().to_4x4()
    mat_alignRotation = Euler([-math.pi, 0.0, 0.0], 'XYZ').to_matrix().to_4x4() # This is necessary, because I have decided to rotate Blenders camera such that it "lies" in
    # the x-y-plane instead of x-z-plane (i.e. a rotation about 90 degrees around the x-axis) and thus is oriented with regards to the procedurally generated city.
    mat_trans = Matrix.Translation(translation)
    BlenderPoseCam2World = mat_trans @ mat_rot @ mat_alignRotation
    
    # Transform from Blender world space to DSO world space
    DSOPose = Blender2DSO @ BlenderPoseCam2World
    
    # Mirror Quaternion along X-Axis is performed by flipping signs of the y and z component
    # Source: Philipp Kurth -> https://stackoverflow.com/questions/32438252/efficient-way-to-apply-mirror-effect-on-quaternion-rotation
    #newQuaternion = quaternion
    #newQuaternion.y = -quaternion.y
    #newQuaternion.z = -quaternion.z
    
    return DSOPose.translation, DSOPose.to_quaternion()


def VSLAMMappingFromDSO2Blender(translation, quaternion):
    '''
    Mapping from DSO to Blender space:
    
    x = -x
    y = -z
    z =  y
    
    Matrix:
        -1   0   0
         0   1   0
         0   0  -1
    '''
    
    # Create space transformation matrix
    DSO2Blender = Euler([-math.pi/2.0, 0.0, 0.0], 'XYZ').to_matrix().to_4x4()
    
    # Create matrix from translation and quaternion
    mat_rot = quaternion.to_matrix().to_4x4()
    mat_alignRotation = Euler([math.pi, 0.0, 0.0], 'XYZ').to_matrix().to_4x4() # This is necessary, because I have decided to rotate Blenders camera such that it "lies" in
    # the x-y-plane instead of x-z-plane (i.e. a rotation about 90 degrees around the x-axis) and thus is oriented with regards to the procedurally generated city.
    mat_trans = Matrix.Translation(translation)
    DSOPoseCam2World = mat_trans @ mat_rot @ mat_alignRotation # * mat_rotTurnAroundY
    
    # Transform from DSO world space to Blender world space
    BlenderPose = DSO2Blender @ DSOPoseCam2World
    
    # Mirror Quaternion along X-Axis is performed by flipping signs of the y and z component
    # Source: Philipp Kurth -> https://stackoverflow.com/questions/32438252/efficient-way-to-apply-mirror-effect-on-quaternion-rotation
    #newQuaternion = quaternion
    #newQuaternion.y = -quaternion.y
    #newQuaternion.z = -quaternion.z
    
    return BlenderPose.translation, BlenderPose.to_quaternion()

def VSLAMTestMappingBlenderDSO():
    # Test Mapping:
    translation = Vector([-1,21,22])
    quaternion = Quaternion((1.0,0.0,0.0,0.0))
    print("Blender (original): \t", translation, " - ", quaternion.to_euler('XYZ'))

    dso_trans, dso_rot = VSLAMMappingFromBlender2DSO(translation, quaternion)
    print("DSO: \t\t\t", dso_trans, " - ", dso_rot)

    blender_trans, blender_rot = VSLAMMappingFromDSO2Blender(dso_trans, dso_rot)
    print("Blender (retrieved): \t", blender_trans, " - ", blender_rot.to_euler('XYZ'))

    if translation == blender_trans and quaternion == blender_rot:
        print("Everything O.K.!")
    else:
        print("Mapping test failed... ... -.-")

class VSLAMImportDSO(Operator):
    '''
    This operator imports camera poses from the DSO.
    '''
    
    bl_idname = "vslam.import_dso"
    bl_label = "Import DSO CSV File (camera_dso.csv)"
    
    
    def execute(self, context):
        scene = context.scene
        camera_csv = "camera_dso.csv"
        filepath = scene.vslam_output_directory + camera_csv
        dso_camera = "Camera_DSO"
        camera_object = scene.objects.get(dso_camera)
        
        if(camera_object is None):
            bpy.ops.object.camera_add(location=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0))
            camera_object = scene.objects.active
            
        camera_object.name = dso_camera
        
        with open(filepath) as poses:  
            line = poses.readline()
            count = 1
            
            while line:
                line = line.strip()
                line = line.split(",")
                
                timestamp = int(line[0])
                frame = round(timestamp / 1000.0 * scene.render.fps, 0)
                location = [ float(line[1]),float(line[2]), float(line[3]) ]
                orientation = [ float(line[4]),float(line[5]),float(line[6]),float(line[7]) ]
                
                print("DSO-Importer: Now setting frame =", frame)
                scene.frame_set(frame)
                
                # Transformation from DSO to Blender
                loc, quat = VSLAMMappingFromDSO2Blender( Vector( location ), Quaternion( orientation ) )
                
                camera_object.location = loc
                camera_object.rotation_quaternion = quat
                
                # camera_object.location = Vector( location )
                camera_object.keyframe_insert(data_path='location')
                camera_object.keyframe_insert(data_path='rotation_quaternion')
                
                
                #print("Line {}: {}".format(count, line.strip()))
                line = poses.readline()
                count += 1
        
            camera_object.rotation_mode = 'QUATERNION'

        return {"FINISHED"}
        
class VSLAMExportCamera(Operator):
    '''
    This operator exports ideal and noisy camera trajectories
    '''
    
    bl_idname = "vslam.export_camera"
    bl_label = "Export noisy camera"
    
    # ExportHelper mixin class uses this
    # filename_ext = ".csv"

    def execute(self, context):
        scene = context.scene
        camera = scene.objects[scene.vslam_camera]
        camera.select_set(True)
        context.view_layer.objects.active = camera
        filepath_ideal_dso = scene.vslam_output_directory + camera.name + "_ideal_dso.csv"
        filepath_ideal_blender = scene.vslam_output_directory + camera.name + "_ideal_blender.csv"
        filepath_noisy_dso = scene.vslam_output_directory + camera.name + "_noisy_dso.csv"
        filepath_noisy_blender = scene.vslam_output_directory + camera.name + "_noisy_blender.csv"
        
        if(scene.vslam_bool_export_ideal):
            FileIdealCameraDSO = open(filepath_ideal_dso, 'w', encoding='utf-8')
            FileIdealCameraBlender = open(filepath_ideal_blender, 'w', encoding='utf-8')
        if(scene.vslam_bool_export_noisy):
            FileNoisyCameraDSO = open(filepath_noisy_dso, 'w', encoding='utf-8')
            FileNoisyCameraBlender = open(filepath_noisy_blender, 'w', encoding='utf-8')
        
        if(scene.objects.get(scene.vslam_altered_camera) is not None):
            # deselect all
            bpy.ops.object.select_all(action='DESELECT')
            # select altered camera object
            scene.objects[scene.vslam_altered_camera].select_set(True)
            # remove it
            bpy.ops.object.delete()
        
        scene.frame_set(scene.frame_start)
        bpy.ops.object.select_all(action='DESELECT')
        scene.objects[scene.vslam_camera].select_set(True)
        bpy.ops.object.duplicate(linked=False, mode="TRANSLATION")
        print(context.view_layer.objects.active)
        altered_camera = context.view_layer.objects.active
        altered_camera.name = scene.vslam_altered_camera
        
        for frame in range(scene.frame_start, scene.frame_end + 1):
            scene.frame_set(frame)
            timestamp = ( (scene.frame_current - scene.frame_start) * (1.0 / scene.render.fps) ) * 1000
            
            # For every frame get camera transformation and write the ideal .csv file
            translation = camera.matrix_world.translation
            orientation = camera.matrix_world.to_quaternion()
            dso_trans, dso_quat = VSLAMMappingFromBlender2DSO(translation, orientation)
            if(scene.vslam_bool_export_ideal):
                if(scene.vslam_bool_export_timestamp):
                    FileIdealCameraDSO.write("%d,%f,%f,%f,%f,%f,%f,%f\n" % (timestamp, dso_trans.x, dso_trans.y, dso_trans.z, dso_quat.w, dso_quat.x, dso_quat.y, dso_quat.z))
                    FileIdealCameraBlender.write("%d,%f,%f,%f,%f,%f,%f,%f\n" % (timestamp, translation.x, translation.y, translation.z, orientation.w, orientation.x, orientation.y, orientation.z))
                else:
                    FileIdealCameraDSO.write("%f,%f,%f,%f,%f,%f,%f\n" % (dso_trans.x, dso_trans.y, dso_trans.z, dso_quat.w, dso_quat.x, dso_quat.y, dso_quat.z))
                    FileIdealCameraBlender.write("%f,%f,%f,%f,%f,%f,%f\n" % (translation.x, translation.y, translation.z, orientation.w, orientation.x, orientation.y, orientation.z))

            # Calculate noise for translation (additive to position)           
            noise = np.random.normal(scene.vslam_noise_translation_mean, scene.vslam_noise_translation_std_deviation, size=3)
            noise_translation = Vector( (noise[0], noise[1], noise[2]) )
            '''
            # ALTERNATIVE approach (not used here, only kept for reference):
            # Idea: sample quaternion rotation randomly
            # Based on: http://planning.cs.uiuc.edu/node198.html
            
            u_1 = noise[3] * 0.5 + 0.5
            u_2 = noise[4] * 0.5 + 0.5
            u_3 = noise[5] * 0.5 + 0.5
            q_w = math.sqrt(1-u_1) * math.sin(2*math.pi*u_2)
            q_x = math.sqrt(1-u_1) * math.cos(2*math.pi*u_2)
            q_y = math.sqrt(u_1) * math.sin(2*math.pi*u_3)
            q_z = math.sqrt(u_1) * math.cos(2*math.pi*u_3)
            noise_orientation = Quaternion( (q_w, q_x, q_y, q_z) )
            
            noise_orientation = orientation
            noise_orientation.x = noise[3]
            noise_orientation.y = noise[4]
            noise_orientation.z = noise[5]
            noise_orientation.w = noise[6]
            '''
            
            # Generate noise for orientation (additive to Euler)
            noise = np.random.normal(scene.vslam_noise_orientation_mean, scene.vslam_noise_orientation_std_deviation, size=3)
            noise_orientation = orientation.to_euler()
            noise_orientation.x += noise[0]
            noise_orientation.y += noise[1]
            noise_orientation.z += noise[2]
            orientation = noise_orientation.to_quaternion()
            
            translation = translation + noise_translation # Vector addition to translate objects
            
            dso_trans, dso_quat = VSLAMMappingFromBlender2DSO(translation, orientation)
            # And now export the noisy camera transformation for every frame and write the noisy .csv file
            if(scene.vslam_bool_export_noisy):
                if(scene.vslam_bool_export_timestamp):
                    FileNoisyCameraDSO.write("%d,%f,%f,%f,%f,%f,%f,%f\n" % (timestamp, dso_trans.x, dso_trans.y, dso_trans.z, dso_quat.w, dso_quat.x, dso_quat.y, dso_quat.z))
                    FileNoisyCameraBlender.write("%d,%f,%f,%f,%f,%f,%f,%f\n" % (timestamp, translation.x, translation.y, translation.z, orientation.w, orientation.x, orientation.y, orientation.z))
                else:
                    FileNoisyCameraDSO.write("%f,%f,%f,%f,%f,%f,%f\n" % (dso_trans.x, dso_trans.y, dso_trans.z, dso_quat.w, dso_quat.x, dso_quat.y, dso_quat.z))
                    FileNoisyCameraBlender.write("%f,%f,%f,%f,%f,%f,%f\n" % (translation.x, translation.y, translation.z, orientation.w, orientation.x, orientation.y, orientation.z))
            
            altered_camera.location = translation
            altered_camera.rotation_euler = orientation.to_euler()
            altered_camera.keyframe_insert(data_path='location')
            altered_camera.keyframe_insert(data_path='rotation_euler')
        
        if(scene.vslam_bool_export_ideal):
            FileIdealCameraDSO.close()
            FileIdealCameraBlender.close()
        if(scene.vslam_bool_export_noisy):
            FileNoisyCameraDSO.close()
            FileNoisyCameraBlender.close()
        
        
        
        
        
        if(scene.vslam_bool_export_dso):
            """
            If set, export additional files (calibration and shell script) to run the dso directly.
            
            ### DSO camera.txt calibration configuration: ###
            Pinhole fx fy cx cy 0
            in_width in_height
            "crop" / "full" / "none" / "fx fy cx cy 0"
            out_width out_height
            """
            image_width = scene.render.resolution_x * (scene.render.resolution_percentage * 0.01)
            image_height = scene.render.resolution_y * (scene.render.resolution_percentage * 0.01)
            cropped_image_width = image_width * scene.vslam_dso_scale_factor
            cropped_image_height = image_height * scene.vslam_dso_scale_factor
            
            cam_data = scene.camera.data
            focal_length = cam_data.lens # [mm]
            sensor_width_mm = cam_data.sensor_width
            sensor_height_mm = cam_data.sensor_height
            
            # Calculate Focal length in Pixels:
            # focal length in pixels = (image width in pixels) * (focal length in mm) / (CCD width in mm)
            # Reference: http://phototour.cs.washington.edu/focal.html
            fx = image_width * (1.0 * focal_length / sensor_width_mm)
            fy = image_height * (1.0 * focal_length / sensor_height_mm)
            cx = image_width / 2
            cy = image_height / 2
            
            # Alternative way to get camera intrinsics based on a post by Daniel on SO
            # Reference: https://blender.stackexchange.com/questions/38009/3x4-camera-matrix-from-blender-camera/120063#120063
            sensor_size_in_mm = sensor_width_mm
            if cam_data.sensor_fit == 'VERTICAL':
                sensor_size_in_mm = sensor_height_mm
            
            sensor_fit = cam_data.sensor_fit
            if sensor_fit == 'AUTO':
                if (scene.render.pixel_aspect_x * image_width) >= (scene.render.pixel_aspect_y * image_height):
                    sensor_fit = 'HORIZONTAL'
                else:
                    sensor_fit = 'VERTICAL'
            
            pixel_aspect_ratio = scene.render.pixel_aspect_y / scene.render.pixel_aspect_x
            if sensor_fit == 'HORIZONTAL':
                view_fac_in_px = image_width
            else:
                view_fac_in_px = pixel_aspect_ratio * image_height
            pixel_size_mm_per_px = (sensor_size_in_mm / focal_length) / view_fac_in_px
            fx = 1.0 / pixel_size_mm_per_px
            fy = (1.0 / pixel_size_mm_per_px) / pixel_aspect_ratio
            cx = (image_width - 1) / 2.0 - cam_data.shift_x * view_fac_in_px
            cy = (image_height - 1) / 2.0 + (cam_data.shift_y * view_fac_in_px) / pixel_aspect_ratio
                        
            dso_calib_file = "camera_" + camera.name + ".txt"
            dso_image_folder = context.scene.vslam_camera + "/" + context.scene.vslam_camera + "_*"
            filepath_dso_calib = scene.vslam_output_directory + dso_calib_file
            FileDSO = open(filepath_dso_calib, 'w', encoding='utf-8')
            FileDSO.write("Pinhole %f %f %f %f 0\n" % (fx, fy, cx, cy))
            FileDSO.write("%d %d\n" % (image_width, image_height))
            FileDSO.write("crop\n")
            FileDSO.write("%d %d\n" % (cropped_image_width, cropped_image_height))
            FileDSO.close()

            """
            ### DSO command line: ###
            bin/dso_dataset \
                files=XXXXX/sequence_XX/images.zip \
                calib=XXXXX/sequence_XX/camera.txt \
                gamma=XXXXX/sequence_XX/pcalib.txt \
                vignette=XXXXX/sequence_XX/vignette.png \
                preset=0 \
                mode=0
            """
            filepath_dso_shell = scene.vslam_output_directory + "run_dso_" + camera.name + ".sh"
            FileDSO = open(filepath_dso_shell, 'w', encoding='utf-8')
            FileDSO.write("bin/dso_dataset files=%s calib=%s" % (dso_image_folder, dso_calib_file))
            FileDSO.close()



        if(scene.vslam_bool_export_libmv):
            # TODO: Add support for libMV as well (Blender's internal camera tracker)
            filepath_libmv = scene.vslam_output_directory + ""
            FileLibMVProject = open(filepath_libmv, 'w', encoding='utf-8')
            FileLibMVProject.close()
            
        return {"FINISHED"}

class VSLAMRenderSequence(Operator):
    '''
    This operator renders the animation as either opengl or via the selected render engine.
    '''

    bl_idname = "vslam.render"
    bl_label = "Render image sequence"
    
    def execute(self, context):
        filepath = context.scene.vslam_output_directory + "/" + context.scene.vslam_camera + "/" + context.scene.vslam_camera + "_"
        filepath = bpy.path.native_pathsep(filepath)
        context.scene.render.filepath = filepath
        if(context.scene.vslam_bool_render_opengl):
            bpy.ops.render.opengl(animation=True)
        else:
            bpy.ops.render.render(animation=True)
        return {"FINISHED"}
    
class VSLAMEffectAutomaticGainControl(Operator):
    '''
    This operator animates the exposure setting for cycles randomly
    '''
    
    bl_idname = "vslam.effect_auto_gain_control"
    bl_label = "Activate Automatic Gain Control"
    
    def execute(self, context):
        print("Automatic Gain Control activated")
        scene = context.scene
        
        for frame in range(scene.frame_start, scene.frame_end + 1):
            scene.frame_set(frame)
            noise = np.random.normal(0.0, context.scene.vslam_automatic_gain_control_std_deviation)
            scene.cycles.film_exposure = 1.0 + noise
            scene.cycles.keyframe_insert("film_exposure")
        
        return {"FINISHED"}
        
class VSLAMEffectRollingShutter(Operator):
    '''
    This operator sets the rolling shutter setting to the one provided by the user
    '''
    
    bl_idname = "vslam.effect_rolling_shutter"
    bl_label = "Activate Rolling Shutter"
    
    def execute(self, context):
        print("Rolling Shutter (and Motion Blur) activated")
        bpy.context.scene.render.use_motion_blur = True
        bpy.context.scene.cycles.rolling_shutter_type = 'TOP'
        bpy.context.scene.cycles.rolling_shutter_duration = context.scene.vslam_rolling_shutter_factor
        return {"FINISHED"}
        
class VSLAMEffectMotionBlur(Operator):
    '''
    This operator activates the motion blur render effect
    '''
    
    bl_idname = "vslam.effect_motion_blur"
    bl_label = "Activate Motion Blur"
    
    def execute(self, context):
        print("Motion Blur activated")
        bpy.context.scene.render.use_motion_blur = True
        bpy.context.scene.render.motion_blur_shutter = context.scene.vslam_motion_blur_duration
        
        return {"FINISHED"}

class VSLAMToolPanel(Panel):
    '''
    This panel will be displayed in the toolbar and provides a simple ui for this addon
    '''
    
    bl_label = "B-SLAM-SIM: Create test data" # Name of the Panel
    #bl_idname = "SCENE_PT_test11"
    bl_space_type = "PROPERTIES" # show in 3D View
    bl_region_type = "WINDOW"  # more specifically: tool panel
    bl_context = "scene"
    bl_category = "VSLAM"
    
    # Draw UI elements
    def draw(self, context):
        layout = self.layout
        settings = layout.box()
        settings.label(text="Settings:")
        row = settings.row()
        row.prop(context.scene, "vslam_output_directory", text="Output directory")
        row = settings.row()
        row.prop_search(context.scene, "vslam_camera", context.scene, "objects", text="Camera to use")
        row = settings.row()
        row.prop(context.scene, "vslam_altered_camera", text="Altered camera")
        col = settings.row()
        box = col.box()
        box.label(text="Noise (Position):", icon="ANIM_DATA")
        box.prop(context.scene, "vslam_noise_translation_mean", text="Mean", expand=True)
        box.prop(context.scene, "vslam_noise_translation_std_deviation", text="Standard deviation", expand=True)
        box = col.box()
        box.label(text="Noise (Orientation):", icon="ANIM_DATA")
        box.prop(context.scene, "vslam_noise_orientation_mean", text="Mean", expand=True)
        box.prop(context.scene, "vslam_noise_orientation_std_deviation", text="Standard deviation", expand=True)
        row = settings.row()
        row.prop(context.scene, "vslam_bool_export_ideal", text="Export ideal camera poses")
        row = settings.row()
        row.prop(context.scene, "vslam_bool_export_noisy", text="Export noisy camera poses")
        row = settings.row()
        row.prop(context.scene, "vslam_bool_export_timestamp", text="Export timestamps")
        row = settings.row()
        row.prop(context.scene, "vslam_bool_export_dso", text="Create a DSO project")
        row.prop(context.scene, "vslam_dso_scale_factor", text="Scale factor:")
        row = settings.row()
        row.enabled = False
        row.prop(context.scene, "vslam_bool_export_libmv", text="Create a libMV project (N/A)", emboss=True)
        row = settings.row()
        row.prop(context.scene, "vslam_bool_render_opengl", text="Render as OpenGL")
        row = settings.row()
        box = row.box() # Box for Automatic Gain Control
        box.label(text="Automatic Gain Control:", icon="ANIM_DATA")
        box.prop(context.scene, "vslam_automatic_gain_control_std_deviation", text="Standard Deviation (image brightness scale)", expand=True)
        row = settings.row()
        box = row.box() # Box for Motion Blur
        box.label(text="Motion Blur:", icon="ANIM_DATA")
        box.prop(context.scene, "vslam_motion_blur_duration", text="Time in frames", expand=True)
        row = settings.row()
        box = row.box() # Box for Rolling Shutter
        box.label(text="Rolling Shutter (0=no, 1=yes):", icon="ANIM_DATA")
        box.prop(context.scene, "vslam_rolling_shutter_factor", text="Rolling Shutter factor", expand=True)
        
        commands = layout.box()
        commands.label(text="Commands:")
        row = commands.row()
        row.operator("vslam.effect_auto_gain_control", text="Simulate Automatic Gain Control (AGC)", icon="ANIM_DATA")
        row = commands.row()
        row.operator("vslam.effect_rolling_shutter", text="Simulate Rolling Shutter", icon="ANIM_DATA")
        row = commands.row()
        row.operator("vslam.effect_motion_blur", text="Simulate Motion Blur", icon="ANIM_DATA")
        row = commands.row()
        row.operator("vslam.render", text="Render image sequence", icon="IMAGE_DATA")
        row = commands.row()
        row.operator("vslam.import_dso", text="Import DSO (\"camera_dso.csv\")", icon="EMPTY_DATA")
        row = commands.row()
        row.operator("vslam.export_camera", text="Export camera poses and project files", icon="CAMERA_DATA")
        
        sep = layout.separator()
        

# Register the new class within Blender
def register():
    bpy.utils.register_class(VSLAMToolPanel)
    bpy.utils.register_class(VSLAMRenderSequence)
    bpy.utils.register_class(VSLAMEffectAutomaticGainControl)
    bpy.utils.register_class(VSLAMEffectRollingShutter)
    bpy.utils.register_class(VSLAMEffectMotionBlur)
    bpy.utils.register_class(VSLAMImportDSO)
    bpy.utils.register_class(VSLAMExportCamera)

    bpy.types.Scene.vslam_output_directory = bpy.props.StringProperty(subtype='DIR_PATH', default=bpy.path.abspath("//"))
    bpy.types.Scene.vslam_camera = bpy.props.StringProperty(default=bpy.context.scene.camera.name if bpy.context.scene.camera is not None else '') #TODO: Dont use string, but bpy.props.PointerProperty
    
    
    bpy.types.Scene.vslam_altered_camera = bpy.props.StringProperty(default="Camera_altered")
    bpy.types.Scene.vslam_noise_translation_mean = bpy.props.FloatVectorProperty(name="Mean", description="Enter the mean for gaussian noise", default=(0.0, 0.0, 0.0), unit="LENGTH", subtype="XYZ")
    bpy.types.Scene.vslam_noise_translation_std_deviation = bpy.props.FloatVectorProperty(name="Standard deviation", description="Enter the standard deviation for gaussian noise", default=(0.01, 0.01, 0.01), unit="LENGTH", subtype="XYZ")
    bpy.types.Scene.vslam_noise_orientation_mean = bpy.props.FloatVectorProperty(name="Mean", description="Enter the mean for gaussian noise", default=(0.0, 0.0, 0.0), unit="ROTATION", subtype="XYZ")
    bpy.types.Scene.vslam_noise_orientation_std_deviation = bpy.props.FloatVectorProperty(name="Standard deviation", description="Enter the standard deviation for gaussian noise", default=(math.radians(1), math.radians(1), math.radians(1)), unit="ROTATION", subtype="XYZ")
    
    # Simulated Camera Effects
    bpy.types.Scene.vslam_automatic_gain_control_std_deviation = bpy.props.FloatProperty(name="AGC Standard deviation", description="Enter the standard deviation for gaussian noise being added to normal 1.0 exposure", default=0.02, unit="TIME")
    bpy.types.Scene.vslam_motion_blur_duration = bpy.props.FloatProperty(name="Motion Blur Shutter duration in frames", description="Enter the time in frames between shutter open and close", default=0.1, min=0.01, max=2.0, subtype="FACTOR")
    bpy.types.Scene.vslam_rolling_shutter_factor = bpy.props.FloatProperty(name="Rolling Shutter factor", description="Enter the percentage of how much rolling shutter there should be in relation to motion blur time (0 = no rolling shutter, 1 = only rolling shutter for the complete motion blur duration)", default=0.1, min=0.0, max=1.0, subtype="PERCENTAGE")
    
    bpy.types.Scene.vslam_bool_export_ideal = bpy.props.BoolProperty(default=True)
    bpy.types.Scene.vslam_bool_export_noisy = bpy.props.BoolProperty(default=True)
    bpy.types.Scene.vslam_bool_export_timestamp = bpy.props.BoolProperty(default=True)
    bpy.types.Scene.vslam_bool_export_dso = bpy.props.BoolProperty(default=True)
    bpy.types.Scene.vslam_dso_scale_factor = bpy.props.FloatProperty(name="DSO: Scale down images", description="Scale images by factor", default=0.25)
    bpy.types.Scene.vslam_bool_export_libmv = bpy.props.BoolProperty(default=False) # TODO!
    bpy.types.Scene.vslam_bool_render_opengl = bpy.props.BoolProperty(default=True)

# Unregister the class when Blender is closed
def unregister():
    bpy.utils.unregister_class(VSLAMToolPanel)
    bpy.utils.unregister_class(VSLAMRenderSequence)
    bpy.utils.unregister_class(VSLAMEffectAutomaticGainControl)
    bpy.utils.unregister_class(VSLAMEffectRollingShutter)
    bpy.utils.unregister_class(VSLAMEffectMotionBlur)
    bpy.utils.unregister_class(VSLAMImportDSO)
    bpy.utils.unregister_class(VSLAMExportCamera)

    del bpy.types.Scene.vslam_output_directory
    del bpy.types.Scene.vslam_camera
    del bpy.types.Scene.vslam_altered_camera
    del bpy.types.Scene.vslam_noise_translation_mean
    del bpy.types.Scene.vslam_noise_translation_std_deviation
    del bpy.types.Scene.vslam_noise_orientation_mean
    del bpy.types.Scene.vslam_noise_orientation_std_deviation

    del bpy.types.Scene.vslam_automatic_gain_control_std_deviation
    del bpy.types.Scene.vslam_motion_blur_duration
    del bpy.types.Scene.vslam_rolling_shutter_factor

    del bpy.types.Scene.vslam_bool_export_ideal
    del bpy.types.Scene.vslam_bool_export_noisy
    del bpy.types.Scene.vslam_bool_export_timestamp
    del bpy.types.Scene.vslam_bool_export_dso
    del bpy.types.Scene.vslam_dso_scale_factor
    del bpy.types.Scene.vslam_bool_export_libmv
    del bpy.types.Scene.vslam_bool_render_opengl

# Needed to run script in Text Editor
if __name__ == "__main__":
    register()


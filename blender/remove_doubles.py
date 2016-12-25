#!BPY

#
# $ blender --python remove_doubles.py
#

import bpy

inputdir = "/home/kanehiro/src/HRP5P/model/"
outputdir = "/tmp/"
filenames = ["HRP-5P_ConceptDesign_Rleg_Link2.wrl",
             "HRP-5P_ConceptDesign_Rarm_Link0.wrl",
             "HRP-5P_ConceptDesign_Chest_Link2.wrl"]

for fn in filenames:
    
    for item in bpy.context.scene.objects:
        if item.type == 'MESH':
            bpy.context.scene.objects.unlink(item)
    for item in bpy.data.objects:
        if item.type == 'MESH':
            bpy.data.objects.remove(item)
    for item in bpy.data.meshes:
        bpy.data.meshes.remove(item)
    for item in bpy.data.materials:
        bpy.data.materials.remove(item)

    inputpath = inputdir+fn
    outputpath = outputdir+fn
    bpy.ops.import_scene.x3d(filepath=inputpath)
    for obj in bpy.data.objects:
        print("obj=",obj)
        if obj.name == "Camera" or obj.name == "Lamp":
            continue
        #obj.select=True
        bpy.context.scene.objects.active=obj
        bpy.ops.object.editmode_toggle()
        bpy.ops.mesh.remove_doubles()
        bpy.ops.object.editmode_toggle()
    bpy.ops.export_scene.vrml2(filepath=outputpath)

bpy.ops.wm.quit_blender()


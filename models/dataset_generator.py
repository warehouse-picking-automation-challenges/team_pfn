# Copyright 2016 Preferred Networks, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import bpy
import numpy as np
import math
import sys
import shutil

cat_list = [
    "i_am_a_bunny_book",
    "laugh_out_loud_joke_book",
    "rawlings_baseball",
    "folgers_classic_roast_coffee",
    "scotch_bubble_mailer",
    "elmers_washable_no_run_school_glue",
    "hanes_tube_socks",
    "womens_knit_gloves",
    "cloud_b_plush_bear",
    "kyjen_squeakin_eggs_plush_puppies",
    "creativity_chenille_stems",
    "oral_b_toothbrush_red",
    "cool_shot_glue_sticks",
    "dr_browns_bottle_brush",
    "fiskars_scissors_red",
    "platinum_pets_dog_bowl",
    "fitness_gear_3lb_dumbbell",
    "rolodex_jumbo_pencil_cup",
    "expo_dry_erase_board_eraser",
    "kleenex_tissue_box",
    "crayola_24_ct",
    "dove_beauty_bar",
    "staples_index_cards",
    "up_glucose_bottle",
    "dasani_water_bottle",
    "cherokee_easy_tee_shirt",
    "peva_shower_curtain_liner",
    "barkely_hide_bones",
    "soft_white_lightbulb",
    "safety_first_outlet_plugs",
    "command_hooks",
    "easter_turtle_sippy_cup",
    "scotch_duct_tape",
    "woods_extension_cord",
    "clorox_utility_brush",
    "kleenex_paper_towels",
    "ticonderoga_12_pencils",
    "jane_eyre_dvd",
    "oral_b_toothbrush_green"
]

for item in bpy.context.scene.objects:
    if 'Camera' in item.name or 'lightsource' in item.name or 'order_bin' in item.name:
        continue
    bpy.context.scene.objects.unlink(item)

# set objects
for i,cat in enumerate(cat_list):
    if np.random.uniform(0, 1)>0.15:
        continue
    d = '/home/user/apc/apc2016/models/%02d_%s.blend/Object'%(i+1,cat)
    fn = '%02d_%s'%(i+1,cat)
    fp = d+'/'+fn

    bpy.ops.wm.append(directory=d,filepath=fp,filename=fn)

    o = bpy.context.scene.objects[fn]
    o.select = True
    bpy.context.scene.objects.active = o
    #bpy.context.scene.objects.active = o
    #bpy.ops.object.duplicate()
    #o = bpy.context.object

    o.location.x = np.random.uniform(-20,20)
    o.location.y = np.random.uniform(-20,20)
    o.location.z = np.random.uniform(20,200)
    o.rotation_euler.x = np.random.uniform(-math.pi, math.pi)
    o.rotation_euler.y = np.random.uniform(-math.pi, math.pi)
    o.rotation_euler.z = np.random.uniform(-math.pi, math.pi)
    bpy.ops.rigidbody.object_add()
    o.rigid_body.collision_shape = 'CONVEX_HULL'
    o.rigid_body.linear_damping = 0.1
    o.rigid_body.angular_damping = 0.1
    o.rigid_body.use_margin = True
    o.rigid_body.collision_margin = 0.01
    o.rigid_body.friction = 0.1

# set camera
o = bpy.context.scene.objects['Camera']
o.select = True
bpy.context.scene.objects.active = o
o.location.x = np.random.uniform(-10,10)
o.location.y = np.random.uniform(-10,10)
o.location.z = np.random.uniform(40,60)
o.rotation_euler.x = np.random.uniform(-math.pi/15, math.pi/15)
o.rotation_euler.y = np.random.uniform(-math.pi/15, math.pi/15)
o.rotation_euler.z = np.random.uniform(-math.pi, math.pi)
bpy.context.object.data.lens = np.random.uniform(3,6)
# set light
for o in bpy.context.scene.objects:
    if "lightsource" in o.name:
        o.select = True
        bpy.context.scene.objects.active = o
        bpy.context.object.data.energy = np.random.uniform(0.1,0.4)
        try:
            bpy.context.object.data.gamma = np.random.uniform(0.02,0.1)
        except:
            pass
        bpy.context.object.data.color = (np.random.uniform(0.5,1),np.random.uniform(0.5,1),np.random.uniform(0.5,1))
        o.location.x = np.random.uniform(-300,300)
        o.location.y = np.random.uniform(-300,300)
        o.location.z = np.random.uniform(200,400)

# set background
o = bpy.context.scene.objects['order_bin']
o.select = True
bpy.context.scene.objects.active = o
bpy.context.object.active_material.diffuse_color = (np.random.uniform(0.0,1),np.random.uniform(0.0,1),np.random.uniform(0.0,1))
bpy.context.object.active_material.diffuse_intensity = np.random.uniform(0,1)
bpy.context.object.active_material.specular_intensity = np.random.uniform(0,1)
bpy.context.object.active_material.specular_hardness = np.random.uniform(0,100)
bpy.context.object.dimensions[0] = np.random.uniform(20,100)
bpy.context.object.dimensions[1] = np.random.uniform(20,100)
bpy.context.object.dimensions[2] = np.random.uniform(20,60)
if np.random.randint(4)==0:
    bpy.context.object.active_material.raytrace_mirror.use = True
    bpy.context.object.active_material.raytrace_mirror.reflect_factor = np.random.uniform(0,1)
    bpy.context.object.active_material.raytrace_mirror.gloss_factor = np.random.uniform(0.9,1)
# set world
bpy.context.scene.world.light_settings.ao_factor = np.random.uniform(0,1)
bpy.context.scene.world.horizon_color = (np.random.uniform(0.0,1),np.random.uniform(0.0,1),np.random.uniform(0.0,1))
bpy.context.scene.gravity[0] = np.random.uniform(-3,3)
bpy.context.scene.gravity[1] = np.random.uniform(-3,3)
bpy.context.scene.gravity[2] = np.random.uniform(-5,-15)


argv = sys.argv

try:
    index = argv.index("--") + 1
except ValueError:
    print('arg error')
    sys.exit(1)

argv = argv[index:]

trial = int(argv[0].strip())


print(trial)


bpy.ops.render.render(animation=True)

bpy.context.scene.frame_set(200)
import os
path_dir = '/home/user/apc/generated_images_0421'
try:
    os.mkdir(path_dir)
except:
    pass
#for i in bpy.data.node_groups:
#    print(i.name)
#bpy.data.node_groups["Compositing Nodetree"].nodes["File Output.001"].base_path = "%s/rgb"%path_dir
#bpy.data.node_groups["Compositing Nodetree"].nodes["File Output"].base_path = "%s/depth"%path_dir


shutil.move('%s/rgb/Image0200.png'%path_dir, '%s/rgb_%d.png'%(path_dir,trial))
shutil.move('%s/depth/Image0200.png'%path_dir, '%s/depth_%d.png'%(path_dir,trial))
#bpy.ops.render.render()
#print(bpy.data.images[1].name)
#bpy.data.images['Render Result'].save_render(filepath='%s/render_%d.png'%(path_dir,trial))

for ic,cat in enumerate(['order_bin']+cat_list):
    for o in bpy.context.scene.objects:
        if cat in o.name:
            o.select = True
            bpy.context.scene.objects.active = o

            mat = bpy.data.materials.new('mat_%d'%ic)
        #    mat.diffuse_color = (np.random.uniform(0,1),np.random.uniform(0,1),np.random.uniform(0,1))
            mat.diffuse_color = (6*(ic)/255.0,6*(ic)/255.0,6*(ic)/255.0)
            mat.diffuse_intensity = 1.0
            mat.specular_intensity = 0.0
            mat.use_shadeless = True
            bpy.context.object.data.materials.append(mat)
            bpy.context.object.active_material_index = len(bpy.context.object.material_slots) - 1

            bpy.ops.object.editmode_toggle()
            bpy.ops.mesh.select_all(action='SELECT')
            bpy.ops.object.material_slot_assign()
            bpy.ops.object.editmode_toggle()
bpy.context.scene.world.horizon_color = (0,0,0)
bpy.context.scene.render.use_antialiasing = False
bpy.ops.render.render()
bpy.context.scene.render.use_antialiasing = True
shutil.move('%s/rgb/Image0200.png'%path_dir, '%s/label_%d.png'%(path_dir,trial))
#print(bpy.data.images[1].name)
#bpy.data.images['Render Result'].save_render(filepath='%s/label_%d.png'%(path_dir,trial))
#for i in bpy.data.images:
#    print(i.name)
#bpy.data.images['Viewer Node'].save_render(filepath='%s/depth_%d.png'%(path_dir,trial))


#for i in range(len(bpy.app.handlers.frame_change_pre)):
#    bpy.app.handlers.frame_change_pre.pop()
#bpy.app.handlers.frame_change_pre.append(frame_change_handler)

import genesis as gs
import numpy as np

gs.init(backend=gs.cpu)

scene = gs.Scene(
    show_viewer = True,
    viewer_options = gs.options.ViewerOptions(
        res           = (1280, 960),
        camera_pos    = (3.5, 0.0, 2.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov    = 40,
        max_FPS       = 120,
    ),
    vis_options = gs.options.VisOptions(
        show_world_frame = True,
        world_frame_size = 1.0,
        show_link_frame  = True,
        show_cameras     = True,
        plane_reflection = True,
        ambient_light    = (0.1, 0.1, 0.1),
    ),
    renderer=gs.renderers.Rasterizer(),
)

plane = scene.add_entity(
    gs.morphs.Plane(),
)

box = scene.add_entity(
    morph = gs.morphs.Box(
        size= (0.5, 0.5, 0.5), 
        pos= (0.0, 0.0, 0.0),
        )
    ,
)

cam = scene.add_camera(
    res    = (640, 480),
    pos    = (3.5, 0.0, 2.5),
    lookat = (0, 0, 0.5),
    fov    = 30,
    GUI    = True,
)

scene.build()

for i in range(300):
    scene.step()
    cam.set_pose(
        transform = np.eye(4)
    )
    rgb, depth, segmentation, normal = cam.render(rgb=True, depth=True, segmentation=True, normal=True)
    
    
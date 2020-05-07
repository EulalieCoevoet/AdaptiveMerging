# Help

The scene file has one `<root>` element that contains bodies and collision properties. 

## Body tags

`<body>` tags have required attributes:  

- `type`: one of box, mesh, sphere, plane, composite  
- `name`: anything you want (for spring attachments and debugging), and need to be unique for springs that go between bodies. 

All bodies currently have unit density, friction and restitution coefficient. Different body types have required attributes that must appear after the tag: 
-	`box`: 
  - `dim = "l w d"` for x y z dimensions 
  - `scale = "s"` for a scale factor on the dimensions (for convenience, see composite below). 
-	`mesh`: 
  - `scale="s"` for a scale factor, 
  - `obj="fname.obj"` for the mesh file, 
  - `st="fname.sph"` for the sphere tree file (undocumented) 
- `plane`: (always pinned) 
  - `p="x y x"` a point on the plane, 
  - `n="x y z"` the normal of the plane 
- `composite`: (always not a factory part, for now) 
  - `obj="fname.obj"` for the mesh file (optional).

## Composite

Composite elements contain a collection of bodies (ideally only box and sphere, avoid mesh and plane) 
thatdefine their inertial properties and collision behaviour. 
Composite bodies should not include planes (pinned) orother composites. They must be limited to under 256 parts, 
and even then if they start to include a largenumber of parts then some BV collision detection should 
be used instead (unimplemented). 

If a mesh is define in the optional obj attribute then these bodies can be drawn instead with the mesh.  
Advice: 
1. Use blender to layer spheres and boxes on top of your mesh 
2. Plan ahead and build your object to an appropriate size! 
3. Read off the locations, rotation, and scale to define the attributes and tags of the subbodies. 
4. Export the obj mesh with options (bottom left hand side of export interface) selection only, Y forward, and Z up, 
5. Note that box scale will be half the dimension so use scale="2" with the dim xml attribute 
6. Note that you want to view the rotation as axis angle and convert the angle to radians. 

## Body element

The body element can contain elements with the following tags: 
-	`<x> x y z </x>`
Specifies translation, default is zero 
-	`<R> x y z radians <R>`
Specifies rotation, default is identity 
-	`<spring pB="x y z" k="ks" d="kd"/>`
Attaches a zero length spring in given body coordinatesto the world with given stiffness and damping. The kand d attributes can be omitted in which case they willhave default values. 
-	`<pinned> bool </pinned>`
Default is false. 
-	`<factorypart> bool </factorypart>`
Default is true, but set to false to not have the bodypersistent, like pinned bodies, and not be generatedrandomly. 
-	`<col> r g b </col>`
A body material colour with [0,1] RGB values. 

## Collision tags

Few collision parameters can also be set from the XML under the `<collision>` tag:

- `enableCompliance` true or false
- `enablePostStabilization` true or false
- `feedbackStiffness` Baumgarte feedback stiffness coefficient
- `iterations` number of PGS iterations for the full LCP solve 




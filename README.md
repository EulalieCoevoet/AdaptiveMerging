# Adaptive Merging for Rigid Body Simulation

[![Paper](https://img.shields.io/badge/Paper-ACMSIGGRAPH-yellow.svg)](https://dl.acm.org/doi/abs/10.1145/3386569.3392417?casa_token=IZlqZXM_C4UAAAAA:NigEEUkma3E9g3b4FqSfGPvcbQUqWqTYdnkDnu3mwudu9lgNpOITf0cnMo4qJYIIQQuDzvpO0YUefQ)
<!---[![Slides](https://img.shields.io/badge/Slides-on_google_drive-blue.svg)]()--->

![TowerPlatform](docs/towerplatform.png "A tower on a mobile platform hit by a projectile.")

We reduce computation time in rigid body simulations by merging collections of bodies when they share a common spatial velocity. Merging relies on monitoring the state of contacts, and a metric that compares the relative linear and angular motion of bodies based on their sizes. Unmerging relies on an inexpensive single iteration projected Gauss-Seidel sweep over contacts between merged bodies, which lets us update internal contact forces over time, and use the same metrics as merging to identify when bodies should unmerge. Furthermore we use a contact ordering for graph traversal refinement of the internal contact forces in collections, which helps to correctly identify all the bodies that must unmerge when there are impacts. The general concept of merging is similar to the common technique of sleeping and waking rigid bodies in the inertial frame, and we exploit this too, but our merging is in moving frames, and unmerging takes place at contacts between bodies rather than at the level of bodies themselves. We discuss the previous relative motion metrics in comparison to ours, and evaluate our method on a variety of scenarios.

[![Teaser](docs/youtubevideo.png "Teaser video")](https://www.youtube.com/watch?v=mmVVRVt8EF4)

# Information

## 3D Java Application

To test our implementation, you can run the Java application `src/mergingBodies3D/LCPApp3D`.
Numerous examples are available in the folder `scene3D`. The scenes are built using XML, a description is provided in a README file under the folder `scenes3D`. If you want to build your own scene you can also have a look at the `scenes3D/python` folder which contains tools to generate the XML, along with some examples.

## 2D Java Application

This repository also holds a 2D implementation of adaptive merging. You can try it by running the Java application `src/mergingBodies2D/LCPApp2D`, just note that the implementation in 2D is **not up to date** with what is described in the paper. Many examples are provided in the `scenes2D` folder, which contains a `README.md` file with more details on how to make your own scene.

# Authors

Eulalie Coevoet, Otman Benchekroun & Paul G. Kry

# Adaptive merging for rigid body simulation

[![Paper](https://img.shields.io/badge/Paper-ACMSIGGRAPH-yellow.svg)]()
[![Slides](https://img.shields.io/badge/Slides-on_google_drive-blue.svg)]()
[![Video](https://img.shields.io/badge/Video-on_youtube-green.svg)](https://www.youtube.com/embed/mmVVRVt8EF4)

![TowerPlatform](https://github.com/EulalieCoevoet/AdaptiveMerging/blob/master/images/towerplatform.png "A tower on a mobile platform hit by a projectile.")

We reduce computation time in rigid body simulations by merging collections of bodies when they share a common spatial velocity. Merging relies on monitoring the state of contacts, and a metric that compares the relative linear and angular motion of bodies based on their sizes. Unmerging relies on an inexpensive single iteration projected Gauss-Seidel sweep over contacts between merged bodies, which lets us update internal contact forces over time, and use the same metrics as merging to identify when bodies should unmerge. Furthermore we use a contact ordering for graph traversal refinement of the internal contact forces in collections, which helps to correctly identify all the bodies that must unmerge when there are impacts. The general concept of merging is similar to the common technique of sleeping and waking rigid bodies in the inertial frame, and we exploit this too, but our merging is in moving frames, and unmerging takes place at contacts between bodies rather than at the level of bodies themselves. We discuss the previous relative motion metrics in comparison to ours, and evaluate our method on a variety of scenarios.

# Authors

Eulalie Coevoet, Otman Benchekroun & Paul G. Kry

# License 


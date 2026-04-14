# Vendored Coal Core

This directory contains a trimmed C++ subset of Coal for this workspace.

Kept:
- primitive shape, BVH, hfield, narrowphase collision, and distance core
- plain CMake package export as `coal::coal`
- Eigen and Boost header dependencies

Removed:
- upstream git metadata, CI, docs, tests, Python bindings, hpp-fcl compatibility headers
- jrl-cmakemodules/fetch-based build flow
- assimp mesh loader, octomap, broadphase managers, serialization, contact patch solvers

Project code should prefer the shallow wrapper in `src/capabilities/collision/include/collision/coal_scene.hpp` instead of using Coal internals directly.

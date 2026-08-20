# Contributing

World Engine accepts focused fixes and measurable optimizations that preserve Sable compatibility.

- Keep addon classes under `com.nstut.worldengine`; never copy a Sable class into the output jar.
- Update `MIGRATION.md` when moving or changing an optimization boundary.
- Add a regression test for scheduler, region, terrain, JNI-buffer, or voxel-collider changes.
- Run `.\gradlew.bat build` and `cargo test -p worldengine_rapier --all-targets` before submitting a change.
- Changes must be work you have the right to contribute.

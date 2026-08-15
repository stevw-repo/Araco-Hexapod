# Araco Hexapod — Phase 0 License and Distribution Audit

Audit date: 2026-08-16

Status: **COMPLETE — PASS FOR PHASE 0 SOURCE FOUNDATION**

This is an engineering review of the source currently present in the
repository. It is not legal advice and does not replace qualified review for a
binary release, commercial distribution, vendor SDK, or app-store submission.

## Selected license

- Project-authored repository and ROS package content uses the MIT License,
  SPDX identifier `MIT`.
- Root `LICENSE` and all nine package-local `LICENSE` files contain identical
  MIT text with `Copyright (c) 2026 Araco Hexapod contributors`.
- Every Phase 0 project source file whose format supports comments carries
  `SPDX-License-Identifier: MIT`.
- Every package manifest declares `<license file="LICENSE">MIT</license>`.
- MIT requires its copyright and permission notice to accompany copies or
  substantial portions and supplies the standard warranty/liability
  disclaimer.
- The earlier GPL-3.0-only checkpoint remains valid for copies received under
  that license. The same rights holder authorized the current MIT relicense,
  which grants the current version under the more permissive terms.

“No license” was considered and rejected. Default copyright would reserve the
rights to reproduce, modify, and distribute, which is inconsistent with a
normal collaborative public open-source repository.

## Direct Phase 0 external dependencies

The dependency licenses below were checked from the ROS 2 Jazzy package
manifests installed under `/opt/ros/jazzy/share` on 2026-08-16.

| Dependency group | Use in Phase 0 | Declared upstream license | Result |
|---|---|---|---|
| `ament_cmake` | Package build system | Apache License 2.0 | Referenced, not bundled; no conflict |
| `ament_cmake_pytest` | Package and generated-IDL tests | Apache License 2.0 | Referenced, not bundled; no conflict |
| `ament_lint_auto`, `ament_lint_common` | Source/package linting | Apache License 2.0 | Referenced, not bundled; no conflict |
| `rosidl_default_generators` | Message/action generation | Apache License 2.0 | Referenced, not bundled; no conflict |
| `rosidl_default_runtime` | Generated interface runtime | Apache License 2.0 | Referenced, not bundled; no conflict |
| `builtin_interfaces`, `geometry_msgs`, `std_msgs` | Types referenced by project IDL | Apache License 2.0 | Referenced, not bundled; no conflict |

Current Phase 0 packages use installed dependencies and do not copy those
projects into this repository. A later binary, container, installer, vendored
dependency, or device-image release requires an artifact-level inventory that
preserves every included upstream license and notice.

Toolchain programs such as CMake, Python, pytest, the compiler, colcon, and
rosdep are supplied by the host environment and are not distributed from this
source repository.

## Bundled-material inventory

No ROS package currently contains object code, vendored libraries, meshes, CAD,
fonts, product logos, sample code, or third-party data. `THIRD_PARTY_NOTICES.md`
is therefore not created in Phase 0. Add it when a future bundled dependency or
asset requires attribution.

The rough-dynamics JSON and input manifest under `tools/fusion/` are project
evidence/data rather than vendor geometry or Autodesk API code. The raw
external Fusion archive is not bundled and is not a runtime input.

## Fusion exporter boundary

`tools/fusion/AracoRobotDescriptionExporter` is project-authored Python source
that imports `adsk.core` and `adsk.fusion`. The repository does not contain an
Autodesk module, SDK binary, API stub, Autodesk sample-code copy, Fusion
installer, or other Autodesk development material. The exporter is offline CAD
evidence tooling and is not a ROS package, build dependency, linked library, or
runtime dependency of the nine Phase 0 packages.

MIT removes the prior GPL linking question but does not remove Autodesk's own
terms:

1. The exporter source is distributed under MIT.
2. Autodesk Fusion and its API remain proprietary third-party systems and are
   not licensed or redistributed here.
3. This repository must not bundle Autodesk API materials or a Fusion runtime.
4. The project makes no claim that MIT grants API access or satisfies an
   Autodesk subscription or Developer License.
5. Before distributing the add-in as a supported third-party product, through
   an app store, in a container/installer, or with Autodesk materials, the
   distributor must verify that their Autodesk entitlement and current terms
   permit that use.

Autodesk publishes public Fusion API add-in samples under open-source licenses,
but that does not prove one entitlement applies to every distributor or use.
The exporter-use/distribution boundary must be re-audited if packaging or use
changes.

## Future robot assets and releases

- Phase 1 may bundle only project-authored or rights-cleared robot resources.
  Detailed vendor CAD with unknown redistribution permission remains excluded.
- Project-generated meshes retain preferred editable source and reproducible
  generation tooling as a project provenance and maintainability requirement.
- Preserve exact creator, source, license, required attribution, modification,
  and redistribution metadata for every bundled non-project asset.
- Repeat this audit for containers, binaries, generated meshes, device images,
  the future Isaac adapter, and any new vendored dependency.

## References checked

- [OSI MIT License](https://opensource.org/license/mit)
- [GitHub licensing guidance](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/licensing-a-repository)
- [ROS REP 149 package manifest license fields](https://docs.ros.org/en/independent/api/rep/html/rep-0149.html)
- [Autodesk Terms of Use, API provisions](https://www.autodesk.com/company/terms-of-use/en/general-terms)
- [Autodesk Fusion API documentation](https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-A92A4B10-3781-4925-94C6-47DA85A4F65A)
- [Autodesk's public Fusion API sample organization](https://github.com/AutodeskFusion360)

## Phase 0 conclusion

The source-only ROS package foundation has no known incompatible linked or
bundled dependency and no missing third-party notice. This conclusion does not
approve a binary/container release, redistribution of vendor CAD, or a packaged
Autodesk integration; those artifacts require their own audit.

All nine source packages and all nine installed package shares contain an exact
copy of the root MIT text. Package metadata and SPDX checks passed as part of
the 111-test Phase 0 suite on 2026-08-16.

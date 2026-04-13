# Branching and Mod Layout

## Branch model

- `base/no-mods`: baseline branch with no new modular mod work (tracking clean base).
- `development/integration`: integration branch where multiple mods are merged and validated together.
- `master`: stable branch for mods that already passed tests.
- `mod/*`: one branch per mod or integration area.

Current mod branches created:

- `mod/softbody`
- `mod/cloth-physics`
- `mod/ui-framework`
- `mod/integration-abmx`
- `mod/integration-boobsettings`
- `mod/integration-moreaccessories`
- `mod/integration-overlay`

## Workflow

1. Start from `development/integration`.
2. Create/switch to one `mod/*` branch.
3. Implement and test only that mod.
4. Merge mod branch into `development/integration` for integration tests.
5. If integration passes, merge into `master`.
6. If a mod fails, discard/revert only that `mod/*` branch.

## Folder organization

### Core

- `Core/Framework/Modules`: shared module contracts and registry infrastructure.
- `Core/Mods/SoftBody`: softbody runtime implementation.
- `Core/Mods/Cloth`: cloth runtime/proxy/data implementation.
- `Core/Mods/Modules`: mod-specific module logic and feature IDs.
- `Core/Mods/Integrations/*`: external plugin integrations per provider.

### GUI

- `GUI/Framework/Modules`: shared module panel contracts/factory.
- `GUI/Mods/SoftBody`: softbody panel UI.
- `GUI/Mods/Cloth`: cloth panel UI.

## Naming conventions

- Core mod logic/runtime: `Core/Mods/<ModName>/...`
- Core integration wrappers: `Core/Mods/Integrations/<Provider>/...`
- UI by mod: `GUI/Mods/<ModName>/...`
- Shared infra: `Core/Framework/...`, `GUI/Framework/...`

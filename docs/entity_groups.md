# Entity Groups

`FxScene` can manage a named set of entities as one thing: delete them together, enable or
disable them together, and — by default — exempt them from colliding with one another. A group
imposes no connectivity or uniformity on its members; builders that populate groups may.

```cpp
auto rig = scene.create_group("rig");          // members won't collide with each other
auto pile = scene.create_group("pile", true);  // self_collide: members do collide

scene.add_to_group(rig, link);   // adds to the scene too, if not already added
rig->size();
rig->members();                  // iterate
rig->set_enabled(false);         // freeze the whole rig
scene.delete_group("rig");       // deletes every member entity, then the group
```

## Intra-group collision filtering

Filtering costs one integer per body, not O(N²) pair exclusions: `FxEntity::collision_group`.
Entities sharing a **negative** value never pair in the broad phase; `0` means no filtering.
Groups created with `self_collide == false` assign each member a unique negative value. The
field is public, so the same mechanism works without a group where you only need the filter.

## Reset

Groups are part of the composition snapshot: `reset()` restores every group with the
membership it had at capture, exactly as it restores entities, joints and constraints.

## Naming

Entities keep their user-given names. Builders that generate members should name them
`<group>_<i>`, using `scene.unique_entity_name(base)` to skip past any taken name rather than
collide. Joints follow the same idea (`<group>_j<i>`), and each joint's constraints are named
`<joint>_<Type>` — unique because joint names are unique, and stable under entity renames.

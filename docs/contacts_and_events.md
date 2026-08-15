# Contacts, Contact Events, and Sensors

`FxScene::step()` has always computed a full contact manifold for every touching
pair, then thrown it away. These APIs retain it, so gameplay and RL code can ask
"what touched what this step?" without redoing collision math.

Everything here is read-only and refreshed by each `step()`. The values stay
valid until the next `step()` or `reset()`.

## Buffered contacts

```cpp
const std::vector<FxContact>& contacts() const;
```

One entry per pair that touched during the step. Each `FxContact` carries:

| Field | Meaning |
|---|---|
| `entity1`, `entity2` | the two bodies, as `shared_ptr<FxEntity>` |
| `count` | number of valid contact points (1 or 2) |
| `position` | up to 2 contact points, in world coordinates |
| `normal` | unit contact normal |
| `penetration_depth` | positive when overlapping |
| `jn_accumulated`, `jt_accumulated` | normal and tangent impulse actually applied |

A pair is usually found in several substeps. The buffer keeps the last one seen,
so the impulses are the ones accumulated by the end of the step.

**On impulses.** `jn_accumulated` is the *velocity-level* impulse — what the
solver applied to cancel closing speed. It is large at impact and decays to zero
once a body settles: a box resting on the ground reports a zero normal impulse,
because nothing is left to cancel. What holds it up from then on is the
position-level penetration solve, which is not an impulse. Use it to measure
impact strength (hit detection, damage, audio), not to measure resting load.

## Begin / end contact events

```cpp
const std::vector<FxContactEvent>& begin_contact_events() const;
const std::vector<FxContactEvent>& end_contact_events() const;
```

`begin` holds pairs touching this step that were not touching last step; `end`
holds pairs that were touching last step and are not now. Each `FxContactEvent`
names `entity1`, `entity2`, and `is_trigger` (true when either is a sensor).

A pair that stays in contact reports `begin` exactly once, not every step. End
events still name their entities even if one was deleted from the scene, because
the previous step's buffer holds a `shared_ptr` to both.

Both lists are sorted by entity id, so repeated runs of the same scene deliver
events in the same order.

## Sensors (triggers)

Set `is_sensor` on an entity, or `sensor: true` under `physics:` in YAML:

```yaml
physics:
    mass: 0.0
    gravity_scale: 0.0
    sensor: true
```

A sensor is detected like any other collider but is skipped by every solver
stage: no penetration resolution, no impulses, no warm starting. Bodies pass
straight through it. Its overlaps appear in `contacts()` (with zero impulses)
and drive begin/end events with `is_trigger == true`.

A sensor also never wakes a sleeping body, since it applies no force that could
disturb one. An overlap with a sleeper is still reported.

## Reading them

Events are built before the step callback runs, so callback code sees them:

```cpp
scene.set_step_callback([](FxScene& scene, double dt) {
    for (const auto& event : scene.begin_contact_events()) {
        if (event.is_trigger) {
            std::cout << event.entity1->get_name() << " entered "
                      << event.entity2->get_name() << "\n";
        }
    }
});
```

Reading them straight after `step()` works equally well:

```cpp
scene.step(dt);
for (const auto& contact : scene.contacts()) {
    float impact = std::fabs(contact.jn_accumulated[0]);
    if (impact > kDamageThreshold) apply_damage(contact.entity1, contact.entity2);
}
```

## Not covered yet

Ray casts, overlap queries, and shape queries are still unimplemented — see item
2 in [ToDo.md](ToDo.md).

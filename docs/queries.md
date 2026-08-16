# Spatial Queries

`FxScene` answers questions about what is where, without stepping: ray casts for line-of-sight
and lidar-style observations, overlap queries for area effects and selection, point queries for
"what is under the cursor".

Overlap queries run the **same narrow phase the simulation uses**, so a query and a contact can
never disagree about what is touching. Disabled entities and entities without collision geometry
are never reported.

## Ray casts

```cpp
bool raycast(const FxVec2f& origin, const FxVec2f& direction, float max_distance,
             FxRayHit& out_hit) const;                     // nearest hit
void raycast_all(const FxVec2f& origin, const FxVec2f& direction, float max_distance,
                 std::vector<FxRayHit>& out_hits) const;   // every hit, nearest first
```

`direction` need not be unit length; `max_distance` and `FxRayHit::distance` are always in world
units either way.

```cpp
struct FxRayHit {
    std::shared_ptr<FxEntity> entity;  // what was struck
    FxVec2f point;                     // where, in world coordinates
    FxVec2f normal;                    // surface normal, always facing back along the ray
    float distance;                    // travel from the origin
    bool hit() const;
};
```

A ray that starts **inside** a body reports that body at distance 0, with the normal pointing
back at the ray. That makes click-picking work whether the cursor is over a body or beside it.

Line of sight between two points:

```cpp
const FxVec2f to_target = target - eye;
FxRayHit hit;
const bool blocked = scene.raycast(eye, to_target, to_target.norm(), hit) &&
                     hit.entity->get_name() != "target";
```

A lidar fan, which is what an RL observation usually wants:

```cpp
std::vector<float> ranges;
for (int i = 0; i < 32; ++i) {
    const float angle = agent->pose.theta() + (static_cast<float>(i) / 32.0f) * 2.0f * FxPif;
    const FxVec2f dir{std::cos(angle), std::sin(angle)};
    FxRayHit hit;
    ranges.push_back(scene.raycast(agent->pose.xy(), dir, kMaxRange, hit) ? hit.distance
                                                                         : kMaxRange);
}
```

Rays are tested against the shape boundary directly rather than routed through the contact
solver, because a ray is a zero-thickness segment and the narrow phase deliberately refuses
those — they carry no volume to resolve. Every shape is vertices plus a skin radius, so the
boundary is the vertex core offset outward by the skin: each edge becomes a pushed-out segment
and each vertex an arc. Testing both and keeping the nearest handles circles, capsules, edges,
polygons and rounded polygons through one path.

## Overlap queries

```cpp
void overlap_circle(const FxVec2f& centre, float radius, std::vector<...>& out) const;
void overlap_box(const FxVec2f& centre, const FxVec2f& extents, std::vector<...>& out) const;
void overlap_point(const FxVec2f& point, std::vector<...>& out) const;
void overlap_shape(const FxShape& shape, const FxVec3f& pose, std::vector<...>& out) const;
std::shared_ptr<FxEntity> entity_at_point(const FxVec2f& point) const;
```

`overlap_shape` takes any `FxShape` at any pose, including rotation, so a swept capsule or a
rounded polygon works as a query volume just as it does as a collider.

An explosion:

```cpp
std::vector<std::shared_ptr<FxEntity>> caught;
scene.overlap_circle(blast_centre, blast_radius, caught);
for (const auto& e : caught) {
    FxVec2f away = e->pose.xy() - blast_centre;
    const float d = away.norm();
    if (d > 1e-4f) e->apply_impulse(away / d * (blast_impulse / std::max(d, 1.0f)));
}
```

Picking, which pairs naturally with [mouse input](input.md) since the cursor is already reported
in scene units:

```cpp
if (scene.input().mouse_pressed(FxMouseButton::Left)) {
    if (auto picked = scene.entity_at_point(scene.input().mouse_position())) {
        // drag it, select it, delete it
    }
}
```

Full containment counts as overlap. The narrow phase alone reports nothing when one shape lies
wholly inside another — there is no penetration axis to separate them along, and coincident
centres leave it no normal to work with. That is correct for solving contacts and wrong for a
query, so overlap adds a containment check on top.

## Performance

Queries scan the entity list with a cheap bounding-circle rejection rather than descending the
broad-phase AABB tree. That is deliberate: the tree is only synced during `step()`, so querying
it between steps — or before the first one — would answer from stale boxes. The rejection is
derived from the shape itself, so a body moved by hand a moment ago is still found correctly.

For a few queries per frame this is not worth optimising. If query volume ever justifies it, the
fix is to sync the tree on demand rather than to trust a stale one.

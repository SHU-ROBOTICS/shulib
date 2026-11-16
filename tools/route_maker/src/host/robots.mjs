// robots.mjs — WHERE the robots' descriptions come from (RMP1 brief A.6). Today: the descriptions BAKED into
// this page when it was built (tools/route_maker/build.py, from `shulib_route_maker describe` on the same
// commit as the library) — so the page knows the robots as of its last build (landmine 18), and says which
// commit that was. Later: a helper's describe of the team's working tree.

/** The baked data: {build: {commit, date, api}, robots: {one, two}, sharedRoute: {present, robots}}. */
export function bakedRobots(doc = typeof document !== 'undefined' ? document : null) {
    const el = doc && doc.getElementById('shulib-robots');
    if (!el) {
        return null;
    }
    try {
        return JSON.parse(el.textContent);
    } catch {
        return null;
    }
}

/**
 * The robots a route's checks run on (R13): its own robot, and the robot it is mirrored for when that robot's
 * build compiles the shared skills-route file (§6.3; true once robot two's update has landed — the baked
 * `sharedRoute.robots` says).
 */
export function routeRobots(baked, route) {
    const out = [];
    const own = baked.robots[route.robot];
    if (own) {
        out.push(own);
    }
    const shared = baked.sharedRoute && baked.sharedRoute.robots ? baked.sharedRoute.robots : [];
    if (route.mirroredFor && baked.robots[route.mirroredFor] && shared.includes(route.mirroredFor)) {
        out.push(baked.robots[route.mirroredFor]);
    }
    return out;
}

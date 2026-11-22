"""Gocool mech bot: survival first, opportunistic kills.

Layers (highest priority first):
  1. zone discipline (be inside the next circle early, never outside late)
  2. defence (face threats, strafe, dodge lasers/shells/bullets, retreat to break line of sight and heal)
  3. combat on our terms (lead gun shots, lightning up close, laser at range, mortar lobs)
  4. growth (weapons, cubes, crates, NPC drones) when nothing threatens us
  5. hide: sit near a wall inside the safe circle, silent, scanning
"""
import heapq
import math
import os
import random
import sys
import time

import numpy as np

import mechfight as mf

HERE = os.path.dirname(os.path.abspath(__file__))
TAU = 2 * math.pi


def log(*a):
    print(*a, file=sys.stderr, flush=True)


def wrap(a):
    return (a + math.pi) % TAU - math.pi


def d2(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


# --------------------------------------------------------------------------- map
class World:
    def __init__(self):
        d = np.load(os.path.join(HERE, "navmap.npz"))
        self.x0, self.y0, nx, ny = d["meta"]
        self.nx, self.ny = int(nx), int(ny)
        self.terr = d["terrain"]
        self.lo = d["lo"].astype(np.float32)
        self.hi = d["hi"].astype(np.float32)
        self.walk = d["walk2"]
        self.surf = d["surf2"]
        self.cover = d["cover2"].astype(np.float32)
        self.near = d["near2"]
        self.H, self.W = self.walk.shape
        self.walk_l = self.walk.ravel().tolist()
        self.roof2 = d["roof2"] if "roof2" in d else np.full(self.walk.shape, -1, np.int16)
        self.roofs = d["roofs"] if "roofs" in d else np.zeros((0, 6), np.float32)
        self.climb = d["climb"] if "climb" in d else np.zeros((0, 3), np.float32)
        self.surf_l = self.surf.ravel().tolist()

    # 2 m nav grid
    def cell(self, x, y):
        i = int(round((x - self.x0) / 2.0))
        j = int(round((y - self.y0) / 2.0))
        return min(max(i, 0), self.W - 1), min(max(j, 0), self.H - 1)

    def cpos(self, i, j):
        return self.x0 + 2.0 * i, self.y0 + 2.0 * j

    def walkable(self, x, y):
        i, j = self.cell(x, y)
        return self.walk_l[j * self.W + i]

    def roof_at(self, x, y):
        i, j = self.cell(x, y)
        return int(self.roof2[j, i])

    def nearest_walk(self, i, j):
        return int(self.near[1, j, i]), int(self.near[0, j, i])

    def ground(self, x, y):
        i = min(max(int(round(x - self.x0)), 0), self.nx - 1)
        j = min(max(int(round(y - self.y0)), 0), self.ny - 1)
        return float(self.terr[j, i])

    def los(self, a, b, step=1.5):
        """True if the straight segment a->b (3D) is not blocked by solids or terrain."""
        dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        L = math.sqrt(dx * dx + dy * dy + dz * dz)
        n = int(L / step)
        if n < 2:
            return True
        t = np.linspace(0.0, 1.0, n + 1)[1:-1]
        x = a[0] + dx * t
        y = a[1] + dy * t
        z = a[2] + dz * t
        i = np.clip(np.rint(x - self.x0).astype(np.int32), 0, self.nx - 1)
        j = np.clip(np.rint(y - self.y0).astype(np.int32), 0, self.ny - 1)
        blocked = (z < self.terr[j, i] - 0.3) | ((z >= self.lo[j, i]) & (z <= self.hi[j, i]))
        return not bool(blocked.any())

    def clear_line(self, a, b):
        """Walkable straight line on the nav grid between 2D points."""
        L = d2(a, b)
        n = max(2, int(L / 1.5))
        for k in range(1, n + 1):
            t = k / n
            if not self.walkable(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t):
                return False
        return True


NB = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
      (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]


class Planner:
    """Reverse A* from a goal; settled g values form a distance-to-goal field we can follow from anywhere
    settled, and the search resumes towards wherever we are now."""

    def __init__(self, world):
        self.w = world
        self.goal = None
        self.g = {}
        self.closed = set()
        self.open = []
        self.target = None

    def set_goal(self, gi, gj):
        w = self.w
        if not w.walk_l[gj * w.W + gi]:
            gi, gj = w.nearest_walk(gi, gj)
        k = gj * w.W + gi
        if self.goal == k:
            return
        self.goal = k
        self.g = {k: 0.0}
        self.closed = set()
        self.open = [(0.0, 0.0, k)]
        self.target = None

    def _h(self, k, tk):
        W = self.w.W
        dx = abs(k % W - tk % W)
        dy = abs(k // W - tk // W)
        return (max(dx, dy) + 0.4142 * min(dx, dy))

    def search(self, si, sj, budget_s):
        w = self.w
        if self.goal is None:
            return False
        if not w.walk_l[sj * w.W + si]:
            si, sj = w.nearest_walk(si, sj)
        sk = sj * w.W + si
        if sk in self.closed:
            return True
        if self.target != sk:
            self.target = sk
            g = self.g
            self.open = [(g[k] + self._h(k, sk), g[k], k) for (_, _, k) in self.open if k not in self.closed]
            heapq.heapify(self.open)
        t_end = time.perf_counter() + budget_s
        W, H = w.W, w.H
        walk, surf = w.walk_l, w.surf_l
        g, closed, op = self.g, self.closed, self.open
        n = 0
        while op:
            n += 1
            if (n & 127) == 0 and time.perf_counter() > t_end:
                return False
            f, gk, k = heapq.heappop(op)
            if k in closed:
                continue
            closed.add(k)
            if k == sk:
                return True
            i, j = k % W, k // W
            hk = surf[k]
            for di, dj, c in NB:
                ii, jj = i + di, j + dj
                if ii < 0 or jj < 0 or ii >= W or jj >= H:
                    continue
                kk = jj * W + ii
                if not walk[kk] or kk in closed:
                    continue
                if di and dj and not (walk[j * W + ii] and walk[jj * W + i]):
                    continue
                if abs(surf[kk] - hk) > 2.3:
                    continue
                ng = gk + c
                if ng < g.get(kk, 1e18):
                    g[kk] = ng
                    heapq.heappush(op, (ng + self._h(kk, sk), ng, kk))
        return False

    def waypoint(self, x, y, look=8):
        """Follow the descent chain from our cell; return a point to steer at (or None)."""
        w = self.w
        i, j = w.cell(x, y)
        if not w.walk_l[j * w.W + i]:
            i, j = w.nearest_walk(i, j)
            return w.cpos(i, j)
        k = j * w.W + i
        if k not in self.closed:
            return None
        g = self.g
        W = w.W
        chain = []
        for _ in range(look):
            if g.get(k, 1e18) <= 0.0:
                break
            ci, cj = k % W, k // W
            best, bk = g[k], None
            for di, dj, c in NB:
                kk = (cj + dj) * W + (ci + di)
                gv = g.get(kk)
                if gv is not None and kk in self.closed and gv < best:
                    best, bk = gv, kk
            if bk is None:
                break
            k = bk
            chain.append(k)
        if not chain:
            return w.cpos(self.goal % W, self.goal // W)
        # furthest chain point reachable in a straight walkable line
        for k in reversed(chain):
            p = w.cpos(k % W, k // W)
            if w.clear_line((x, y), p):
                return p
        k = chain[0]
        return w.cpos(k % W, k // W)


# --------------------------------------------------------------------------- bot
# Tunable tactics (one experiment = one change here)
P = {
    "engage_r": 240.0,      # start fighting visible mechs within this range
    "passive": False,       # if True: only fight when close (< close_r) or recently shot at
    "close_r": 100.0,
    "strafe_min": 3.0, "strafe_max": 5.0,
    "want_gun": 80.0, "want_laser": 100.0, "want_lightning": 28.0,
    "retreat_hp": 0.22, "retreat_hp_multi": 0.38,
    "loot_r": 110.0, "npc_hp": 0.45, "crates": True,
    "laser_first": True,
    "lightning_top": False,     # rank lightning above laser
    "reload_cover": False,      # step out of sight while the gun reloads
    "hide_shuffle": False,      # never stand perfectly still while hiding
    "pacifist": False,          # diagnostic: never fight, only hide/evade
    "stealth": False,            # survive to the final burn: stay unseen, silent, fight only when cornered
    "stealth_fight_r": 50.0,
    "power_pos": False,          # midgame: hold a rooftop inside the next zone, shoot from its edge
    "pp_start": 150.0, "pp_end": 330.0, "pp_climb_s": 7.0,
    "assassin": True,
    "mode_detect": True,        # deaths we did not cause early -> real free-for-all -> survival mode
    "backstab_laser_first": True,
    "smart_targets": True,      # prefer wounded / facing-away / exposed / laser-charging targets, avoid escorted ones
    "gang_escape": False,        # v19: hit by >= gang_n different mechs within gang_window s -> retreat gang_hold s
    "gang_n": 2, "gang_window": 3.0, "gang_hold": 4.0,
    "converge_n": 3,            # this many enemies with eyes on us -> break line of sight first
    "recover_hp": 0.6,          # after a retreat, only re-engage above this HP (or for an easy finish)
    "endgame_t": 330.0,
    "commit": True,            # v13-B: stick to one target until dead, prefer isolated ones, burst->survival on heat
    "heat_max": 2,
    "ghost_max_age": 12.0,      # v13-C sets 5.5 (only stalk while the estimate is trustworthy)
    "roof_safe": False,         # v13-C: opportunistic rooftop with safe exits (retreat/convergence always win)
    "standoff": True,           # v13-A: laser (hitscan) vs gun-only bots at ~180 m, where their bullets need 0.7 s
    "standoff_r": 180.0,
    "laser_min_r": 0.0,         # v17 (lab flag; 75 to enable): keep at least this far from laser carriers (0 = off)
    "laser_min_finish": 0.15,   # ...unless the carrier is below this HP fraction (finish it)
    "laser_cd_window": 1.3,     # ...or it fired its laser within this many seconds (cooldown 1.65 s)
    "powerup_until": 100.0,     # opening: cubes first (crates, drones), only fight bots that are close or shooting us
    "powerup_r": 150.0,         # final-burn prep: only close / finishing / forced fights after this           # roam to find house bots early and prefer ones facing away (rear hits x1.5)
    "roam_until": 300.0, "roam_hp": 0.6,
    "aggressive": True,
    "hunt_hidden": True,
    "hold_angles": False,
    "ghost_hunt": True,         # keep stalking enemies for 12 s after the 5 s memory ends        # hidden target pushing us -> hold still behind cover with a charged laser
    "edge_hide": False,          # hide with back to a wall near the zone edge        # peek for a firing line, pre-charge laser, hunt toward sounds         # constant motion + boost jinks, chase wounded, retreat late
    "cube_r": 110.0, "crate_r": 110.0, "npc_r": 160.0, "npc_stand": 62.0,
    "laser_hunt_r": 320.0, "open_scan_s": 3.0,
    "drops": True, "drop_r": 200.0, "heal_to": 0.6,
    "drop_heal": True,          # v15: supply drop = full heal + 3 cubes; go for it mid-fight, keep moving while capturing
}
P.update({"press_hp": 0.45, "press_gap": 0.25, "press_r": 45.0, "forget_dead": False, "farm_hp_gate": False, "drone_avoid": False, "drone_avoid_hp": 0.8,
          "press_multi_block": False, "fight_boost": True, "retreat_hidden_w": 60.0, "gang_fire": True})
# v22 INFORMED BERSERK: keep the caution that real scores proved (survival switch @2, pack skip,
# break LOS @3 watchers); drop untested caution that keeps us out of fights; add lab bug fixes.
P.update({"laser_grab": True, "laser_grab_r": 90.0,
          "recover_hp": 0.0, "endgame_t": 9999.0, "retreat_hp_multi": 0.25,
          "drone_avoid": True, "forget_dead": True, "farm_hp_gate": True})
# v35 knobs (default off = v33b behaviour)
P.update({"quiet": False, "quiet_scale": 0.4,      # move silently (< 5 m/s, no boost) when not fighting/retreating/zone
          "heal_first": False,                     # heal (hide + regen) before roaming/hunting/looting
          "sit_heal": False,                       # while healing, stand still once at the hide spot (already) / or wherever we are
          "cube_fight_r": 0.0,                     # grab visible cubes within this range even while a target exists
          "cube_fight_hp": 0.35,
          "upg": "base",                           # "armor": plating, armor, laser_damage, laser_charge, nanites, gun...
          "lose_rule": False,                      # retreat when 2 near & hp<.55, 3 near & hp<.8, or losing the trade
          "lethal_laser": False,                   # release a partial laser charge when it kills
          "fight_expo": 0.0,
          "cover_retreat": False,                  # v38: retreat = go to nearest spot hidden from threats and hold; none -> fight
          "cover_max": 60.0, "cornered_fight": True,
          "avoid_r": 0.0,                          # v38 (Xero idea): when not fighting, steer away from known mechs within this
          "avoid_w": 1.2, "avoid_loot": 60.0,
          "stick": 0.0,                            # v38: score bonus for the current target (stop target thrashing)
          "stick_hurt": 0.0,                       # extra bonus per missing HP fraction of the target
          "solo_duel": False, "solo_hp": 0.25,
          "econ": False,                           # v38: remember crates/drones/cubes, walk to best value/(d+18)
          "econ_r": 260.0, "econ_npc_hp": 0.6, "econ_boost": False, "econ_give_up": 30.0,
          "econ_v_cube": 110.0, "econ_v_crate": 105.0, "econ_v_npc": 170.0,
          "threat_pri": 0.0,                       # bonus for mechs actively hunting us (hit us / aimed+closing)
          "pace_s": 0.0,                           # after a kill: no new non-threat targets for this long or until healed
          "pace_hp": 0.9,
          "reactive": False,                       # only engage mechs that are close (<60), hit us, close in, are hurt, or aim at us
          "react_r": 60.0,
          "regen": False, "regen_hp": 0.8, "regen_to": 0.95, "regen_fin": 0.3,   # break LOS, hold fire, regen to full
          "fight_spot": False, "spot_expo": 30.0, "spot_los": 45.0, "spot_duck": True,  # v37 fight core: score nearby spots
          "idle_still": False,                     # nothing to do: stand still where we are (giddy waits ~50% of its life)
          "idle_still_hp": 0.0,                    # ...only while hp >= this (0 = always)                       # fight spot choice: penalty per other enemy that can see the spot
          })
# v36 defaults: best sim knobs on top of v33b (giddy-style retreat 38% -> recover 80%, lethal laser release, armor-first upgrades)
P.update({"retreat_hp": 0.38, "recover_hp": 0.8, "lethal_laser": True, "upg": "armor"})
# v38 probe: loot memory + grab cubes in fights + hunters first (sim +0.14 vs v36)
P.update({"econ": True, "cube_fight_r": 70.0, "cube_fight_hp": 0.3, "threat_pri": 90.0})
# v41 (random search r14, sim +0.61 on 150 seeds vs v38)
P.update({"retreat_hp_multi": 0.5, "lose_rule": False, "fight_expo": 30.0, "strafe_min": 3.0, "strafe_max": 5.0, "want_gun": 100.0, "engage_r": 300.0, "smart_targets": False, "commit": False, "laser_grab_r": 60.0})
# v42: top-team real-proven ideas on top of v41 (4.60): Xero solo duel (real +0.19), heal to 80% after a retreat
# (sim +0.01 on r14), r41 extras (sim +0.57 search): retreat when hit at 45%, avoid idle mechs within 80 m, laser 130 m
P.update({"solo_duel": True, "solo_hp": 0.25, "recover_hp": 0.8, "retreat_hp": 0.45, "avoid_r": 80.0, "want_laser": 130.0})

LABLOG = bool(os.environ.get("GOCOOL_LOG"))
try:   # lab only: variant overrides from the environment (never set in the match container)
    import json as _json
    P.update(_json.loads(os.environ.get("GOCOOL_P", "{}") or "{}"))
except Exception:
    pass

UPGRADE_PRIORITY = ["plating", "gun_mag", "gun_damage", "laser_damage", "nanites", "armor", "laser_charge", "gun_rate",
                    "lightning_efficiency", "servos", "gun_velocity", "sensors", "lightning_range", "gyros",
                    "lightning_chain", "reactor", "capacitor", "jets", "mortar_reload", "mortar_velocity",
                    "mortar_splash", "laser_pierce"]  # v27: gun is our main damage (always held); mag = +31% sustained DPS
UPG_ARMOR = ["plating", "armor", "laser_damage", "laser_charge", "nanites", "gun_damage", "gun_mag", "gun_rate",
             "lightning_efficiency", "lightning_range", "servos", "gun_velocity", "gyros", "lightning_chain", "reactor",
             "capacitor", "sensors", "jets", "mortar_reload", "mortar_velocity", "mortar_splash", "laser_pierce"]
# v45: upgrade order with repair nanites first (+60% out-of-combat regen, starts 1.5 s sooner; Marwan's practice
# valuation ranks it top) then plating / armor / laser
UPG_ARMOR[:] = ["nanites", "plating", "armor", "laser_damage", "laser_charge"] + [u for u in UPG_ARMOR if u not in
              ("nanites", "plating", "armor", "laser_damage", "laser_charge")]
WEAPON_RANK = {"lightning": 3.0, "laser": 2.6, "mortar": 1.0, "gun": 0.0}
if P["laser_first"]:
    WEAPON_RANK["laser"] = 4.0
    WEAPON_RANK["lightning"] = 2.5
if P.get("lightning_top"):
    WEAPON_RANK["lightning"] = 3.5


class Bot:
    name = "gocool"

    def __init__(self, hello):
        self.cfg = hello["config"]
        random.seed(hello.get("seed", 1))
        self.w = World()
        self.zone_plan = Planner(self.w)
        self.local_plan = Planner(self.w)
        self.local_goal = None
        self.strafe_dir = 1
        self.strafe_until = 0.0
        self.last_pos = []
        self.stuck_until = 0.0
        self.unstick_dir = None
        self.threat_bearing = None
        self.threat_t = -99.0
        self.last_damage_t = -99.0
        self.hits_by = []
        self.hit_from = {}
        self.regen_on = False         # v19: (t, attacker id) for recent damage
        self.gang_until = -99.0
        self.cur_threats = []
        self.cur_enemies = []
        self.cur_cubes = []
        self.scan_phase = random.random() * TAU
        self.hide_goal = None
        self.hide_key = None
        self.mode = "start"
        self.shorted_until = 0.0
        self.t = 0.0
        self.enemy_seen_t = -99.0
        self.ignore_crates = {}
        self.ok = True
        self.known_pk = {}   # pickup id -> [kind, x, y, z, last_seen_t]
        self.scan_start = None
        self.peek = None          # (x, y, target_id, until_t)
        self.heard = None         # (bearing, dist_m, t)
        self.tracks = {}          # enemy id -> dict(pos, vel, t, hp, max_hp, weapons)
        self.laser_fired = {}     # v17: enemy id -> time we last saw its laser fire (beam or charge ending)
        self.was_charging = {}
        self.pp = None            # power position: dict(roof, state, t0, climb)
        self.my_kills = 0
        self.cover_t, self.cover_spot_ = -99.0, None
        self.mem_crates, self.mem_cubes, self.mem_npcs = {}, {}, {}
        self.loot_id, self.loot_since, self.loot_bad = None, 0.0, {}
        self.last_kill_t = -99.0
        self.hit_ids = set()      # mechs we have already damaged (no longer unaware)
        self.recovering = False
        self.cur_tid = None
        self.seen_at = {}         # enemy id -> last time seen (for "heat": how many bots are on us)
        self.survival = False
        self.roof_since = None
        self.ffa = False          # True once we see deaths we did not cause (real free-for-all)
        self.roof_black = {}      # roof id -> blacklisted until t
        self.roof_peek = None

    # ------------------------------------------------------------------ helpers
    def zone_goal(self, obs):
        z = obs["zone"]
        me = obs["self"]["pos"]
        t = obs["t"]
        if t >= z["shrink_end"] - 0.01 and z["next_radius"] >= z["radius"] - 0.5:
            cx, cy = z["center"]
            r = z["radius"]
        else:
            cx, cy = z["next_center"]
            r = z["next_radius"]
        if r <= 30:
            margin_r = max(0.0, r * 0.35)
        elif r <= 80:
            margin_r = r * 0.5
        else:
            margin_r = max(r * 0.55, r - 60.0)
        dx, dy = me[0] - cx, me[1] - cy
        dist = math.hypot(dx, dy)
        if dist <= margin_r:
            return None, (cx, cy, r, margin_r)
        s = margin_r / max(dist, 1e-6)
        return (cx + dx * s, cy + dy * s), (cx, cy, r, margin_r)

    def pick_hide(self, obs, circle):
        """A walkable cell near a tall wall, inside the safe circle, near where we are."""
        cx, cy, r, mr = circle
        me = obs["self"]["pos"]
        key = (round(cx), round(cy), round(r))
        if self.hide_key == key and self.hide_goal is not None:
            return self.hide_goal
        w = self.w
        # search a window around the closest safe point
        dx, dy = me[0] - cx, me[1] - cy
        dist = math.hypot(dx, dy)
        s = min(1.0, mr / max(dist, 1e-6))
        px, py = cx + dx * s, cy + dy * s
        rad = 40.0
        i0, j0 = w.cell(px - rad, py - rad)
        i1, j1 = w.cell(px + rad, py + rad)
        sub_w = w.walk[j0:j1 + 1, i0:i1 + 1]
        sub_c = w.cover[j0:j1 + 1, i0:i1 + 1]
        if sub_w.size == 0 or not sub_w.any():
            self.hide_goal, self.hide_key = (px, py), key
            return self.hide_goal
        jj, ii = np.mgrid[j0:j1 + 1, i0:i1 + 1]
        X = w.x0 + 2.0 * ii
        Y = w.y0 + 2.0 * jj
        inside = np.hypot(X - cx, Y - cy) <= mr
        ok = sub_w & inside
        if not ok.any():
            self.hide_goal, self.hide_key = (px, py), key
            return self.hide_goal
        score = np.hypot(X - px, Y - py) * 0.25 + np.clip(sub_c - 3.5, 0, 30) * 1.5
        if P.get("edge_hide"):
            score = score + np.abs(np.hypot(X - cx, Y - cy) - mr * 0.85) * 0.15
        score = np.where(ok, score, 1e9)
        k = int(np.argmin(score))
        self.hide_goal = (float(X.ravel()[k]), float(Y.ravel()[k]))
        self.hide_key = key
        return self.hide_goal

    def steer(self, me, desired, avoid_pts=()):
        """Pick a walkable heading close to the desired (unnormalised) vector."""
        x, y = me["pos"][0], me["pos"][1]
        mag = math.hypot(desired[0], desired[1])
        if mag < 1e-6:
            return [0.0, 0.0]
        dx, dy = desired[0] / mag, desired[1] / mag
        w = self.w
        here_ok = w.walkable(x, y)
        best, bh = -9.0, (dx, dy)
        base = math.atan2(dy, dx)
        for k in range(16):
            a = base + (k if k <= 8 else k - 16) * (TAU / 16)
            hx, hy = math.cos(a), math.sin(a)
            sc = hx * dx + hy * dy
            if here_ok:
                if not w.walkable(x + hx * 3.0, y + hy * 3.0):
                    sc -= 3.0
                elif not w.walkable(x + hx * 6.0, y + hy * 6.0):
                    sc -= 0.6
            if sc > best:
                best, bh = sc, (hx, hy)
        m = min(1.0, mag)
        return [bh[0] * m, bh[1] * m]

    def go_to(self, me, planner, goal, t):
        """Desired move vector towards goal using the planner field (straight if not ready)."""
        x, y = me["pos"][0], me["pos"][1]
        gi, gj = self.w.cell(goal[0], goal[1])
        planner.set_goal(gi, gj)
        si, sj = self.w.cell(x, y)
        planner.search(si, sj, 0.012)
        wp = planner.waypoint(x, y)
        if wp is None:
            wp = goal
        vx, vy = wp[0] - x, wp[1] - y
        n = math.hypot(vx, vy)
        if n < 1e-6:
            return [0.0, 0.0]
        return [vx / n, vy / n]

    # ------------------------------------------------------------------ aiming
    def lead(self, me, tgt, speed, scale=1.0):
        mz = me["pos"][2] + me["stats"].get("muzzle_height", 7.0)
        mx, my = me["pos"][0], me["pos"][1]
        tsize = 1.0 + 0.05 * tgt.get("cubes", 0) if tgt.get("kind") == "mech" else 0.39
        tx, ty, tz = tgt["pos"][0], tgt["pos"][1], tgt["pos"][2] + 4.5 * tsize
        vx, vy, vz = tgt["vel"]
        if abs(vz) < 1.0:
            vz = 0.0
        t = 0.0
        for _ in range(3):
            px, py, pz = tx + vx * t, ty + vy * t, tz + vz * t * 0.5
            dist = math.sqrt((px - mx) ** 2 + (py - my) ** 2 + (pz - mz) ** 2)
            t = dist / speed if speed > 0 else 0.0
        px, py, pz = tx + vx * t, ty + vy * t, tz + vz * t * 0.5
        hd = math.hypot(px - mx, py - my)
        return math.atan2(py - my, px - mx), math.atan2(pz - mz, max(hd, 0.1)), hd, (px, py, pz)

    def mortar_aim(self, me, tgt, w):
        v = w["stats"].get("speed", 72.0)
        g = self.cfg["weapons"]["mortar"].get("gravity", 22.0)
        mx, my = me["pos"][0], me["pos"][1]
        mz = me["pos"][2] + me["stats"].get("height", 9.0) + 0.5
        tx, ty, tz = tgt["pos"][0], tgt["pos"][1], tgt["pos"][2] + 1.0
        vx, vy, _ = tgt["vel"]
        T = 1.0
        for _ in range(3):
            px, py = tx + vx * T * 0.6, ty + vy * T * 0.6
            D = math.hypot(px - mx, py - my)
            dz = tz - mz
            disc = v ** 4 - g * (g * D * D + 2 * dz * v * v)
            if disc < 0:
                return None
            th = math.atan((v * v - math.sqrt(disc)) / (g * max(D, 1.0)))
            T = D / max(v * math.cos(th), 1.0)
        return math.atan2(py - my, px - mx), th, D

    # ------------------------------------------------------------------ main
    def act(self, obs):
        t0 = time.perf_counter()
        try:
            act = self._act(obs)
        except Exception as e:  # never stand still: fall back to walking at the zone
            import traceback
            log("act error", obs.get("tick"), traceback.format_exc())
            me = obs["self"]
            z = obs["zone"]
            c = z["next_center"]
            yaw = math.atan2(c[1] - me["pos"][1], c[0] - me["pos"][0])
            act = {"move": [math.cos(yaw), math.sin(yaw)], "aim": [yaw, 0.0], "boost": not me["in_zone"]}
        dt = (time.perf_counter() - t0) * 1000
        if dt > 30:
            log(f"slow tick {obs['tick']}: {dt:.1f} ms")
        return act

    def _act(self, obs):
        t = obs["t"]
        self.t = t
        me = obs["self"]
        pos = me["pos"]
        x, y, z = pos
        w = self.w
        act = {}
        hp_frac = me["hp"] / max(me["max_hp"], 1.0)
        eye = (x, y, z + me["stats"].get("eye_height", 8.0))
        muzzle = (x, y, z + me["stats"].get("muzzle_height", 7.0))

        # ---- offers
        offer = obs.get("offer")
        if offer:
            ch = offer["choices"]
            best, bi = 999, 0
            for i, c in enumerate(ch):
                UP = UPG_ARMOR if P["upg"] == "armor" else UPGRADE_PRIORITY
                r = UP.index(c["id"]) if c["id"] in UP else 50
                if r < best:
                    best, bi = r, i
            act["pick"] = bi

        # ---- events
        for ev in obs.get("events", []):
            et = ev.get("type")
            if et == "damaged":
                if ev.get("weapon") != "zone":
                    self.last_damage_t = t
                    if ev.get("by") is not None and ev["by"] < 100:   # mechs only, not drones
                        self.hits_by.append((t, ev["by"]))
                        self.hit_from[ev["by"]] = t
                    if ev.get("bearing") is not None:
                        self.threat_bearing = ev["bearing"]
                        self.threat_t = t
            elif et == "hit" and ev.get("target") is not None:
                self.hit_ids.add(ev["target"])
            elif et == "kill":
                self.my_kills += 1
                self.last_kill_t = t
                if P["forget_dead"]:
                    kid = ev.get("target")          # v16: forget the corpse so we don't stalk/charge at it
                    self.tracks.pop(kid, None)
                    if self.cur_tid == kid:
                        self.cur_tid = None
                    self.peek = None
            elif et == "effect" and ev.get("effect") == "short":
                self.shorted_until = t + 2.0
            elif et == "take_failed":
                pass

        if P["mode_detect"] and not self.ffa:
            others_dead = (16 - obs.get("alive", 16)) - self.my_kills
            if others_dead >= 2 and t < 150.0:
                self.ffa = True
                log(f"mode: free-for-all detected at t={t:.1f} (others dead {others_dead})")
                # survival profile for a real FFA: stay unseen, fight only when cornered, no roaming
                P["stealth"] = True
                P["assassin"] = False
                P["ghost_hunt"] = False
        for snd in obs.get("sounds", []):
            if snd["kind"] in ("gun", "laser", "laser_charge", "lightning", "step", "boost", "reload", "jet", "mortar") and snd["dist"] in ("near", "mid"):
                dm = 35.0 if snd["dist"] == "near" else 105.0
                if self.heard is None or dm <= self.heard[1] or t - self.heard[2] > 2.0:
                    self.heard = (snd["bearing"], dm, t)
        enemies = [m for m in obs["mechs"] if m["kind"] == "mech" and m["id"] != me["id"]]
        npcs = [m for m in obs["mechs"] if m["kind"] == "npc"]
        for e in enemies:
            e["_d"] = d2(e["pos"], pos)
        enemies.sort(key=lambda e: e["_d"])
        self.cur_enemies = enemies
        self.cur_cubes = obs.get("cubes", []) or []
        for e in enemies:
            self.tracks[e["id"]] = {"pos": list(e["pos"]), "vel": list(e["vel"]), "t": t, "hp": e.get("hp", 1400),
                                    "max_hp": e.get("max_hp", 1400), "weapons": e.get("weapons", []), "cubes": e.get("cubes", 0)}
        cur_ids = {e["id"] for e in enemies}
        for e in enemies:
            self.seen_at[e["id"]] = t
        # v17: track enemy laser cooldowns (beam seen, or a charge we watched ended)
        for bm in obs.get("beams", []) or []:
            if bm.get("kind") == "laser" and bm.get("by") is not None:
                self.laser_fired[bm["by"]] = t - (bm.get("age", 0) or 0)
        for e in enemies:
            ch = bool(e.get("charging"))
            if self.was_charging.get(e["id"]) and not ch:
                self.laser_fired[e["id"]] = max(self.laser_fired.get(e["id"], -99.0), t - 0.1)
            self.was_charging[e["id"]] = ch
        heat = sum(1 for v in self.seen_at.values() if t - v < 20.0)
        if P["commit"]:
            hpf_now = me["hp"] / max(me["max_hp"], 1.0)
            if not self.survival and (heat >= P["heat_max"] and hpf_now < 0.75):
                self.survival = True
            elif self.survival and heat <= 1 and hpf_now > 0.85 and t < P["endgame_t"]:
                self.survival = False
        ghost = None
        for gid, tr in list(self.tracks.items()):
            age = t - tr["t"]
            if age > P["ghost_max_age"]:
                del self.tracks[gid]
                continue
            if gid in cur_ids or age < 0.2:
                continue
            k = min(age, 3.0) if P["ghost_max_age"] > 6.0 else min(age, 1.5) * 0.7
            gp = [tr["pos"][0] + tr["vel"][0] * k, tr["pos"][1] + tr["vel"][1] * k, tr["pos"][2]]
            dd = d2(gp, pos)
            if dd < 220 and (ghost is None or dd < ghost["_d"]):
                ghost = {"id": gid, "kind": "mech", "pos": gp, "vel": [0.0, 0.0, 0.0], "hp": tr["hp"],
                         "max_hp": tr["max_hp"], "weapons": tr["weapons"], "cubes": tr["cubes"], "_d": dd, "_los": False}
        if enemies:
            self.enemy_seen_t = t

        # ---- weapons
        weps = me["weapons"]
        slots = {}
        for i, wp in enumerate(weps):
            if wp:
                slots[wp["kind"]] = i

        # ---- target selection
        target = None
        best = 1e18
        recently_hit = (t - self.last_damage_t) < 3.0
        for e in enemies:
            if P["pacifist"]:
                break
            if P["stealth"] and e["_d"] > P["stealth_fight_r"] and not recently_hit:
                continue
            if t < P["powerup_until"] and e["_d"] > 120 and not recently_hit:
                continue
            if e["_d"] > P["engage_r"]:
                continue
            if P["passive"] and e["_d"] > P["close_r"] and not recently_hit:
                continue
            if P["reactive"]:
                b_e2me_ = math.atan2(y - e["pos"][1], x - e["pos"][0])
                aimed_ = abs(wrap(e["aim"][0] - b_e2me_)) < 0.22
                rx__, ry__ = x - e["pos"][0], y - e["pos"][1]
                closing_ = (e["vel"][0] * rx__ + e["vel"][1] * ry__) / max(e["_d"], 1.0)
                ehp_ = e.get("hp", 1400) / max(e.get("max_hp", 1400), 1)
                if not (e["_d"] < P["react_r"] or t - self.hit_from.get(e["id"], -99.0) < 6.0
                        or (closing_ > 3 and e["_d"] < 180) or (ehp_ < 0.4 and e["_d"] < 160)
                        or (aimed_ and e["_d"] < 190) or e["id"] == self.cur_tid):
                    continue
            if P["threat_pri"] or P["pace_s"]:
                b_e2me_ = math.atan2(y - e["pos"][1], x - e["pos"][0])
                rx__, ry__ = x - e["pos"][0], y - e["pos"][1]
                closing_ = (e["vel"][0] * rx__ + e["vel"][1] * ry__) / max(e["_d"], 1.0)
                threat_ = (t - self.hit_from.get(e["id"], -99.0) < 6.0
                           or (abs(wrap(e["aim"][0] - b_e2me_)) < 0.22 and (closing_ > 1 or e["_d"] < 120))
                           or e["_d"] < 45)
                ehp_ = e.get("hp", 1400) / max(e.get("max_hp", 1400), 1)
                if (P["pace_s"] and not threat_ and ehp_ > 0.4 and e["id"] != self.cur_tid
                        and t - self.last_kill_t < P["pace_s"] and me["hp"] / max(me["max_hp"], 1) < P["pace_hp"]):
                    continue
            ez = e["pos"][2] + 4.5
            vis = w.los(muzzle, (e["pos"][0], e["pos"][1], ez))
            e["_los"] = vis
            # cheapest point first: weight remaining HP heavily (a wounded bot is a near-free kill)
            sc = e["_d"] * 0.5 + (0 if vis else 120) + e.get("hp", 1400) * 0.08
            if P["threat_pri"]:
                sc += -P["threat_pri"] if threat_ else P["threat_pri"] * 0.5
            if P["stick"] and e["id"] == self.cur_tid:
                sc -= P["stick"] + P["stick_hurt"] * (1.0 - e.get("hp", 1400) / max(e.get("max_hp", 1400), 1))
            if P["smart_targets"]:
                escorts = sum(1 for o in enemies if o is not e and d2(o["pos"], e["pos"]) < 50)
                sc += escorts * 40.0                      # escorted targets are expensive
                if e.get("charging"):
                    sc -= 25.0                            # half-speed while charging: exposed
                if e.get("hp", 1400) < 0.3 * e.get("max_hp", 1400):
                    sc -= 60.0                            # finish it
            thp = e.get("hp", 1400) / max(e.get("max_hp", 1400), 1)
            easy = thp < 0.35 or e["_d"] < 40 or recently_hit
            if P["smart_targets"] and self.recovering and not easy:
                continue
            if P["smart_targets"] and t >= P["endgame_t"] and not easy and e["_d"] > 60:
                continue
            if P["assassin"]:
                b_e2me = math.atan2(y - e["pos"][1], x - e["pos"][0])
                if abs(wrap(e["aim"][0] - b_e2me)) > math.pi / 2:
                    sc -= 60.0  # it is facing away: our hits land on its back
            if P["commit"]:
                escorts_b = sum(1 for o in enemies if o is not e and d2(o["pos"], e["pos"]) < 60)
                if escorts_b >= 2 and thp >= 0.3:
                    continue                              # attacking a pack invites the squad
                sc += escorts_b * 30.0
                if e["id"] == self.cur_tid:
                    sc -= 50.0 + (40.0 if thp < 0.6 else 0.0)   # finish what we started
                if self.survival and not easy:
                    continue
            if sc < best:
                best, target = sc, e
        self.cur_tid = target["id"] if target is not None else (
            self.cur_tid if self.cur_tid in self.tracks and t - self.tracks[self.cur_tid]["t"] < 5.0 else None)
        # NPC/crate farming only when no mechs around
        farm = None
        calm = (t - self.enemy_seen_t > 4.0) and (t - self.last_damage_t > 3.0)

        # ---- zone
        zgoal, circle = self.zone_goal(obs)
        zone = obs["zone"]
        time_to_shrink = zone["shrink_start"] - t
        urgent_zone = (not me["in_zone"]) or (zgoal is not None and (
            time_to_shrink < d2(zgoal, pos) / 9.0 + 8.0 or t >= zone["shrink_start"]))

        # ---- loot targets (only when calm)
        loot_goal = None
        take = None
        reach = self.cfg["loot"]["pickup_radius"]
        for p in obs.get("pickups", []):
            kind = p["kind"]
            if kind in slots:
                continue
            cur1 = weps[1]["kind"] if len(weps) > 1 and weps[1] else None
            if cur1 and WEAPON_RANK.get(kind, 0) <= WEAPON_RANK.get(cur1, 0) + 0.1:
                continue
            dd = d2(p["pos"], pos)
            dz = abs(p["pos"][2] - z)
            if dd < reach - 0.3 and dz < 4.0:
                laser_busy = cur1 == "laser" and weps[1].get("charge", 0) > 0
                if not laser_busy:
                    take = (p["id"], 1 if cur1 else None)
            elif dd < P["loot_r"] and w.walkable(p["pos"][0], p["pos"][1]) and abs(p["pos"][2] - w.ground(p["pos"][0], p["pos"][1])) < 2.0:
                if loot_goal is None or dd < loot_goal[2]:
                    loot_goal = (p["pos"][0], p["pos"][1], dd, "weapon")
        seen_ids = set()
        for p in obs.get("pickups", []):
            self.known_pk[p["id"]] = [p["kind"], p["pos"][0], p["pos"][1], p["pos"][2], t]
            seen_ids.add(p["id"])
        for pid in list(self.known_pk):
            k = self.known_pk[pid]
            # we see everything within ~40 m: if it is not listed, someone took it
            if pid not in seen_ids and (d2((k[1], k[2]), pos) < 38.0 or t - k[4] > 90.0):
                del self.known_pk[pid]
        laser_goal = None
        if "laser" not in slots:
            for pid, k in self.known_pk.items():
                if k[0] != "laser":
                    continue
                p = {"pos": [k[1], k[2], k[3]]}
                dd = d2(p["pos"], pos)
                if dd < P["laser_hunt_r"] and abs(p["pos"][2] - w.ground(p["pos"][0], p["pos"][1])) < 2.5 and w.walkable(p["pos"][0], p["pos"][1]):
                    if laser_goal is None or dd < laser_goal[2]:
                        laser_goal = (p["pos"][0], p["pos"][1], dd)
        if take:
            act["take"] = take[0]
            if take[1] is not None:
                act["drop"] = take[1]
        # ---- zone-feasibility of a goal point
        ncx, ncy = zone["next_center"]
        nr = zone["next_radius"]

        def zone_ok(px, py):
            dn = math.hypot(px - ncx, py - ncy)
            if dn <= nr * 0.85:
                return True
            # outside the next circle: only while there is plenty of time before the shrink
            return time_to_shrink > d2((px, py), pos) / 9.0 + dn / 9.0 + 15.0 and t < zone["shrink_start"]

        econ_goal = None
        if P["econ"]:
            cur_c = {c["id"]: c for c in obs.get("crates", [])}
            cur_q = {c["id"]: c for c in obs.get("cubes", [])}
            cur_n = {n["id"]: n for n in npcs}
            for mem, cur in ((self.mem_crates, cur_c), (self.mem_cubes, cur_q), (self.mem_npcs, cur_n)):
                for i_, o_ in cur.items():
                    mem[i_] = (list(o_["pos"]), t)
                for i_ in list(mem):
                    if i_ not in cur and d2(mem[i_][0], pos) < 22:
                        del mem[i_]
            for i_ in list(self.mem_npcs):
                if t - self.mem_npcs[i_][1] > 40:
                    del self.mem_npcs[i_]
            hurt_ = hp_frac <= 0.45
            mech_near_ = (t - self.enemy_seen_t) < 0.3
            best_ = None
            bu_ = 0.0
            def cons_(kind, i_, p_, v_):
                nonlocal best_, bu_
                if self.loot_bad.get(i_, -1.0) > t:
                    return
                dd_ = d2(p_, pos)
                if dd_ > P["econ_r"] or (hurt_ and dd_ > 35 and kind != "npc"):
                    return
                if not zone_ok(p_[0], p_[1]) or p_[2] - w.ground(p_[0], p_[1]) > 3.2:
                    return
                if P["avoid_r"] > 0 and kind != "cube" and any(d2(e["pos"], p_) < P["avoid_loot"] for e in enemies):
                    return
                u_ = v_ / (dd_ + 18.0) * (1.35 if i_ == self.loot_id else 1.0)
                if u_ > bu_:
                    bu_, best_ = u_, (kind, i_, p_, dd_)
            for i_, (p_, _) in self.mem_cubes.items():
                cons_("cube", i_, p_, P["econ_v_cube"])
            if P["crates"]:
                for i_, (p_, _) in self.mem_crates.items():
                    cons_("crate", i_, p_, P["econ_v_crate"])
            if hp_frac > P["econ_npc_hp"] and not mech_near_:
                for i_, (p_, ts_) in self.mem_npcs.items():
                    cons_("npc", i_, p_, P["econ_v_npc"] - (15.0 if t - ts_ > 12 else 0.0))
            if best_ is not None:
                if best_[1] != self.loot_id:
                    self.loot_id, self.loot_since = best_[1], t
                elif t - self.loot_since > P["econ_give_up"]:
                    self.loot_bad[best_[1]] = t + 60.0
                    self.loot_id = None
                    best_ = None
            else:
                self.loot_id = None
            econ_goal = best_
            if econ_goal is not None:
                econ_goal = econ_goal + ((cur_c if econ_goal[0] == "crate" else cur_n if econ_goal[0] == "npc" else cur_q).get(econ_goal[1]),)

        cube_goal = None
        for c in obs.get("cubes", []):
            dd = d2(c["pos"], pos)
            if dd < P["cube_r"] and abs(c["pos"][2] - w.ground(c["pos"][0], c["pos"][1])) < 2.5 and w.walkable(c["pos"][0], c["pos"][1]) and zone_ok(c["pos"][0], c["pos"][1]):
                if cube_goal is None or dd < cube_goal[2]:
                    cube_goal = (c["pos"][0], c["pos"][1], dd)
        crate_tgt = None
        for c in obs.get("crates", []):
            dd = d2(c["pos"], pos)
            if dd < (P["powerup_r"] if t < P["powerup_until"] else P["crate_r"]) and abs(c["pos"][2] - w.ground(c["pos"][0], c["pos"][1])) < 3.0 and self.ignore_crates.get(c["id"], 0) < 120 and zone_ok(c["pos"][0], c["pos"][1]):
                if crate_tgt is None or dd < crate_tgt[1]:
                    crate_tgt = (c, dd)
        npc_tgt = None
        if hp_frac > P["npc_hp"]:
            for n in npcs:
                dd = d2(n["pos"], pos)
                if dd < (max(P["npc_r"], 200.0) if t < P["powerup_until"] else P["npc_r"]) and zone_ok(n["pos"][0], n["pos"][1]):
                    if npc_tgt is None or dd < npc_tgt[1]:
                        npc_tgt = (n, dd)
        drop_goal = None
        if P["drops"]:
            for dr in obs.get("drops", []):
                dp = dr["pos"]
                dd = d2(dp, pos)
                hold = dr.get("holder")
                if hold is not None and hold != me["id"]:
                    continue
                if dd < P["drop_r"] and zone_ok(dp[0], dp[1]) and dr["lands_in"] < dd / 10.0 + 6.0:
                    if not any(d2(e["pos"], dp) < (45 if P["drop_heal"] else 70) for e in enemies):
                        if drop_goal is None or dd < drop_goal[2]:
                            drop_goal = (dp[0], dp[1], dd, dr.get("radius", 8.0))

        # ---- threat model
        threats = [e for e in enemies if e["_d"] < 200]
        self.cur_threats = [e for e in threats if e["_d"] < 150]
        under_fire = (t - self.last_damage_t) < 1.5
        n_threats = len(threats)
        my_hp = me["hp"]
        strongest = max((e.get("hp", 1400) for e in threats), default=0)
        eyes_on_us = 0
        for e in threats:
            b_e2me = math.atan2(y - e["pos"][1], x - e["pos"][0])
            if e["_d"] < 150 and abs(wrap(e["aim"][0] - b_e2me)) < 0.35:
                eyes_on_us += 1
        converging = P["smart_targets"] and eyes_on_us >= P["converge_n"]
        # v19: replays show mechs hit by 2+ different attackers survive far more often when they
        # back off at once (66-89% vs 50-67% holding, 33-62% pushing in). Break off for a few seconds.
        if P["gang_escape"]:
            self.hits_by = [(ht, hb) for ht, hb in self.hits_by if t - ht < P["gang_window"]]
            if len({hb for _, hb in self.hits_by}) >= P["gang_n"]:
                self.gang_until = t + P["gang_hold"]
            if t < self.gang_until and threats:
                converging = True
        rh_ = P["retreat_hp"]
        if P["solo_duel"] and n_threats <= 1 and len([e for e in enemies if e["_d"] < 200]) <= 1:
            rh_ = min(rh_, P["solo_hp"])          # Xero real A/B: lone hunter -> fight it out, retreat only below 25%
        losing = converging or (P["pacifist"] and bool(threats)) or (P["stealth"] and bool(threats) and target is None) or threats and (hp_frac < rh_ or (hp_frac < P["retreat_hp_multi"] and (n_threats >= 2 or strongest > my_hp * 1.3)))
        if P["lose_rule"] and threats:
            near_n = sum(1 for e in enemies if e["_d"] < 160)
            lr = (near_n >= 2 and hp_frac < 0.55) or (near_n >= 3 and hp_frac < 0.8)
            if target is not None:
                tf = target.get("hp", 1400) / max(target.get("max_hp", 1400), 1)
                if hp_frac < 0.35 and tf > hp_frac + 0.1:
                    lr = True
                if tf < 0.25 and hp_frac > 0.2 and near_n <= 2:
                    lr = False
                    losing = False
            losing = losing or lr
        near_enemy = min((e["_d"] for e in enemies), default=999.0)
        heal_mode = hp_frac < (0.85 if self.survival else P["heal_to"]) and not threats and (t - self.last_damage_t) > 0.5
        if losing:
            self.recovering = True
        elif self.recovering and hp_frac >= P["recover_hp"]:
            self.recovering = False

        # ---- power position (rooftop) for the midgame
        pp_res = None
        no_steer = False
        if P["power_pos"] and not (urgent_zone and zgoal is not None):
            pp_res = self.power_position(obs, me, circle, target, enemies, t, hp_frac, muzzle)
        elif self.pp is not None:
            self.pp = None

        # ---- movement decision
        move = [0.0, 0.0]
        boost = False
        jump = False
        speed_scale = 1.0
        label = ""
        farm_shoot = None

        regen_defend = False
        if P["regen"]:
            fin = target is not None and target.get("hp", 1400) / max(target.get("max_hp", 1400), 1) < P["regen_fin"] and target["_d"] < 120
            if not self.regen_on and hp_frac < P["regen_hp"] and not fin:
                self.regen_on = True
            if self.regen_on and (hp_frac >= P["regen_to"] or fin):
                self.regen_on = False
            regen_defend = self.regen_on and under_fire and near_enemy < 60 and hp_frac > 0.35
        if urgent_zone and zgoal is not None:
            label = "zone"
            move = self.go_to(me, self.zone_plan, zgoal, t)
            boost = (not me["in_zone"]) or (time_to_shrink < d2(zgoal, pos) / 11.0 + 3) or (t >= zone["shrink_start"])
        elif pp_res is not None and pp_res["hold"] and not losing:
            label = pp_res["label"]
            move, boost, jump = pp_res["move"], pp_res["boost"], pp_res["jump"]
            no_steer = True
        elif self.regen_on and not regen_defend:
            seen_by = [e for e in enemies if e["_d"] < 220 and w.los((e["pos"][0], e["pos"][1], e["pos"][2] + 5.0), (x, y, z + 3.5))]
            if seen_by:
                label = "regen-hide"
                move = self.retreat_vector(me, seen_by, circle)
                boost = me["energy"] > 25
            else:
                label = "regen"
                move = [0.0, 0.0]
        elif losing and P["cover_retreat"]:
            if t - self.cover_t > 0.5:
                self.cover_t = t
                self.cover_spot_ = self.cover_spot(me, threats, circle)
            sp_ = self.cover_spot_
            if sp_ is None:
                if target is not None and P["cornered_fight"]:
                    label = f"fight {target['id']} cornered"
                    move = self.fight_vector(me, target, slots, weps, circle)
                else:
                    label = "retreat"
                    move = self.retreat_vector(me, threats, circle)
                    boost = me["energy"] > 25
            elif d2(sp_, pos) < 2.0:
                label = "retreat cover-hold"
                move = [0.0, 0.0]
            else:
                label = "retreat cover"
                move = self.go_to(me, self.local_plan, sp_, t)
                boost = me["energy"] > 10
        elif losing:
            label = "retreat"
            move = self.retreat_vector(me, threats, circle)
            boost = me["energy"] > 25
        elif P["heal_first"] and heal_mode and target is None:
            label = "heal"
            if P["sit_heal"]:
                move = [0.0, 0.0]
            else:
                hide = self.pick_hide(obs, circle)
                if d2(hide, pos) > 3.0:
                    move = self.go_to(me, self.zone_plan, hide, t)
                    speed_scale = 0.4
        elif (P["cube_fight_r"] > 0 and cube_goal is not None and cube_goal[2] < P["cube_fight_r"]
              and hp_frac > P["cube_fight_hp"]):
            label = "cube-grab"
            move = self.go_to(me, self.local_plan, (cube_goal[0], cube_goal[1]), t)
            boost = cube_goal[2] > 12 and me["energy"] > 30
        elif P["laser_grab"] and laser_goal is not None and laser_goal[2] < P["laser_grab_r"] and hp_frac > 0.3:
            # v24: lasers mostly come from dead mechs (1272 drops vs 300 spawns in lab replays); we picked up
            # only 5 of 44 lasers dropped by our own kills, usually lying 40-65 m away. Go take it now;
            # aiming/firing at the current target continues while we move.
            label = "grab-laser"
            move = self.go_to(me, self.local_plan, (laser_goal[0], laser_goal[1]), t)
            boost = laser_goal[2] > 12 and me["energy"] > 30
        elif laser_goal is not None and (target is None or (target["_d"] > 110 and laser_goal[2] < 70)) and min((e["_d"] for e in enemies), default=999) > 70:
            label = "get-laser"
            move = self.go_to(me, self.local_plan, (laser_goal[0], laser_goal[1]), t)
            boost = laser_goal[2] > 20 and me["energy"] > 30
        elif (P["drop_heal"] and drop_goal is not None and (t - self.last_damage_t) > 1.5
              and (hp_frac < 0.85 or me.get("cubes", 0) < 10)):
            dd = drop_goal[2]
            if dd > 5.0:
                label = "drop-heal"
                move = self.go_to(me, self.local_plan, (drop_goal[0], drop_goal[1]), t)
                boost = dd > 15 and me["energy"] > 30
            else:
                label = "capture-move"   # stay inside the ring but never stand still
                a = math.atan2(y - drop_goal[1], x - drop_goal[0]) + math.pi / 2
                move = [0.5 * math.cos(a), 0.5 * math.sin(a)]
        elif target is not None:
            label = f"fight {target['id']}"
            move = self.fight_vector(me, target, slots, weps, circle)
            pushing = False
            if not target.get("_los", True):
                tvx, tvy = target["vel"][0], target["vel"][1]
                rx_, ry_ = x - target["pos"][0], y - target["pos"][1]
                rn_ = math.hypot(rx_, ry_) or 1.0
                closing = (tvx * rx_ + tvy * ry_) / rn_
                pushing = closing > 1.5 or target["_d"] < 55
            if P["hold_angles"] and not target.get("_los", True) and pushing and hp_frac > 0.3:
                label += " hold"
                move = [0.0, 0.0]
            elif P["hunt_hidden"] and not target.get("_los", True):
                pk = self.peek_point(me, target, muzzle, t)
                if pk is not None:
                    label += " peek"
                    move = self.go_to(me, self.local_plan, pk, t)
            if P["aggressive"]:
                charging = "laser" in slots and (weps[slots["laser"]].get("charge", 0) or 0) > 0
                boost = P["fight_boost"] and me["energy"] > 45 and not charging and (int(t * 2.0) % 3 == 0)
            if P["reload_cover"] and target["_d"] < 180:
                gi_ = slots.get("gun")
                reloading = gi_ is not None and weps[gi_].get("reload", 0) > 0.4
                other_ready = ("lightning" in slots and target["_d"] < 44 and me["energy"] > 10) or (
                    "laser" in slots and weps[slots["laser"]].get("cooldown", 0) < 0.3)
                if reloading and not other_ready:
                    label += " reload-cover"
                    move = self.retreat_vector(me, threats or [target], circle)
        elif econ_goal is not None:
            kind_, i_, p_, dd_, obj_ = econ_goal
            label = "econ " + kind_
            if kind_ == "npc" and obj_ is not None and dd_ < 75 and w.los(muzzle, (p_[0], p_[1], p_[2] + 1.75)):
                move = self.fight_vector(me, obj_, slots, weps, circle, want=P["npc_stand"])
                farm_shoot = ("npc", obj_)
            elif kind_ == "crate" and obj_ is not None and dd_ < 50 and w.los(muzzle, (p_[0], p_[1], p_[2] + 1.2)):
                farm_shoot = ("crate", obj_)
                if dd_ < 12:
                    move = [(x - p_[0]) / max(dd_, 0.1), (y - p_[1]) / max(dd_, 0.1)]
            else:
                move = self.go_to(me, self.local_plan, (p_[0], p_[1]), t)
                boost = P["econ_boost"] and dd_ > 40 and me["energy"] > 40 and t - self.enemy_seen_t > 3
        elif pp_res is not None:
            label = pp_res["label"]
            move, boost = pp_res["move"], pp_res["boost"]
        elif (P["ghost_hunt"] and not P["stealth"] and not self.survival and t >= P["powerup_until"] and ghost is not None and hp_frac > 0.55 and not heal_mode
              and zone_ok(ghost["pos"][0], ghost["pos"][1])):
            label = f"stalk {ghost['id']}"
            pk = self.peek_point(me, ghost, muzzle, t)
            move = self.go_to(me, self.local_plan, pk if pk is not None else (ghost["pos"][0], ghost["pos"][1]), t)
            farm_shoot = ("ghost", ghost)
        elif (P["hunt_hidden"] and not P["stealth"] and not self.survival and t >= P["powerup_until"] and self.heard is not None and t - self.heard[2] < 3.0 and hp_frac > 0.5
              and not heal_mode and zone_ok(x + math.cos(self.heard[0]) * self.heard[1], y + math.sin(self.heard[0]) * self.heard[1])):
            b, dm, _ = self.heard
            label = "hunt-sound"
            gx, gy = x + math.cos(b) * dm, y + math.sin(b) * dm
            move = self.go_to(me, self.local_plan, (gx, gy), t)
        elif cube_goal is not None and (near_enemy > 60 or hp_frac > 0.5):
            label = "cube"
            move = self.go_to(me, self.local_plan, (cube_goal[0], cube_goal[1]), t)
            boost = cube_goal[2] > 30 and me["energy"] > 40
        elif drop_goal is not None and calm:
            dd = drop_goal[2]
            if dd > drop_goal[3] - 3.0:
                label = "drop"
                move = self.go_to(me, self.local_plan, (drop_goal[0], drop_goal[1]), t)
                boost = me["energy"] > 30
            else:
                label = "capture"
                move = [0.0, 0.0]
        elif heal_mode:
            label = "heal"
            hide = self.pick_hide(obs, circle)
            if d2(hide, pos) > 3.0:
                move = self.go_to(me, self.zone_plan, hide, t)
                speed_scale = 0.4
        elif calm and loot_goal is not None:
            label = "loot " + loot_goal[3]
            move = self.go_to(me, self.local_plan, (loot_goal[0], loot_goal[1]), t)
        elif (calm or (t < P["powerup_until"] and target is None)) and npc_tgt is not None and not P["stealth"] and (not P["farm_hp_gate"] or hp_frac >= 0.9 or t < P["powerup_until"]):
            n, dd = npc_tgt
            label = "npc"
            vis = w.los(muzzle, (n["pos"][0], n["pos"][1], n["pos"][2] + 1.75))
            stand = P["npc_stand"]
            if dd > stand + 12 or not vis:
                move = self.go_to(me, self.local_plan, (n["pos"][0], n["pos"][1]), t)
            elif dd < stand - 10:
                ax, ay = x - n["pos"][0], y - n["pos"][1]
                nn = math.hypot(ax, ay) or 1
                move = [ax / nn, ay / nn]
            else:
                move = self.fight_vector(me, n, slots, weps, circle, want=stand)
            if vis:
                farm_shoot = ("npc", n)
        elif (calm or (t < P["powerup_until"] and target is None)) and crate_tgt is not None and P["crates"] and not P["stealth"] and (not P["farm_hp_gate"] or hp_frac >= 0.9 or t < P["powerup_until"]):
            c, dd = crate_tgt
            label = "crate"
            cz = c["pos"][2] + 1.2
            vis = w.los(muzzle, (c["pos"][0], c["pos"][1], cz))
            if dd > 35 or not vis:
                move = self.go_to(me, self.local_plan, (c["pos"][0], c["pos"][1]), t)
            if vis and dd < 120:
                farm_shoot = ("crate", c)
        elif P["assassin"] and not self.survival and t < P["roam_until"] and hp_frac > P["roam_hp"] and not heal_mode:
            label = "roam"
            rp = self.roam_point(obs, circle, t)
            move = self.go_to(me, self.zone_plan, rp, t)
        elif P["idle_still"] and hp_frac >= P["idle_still_hp"] and (zgoal is None or not urgent_zone):
            label = "wait"
            move = [0.0, 0.0]
        else:
            hide = self.pick_hide(obs, circle)
            dd = d2(hide, pos)
            if dd > 3.0:
                label = "to-hide"
                move = self.go_to(me, self.zone_plan, hide, t)
                if zgoal is None:
                    speed_scale = 0.38 if dd < 120 and time_to_shrink > dd / 4.0 + 10 else 1.0
            else:
                label = "hide"
                move = [0.0, 0.0]
                if P["hide_shuffle"]:
                    ph = t * 0.9 + self.scan_phase
                    move = [0.4 * math.cos(ph), 0.4 * math.sin(ph)]

        if P["avoid_r"] > 0 and not (urgent_zone and zgoal is not None) and not label.startswith(("fight", "retreat", "zone")):
            ax_, ay_ = 0.0, 0.0
            for e in enemies:
                if e["_d"] < P["avoid_r"]:
                    k_ = (P["avoid_r"] - e["_d"]) / P["avoid_r"]
                    ax_ += (x - e["pos"][0]) / max(e["_d"], 1.0) * k_
                    ay_ += (y - e["pos"][1]) / max(e["_d"], 1.0) * k_
            if ax_ or ay_:
                nx_, ny_ = move[0] + ax_ * P["avoid_w"], move[1] + ay_ * P["avoid_w"]
                if zone_ok(x + nx_ * 10.0, y + ny_ * 10.0):
                    n_ = math.hypot(nx_, ny_) or 1.0
                    move = [nx_ / n_, ny_ / n_]

        # ---- stealth: move quietly (< step_speed) unless running from the zone or from fire
        if P["stealth"] and not (urgent_zone and zgoal is not None):
            if label.startswith("retreat"):
                boost = boost and (hp_frac < 0.5 or (t - self.last_damage_t) < 2.0)
            elif not label.startswith("fight"):
                boost = False
                speed_scale = min(speed_scale, 0.4)

        if P["quiet"] and not (urgent_zone and zgoal is not None) and not label.startswith(("fight", "retreat", "cube-grab", "grab-laser", "drop-heal")):
            boost = False
            speed_scale = min(speed_scale, P["quiet_scale"])

        # ---- lab finding: hurt bots get worn down by drones (45 m guard radius). Keep out of it unless farming.
        if P["drone_avoid"] and hp_frac < P["drone_avoid_hp"] and not label.startswith("npc") and not label.startswith("zone"):
            for n in npcs:
                dn = d2(n["pos"], pos)
                if dn < 55.0:
                    ax, ay = x - n["pos"][0], y - n["pos"][1]
                    k = (55.0 - dn) / 55.0 * 1.5 / (dn or 1.0)
                    move = [move[0] + ax * k, move[1] + ay * k]
        # ---- dodging (adds a lateral component)
        dodge = self.dodge_vector(obs, me, enemies)
        if dodge is not None:
            move = [move[0] * 0.4 + dodge[0], move[1] * 0.4 + dodge[1]]
            boost = boost or me["energy"] > 30
            speed_scale = 1.0
            label += " dodge"

        # ---- stuck handling
        self.last_pos.append((t, x, y))
        while self.last_pos and t - self.last_pos[0][0] > 1.6:
            self.last_pos.pop(0)
        mv = math.hypot(move[0], move[1])
        if no_steer:
            self.last_pos.clear()
        elif t < self.stuck_until and self.unstick_dir is not None:
            move = list(self.unstick_dir)
            jump = me.get("grounded", True)
            boost = True
        elif mv > 0.5 and len(self.last_pos) > 10 and t - self.last_pos[0][0] > 1.4:
            moved = math.hypot(x - self.last_pos[0][1], y - self.last_pos[0][2])
            if moved < 1.5 * speed_scale:
                a = math.atan2(move[1], move[0]) + random.choice([-1, 1]) * random.uniform(0.8, 2.0)
                self.unstick_dir = (math.cos(a), math.sin(a))
                self.stuck_until = t + 0.8
                self.last_pos.clear()
                self.zone_plan.goal = None
                self.local_plan.goal = None

        if not (t < self.stuck_until) and not no_steer:
            move = self.steer(me, move)
        move = [move[0] * speed_scale, move[1] * speed_scale]

        # ---- aiming and firing
        fire = [False, False]
        aim = None
        if target is not None:
            aim, fire = self.shoot(me, target, slots, weps, muzzle, t)
        elif farm_shoot is not None and farm_shoot[0] == "ghost":
            gh = farm_shoot[1]
            aim, fire = self.shoot(me, gh, slots, weps, muzzle, t)
            fire = [f if (slots.get("laser") == i) else False for i, f in enumerate(fire)]
        elif farm_shoot is not None and farm_shoot[0] == "npc":
            aim, fire = self.shoot(me, farm_shoot[1], slots, weps, muzzle, t, npc=True)
        elif farm_shoot is not None:
            c = farm_shoot[1]
            fake = {"pos": [c["pos"][0], c["pos"][1], c["pos"][2] - 3.3], "vel": [0, 0, 0], "kind": "crate"}
            yaw, pitch, hd, _ = self.lead(me, fake, 252.0)
            aim = [yaw, pitch]
            gi = slots.get("gun")
            if gi is not None and self.aligned(me, yaw, pitch, hd, 0.8):
                fire[gi] = True
            li = slots.get("lightning")
            if li is not None and hd < 40 and abs(wrap(me["aim"][0] - yaw)) < 0.15 and me["energy"] > 40:
                fire[li] = True
            self.ignore_crates[c["id"]] = self.ignore_crates.get(c["id"], 0) + 1
        if aim is None:
            aim = self.idle_aim(obs, me, enemies, t, move)
        if label.startswith("regen"):
            fire = [False, False]   # dealing damage resets the 5 s regen timer

        # reload when idle and low
        g = slots.get("gun")
        if g is not None and target is None:
            gw = weps[g]
            if gw.get("ammo", 12) <= 5 and gw.get("reload", 0) == 0:
                act["reload"] = True

        act.update(move=move, boost=bool(boost), jump=bool(jump), aim=aim, fire=fire)
        if LABLOG and int(t * 10) % 10 == 0:
            vis_n = sum(1 for e in enemies if e.get("_los"))
            sys.stderr.write("L %.0f %s|tgt=%s|d=%s|n=%d|vis=%d|hp=%.2f|surv=%d|rec=%d|fire=%s\n" % (
                t, label, target["id"] if target is not None else "-", int(target["_d"]) if target is not None else "-",
                len(enemies), vis_n, hp_frac, int(self.survival), int(self.recovering), "".join("1" if f else "0" for f in fire)))
        mf.draw.label(f"{label} hp{me['hp']:.0f}")
        return act

    # ------------------------------------------------------------------ behaviours
    def aligned(self, me, yaw, pitch, dist, tol_m):
        ay, ap = me["aim"]
        e_yaw = abs(wrap(ay - yaw))
        e_p = abs(ap - pitch)
        tol = math.atan2(tol_m, max(dist, 1.0)) + 0.004
        return e_yaw <= tol and e_p <= tol * 1.5 + 0.01

    def shoot(self, me, tgt, slots, weps, muzzle, t, npc=False):
        fire = [False, False]
        d = d2(tgt["pos"], me["pos"])
        tz = tgt["pos"][2] + (1.75 if npc else 4.5)
        vis = tgt.get("_los")
        if vis is None:
            vis = self.w.los(muzzle, (tgt["pos"][0], tgt["pos"][1], tz))
        aim = None
        # lightning: point at it, fire when in cone and range
        li = slots.get("lightning")
        if li is not None:
            lw = weps[li]
            rng = lw["stats"].get("range", 45)
            if d < rng * 0.95 and vis:
                yaw = math.atan2(tgt["pos"][1] - me["pos"][1], tgt["pos"][0] - me["pos"][0])
                aim = [yaw, 0.0]
                if abs(wrap(me["aim"][0] - yaw)) < math.radians(lw["stats"].get("cone_deg", 30) * 0.4) and lw.get("cooldown", 0) <= 0.1 and me["energy"] >= lw["stats"].get("energy", 3):
                    fire[li] = True
        # laser (hitscan) + gun: while the laser charges we aim with gun lead so the gun keeps firing;
        # in the last moment we switch to the exact (no-lead) laser line and release when aligned.
        la = slots.get("laser")
        gi = slots.get("gun")
        laser_aim_now = False
        if la is not None and not (li is not None and fire[li]):
            lw = weps[la]
            rng = lw["stats"].get("range", 280)
            charge = lw.get("charge", 0) or 0
            full = lw["stats"].get("charge", 0.75)
            hold = lw["stats"].get("hold_s", 2.0)
            ly, lp, lhd, _ = self.lead(me, tgt, 1e9)
            if d < rng * 0.97 and vis and t >= self.shorted_until and (li is None or d > 30):
                if lw.get("cooldown", 0) <= 0.15:
                    lethal = False
                    if P["lethal_laser"] and tgt.get("kind") == "mech" and charge >= lw["stats"].get("min_charge", 0.15) and charge < full - 0.12:
                        st = lw["stats"]
                        dmg = st.get("damage", 315) * (st.get("min_frac", 0.35) + (1 - st.get("min_frac", 0.35)) * charge / full)
                        dmg *= 1 + self.cfg["cubes"].get("damage_per_cube", 0.04) * min(me.get("cubes", 0), self.cfg["cubes"].get("stat_cap", 10))
                        lethal = dmg * 0.7 >= tgt.get("hp", 1400)
                    if lethal:
                        laser_aim_now = True
                        aim = [ly, lp]
                        fire[la] = not self.aligned(me, ly, lp, lhd, 1.2)
                    elif charge < full - 0.12:
                        fire[la] = True  # keep charging
                    else:
                        laser_aim_now = True
                        aim = [ly, lp]
                        if self.aligned(me, ly, lp, lhd, 1.4):
                            fire[la] = False  # release -> beam fires
                        else:
                            fire[la] = charge < full + hold - 0.25
            elif charge > 0.2:
                if vis or charge > full + hold - 0.3:
                    fire[la] = False
                else:
                    fire[la] = True  # keep the charge while we wait for a line
            elif (P["hunt_hidden"] and not vis and d < 160 and lw.get("cooldown", 0) <= 0.15
                  and t >= self.shorted_until and charge < full):
                fire[la] = True  # pre-charge so we fire the moment it steps out
                aim = [ly, lp]
        # mortar
        mo = slots.get("mortar")
        if mo is not None and not npc:
            mw = weps[mo]
            if 28 < d < 220 and mw.get("cooldown", 0) <= 0.1:
                sol = self.mortar_aim(me, tgt, mw)
                if sol is not None and (aim is None or not vis):
                    yaw, pitch, D = sol
                    aim = [yaw, pitch]
                    if self.aligned(me, yaw, pitch, D, 2.0):
                        fire[mo] = True
        # gun
        if gi is not None:
            gw = weps[gi]
            spd = gw["stats"].get("speed", 252)
            rng = gw["stats"].get("range", 230)
            yaw, pitch, hd, _ = self.lead(me, tgt, spd)
            if aim is None or (not laser_aim_now and li is None):
                aim = [yaw, pitch]
            if vis and hd < rng * 0.97 and (gw.get("ammo", 1) > 0 or 0 < gw.get("reload", 0) <= 0.1) and gw.get("reload", 0) <= 0.1:
                tol = 2.2 if not npc else 1.2
                if self.aligned(me, yaw, pitch, hd, tol):
                    fire[gi] = True
            # an unaware target (facing away): let the full laser land on its back first, then the gun
            if (P["backstab_laser_first"] and fire[gi] and la is not None and tgt.get("kind") == "mech"
                    and tgt["id"] not in self.hit_ids
                    and weps[la].get("cooldown", 0) <= 0.15 and t >= self.shorted_until):
                b_t2me = math.atan2(me["pos"][1] - tgt["pos"][1], me["pos"][0] - tgt["pos"][0])
                if abs(wrap(tgt["aim"][0] - b_t2me)) > math.pi / 2:
                    fire[gi] = False
        if aim is None:
            yaw = math.atan2(tgt["pos"][1] - me["pos"][1], tgt["pos"][0] - me["pos"][0])
            aim = [yaw, 0.0]
        return aim, fire

    def spot_vector(self, me, tgt, slots, weps, want, sx, sy):
        """v37: pick the best nearby spot: range, line of fire when ready / cover when not, few other watchers, cubes."""
        w = self.w
        x, y, z = me["pos"]
        tx, ty, tz = tgt["pos"]
        mh = me.get("stats", {}).get("muzzle_height", 3.6)
        tc = (tx, ty, tz + 3.0)
        ready = True
        if P["spot_duck"]:
            gi, la, li = slots.get("gun"), slots.get("laser"), slots.get("lightning")
            g_ok = gi is not None and weps[gi].get("ammo", 1) > 0 and weps[gi].get("reload", 0) <= 0.25
            l_ok = la is not None and weps[la].get("cooldown", 0) <= 0.3
            z_ok = li is not None and tgt["_d"] < 50 and me["energy"] > 15
            ready = g_ok or l_ok or z_ok
        want_los = 1.0 if ready else -1.0
        others = [e for e in self.cur_enemies if e.get("id") != tgt.get("id") and e.get("_d", 999) < 170][:4]
        cube = None
        for c in self.cur_cubes:
            dc = d2(c["pos"], (x, y))
            if dc < 40 and (cube is None or dc < cube[2]):
                cube = (c["pos"][0], c["pos"][1], dc)
        best, bv = None, None
        cands = [(0.0, 0.0, 0.0)] + [(math.cos(k * math.pi / 8), math.sin(k * math.pi / 8), 6.0) for k in range(16)] + \
                [(math.cos(k * math.pi / 4 + 0.39), math.sin(k * math.pi / 4 + 0.39), 12.0) for k in range(8)]
        for dx, dy, r in cands:
            cx, cy = x + dx * r, y + dy * r
            if r > 0 and (not w.walkable(cx, cy) or not w.clear_line((x, y), (cx, cy))):
                continue
            gz = w.ground(cx, cy) if r > 0 else z
            s_ = -1.1 * abs(math.hypot(tx - cx, ty - cy) - want) - r * 0.4
            los = w.los((cx, cy, gz + mh), tc)
            if los:
                s_ += P["spot_los"] * want_los
            body = (cx, cy, gz + 3.0)
            for e in others:
                if w.los((e["pos"][0], e["pos"][1], e["pos"][2] + 5.0), body):
                    s_ -= P["spot_expo"]
            if cube is not None:
                s_ += 1.5 * (cube[2] - math.hypot(cube[0] - cx, cube[1] - cy))
            if want_los > 0:
                s_ += 9.0 * (dx * sx + dy * sy)
            if r == 0:
                s_ -= 8.0
                if want_los < 0 and not los:
                    s_ += 14.0
            if best is None or s_ > best:
                best, bv = s_, (dx, dy)
        if bv is None:
            return None
        return [bv[0], bv[1]]

    def fight_vector(self, me, tgt, slots, weps, circle, want=None):
        x, y = me["pos"][0], me["pos"][1]
        tx, ty = tgt["pos"][0], tgt["pos"][1]
        d = d2((x, y), (tx, ty))
        rx, ry = (tx - x) / max(d, 1e-6), (ty - y) / max(d, 1e-6)
        px, py = -ry, rx
        # v33 steady strafe: long runs in one direction (reverse every 3-5 s, or early at a wall/edge)
        ahead_x, ahead_y = x + px * self.strafe_dir * 7.0, y + py * self.strafe_dir * 7.0
        if self.t > self.strafe_until or (self.t > self.strafe_until - P["strafe_max"] + 0.8
                                          and not self.w.walkable(ahead_x, ahead_y)):
            self.strafe_dir = -self.strafe_dir
            self.strafe_until = self.t + random.uniform(P["strafe_min"], P["strafe_max"])
        # preferred range
        if want is not None:
            pass
        elif "lightning" in slots and me["energy"] > 20:
            want = P["want_lightning"]
        elif "laser" in slots:
            want = P["want_laser"]
        else:
            want = P["want_gun"]
        if P["aggressive"]:
            thp = tgt.get("hp", 1400) / max(tgt.get("max_hp", 1400), 1)
            myhp = me["hp"] / max(me["max_hp"], 1)
            crowd = sum(1 for e in self.cur_threats if e.get("id") != tgt.get("id")) if P["press_multi_block"] else 0
            if (thp < P["press_hp"] or thp < myhp - P["press_gap"]) and crowd == 0:
                want = min(want, P["press_r"])  # press the kill
        their = tgt.get("weapons", [])
        if "lightning" in their and "lightning" not in slots:
            want = max(want, 60.0)
        if P["laser_min_r"] and "laser" in their and tgt.get("kind") == "mech":
            # v17: house-bot lasers hit 48% inside 60 m vs 18% at 60-120 m (replays). Stay out of that band while
            # their laser is ready/charging; close in (press, lightning) only while it is on cooldown.
            thp_ = tgt.get("hp", 1400) / max(tgt.get("max_hp", 1400), 1)
            on_cd = (self.t - self.laser_fired.get(tgt.get("id"), -99.0)) < P["laser_cd_window"]
            if thp_ > P["laser_min_finish"] and (tgt.get("charging") or not on_cd):
                want = max(want, P["laser_min_r"])
        if (P["standoff"] and "laser" in slots and "laser" not in their and tgt.get("kind") == "mech"
                and self.t >= self.shorted_until):
            thp = tgt.get("hp", 1400) / max(tgt.get("max_hp", 1400), 1)
            want = P["standoff_r"] if thp > 0.2 else 120.0
        radial = max(-1.0, min(1.0, (d - want) / 30.0))
        mvx = px * self.strafe_dir * 0.9 + rx * radial * 0.7
        mvy = py * self.strafe_dir * 0.9 + ry * radial * 0.7
        if P["fight_spot"] and tgt.get("kind") == "mech":
            v = self.spot_vector(me, tgt, slots, weps, want, px * self.strafe_dir, py * self.strafe_dir)
            if v is not None:
                mvx, mvy = v
        elif P["fight_expo"] > 0 and tgt.get("kind") == "mech":
            others = [e for e in self.cur_enemies if e.get("id") != tgt.get("id") and e.get("_d", 999) < 170]
            if others:
                best_s, best_v = None, None
                mz = me["pos"][2]
                for k in range(16):
                    a = k * math.pi / 8
                    dx, dy = math.cos(a), math.sin(a)
                    cx_, cy_ = x + dx * 7.0, y + dy * 7.0
                    if not self.w.walkable(cx_, cy_):
                        continue
                    s_ = -1.1 * abs(d2((cx_, cy_), (tx, ty)) - want) + 12.0 * (dx * px * self.strafe_dir + dy * py * self.strafe_dir)
                    gz = self.w.ground(cx_, cy_)
                    body = (cx_, cy_, gz + 3.5)
                    for e in others:
                        if self.w.los((e["pos"][0], e["pos"][1], e["pos"][2] + 5.0), body):
                            s_ -= P["fight_expo"]
                    if best_s is None or s_ > best_s:
                        best_s, best_v = s_, (dx, dy)
                if best_v is not None:
                    mvx, mvy = best_v[0] * 1.1, best_v[1] * 1.1
        # do not leave the safe circle
        cx, cy, r, mr = circle
        if math.hypot(x - cx, y - cy) > mr:
            ax, ay = cx - x, cy - y
            n = math.hypot(ax, ay) or 1
            mvx += ax / n * 0.8
            mvy += ay / n * 0.8
        return [mvx, mvy]

    def power_position(self, obs, me, circle, target, enemies, t, hp_frac, muzzle):
        """Rooftop power position. Returns None (not active) or a dict with move/boost/jump/label and
        hold=True when we are climbing or on the roof (then this overrides fighting/retreat logic)."""
        if not (P["pp_start"] <= t < P["pp_end"]):
            self.pp = None
            return None
        w = self.w
        if P["roof_safe"]:
            recent2 = sum(1 for v in self.seen_at.values() if t - v < 5.0)
            last_seen = max(self.seen_at.values(), default=-99.0)
            x_, y_, z_ = me["pos"]
            on_any = w.roof_at(x_, y_) >= 0 and z_ > w.ground(x_, y_) + 6.0
            if on_any and self.roof_since is None:
                self.roof_since = t
            if not on_any:
                self.roof_since = None
            if self.pp is not None and (recent2 >= 2 or (self.roof_since is not None and t - self.roof_since > 30.0)):
                self.roof_black[self.pp["roof"]] = t + 60.0       # leave: squad converging or stayed too long
                self.pp = None
                return None
            if self.pp is None and (hp_frac < 0.75 or t - last_seen < 8.0):
                return None                                     # only climb when healthy and unwatched
        if len(w.roofs) == 0:
            return None
        x, y, z = me["pos"]
        cx, cy, r, mr = circle
        # choose a roof inside the safe circle
        if self.pp is not None:
            k = self.pp["roof"]
            rf = w.roofs[k]
            if math.hypot(rf[3] - cx, rf[4] - cy) > max(mr * 0.95, 15.0) or self.roof_black.get(k, -1) > t:
                self.pp = None
        if self.pp is None:
            best = None
            for rf in w.roofs:
                k = int(rf[0])
                if self.roof_black.get(k, -1) > t:
                    continue
                if math.hypot(rf[3] - cx, rf[4] - cy) > mr * 0.8:
                    continue
                dd = math.hypot(rf[3] - x, rf[4] - y)
                if P["roof_safe"]:
                    if dd > 140.0:
                        continue
                    sc = dd + abs(rf[2] - 18.0) * 3.0 - min(rf[5], 3000) * 0.01 + math.hypot(rf[3] - cx, rf[4] - cy) * 0.3
                else:
                    sc = dd + rf[2] * 0.5 - min(rf[5], 3000) * 0.004
                if best is None or sc < best[0]:
                    best = (sc, k)
            if best is None:
                return None
            k = best[1]
            pts = w.climb[w.climb[:, 0] == k]
            if len(pts) == 0:
                self.roof_black[k] = t + 120
                return None
            dd = np.hypot(pts[:, 1] - x, pts[:, 2] - y)
            cp = pts[int(np.argmin(dd))]
            self.pp = {"roof": k, "state": "travel", "t0": t, "climb": (float(cp[1]), float(cp[2]))}
        k = self.pp["roof"]
        rf = w.roofs[k]
        top, ctr = float(rf[1]), (float(rf[3]), float(rf[4]))
        on_roof = w.roof_at(x, y) == k and z > top - 1.5
        res = {"move": [0.0, 0.0], "boost": False, "jump": False, "hold": False, "label": ""}
        if on_roof:
            self.pp["state"] = "on"
            res["hold"] = True
            gi = None
            for i_, wp in enumerate(me["weapons"]):
                if wp and wp["kind"] == "gun":
                    gi = i_
            reloading = gi is not None and me["weapons"][gi].get("reload", 0) > 0.3
            tgt = target
            if tgt is None and enemies:
                tgt = enemies[0]
            if tgt is not None and hp_frac > 0.35 and not reloading:
                pk = self.roof_peek_point(me, k, top, tgt, t)
                goal = pk if pk is not None else ctr
                res["label"] = "roof-peek"
            else:
                goal = ctr
                res["label"] = "roof-hold"
            vx, vy = goal[0] - x, goal[1] - y
            n = math.hypot(vx, vy)
            if n > 1.5:
                sp = 1.0 if n > 6 else 0.5
                res["move"] = [vx / n * sp, vy / n * sp]
            return res
        if self.pp["state"] == "climb":
            if t - self.pp["t0"] > P["pp_climb_s"]:
                self.roof_black[k] = t + 90
                self.pp = None
                return None
            res["hold"] = True
            res["label"] = "climb"
            vx, vy = ctr[0] - x, ctr[1] - y
            n = math.hypot(vx, vy) or 1.0
            res["move"] = [vx / n, vy / n]
            if me.get("grounded", True):
                res["jump"] = True
            res["boost"] = z < top + 1.5 or not me.get("grounded", True)
            return res
        # travel to the climb point on the ground
        cp = self.pp["climb"]
        dcp = math.hypot(cp[0] - x, cp[1] - y)
        if dcp < 3.5 and me["energy"] >= min(95.0, 30.0 + (top - z) * 1.8):
            self.pp["state"] = "climb"
            self.pp["t0"] = t
            res["hold"] = True
            res["label"] = "climb"
            res["jump"] = True
            return res
        if t - self.pp["t0"] > 60.0:
            self.roof_black[k] = t + 90
            self.pp = None
            return None
        res["label"] = "to-roof"
        if dcp < 3.5:
            res["label"] = "to-roof wait-energy"
            return res
        res["move"] = self.go_to(me, self.local_plan, cp, t)
        return res

    def roof_peek_point(self, me, k, top, tgt, t):
        """A spot on our roof from which our muzzle sees the target (nearest to us)."""
        if self.roof_peek is not None and self.roof_peek[2] == tgt["id"] and t < self.roof_peek[3]:
            return (self.roof_peek[0], self.roof_peek[1])
        w = self.w
        x, y, z = me["pos"]
        mh = me["stats"].get("muzzle_height", 7.0)
        tp = (tgt["pos"][0], tgt["pos"][1], tgt["pos"][2] + 4.5)
        base = math.atan2(tp[1] - y, tp[0] - x)
        best = None
        for ring in (0.0, 4.0, 8.0, 12.0, 18.0, 26.0):
            for kk in range(1 if ring == 0 else 10):
                a = base + (kk - 5) * (TAU / 20)
                px, py = x + math.cos(a) * ring, y + math.sin(a) * ring
                if w.roof_at(px, py) != k:
                    continue
                if w.los((px, py, top + mh), tp, step=2.0):
                    best = (px, py)
                    break
            if best is not None:
                break
        if best is None:
            self.roof_peek = None
            return None
        self.roof_peek = (best[0], best[1], tgt["id"], t + 0.8)
        return best

    def roam_point(self, obs, circle, t):
        """Where house bots are likely: inside the next circle, away from where we already looked."""
        cx, cy, r, mr = circle
        me = obs["self"]["pos"]
        if getattr(self, "_roam", None) is not None and t < self._roam[2] and d2(self._roam, me) > 15.0:
            return (self._roam[0], self._roam[1])
        best = None
        for k in range(10):
            a = random.uniform(-math.pi, math.pi)
            rr = random.uniform(0.15, 0.75) * max(mr, 20.0)
            px, py = cx + math.cos(a) * rr, cy + math.sin(a) * rr
            if not self.w.walkable(px, py):
                continue
            dd = d2((px, py), me)
            sc = abs(dd - 120.0)  # a fresh area ~120 m away
            if best is None or sc < best[0]:
                best = (sc, px, py)
        if best is None:
            best = (0, cx, cy)
        self._roam = (best[1], best[2], t + 25.0)
        return (best[1], best[2])

    def peek_point(self, me, tgt, muzzle, t):
        """Nearest walkable spot from which our muzzle has a line to a hidden (remembered) target."""
        if self.peek is not None and self.peek[2] == tgt["id"] and t < self.peek[3]:
            return (self.peek[0], self.peek[1])
        x, y, z = me["pos"]
        w = self.w
        mh = me["stats"].get("muzzle_height", 7.0)
        tp = (tgt["pos"][0], tgt["pos"][1], tgt["pos"][2] + 4.5)
        best = None
        base = math.atan2(tgt["pos"][1] - y, tgt["pos"][0] - x)
        for ring in (8.0, 16.0, 26.0, 38.0):
            for k in range(12):
                a = base + (k - 6) * (TAU / 12)
                px, py = x + math.cos(a) * ring, y + math.sin(a) * ring
                if not w.walkable(px, py):
                    continue
                pz = w.ground(px, py) + mh
                if w.los((px, py, pz), tp, step=2.0):
                    sc = ring + abs(wrap(a - base)) * 4.0
                    if best is None or sc < best[0]:
                        best = (sc, px, py)
            if best is not None:
                break
        if best is None:
            self.peek = None
            return None
        self.peek = (best[1], best[2], tgt["id"], t + 1.0)
        return (best[1], best[2])

    def cover_spot(self, me, threats, circle):
        x, y, z = me["pos"]
        w = self.w
        cx, cy, r, mr = circle
        ths = sorted(threats, key=lambda e: e["_d"])[:4]

        def hidden(px, py):
            g = w.ground(px, py)
            for e in ths:
                ez = (e["pos"][0], e["pos"][1], e["pos"][2] + 5.0)
                if w.los(ez, (px, py, g + 5.5), step=2.0) or w.los(ez, (px, py, g + 2.5), step=2.0):
                    return False
            return True
        if hidden(x, y):
            return (x, y)
        best, bs = None, 1e18
        for ring in (5.0, 10.0, 16.0, 24.0, 34.0, 46.0, 60.0):
            if ring > P["cover_max"]:
                break
            for k in range(20):
                a = k * TAU / 20
                px, py = x + math.cos(a) * ring, y + math.sin(a) * ring
                if not w.walkable(px, py) or math.hypot(px - cx, py - cy) > max(r - 5, mr):
                    continue
                if abs(w.ground(px, py) - w.ground(x, y)) > 3.0:
                    continue
                sc = ring * (1.0 if w.clear_line((x, y), (px, py)) else 1.8)
                sc -= 0.15 * sum(d2((px, py), e["pos"]) - e["_d"] for e in ths)
                if sc < bs and hidden(px, py):
                    bs, best = sc, (px, py)
            if best is not None and ring >= 10.0:
                break
        return best

    def retreat_vector(self, me, threats, circle):
        x, y, z = me["pos"]
        w = self.w
        cx, cy, r, mr = circle
        best, bv = -1e18, None
        for ring in (14.0, 28.0, 45.0):
            for k in range(16):
                a = k * TAU / 16
                px, py = x + math.cos(a) * ring, y + math.sin(a) * ring
                if not w.walkable(px, py):
                    continue
                pz = w.ground(px, py) + 5.0
                hidden = 0
                for e in threats[:4]:
                    ez = e["pos"][2] + 8.0
                    if not w.los((e["pos"][0], e["pos"][1], ez), (px, py, pz), step=2.0):
                        hidden += 1
                away = sum(d2((px, py), e["pos"]) - e["_d"] for e in threats[:4])
                inz = math.hypot(px - cx, py - cy) <= max(r - 5, mr)
                sc = hidden * P["retreat_hidden_w"] + away * 0.6 - ring * 0.5 + (0 if inz else -200)
                if not w.clear_line((x, y), (px, py)):
                    sc -= 80
                if sc > best:
                    best, bv = sc, (px, py)
        if bv is None:
            e = threats[0]
            ax, ay = x - e["pos"][0], y - e["pos"][1]
            n = math.hypot(ax, ay) or 1
            return [ax / n, ay / n]
        vx, vy = bv[0] - x, bv[1] - y
        n = math.hypot(vx, vy) or 1
        return [vx / n, vy / n]

    def dodge_vector(self, obs, me, enemies):
        x, y, z = me["pos"]
        cz = z + 4.5
        best = None
        # lasers charging at us
        for e in enemies:
            if e.get("charging") and e["_d"] < 290:
                b = math.atan2(y - e["pos"][1], x - e["pos"][0])
                if abs(wrap(e["aim"][0] - b)) < math.atan2(6.0, max(e["_d"], 1)) + 0.03:
                    px, py = -math.sin(b), math.cos(b)
                    s = self.strafe_dir
                    best = (px * s, py * s)
        # bullets / shells
        for p in obs.get("projectiles", []):
            px, py, pz = p["pos"]
            vx, vy, vz = p["vel"]
            if p["kind"] == "mortar":
                g = self.cfg["weapons"]["mortar"].get("gravity", 22.0)
                # landing time approx at our height
                a, b, c = -0.5 * g, vz, pz - z
                disc = b * b - 4 * a * c
                if disc < 0:
                    continue
                tl = (-b - math.sqrt(disc)) / (2 * a)
                if tl < 0 or tl > 4:
                    continue
                lx, ly = px + vx * tl, py + vy * tl
                dd = math.hypot(lx - x, ly - y)
                if dd < 10:
                    ax, ay = x - lx, y - ly
                    n = math.hypot(ax, ay) or 1
                    best = (ax / n, ay / n)
            else:
                rx, ry, rz = x - px, y - py, cz - pz
                vv = vx * vx + vy * vy + vz * vz
                if vv < 1:
                    continue
                tc = (rx * vx + ry * vy + rz * vz) / vv
                if tc < 0 or tc > 1.0:
                    continue
                cxp, cyp = px + vx * tc - x, py + vy * tc - y
                miss = math.hypot(cxp, cyp)
                if miss < 4.0 and False:  # v33b: no per-bullet sidesteps (keep the steady strafe)
                    vh = math.hypot(vx, vy) or 1
                    nx_, ny_ = -vy / vh, vx / vh
                    s = 1 if (nx_ * -cxp + ny_ * -cyp) < 0 else -1
                    if miss < 0.5:
                        s = self.strafe_dir
                    if best is None:
                        best = (nx_ * s, ny_ * s)
        return best

    def idle_aim(self, obs, me, enemies, t, move):
        if not enemies and t < P["open_scan_s"]:
            if self.scan_start is None:
                self.scan_start = (t, me["aim"][0])
            return [wrap(self.scan_start[1] + (t - self.scan_start[0] + 0.3) * 2.6), 0.0]
        if enemies:
            e = enemies[0]
            return [math.atan2(e["pos"][1] - me["pos"][1], e["pos"][0] - me["pos"][0]), 0.0]
        if self.threat_bearing is not None and t - self.threat_t < 4.0:
            return [self.threat_bearing, 0.0]
        # loudest recent combat sound
        best = None
        for s in obs.get("sounds", []):
            if s["kind"] in ("step", "boost", "jet", "gun", "laser_charge", "lightning", "reload") and s["dist"] != "far":
                best = s["bearing"]
        if best is not None:
            self.threat_bearing = best
            self.threat_t = t - 2.0
            return [best, 0.0]
        mv = math.hypot(move[0], move[1])
        if mv > 0.6:
            base = math.atan2(move[1], move[0])
            return [base + 0.6 * math.sin(t * 1.5 + self.scan_phase), 0.0]
        # sweep facing away from the nearest wall (towards open ground)
        z = obs["zone"]
        base = math.atan2(z["center"][1] - me["pos"][1], z["center"][0] - me["pos"][0])
        return [wrap(base + 1.3 * math.sin(t * 0.9 + self.scan_phase)), 0.0]


if __name__ == "__main__":
    mf.run(Bot)

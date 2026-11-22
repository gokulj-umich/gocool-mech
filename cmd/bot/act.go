package main

import (
	"fmt"
	"math"
	"sort"
	"strings"
)

func idxOf(l []string, s string) int {
	for i, x := range l {
		if x == s {
			return i
		}
	}
	return -1
}

// ---- state helpers (b.S["x"])
func (b *Bot) sf(k string) float64 { return num(b.S[k]) }
func (b *Bot) sb(k string) bool    { v, _ := b.S[k].(bool); return v }

func smapF(b *Bot, k string) map[float64]float64 {
	m, ok := b.S[k].(map[float64]float64)
	if !ok {
		m = map[float64]float64{}
		b.S[k] = m
	}
	return m
}
func smapB(b *Bot, k string) map[float64]bool {
	m, ok := b.S[k].(map[float64]bool)
	if !ok {
		m = map[float64]bool{}
		b.S[k] = m
	}
	return m
}

type memEnt struct {
	pos []float64
	t   float64
}

func smapMem(b *Bot, k string) map[float64]memEnt {
	m, ok := b.S[k].(map[float64]memEnt)
	if !ok {
		m = map[float64]memEnt{}
		b.S[k] = m
	}
	return m
}
func stracks(b *Bot) map[float64]M {
	m, ok := b.S["tracks"].(map[float64]M)
	if !ok {
		m = map[float64]M{}
		b.S["tracks"] = m
	}
	return m
}
func sknown(b *Bot) map[float64][]any {
	m, ok := b.S["known_pk"].(map[float64][]any)
	if !ok {
		m = map[float64][]any{}
		b.S["known_pk"] = m
	}
	return m
}

type econG struct {
	kind string
	id   float64
	p    []float64
	dd   float64
	obj  M
}

func anyv(v []float64) []any {
	o := make([]any, len(v))
	for i, x := range v {
		o[i] = x
	}
	return o
}
func hasKey(m M, k string) bool { _, ok := m[k]; return ok }
func getOr(m M, k string, d float64) float64 {
	if v, ok := m[k]; ok && v != nil {
		return num(v)
	}
	return d
}
func b2f(v bool) float64 {
	if v {
		return 1
	}
	return 0
}

func (b *Bot) actInner(obs M) M {
	t := mf(obs, "t")
	b.S["t"] = t
	me := mm(obs, "self")
	pos := vec(me["pos"])
	x, y, z := pos[0], pos[1], pos[2]
	w := b.W
	act := M{}
	maxhp := math.Max(mf(me, "max_hp"), 1.0)
	hpFrac := mf(me, "hp") / maxhp
	stats := mm(me, "stats")
	eyeH := 8.0
	if stats != nil && hasKey(stats, "eye_height") {
		eyeH = mf(stats, "eye_height")
	}
	_ = eyeH
	muzH := 7.0
	if stats != nil && hasKey(stats, "muzzle_height") {
		muzH = mf(stats, "muzzle_height")
	}
	muzzle := []float64{x, y, z + muzH}
	meID := num(me["id"])

	// ---- offers
	if offer := mm(obs, "offer"); offer != nil {
		ch := ma(offer, "choices")
		best, bi := 999, 0
		for i, cc := range ch {
			c := cc.(M)
			UP := upgradePriority
			if ps("upg") == "armor" {
				UP = upgArmor
			}
			r := idxOf(UP, ms(c, "id"))
			if r < 0 {
				r = 50
			}
			if r < best {
				best, bi = r, i
			}
		}
		act["pick"] = bi
	}

	// ---- events
	hitFrom := smapF(b, "hit_from")
	for _, evv := range ma(obs, "events") {
		ev := evv.(M)
		switch ms(ev, "type") {
		case "damaged":
			if ms(ev, "weapon") != "zone" {
				b.S["last_damage_t"] = t
				if ev["by"] != nil && num(ev["by"]) < 100 {
					hb, _ := b.S["hits_by"].([][2]float64)
					b.S["hits_by"] = append(hb, [2]float64{t, num(ev["by"])})
					hitFrom[num(ev["by"])] = t
				}
				if ev["bearing"] != nil {
					b.S["threat_bearing"] = num(ev["bearing"])
					b.S["threat_t"] = t
				}
			}
		case "hit":
			if ev["target"] != nil {
				smapB(b, "hit_ids")[num(ev["target"])] = true
			}
		case "kill":
			b.S["my_kills"] = b.sf("my_kills") + 1
			b.S["last_kill_t"] = t
			if pb("forget_dead") {
				kid := num(ev["target"])
				delete(stracks(b), kid)
				if cid, ok := b.S["cur_tid"].(float64); ok && cid == kid {
					b.S["cur_tid"] = nil
				}
				b.S["peek"] = nil
			}
		case "effect":
			if ms(ev, "effect") == "short" {
				b.S["shorted_until"] = t + 2.0
			}
		}
	}

	if pb("mode_detect") && !b.sb("ffa") {
		alive := 16.0
		if hasKey(obs, "alive") {
			alive = mf(obs, "alive")
		}
		othersDead := (16 - alive) - b.sf("my_kills")
		if othersDead >= 2 && t < 150.0 {
			b.S["ffa"] = true
			P["stealth"] = true
			P["assassin"] = false
			P["ghost_hunt"] = false
		}
	}
	for _, sv := range ma(obs, "sounds") {
		snd := sv.(M)
		switch ms(snd, "kind") {
		case "gun", "laser", "laser_charge", "lightning", "step", "boost", "reload", "jet", "mortar":
		default:
			continue
		}
		dist := ms(snd, "dist")
		if dist == "near" || dist == "mid" {
			dm := 105.0
			if dist == "near" {
				dm = 35.0
			}
			h, _ := b.S["heard"].([]float64)
			if h == nil || dm <= h[1] || t-h[2] > 2.0 {
				b.S["heard"] = []float64{num(snd["bearing"]), dm, t}
			}
		}
	}
	var enemies, npcs []M
	for _, mv := range ma(obs, "mechs") {
		m := mv.(M)
		if ms(m, "kind") == "mech" && num(m["id"]) != meID {
			enemies = append(enemies, m)
		}
		if ms(m, "kind") == "npc" {
			npcs = append(npcs, m)
		}
	}
	for _, e := range enemies {
		e["_d"] = d2(vec(e["pos"]), pos)
	}
	sort.SliceStable(enemies, func(i, j int) bool { return mf(enemies[i], "_d") < mf(enemies[j], "_d") })
	b.S["cur_enemies"] = enemies
	b.S["cur_cubes"] = ma(obs, "cubes")
	tracks := stracks(b)
	for _, e := range enemies {
		hp, mhp := 1400.0, 1400.0
		if hasKey(e, "hp") {
			hp = mf(e, "hp")
		}
		if hasKey(e, "max_hp") {
			mhp = mf(e, "max_hp")
		}
		tracks[num(e["id"])] = M{"pos": vec(e["pos"]), "vel": vec(e["vel"]), "t": t, "hp": hp, "max_hp": mhp,
			"weapons": e["weapons"], "cubes": mf(e, "cubes")}
	}
	curIDs := map[float64]bool{}
	seenAt := smapF(b, "seen_at")
	for _, e := range enemies {
		curIDs[num(e["id"])] = true
		seenAt[num(e["id"])] = t
	}
	laserFired := smapF(b, "laser_fired")
	for _, bv := range ma(obs, "beams") {
		bm := bv.(M)
		if ms(bm, "kind") == "laser" && bm["by"] != nil {
			laserFired[num(bm["by"])] = t - mf(bm, "age")
		}
	}
	wasCharging := smapB(b, "was_charging")
	for _, e := range enemies {
		id := num(e["id"])
		ch := mb(e, "charging")
		if wasCharging[id] && !ch {
			old, ok := laserFired[id]
			if !ok {
				old = -99.0
			}
			laserFired[id] = math.Max(old, t-0.1)
		}
		wasCharging[id] = ch
	}
	heat := 0
	for _, v := range seenAt {
		if t-v < 20.0 {
			heat++
		}
	}
	if pb("commit") {
		hpfNow := hpFrac
		if !b.sb("survival") && (float64(heat) >= pf("heat_max") && hpfNow < 0.75) {
			b.S["survival"] = true
		} else if b.sb("survival") && heat <= 1 && hpfNow > 0.85 && t < pf("endgame_t") {
			b.S["survival"] = false
		}
	}
	var ghost M
	for gid, tr := range tracks {
		age := t - mf(tr, "t")
		if age > pf("ghost_max_age") {
			delete(tracks, gid)
			continue
		}
		if curIDs[gid] || age < 0.2 {
			continue
		}
		var k float64
		if pf("ghost_max_age") > 6.0 {
			k = math.Min(age, 3.0)
		} else {
			k = math.Min(age, 1.5) * 0.7
		}
		tp, tv := tr["pos"].([]float64), tr["vel"].([]float64)
		gp := []float64{tp[0] + tv[0]*k, tp[1] + tv[1]*k, tp[2]}
		dd := d2(gp, pos)
		if dd < 220 && (ghost == nil || dd < mf(ghost, "_d")) {
			ghost = M{"id": gid, "kind": "mech", "pos": anyv(gp), "vel": []any{0.0, 0.0, 0.0}, "hp": tr["hp"],
				"max_hp": tr["max_hp"], "weapons": tr["weapons"], "cubes": tr["cubes"], "_d": dd, "_los": false}
		}
	}
	if len(enemies) > 0 {
		b.S["enemy_seen_t"] = t
	}

	// ---- weapons
	weps := ma(me, "weapons")
	wep := func(i int) M {
		if i < 0 || i >= len(weps) {
			return nil
		}
		m, _ := weps[i].(M)
		return m
	}
	slots := M{}
	for i := range weps {
		if wp := wep(i); wp != nil {
			slots[ms(wp, "kind")] = i
		}
	}
	slot := func(k string) (int, bool) {
		v, ok := slots[k]
		if !ok {
			return 0, false
		}
		return v.(int), true
	}
	_, hasLaser := slot("laser")

	// ---- target selection
	var target M
	best := 1e18
	recentlyHit := (t - b.sf("last_damage_t")) < 3.0
	curTid, hasCur := b.S["cur_tid"].(float64)
	hf := func(id float64) float64 {
		if v, ok := hitFrom[id]; ok {
			return v
		}
		return -99.0
	}
	ehpOf := func(e M) float64 { return getOr(e, "hp", 1400) / math.Max(getOr(e, "max_hp", 1400), 1) }
	for _, e := range enemies {
		if pb("pacifist") {
			break
		}
		ed := mf(e, "_d")
		epos := vec(e["pos"])
		eid := num(e["id"])
		if pb("stealth") && ed > pf("stealth_fight_r") && !recentlyHit {
			continue
		}
		if t < pf("powerup_until") && ed > 120 && !recentlyHit {
			continue
		}
		if ed > pf("engage_r") {
			continue
		}
		if pb("passive") && ed > pf("close_r") && !recentlyHit {
			continue
		}
		evel := vec(e["vel"])
		eaim := vec(e["aim"])
		var threat bool
		if pb("reactive") {
			be2me := math.Atan2(y-epos[1], x-epos[0])
			aimed := math.Abs(wrap(eaim[0]-be2me)) < 0.22
			rx, ry := x-epos[0], y-epos[1]
			closing := (evel[0]*rx + evel[1]*ry) / math.Max(ed, 1.0)
			ehp := ehpOf(e)
			if !(ed < pf("react_r") || t-hf(eid) < 6.0 ||
				(closing > 3 && ed < 180) || (ehp < 0.4 && ed < 160) ||
				(aimed && ed < 190) || (hasCur && eid == curTid)) {
				continue
			}
		}
		if pb("threat_pri") || pf("pace_s") != 0 {
			be2me := math.Atan2(y-epos[1], x-epos[0])
			rx, ry := x-epos[0], y-epos[1]
			closing := (evel[0]*rx + evel[1]*ry) / math.Max(ed, 1.0)
			threat = t-hf(eid) < 6.0 ||
				(math.Abs(wrap(eaim[0]-be2me)) < 0.22 && (closing > 1 || ed < 120)) ||
				ed < 45
			ehp := ehpOf(e)
			if pf("pace_s") != 0 && !threat && ehp > 0.4 && !(hasCur && eid == curTid) &&
				t-b.sf("last_kill_t") < pf("pace_s") && mf(me, "hp")/math.Max(maxhp, 1) < pf("pace_hp") {
				continue
			}
		}
		ez := epos[2] + 4.5
		vis := w.Los(muzzle, []float64{epos[0], epos[1], ez}, 1.5)
		e["_los"] = vis
		sc := ed*0.5 + 0.0 + getOr(e, "hp", 1400)*0.08
		if !vis {
			sc += 120
		}
		if pb("g_target") {
			rxg, ryg := x-epos[0], y-epos[1]
			clg := (evel[0]*rxg + evel[1]*ryg) / math.Max(ed, 1.0)
			sc = ed + 0.18*getOr(e, "hp", 1400)
			if !vis {
				sc += 240
			}
			if getOr(e, "hp", 1400) <= 0.3*getOr(e, "max_hp", 1400) {
				sc -= 200
			}
			if gid, ok := b.S["g_stick_id"].(float64); ok && eid == gid && t-b.sf("g_stick_t") < 2.0 {
				sc -= 200
			}
			if t-hf(eid) < 4.0 {
				sc -= 40
			}
			if clg > 3 {
				sc -= 15
			}
		}
		if pb("threat_pri") {
			if threat {
				sc += -pf("threat_pri")
			} else {
				sc += pf("threat_pri") * 0.5
			}
		}
		if pb("stick") && hasCur && eid == curTid {
			sc -= pf("stick") + pf("stick_hurt")*(1.0-ehpOf(e))
		}
		if pb("smart_targets") {
			escorts := 0
			for _, o := range enemies {
				if num(o["id"]) != eid && d2(vec(o["pos"]), epos) < 50 {
					escorts++
				}
			}
			sc += float64(escorts) * 40.0
			if mb(e, "charging") {
				sc -= 25.0
			}
			if getOr(e, "hp", 1400) < 0.3*getOr(e, "max_hp", 1400) {
				sc -= 60.0
			}
		}
		thp := ehpOf(e)
		easy := thp < 0.35 || ed < 40 || recentlyHit
		if pb("smart_targets") && b.sb("recovering") && !easy {
			continue
		}
		if pb("smart_targets") && t >= pf("endgame_t") && !easy && ed > 60 {
			continue
		}
		if pb("assassin") {
			be2me := math.Atan2(y-epos[1], x-epos[0])
			if math.Abs(wrap(eaim[0]-be2me)) > math.Pi/2 {
				sc -= 60.0
			}
		}
		if pb("commit") {
			escortsB := 0
			for _, o := range enemies {
				if num(o["id"]) != eid && d2(vec(o["pos"]), epos) < 60 {
					escortsB++
				}
			}
			if escortsB >= 2 && thp >= 0.3 {
				continue
			}
			sc += float64(escortsB) * 30.0
			if hasCur && eid == curTid {
				sc -= 50.0
				if thp < 0.6 {
					sc -= 40.0
				}
			}
			if b.sb("survival") && !easy {
				continue
			}
		}
		if sc < best {
			best, target = sc, e
		}
	}
	if target != nil {
		b.S["g_stick_id"], b.S["g_stick_t"] = num(target["id"]), t
		b.S["cur_tid"] = num(target["id"])
	} else {
		keep := false
		if hasCur {
			if tr, ok := tracks[curTid]; ok && t-mf(tr, "t") < 5.0 {
				keep = true
			}
		}
		if keep {
			b.S["cur_tid"] = curTid
		} else {
			b.S["cur_tid"] = nil
		}
	}
	curTid, hasCur = b.S["cur_tid"].(float64)
	calm := (t-b.sf("enemy_seen_t") > 4.0) && (t-b.sf("last_damage_t") > 3.0)

	// ---- zone
	zgoal, circle := b.ZoneGoal(obs)
	zone := mm(obs, "zone")
	timeToShrink := mf(zone, "shrink_start") - t
	inZone := mb(me, "in_zone")
	urgentZone := !inZone || (zgoal != nil && (timeToShrink < d2(zgoal, pos)/9.0+8.0 || t >= mf(zone, "shrink_start")))

	// ---- loot targets
	type lg struct {
		x, y, dd float64
		kind     string
		r        float64
	}
	var lootGoal *lg
	var takeID any
	takeDrop := -1
	reach := num(mm(mm(b.Cfg, "loot"), "pickup_radius"))
	for _, pv := range ma(obs, "pickups") {
		p := pv.(M)
		kind := ms(p, "kind")
		if _, ok := slots[kind]; ok {
			continue
		}
		cur1 := ""
		if len(weps) > 1 && wep(1) != nil {
			cur1 = ms(wep(1), "kind")
		}
		if cur1 != "" && weaponRank[kind] <= weaponRank[cur1]+0.1 {
			continue
		}
		pp := vec(p["pos"])
		dd := d2(pp, pos)
		dz := math.Abs(pp[2] - z)
		if dd < reach-0.3 && dz < 4.0 {
			laserBusy := cur1 == "laser" && mf(wep(1), "charge") > 0
			if !laserBusy {
				takeID = p["id"]
				if cur1 != "" {
					takeDrop = 1
				} else {
					takeDrop = -1
				}
			}
		} else if dd < pf("loot_r") && w.Walkable(pp[0], pp[1]) && math.Abs(pp[2]-w.Ground(pp[0], pp[1])) < 2.0 {
			if lootGoal == nil || dd < lootGoal.dd {
				lootGoal = &lg{pp[0], pp[1], dd, "weapon", 0}
			}
		}
	}
	known := sknown(b)
	seenIDs := map[float64]bool{}
	for _, pv := range ma(obs, "pickups") {
		p := pv.(M)
		pp := vec(p["pos"])
		known[num(p["id"])] = []any{ms(p, "kind"), pp[0], pp[1], pp[2], t}
		seenIDs[num(p["id"])] = true
	}
	for pid, k := range known {
		if !seenIDs[pid] && (d2([]float64{num(k[1]), num(k[2])}, pos) < 38.0 || t-num(k[4]) > 90.0) {
			delete(known, pid)
		}
	}
	var laserGoal *lg
	if !hasLaser {
		for _, k := range known {
			if k[0].(string) != "laser" {
				continue
			}
			pp := []float64{num(k[1]), num(k[2]), num(k[3])}
			dd := d2(pp, pos)
			if dd < pf("laser_hunt_r") && math.Abs(pp[2]-w.Ground(pp[0], pp[1])) < 2.5 && w.Walkable(pp[0], pp[1]) {
				if laserGoal == nil || dd < laserGoal.dd {
					laserGoal = &lg{pp[0], pp[1], dd, "", 0}
				}
			}
		}
	}
	if takeID != nil {
		act["take"] = takeID
		if takeDrop >= 0 {
			act["drop"] = takeDrop
		}
	}
	// ---- zone feasibility
	nc := vec(zone["next_center"])
	ncx, ncy := nc[0], nc[1]
	nr := mf(zone, "next_radius")
	zoneOk := func(px, py float64) bool {
		dn := math.Hypot(px-ncx, py-ncy)
		if dn <= nr*0.85 {
			return true
		}
		return timeToShrink > d2([]float64{px, py}, pos)/9.0+dn/9.0+15.0 && t < mf(zone, "shrink_start")
	}

	var econGoal *econG
	enemySeenT := b.sf("enemy_seen_t")
	if pb("econ") {
		curC, curQ, curN := map[float64]M{}, map[float64]M{}, map[float64]M{}
		for _, c := range ma(obs, "crates") {
			curC[num(c.(M)["id"])] = c.(M)
		}
		for _, c := range ma(obs, "cubes") {
			curQ[num(c.(M)["id"])] = c.(M)
		}
		for _, n := range npcs {
			curN[num(n["id"])] = n
		}
		memCrates, memCubes, memNpcs := smapMem(b, "mem_crates"), smapMem(b, "mem_cubes"), smapMem(b, "mem_npcs")
		for _, pr := range []struct {
			mem map[float64]memEnt
			cur map[float64]M
		}{{memCrates, curC}, {memCubes, curQ}, {memNpcs, curN}} {
			for i, o := range pr.cur {
				pr.mem[i] = memEnt{vec(o["pos"]), t}
			}
			for i, me_ := range pr.mem {
				if _, ok := pr.cur[i]; !ok && d2(me_.pos, pos) < 22 {
					delete(pr.mem, i)
				}
			}
		}
		for i, v := range memNpcs {
			if t-v.t > 40 {
				delete(memNpcs, i)
			}
		}
		hurt := hpFrac <= 0.45
		mechNear := (t - enemySeenT) < 0.3
		var bestE *econG
		bu := 0.0
		lootBad := smapF(b, "loot_bad")
		lootID, lootIDok := b.S["loot_id"].(float64)
		cons := func(kind string, i float64, p []float64, v float64) {
			if lb, ok := lootBad[i]; ok && lb > t {
				return
			}
			dd := d2(p, pos)
			if dd > pf("econ_r") || (hurt && dd > 35 && kind != "npc") {
				return
			}
			if !zoneOk(p[0], p[1]) || p[2]-w.Ground(p[0], p[1]) > 3.2 {
				return
			}
			if pf("avoid_r") > 0 && kind != "cube" {
				for _, e := range enemies {
					if d2(vec(e["pos"]), p) < pf("avoid_loot") {
						return
					}
				}
			}
			mult := 1.0
			if lootIDok && i == lootID {
				mult = 1.35
			}
			u := v / (dd + 18.0) * mult
			if u > bu {
				bu, bestE = u, &econG{kind: kind, id: i, p: p, dd: dd}
			}
		}
		for i, e := range memCubes {
			cons("cube", i, e.pos, pf("econ_v_cube"))
		}
		if pb("crates") {
			for i, e := range memCrates {
				cons("crate", i, e.pos, pf("econ_v_crate"))
			}
		}
		if hpFrac > pf("econ_npc_hp") && !mechNear {
			for i, e := range memNpcs {
				off := 0.0
				if t-e.t > 12 {
					off = 15.0
				}
				cons("npc", i, e.pos, pf("econ_v_npc")-off)
			}
		}
		if bestE != nil {
			if !lootIDok || bestE.id != lootID {
				b.S["loot_id"], b.S["loot_since"] = bestE.id, t
			} else if t-b.sf("loot_since") > pf("econ_give_up") {
				lootBad[bestE.id] = t + 60.0
				b.S["loot_id"] = nil
				bestE = nil
			}
		} else {
			b.S["loot_id"] = nil
		}
		econGoal = bestE
		if econGoal != nil {
			switch econGoal.kind {
			case "crate":
				econGoal.obj = curC[econGoal.id]
			case "npc":
				econGoal.obj = curN[econGoal.id]
			default:
				econGoal.obj = curQ[econGoal.id]
			}
		}
	}

	var cubeGoal *lg
	for _, cv := range ma(obs, "cubes") {
		c := cv.(M)
		cp := vec(c["pos"])
		dd := d2(cp, pos)
		if dd < pf("cube_r") && math.Abs(cp[2]-w.Ground(cp[0], cp[1])) < 2.5 && w.Walkable(cp[0], cp[1]) && zoneOk(cp[0], cp[1]) {
			if cubeGoal == nil || dd < cubeGoal.dd {
				cubeGoal = &lg{cp[0], cp[1], dd, "", 0}
			}
		}
	}
	ignoreCrates := smapF(b, "ignore_crates")
	var crateTgt M
	crateDD := 0.0
	for _, cv := range ma(obs, "crates") {
		c := cv.(M)
		cp := vec(c["pos"])
		dd := d2(cp, pos)
		lim := pf("crate_r")
		if t < pf("powerup_until") {
			lim = pf("powerup_r")
		}
		if dd < lim && math.Abs(cp[2]-w.Ground(cp[0], cp[1])) < 3.0 && ignoreCrates[num(c["id"])] < 120 && zoneOk(cp[0], cp[1]) {
			if crateTgt == nil || dd < crateDD {
				crateTgt, crateDD = c, dd
			}
		}
	}
	var npcTgt M
	npcDD := 0.0
	if hpFrac > pf("npc_hp") {
		for _, n := range npcs {
			np := vec(n["pos"])
			dd := d2(np, pos)
			lim := pf("npc_r")
			if t < pf("powerup_until") {
				lim = math.Max(pf("npc_r"), 200.0)
			}
			if dd < lim && zoneOk(np[0], np[1]) {
				if npcTgt == nil || dd < npcDD {
					npcTgt, npcDD = n, dd
				}
			}
		}
	}
	var dropGoal []float64 // x, y, dd, radius
	if pb("drops") {
		for _, dv := range ma(obs, "drops") {
			dr := dv.(M)
			dp := vec(dr["pos"])
			dd := d2(dp, pos)
			if dr["holder"] != nil && num(dr["holder"]) != meID {
				continue
			}
			if dd < pf("drop_r") && zoneOk(dp[0], dp[1]) && mf(dr, "lands_in") < dd/10.0+6.0 {
				lim := 70.0
				if pb("drop_heal") {
					lim = 45
				}
				near := false
				for _, e := range enemies {
					if d2(vec(e["pos"]), dp) < lim {
						near = true
						break
					}
				}
				if !near && (dropGoal == nil || dd < dropGoal[2]) {
					dropGoal = []float64{dp[0], dp[1], dd, getOr(dr, "radius", 8.0)}
				}
			}
		}
	}

	// ---- threat model
	var threats []M
	for _, e := range enemies {
		if mf(e, "_d") < 200 {
			threats = append(threats, e)
		}
	}
	var curThreats []M
	for _, e := range threats {
		if mf(e, "_d") < 150 {
			curThreats = append(curThreats, e)
		}
	}
	b.S["cur_threats"] = curThreats
	underFire := (t - b.sf("last_damage_t")) < 1.5
	nThreats := len(threats)
	myHp := mf(me, "hp")
	strongest := 0.0
	for i, e := range threats {
		if v := getOr(e, "hp", 1400); i == 0 || v > strongest {
			strongest = v
		}
	}
	eyesOnUs := 0
	for _, e := range threats {
		ep := vec(e["pos"])
		be2me := math.Atan2(y-ep[1], x-ep[0])
		if mf(e, "_d") < 150 && math.Abs(wrap(vec(e["aim"])[0]-be2me)) < 0.35 {
			eyesOnUs++
		}
	}
	converging := pb("smart_targets") && float64(eyesOnUs) >= pf("converge_n")
	if pb("gang_escape") {
		hb, _ := b.S["hits_by"].([][2]float64)
		var nh [][2]float64
		ids := map[float64]bool{}
		for _, h := range hb {
			if t-h[0] < pf("gang_window") {
				nh = append(nh, h)
				ids[h[1]] = true
			}
		}
		b.S["hits_by"] = nh
		if float64(len(ids)) >= pf("gang_n") {
			b.S["gang_until"] = t + pf("gang_hold")
		}
		if t < b.sf("gang_until") && len(threats) > 0 {
			converging = true
		}
	}
	if pb("g_outgun") && target != nil && hpFrac < 0.8 && mf(target, "_d") > 55 {
		if !hasLaser {
			for _, wk := range ma(target, "weapons") {
				if s, _ := wk.(string); s == "laser" {
					converging = true
				}
			}
		}
	}
	if pb("g_swamp") {
		crowd := 0
		for _, e := range enemies {
			if l, _ := e["_los"].(bool); l && (mf(e, "_d") < 150 || t-hf(num(e["id"])) < 4.0) {
				crowd++
			}
		}
		if crowd >= 3 || (crowd == 2 && hpFrac < 0.65) {
			converging = true
		}
	}
	rh := pf("retreat_hp")
	if pb("solo_duel") && nThreats <= 1 {
		cnt := 0
		for _, e := range enemies {
			if mf(e, "_d") < 200 {
				cnt++
			}
		}
		if cnt <= 1 {
			rh = math.Min(rh, pf("solo_hp"))
		}
	}
	hasThr := len(threats) > 0
	losing := converging || (pb("pacifist") && hasThr) || (pb("stealth") && hasThr && target == nil) ||
		(hasThr && (hpFrac < rh || (hpFrac < pf("retreat_hp_multi") && (nThreats >= 2 || strongest > myHp*1.3))))
	if pb("lose_rule") && hasThr {
		nearN := 0
		for _, e := range enemies {
			if mf(e, "_d") < 160 {
				nearN++
			}
		}
		lr := (nearN >= 2 && hpFrac < 0.55) || (nearN >= 3 && hpFrac < 0.8)
		if target != nil {
			tf := ehpOf(target)
			if hpFrac < 0.35 && tf > hpFrac+0.1 {
				lr = true
			}
			if tf < 0.25 && hpFrac > 0.2 && nearN <= 2 {
				lr = false
				losing = false
			}
		}
		losing = losing || lr
	}
	nearEnemy := 999.0
	for _, e := range enemies {
		nearEnemy = math.Min(nearEnemy, mf(e, "_d"))
	}
	healTo := pf("heal_to")
	if b.sb("survival") {
		healTo = 0.85
	}
	healMode := hpFrac < healTo && !hasThr && (t-b.sf("last_damage_t")) > 0.5
	if losing {
		b.S["recovering"] = true
	} else if b.sb("recovering") && hpFrac >= pf("recover_hp") {
		b.S["recovering"] = false
	}

	// ---- power position
	var ppRes M
	noSteer := false
	if pb("power_pos") && !(urgentZone && zgoal != nil) {
		ppRes = b.PowerPosition(obs, me, circle, target, enemies, t, hpFrac, muzzle)
	} else if b.S["pp"] != nil {
		b.S["pp"] = nil
	}

	// ---- movement decision
	move := []float64{0.0, 0.0}
	boost := false
	jump := false
	speedScale := 1.0
	label := ""
	var farmKind string
	var farmObj M
	energy := mf(me, "energy")

	regenDefend := false
	if pb("regen") {
		fin := target != nil && ehpOf(target) < pf("regen_fin") && mf(target, "_d") < 120
		if !b.sb("regen_on") && hpFrac < pf("regen_hp") && !fin {
			b.S["regen_on"] = true
		}
		if b.sb("regen_on") && (hpFrac >= pf("regen_to") || fin) {
			b.S["regen_on"] = false
		}
		regenDefend = b.sb("regen_on") && underFire && nearEnemy < 60 && hpFrac > 0.35
	}
	if urgentZone && zgoal != nil {
		label = "zone"
		move = b.GoTo(me, b.Zone, zgoal, t)
		boost = !inZone || (timeToShrink < d2(zgoal, pos)/11.0+3) || (t >= mf(zone, "shrink_start"))
	} else if ppRes != nil && mb(ppRes, "hold") && !losing {
		label = ms(ppRes, "label")
		move, boost, jump = ppRes["move"].([]float64), mb(ppRes, "boost"), mb(ppRes, "jump")
		noSteer = true
	} else if b.sb("regen_on") && !regenDefend {
		var seenBy []M
		for _, e := range enemies {
			ep := vec(e["pos"])
			if mf(e, "_d") < 220 && w.Los([]float64{ep[0], ep[1], ep[2] + 5.0}, []float64{x, y, z + 3.5}, 1.5) {
				seenBy = append(seenBy, e)
			}
		}
		if len(seenBy) > 0 {
			label = "regen-hide"
			move = b.RetreatVector(me, seenBy, circle)
			boost = energy > 25
		} else {
			label = "regen"
			move = []float64{0.0, 0.0}
		}
	} else if losing && pb("cover_retreat") {
		if t-b.sf("cover_t") > 0.5 {
			b.S["cover_t"] = t
			b.S["cover_spot_"] = b.CoverSpot(me, threats, circle)
		}
		sp, _ := b.S["cover_spot_"].([]float64)
		if sp == nil {
			if target != nil && pb("cornered_fight") {
				label = fmt.Sprintf("fight %v cornered", target["id"])
				move = b.FightVector(me, target, slots, weps, circle, -1)
			} else {
				label = "retreat"
				move = b.RetreatVector(me, threats, circle)
				boost = energy > 25
			}
		} else if d2(sp, pos) < 2.0 {
			label = "retreat cover-hold"
			move = []float64{0.0, 0.0}
		} else {
			label = "retreat cover"
			move = b.GoTo(me, b.Local, sp, t)
			boost = energy > 10
		}
	} else if losing {
		label = "retreat"
		move = b.RetreatVector(me, threats, circle)
		boost = energy > 25
	} else if pb("heal_first") && healMode && target == nil {
		label = "heal"
		if pb("sit_heal") {
			move = []float64{0.0, 0.0}
		} else {
			hide := b.PickHide(obs, circle)
			if d2(hide, pos) > 3.0 {
				move = b.GoTo(me, b.Zone, hide, t)
				speedScale = 0.4
			}
		}
	} else if pf("cube_fight_r") > 0 && cubeGoal != nil && cubeGoal.dd < pf("cube_fight_r") && hpFrac > pf("cube_fight_hp") {
		label = "cube-grab"
		move = b.GoTo(me, b.Local, []float64{cubeGoal.x, cubeGoal.y}, t)
		boost = cubeGoal.dd > 12 && energy > 30
	} else if pb("laser_grab") && laserGoal != nil && laserGoal.dd < pf("laser_grab_r") && hpFrac > 0.3 {
		label = "grab-laser"
		move = b.GoTo(me, b.Local, []float64{laserGoal.x, laserGoal.y}, t)
		boost = laserGoal.dd > 12 && energy > 30
	} else if laserGoal != nil && (target == nil || (mf(target, "_d") > 110 && laserGoal.dd < 70)) && nearEnemy > 70 {
		label = "get-laser"
		move = b.GoTo(me, b.Local, []float64{laserGoal.x, laserGoal.y}, t)
		boost = laserGoal.dd > 20 && energy > 30
	} else if pb("drop_heal") && dropGoal != nil && (t-b.sf("last_damage_t")) > 1.5 && (hpFrac < 0.85 || mf(me, "cubes") < 10) {
		dd := dropGoal[2]
		if dd > 5.0 {
			label = "drop-heal"
			move = b.GoTo(me, b.Local, []float64{dropGoal[0], dropGoal[1]}, t)
			boost = dd > 15 && energy > 30
		} else {
			label = "capture-move"
			a := math.Atan2(y-dropGoal[1], x-dropGoal[0]) + math.Pi/2
			move = []float64{0.5 * math.Cos(a), 0.5 * math.Sin(a)}
		}
	} else if target != nil {
		label = fmt.Sprintf("fight %v", target["id"])
		move = b.FightVector(me, target, slots, weps, circle, -1)
		pushing := false
		tl, tlOk := target["_los"].(bool)
		if !tlOk {
			tl = true
		}
		if !tl {
			tv := vec(target["vel"])
			tp := vec(target["pos"])
			rx, ry := x-tp[0], y-tp[1]
			rn := math.Hypot(rx, ry)
			if rn == 0 {
				rn = 1.0
			}
			closing := (tv[0]*rx + tv[1]*ry) / rn
			pushing = closing > 1.5 || mf(target, "_d") < 55
		}
		if pb("hold_angles") && !tl && pushing && hpFrac > 0.3 {
			label += " hold"
			move = []float64{0.0, 0.0}
		} else if pb("hunt_hidden") && !tl {
			pk := b.PeekPoint(me, target, muzzle, t)
			if pk != nil {
				label += " peek"
				move = b.GoTo(me, b.Local, pk, t)
			}
		}
		if pb("aggressive") {
			charging := false
			if li, ok := slot("laser"); ok {
				charging = mf(wep(li), "charge") > 0
			}
			boost = pb("fight_boost") && energy > 45 && !charging && (int(t*2.0)%3 == 0)
		}
		if pb("reload_cover") && mf(target, "_d") < 180 {
			reloading := false
			if gi, ok := slot("gun"); ok {
				reloading = mf(wep(gi), "reload") > 0.4
			}
			_, hasLight := slot("lightning")
			otherReady := (hasLight && mf(target, "_d") < 44 && energy > 10)
			if li, ok := slot("laser"); ok && mf(wep(li), "cooldown") < 0.3 {
				otherReady = true
			}
			if reloading && !otherReady {
				label += " reload-cover"
				th := threats
				if len(th) == 0 {
					th = []M{target}
				}
				move = b.RetreatVector(me, th, circle)
			}
		}
	} else if econGoal != nil {
		p := econGoal.p
		dd := econGoal.dd
		label = "econ " + econGoal.kind
		obj := econGoal.obj
		if econGoal.kind == "npc" && obj != nil && dd < 75 && w.Los(muzzle, []float64{p[0], p[1], p[2] + 1.75}, 1.5) {
			move = b.FightVector(me, obj, slots, weps, circle, pf("npc_stand"))
			farmKind, farmObj = "npc", obj
		} else if econGoal.kind == "crate" && obj != nil && dd < 50 && w.Los(muzzle, []float64{p[0], p[1], p[2] + 1.2}, 1.5) {
			farmKind, farmObj = "crate", obj
			if dd < 12 {
				move = []float64{(x - p[0]) / math.Max(dd, 0.1), (y - p[1]) / math.Max(dd, 0.1)}
			}
		} else {
			move = b.GoTo(me, b.Local, []float64{p[0], p[1]}, t)
			boost = pb("econ_boost") && dd > 40 && energy > 40 && t-enemySeenT > 3
		}
	} else if ppRes != nil {
		label = ms(ppRes, "label")
		move, boost = ppRes["move"].([]float64), mb(ppRes, "boost")
	} else if hs, _ := b.S["heard"].([]float64); pb("ghost_hunt") && !pb("stealth") && !b.sb("survival") && t >= pf("powerup_until") && ghost != nil && hpFrac > 0.55 && !healMode &&
		zoneOk(vec(ghost["pos"])[0], vec(ghost["pos"])[1]) {
		label = fmt.Sprintf("stalk %v", ghost["id"])
		pk := b.PeekPoint(me, ghost, muzzle, t)
		if pk == nil {
			gp := vec(ghost["pos"])
			pk = []float64{gp[0], gp[1]}
		}
		move = b.GoTo(me, b.Local, pk, t)
		farmKind, farmObj = "ghost", ghost
		_ = hs
	} else if hs, _ := b.S["heard"].([]float64); pb("hunt_hidden") && !pb("stealth") && !b.sb("survival") && t >= pf("powerup_until") && hs != nil && t-hs[2] < 3.0 && hpFrac > 0.5 &&
		!healMode && zoneOk(x+math.Cos(hs[0])*hs[1], y+math.Sin(hs[0])*hs[1]) {
		label = "hunt-sound"
		gx, gy := x+math.Cos(hs[0])*hs[1], y+math.Sin(hs[0])*hs[1]
		move = b.GoTo(me, b.Local, []float64{gx, gy}, t)
	} else if cubeGoal != nil && (nearEnemy > 60 || hpFrac > 0.5) {
		label = "cube"
		move = b.GoTo(me, b.Local, []float64{cubeGoal.x, cubeGoal.y}, t)
		boost = cubeGoal.dd > 30 && energy > 40
	} else if dropGoal != nil && calm {
		dd := dropGoal[2]
		if dd > dropGoal[3]-3.0 {
			label = "drop"
			move = b.GoTo(me, b.Local, []float64{dropGoal[0], dropGoal[1]}, t)
			boost = energy > 30
		} else {
			label = "capture"
			move = []float64{0.0, 0.0}
		}
	} else if healMode {
		label = "heal"
		hide := b.PickHide(obs, circle)
		if d2(hide, pos) > 3.0 {
			move = b.GoTo(me, b.Zone, hide, t)
			speedScale = 0.4
		}
	} else if calm && lootGoal != nil {
		label = "loot " + lootGoal.kind
		move = b.GoTo(me, b.Local, []float64{lootGoal.x, lootGoal.y}, t)
	} else if (calm || (t < pf("powerup_until") && target == nil)) && npcTgt != nil && !pb("stealth") && (!pb("farm_hp_gate") || hpFrac >= 0.9 || t < pf("powerup_until")) {
		n, dd := npcTgt, npcDD
		np := vec(n["pos"])
		label = "npc"
		vis := w.Los(muzzle, []float64{np[0], np[1], np[2] + 1.75}, 1.5)
		stand := pf("npc_stand")
		if dd > stand+12 || !vis {
			move = b.GoTo(me, b.Local, []float64{np[0], np[1]}, t)
		} else if dd < stand-10 {
			ax, ay := x-np[0], y-np[1]
			nn := math.Hypot(ax, ay)
			if nn == 0 {
				nn = 1
			}
			move = []float64{ax / nn, ay / nn}
		} else {
			move = b.FightVector(me, n, slots, weps, circle, stand)
		}
		if vis {
			farmKind, farmObj = "npc", n
		}
	} else if (calm || (t < pf("powerup_until") && target == nil)) && crateTgt != nil && pb("crates") && !pb("stealth") && (!pb("farm_hp_gate") || hpFrac >= 0.9 || t < pf("powerup_until")) {
		c, dd := crateTgt, crateDD
		cp := vec(c["pos"])
		label = "crate"
		vis := w.Los(muzzle, []float64{cp[0], cp[1], cp[2] + 1.2}, 1.5)
		if dd > 35 || !vis {
			move = b.GoTo(me, b.Local, []float64{cp[0], cp[1]}, t)
		}
		if vis && dd < 120 {
			farmKind, farmObj = "crate", c
		}
	} else if pb("assassin") && !b.sb("survival") && t < pf("roam_until") && hpFrac > pf("roam_hp") && !healMode {
		label = "roam"
		rp := b.RoamPoint(obs, circle, t)
		move = b.GoTo(me, b.Zone, rp, t)
	} else if pb("idle_still") && hpFrac >= pf("idle_still_hp") && (zgoal == nil || !urgentZone) {
		label = "wait"
		move = []float64{0.0, 0.0}
	} else {
		hide := b.PickHide(obs, circle)
		dd := d2(hide, pos)
		if dd > 3.0 {
			label = "to-hide"
			move = b.GoTo(me, b.Zone, hide, t)
			if zgoal == nil {
				if dd < 120 && timeToShrink > dd/4.0+10 {
					speedScale = 0.38
				} else {
					speedScale = 1.0
				}
			}
		} else {
			label = "hide"
			move = []float64{0.0, 0.0}
			if pb("hide_shuffle") {
				ph := t*0.9 + b.sf("scan_phase")
				move = []float64{0.4 * math.Cos(ph), 0.4 * math.Sin(ph)}
			}
		}
	}

	hasPre := func(prefixes ...string) bool {
		for _, p := range prefixes {
			if strings.HasPrefix(label, p) {
				return true
			}
		}
		return false
	}
	if pf("avoid_r") > 0 && !(urgentZone && zgoal != nil) && !hasPre("fight", "retreat", "zone") {
		ax, ay := 0.0, 0.0
		for _, e := range enemies {
			ed := mf(e, "_d")
			if ed < pf("avoid_r") {
				k := (pf("avoid_r") - ed) / pf("avoid_r")
				ep := vec(e["pos"])
				ax += (x - ep[0]) / math.Max(ed, 1.0) * k
				ay += (y - ep[1]) / math.Max(ed, 1.0) * k
			}
		}
		if ax != 0 || ay != 0 {
			nx, ny := move[0]+ax*pf("avoid_w"), move[1]+ay*pf("avoid_w")
			if zoneOk(x+nx*10.0, y+ny*10.0) {
				nn := math.Hypot(nx, ny)
				if nn == 0 {
					nn = 1.0
				}
				move = []float64{nx / nn, ny / nn}
			}
		}
	}

	// ---- stealth
	if pb("stealth") && !(urgentZone && zgoal != nil) {
		if hasPre("retreat") {
			boost = boost && (hpFrac < 0.5 || (t-b.sf("last_damage_t")) < 2.0)
		} else if !hasPre("fight") {
			boost = false
			speedScale = math.Min(speedScale, 0.4)
		}
	}
	if pb("quiet") && !(urgentZone && zgoal != nil) && !hasPre("fight", "retreat", "cube-grab", "grab-laser", "drop-heal") {
		boost = false
		speedScale = math.Min(speedScale, pf("quiet_scale"))
	}
	// ---- drone avoidance
	if pb("drone_avoid") && hpFrac < pf("drone_avoid_hp") && !hasPre("npc") && !hasPre("zone") {
		for _, n := range npcs {
			np := vec(n["pos"])
			dn := d2(np, pos)
			if dn < 55.0 {
				ax, ay := x-np[0], y-np[1]
				den := dn
				if den == 0 {
					den = 1
				}
				k := (55.0 - dn) / 55.0 * 1.5 / den
				move = []float64{move[0] + ax*k, move[1] + ay*k}
			}
		}
	}
	// ---- dodging
	if dodge := b.DodgeVector(obs, me, enemies); dodge != nil {
		move = []float64{move[0]*0.4 + dodge[0], move[1]*0.4 + dodge[1]}
		boost = boost || energy > 30
		speedScale = 1.0
		label += " dodge"
	}

	// ---- stuck handling
	lp, _ := b.S["last_pos"].([][3]float64)
	lp = append(lp, [3]float64{t, x, y})
	for len(lp) > 0 && t-lp[0][0] > 1.6 {
		lp = lp[1:]
	}
	mv := math.Hypot(move[0], move[1])
	stuckUntil := b.sf("stuck_until")
	ud, _ := b.S["unstick_dir"].([]float64)
	if noSteer {
		lp = nil
	} else if t < stuckUntil && ud != nil {
		move = []float64{ud[0], ud[1]}
		g := true
		if hasKey(me, "grounded") {
			g = mb(me, "grounded")
		}
		jump = g
		boost = true
	} else if mv > 0.5 && len(lp) > 10 && t-lp[0][0] > 1.4 {
		moved := math.Hypot(x-lp[0][1], y-lp[0][2])
		if moved < 1.5*speedScale {
			sgn := 1.0
			if b.Rng.Intn(2) == 0 {
				sgn = -1.0
			}
			a := math.Atan2(move[1], move[0]) + sgn*(0.8+b.Rng.Float64()*1.2)
			b.S["unstick_dir"] = []float64{math.Cos(a), math.Sin(a)}
			b.S["stuck_until"] = t + 0.8
			stuckUntil = t + 0.8
			lp = nil
			b.Zone.Goal = -1
			b.Local.Goal = -1
		}
	}
	b.S["last_pos"] = lp

	if !(t < stuckUntil) && !noSteer {
		move = b.Steer(me, move, nil)
	}
	move = []float64{move[0] * speedScale, move[1] * speedScale}

	// ---- aiming and firing
	fire := []bool{false, false}
	var aim []float64
	if target != nil {
		aim, fire = b.Shoot(me, target, slots, weps, muzzle, t, false)
	} else if farmKind == "ghost" {
		aim, fire = b.Shoot(me, farmObj, slots, weps, muzzle, t, false)
		li, ok := slot("laser")
		nf := make([]bool, len(fire))
		for i, f := range fire {
			nf[i] = f && ok && li == i
		}
		fire = nf
	} else if farmKind == "npc" {
		aim, fire = b.Shoot(me, farmObj, slots, weps, muzzle, t, true)
	} else if farmKind != "" {
		c := farmObj
		cp := vec(c["pos"])
		fake := M{"pos": []any{cp[0], cp[1], cp[2] - 3.3}, "vel": []any{0.0, 0.0, 0.0}, "kind": "crate"}
		l := b.Lead(me, fake, 252.0, 1.0)
		yaw, pitch, hd := l[0], l[1], l[2]
		aim = []float64{yaw, pitch}
		if gi, ok := slot("gun"); ok && b.Aligned(me, yaw, pitch, hd, 0.8) {
			fire[gi] = true
		}
		if li, ok := slot("lightning"); ok && hd < 40 && math.Abs(wrap(vec(me["aim"])[0]-yaw)) < 0.15 && energy > 40 {
			fire[li] = true
		}
		ignoreCrates[num(c["id"])]++
	}
	if aim == nil {
		aim = b.IdleAim(obs, me, enemies, t, move)
	}
	if hasPre("regen") {
		fire = []bool{false, false}
	}

	// reload when idle and low
	if g, ok := slot("gun"); ok && target == nil {
		gw := wep(g)
		if getOr(gw, "ammo", 12) <= 5 && mf(gw, "reload") == 0 {
			act["reload"] = true
		}
	}
	tb, hasTB := b.S["threat_bearing"].(float64)
	if gi, ok := slot("gun"); pb("blind_fire") && target == nil && hasTB && t-b.sf("threat_t") < 1.5 && ok && !anyTrue(fire) {
		aim = []float64{tb, 0.0}
		nf := append([]bool{}, fire...)
		for len(nf) <= gi {
			nf = append(nf, false)
		}
		nf[gi] = true
		fire = nf
	}
	if pb("retreat_holdfire") && hasPre("retreat") && nearEnemy > 40.0 {
		fire = make([]bool, len(fire))
	}
	act["move"] = move
	act["boost"] = boost
	act["jump"] = jump
	act["aim"] = aim
	act["fire"] = fire
	return act
}

func anyTrue(f []bool) bool {
	for _, v := range f {
		if v {
			return true
		}
	}
	return false
}

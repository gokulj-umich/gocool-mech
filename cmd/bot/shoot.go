package main

import "math"

// shStat mirrors python dict.get(k, def) on a stats map.
func shStat(m M, k string, def float64) float64 {
	if v, ok := m[k]; ok && v != nil {
		return num(v)
	}
	return def
}

func shHit(set any, id any) bool {
	switch s := set.(type) {
	case map[any]bool:
		return s[id]
	case map[string]bool:
		k, _ := id.(string)
		return s[k]
	case map[float64]bool:
		return s[num(id)]
	case map[int]bool:
		return s[int(num(id))]
	case map[any]struct{}:
		_, ok := s[id]
		return ok
	case []any:
		for _, x := range s {
			if x == id || (num(x) == num(id) && num(id) != 0) {
				return true
			}
		}
	}
	return false
}

func (b *Bot) Aligned(me M, yaw, pitch, dist, tolM float64) bool {
	aim := vec(me["aim"])
	eYaw := math.Abs(wrap(aim[0] - yaw))
	eP := math.Abs(aim[1] - pitch)
	tol := math.Atan2(tolM, math.Max(dist, 1.0)) + 0.004
	return eYaw <= tol && eP <= tol*1.5+0.01
}

func (b *Bot) Shoot(me, tgt M, slots M, weps []any, muzzle []float64, t float64, npc bool) ([]float64, []bool) {
	fire := []bool{false, false}
	tp := vec(tgt["pos"])
	mp := vec(me["pos"])
	mAim := vec(me["aim"])
	d := d2(tp, mp)
	add := 4.5
	if npc {
		add = 1.75
	}
	tz := tp[2] + add
	var vis bool
	if v, ok := tgt["_los"]; ok && v != nil {
		vis = num(v) != 0
	} else {
		vis = b.W.Los(muzzle, []float64{tp[0], tp[1], tz}, 1.5)
	}
	var aim []float64
	shortedUntil := num(b.S["shorted_until"])
	li, hasLi := -1, false
	if v, ok := slots["lightning"]; ok && v != nil {
		li, hasLi = int(num(v)), true
	}
	if hasLi {
		lw := weps[li].(M)
		st := mm(lw, "stats")
		rng := shStat(st, "range", 45)
		if d < rng*0.95 && vis {
			yaw := math.Atan2(tp[1]-mp[1], tp[0]-mp[0])
			aim = []float64{yaw, 0.0}
			if math.Abs(wrap(mAim[0]-yaw)) < shStat(st, "cone_deg", 30)*0.4*math.Pi/180 && mf(lw, "cooldown") <= 0.1 && mf(me, "energy") >= shStat(st, "energy", 3) {
				fire[li] = true
			}
		}
	}
	la, hasLa := -1, false
	if v, ok := slots["laser"]; ok && v != nil {
		la, hasLa = int(num(v)), true
	}
	gi, hasGi := -1, false
	if v, ok := slots["gun"]; ok && v != nil {
		gi, hasGi = int(num(v)), true
	}
	laserAimNow := false
	if hasLa && !(hasLi && fire[li]) {
		lw := weps[la].(M)
		st := mm(lw, "stats")
		rng := shStat(st, "range", 280)
		charge := mf(lw, "charge")
		full := shStat(st, "charge", 0.75)
		hold := shStat(st, "hold_s", 2.0)
		L := b.Lead(me, tgt, 1e9, 1.0)
		ly, lp, lhd := L[0], L[1], L[2]
		if d < rng*0.97 && vis && t >= shortedUntil && (!hasLi || d > 30) {
			if mf(lw, "cooldown") <= 0.15 {
				lethal := false
				if pb("lethal_laser") && ms(tgt, "kind") == "mech" && charge >= shStat(st, "min_charge", 0.15) && charge < full-0.12 {
					mfrac := shStat(st, "min_frac", 0.35)
					dmg := shStat(st, "damage", 315) * (mfrac + (1-mfrac)*charge/full)
					cub := mm(b.Cfg, "cubes")
					dmg *= 1 + shStat(cub, "damage_per_cube", 0.04)*math.Min(mf(me, "cubes"), shStat(cub, "stat_cap", 10))
					hp := 1400.0
					if v, ok := tgt["hp"]; ok {
						hp = num(v)
					}
					lethal = dmg*0.7 >= hp
				}
				if lethal {
					laserAimNow = true
					aim = []float64{ly, lp}
					fire[la] = !b.Aligned(me, ly, lp, lhd, 1.2)
				} else if charge < full-0.12 {
					fire[la] = true
				} else {
					laserAimNow = true
					aim = []float64{ly, lp}
					if b.Aligned(me, ly, lp, lhd, 1.4) {
						fire[la] = false
					} else {
						fire[la] = charge < full+hold-0.25
					}
				}
			}
		} else if charge > 0.2 {
			if vis || charge > full+hold-0.3 {
				fire[la] = false
			} else {
				fire[la] = true
			}
		} else if pb("hunt_hidden") && !vis && d < 160 && mf(lw, "cooldown") <= 0.15 && t >= shortedUntil && charge < full {
			fire[la] = true
			aim = []float64{ly, lp}
		}
	}
	if v, ok := slots["mortar"]; ok && v != nil && !npc {
		mo := int(num(v))
		mw := weps[mo].(M)
		if 28 < d && d < 220 && mf(mw, "cooldown") <= 0.1 {
			sol := b.MortarAim(me, tgt, mw)
			if sol != nil && (aim == nil || !vis) {
				aim = []float64{sol[0], sol[1]}
				if b.Aligned(me, sol[0], sol[1], sol[2], 2.0) {
					fire[mo] = true
				}
			}
		}
	}
	if hasGi {
		gw := weps[gi].(M)
		st := mm(gw, "stats")
		spd := shStat(st, "speed", 252)
		rng := shStat(st, "range", 230)
		G := b.Lead(me, tgt, spd, 1.0)
		yaw, pitch, hd := G[0], G[1], G[2]
		if aim == nil || (!laserAimNow && !hasLi) {
			aim = []float64{yaw, pitch}
		}
		ammo := 1.0
		if v, ok := gw["ammo"]; ok {
			ammo = num(v)
		}
		reload := mf(gw, "reload")
		if vis && hd < rng*0.97 && (ammo > 0 || (0 < reload && reload <= 0.1)) && reload <= 0.1 {
			tol := 2.2
			if npc {
				tol = 1.2
			}
			if b.Aligned(me, yaw, pitch, hd, tol) {
				fire[gi] = true
			}
		}
		if pb("backstab_laser_first") && fire[gi] && hasLa && ms(tgt, "kind") == "mech" &&
			!shHit(b.S["hit_ids"], tgt["id"]) &&
			mf(weps[la].(M), "cooldown") <= 0.15 && t >= shortedUntil {
			b2 := math.Atan2(mp[1]-tp[1], mp[0]-tp[0])
			if math.Abs(wrap(vec(tgt["aim"])[0]-b2)) > math.Pi/2 {
				fire[gi] = false
			}
		}
	}
	if aim == nil {
		yaw := math.Atan2(tp[1]-mp[1], tp[0]-mp[0])
		aim = []float64{yaw, 0.0}
	}
	return aim, fire
}

func (b *Bot) DodgeVector(obs, me M, enemies []M) []float64 {
	pos := vec(me["pos"])
	x, y, z := pos[0], pos[1], pos[2]
	_ = z + 4.5
	var best []float64
	for _, e := range enemies {
		if mb(e, "charging") && mf(e, "_d") < 290 {
			bb := math.Atan2(y-vec(e["pos"])[1], x-vec(e["pos"])[0])
			ed := mf(e, "_d")
			if math.Abs(wrap(vec(e["aim"])[0]-bb)) < math.Atan2(6.0, math.Max(ed, 1))+0.03 {
				px, py := -math.Sin(bb), math.Cos(bb)
				s := num(b.S["strafe_dir"])
				best = []float64{px * s, py * s}
			}
		}
	}
	for _, pi := range ma(obs, "projectiles") {
		p, _ := pi.(M)
		pp := vec(p["pos"])
		pv := vec(p["vel"])
		px, py, pz := pp[0], pp[1], pp[2]
		vx, vy, vz := pv[0], pv[1], pv[2]
		if ms(p, "kind") == "mortar" {
			g := 22.0
			if w := mm(mm(b.Cfg, "weapons"), "mortar"); w != nil {
				g = shStat(w, "gravity", 22.0)
			}
			a, bq, c := -0.5*g, vz, pz-z
			disc := bq*bq - 4*a*c
			if disc < 0 {
				continue
			}
			tl := (-bq - math.Sqrt(disc)) / (2 * a)
			if tl < 0 || tl > 4 {
				continue
			}
			lx, ly := px+vx*tl, py+vy*tl
			dd := math.Hypot(lx-x, ly-y)
			if dd < 10 {
				ax, ay := x-lx, y-ly
				n := math.Hypot(ax, ay)
				if n == 0 {
					n = 1
				}
				best = []float64{ax / n, ay / n}
			}
		}
		// other projectiles: v33b disabled per-bullet sidesteps (no-op in python)
	}
	return best
}

func (b *Bot) IdleAim(obs, me M, enemies []M, t float64, move []float64) []float64 {
	mp := vec(me["pos"])
	if len(enemies) == 0 && t < pf("open_scan_s") {
		ss, _ := b.S["scan_start"].([]float64)
		if ss == nil {
			ss = []float64{t, vec(me["aim"])[0]}
			b.S["scan_start"] = ss
		}
		return []float64{wrap(ss[1] + (t-ss[0]+0.3)*2.6), 0.0}
	}
	if len(enemies) > 0 {
		ep := vec(enemies[0]["pos"])
		return []float64{math.Atan2(ep[1]-mp[1], ep[0]-mp[0]), 0.0}
	}
	if tb, ok := b.S["threat_bearing"].(float64); ok && t-num(b.S["threat_t"]) < 4.0 {
		return []float64{tb, 0.0}
	}
	var best *float64
	for _, si := range ma(obs, "sounds") {
		s, _ := si.(M)
		switch ms(s, "kind") {
		case "step", "boost", "jet", "gun", "laser_charge", "lightning", "reload":
			if ms(s, "dist") != "far" {
				v := mf(s, "bearing")
				best = &v
			}
		}
	}
	if best != nil {
		b.S["threat_bearing"] = *best
		b.S["threat_t"] = t - 2.0
		return []float64{*best, 0.0}
	}
	phase := num(b.S["scan_phase"])
	mv := math.Hypot(move[0], move[1])
	if mv > 0.6 {
		base := math.Atan2(move[1], move[0])
		return []float64{base + 0.6*math.Sin(t*1.5+phase), 0.0}
	}
	z := mm(obs, "zone")
	c := vec(z["center"])
	base := math.Atan2(c[1]-mp[1], c[0]-mp[0])
	return []float64{wrap(base + 1.3*math.Sin(t*0.9+phase)), 0.0}
}

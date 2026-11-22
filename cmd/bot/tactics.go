// Tactics: 1:1 port of Bot.spot_vector / fight_vector / power_position / roof_peek_point /
// roam_point / peek_point / cover_spot / retreat_vector from bot.py.
package main

import (
	"math"
	"math/rand"
	"reflect"
	"sort"
)

// ---- local helpers (tx prefix to avoid collisions) ----

func txPos(v any) []float64 {
	switch p := v.(type) {
	case []float64:
		return p
	case []any:
		return vec(p)
	}
	return []float64{0, 0, 0}
}

func txGet(m M, k string, def float64) float64 {
	if v, ok := m[k]; ok && v != nil {
		return num(v)
	}
	return def
}

func txMs(v any) []M {
	switch a := v.(type) {
	case []M:
		return a
	case []any:
		out := make([]M, 0, len(a))
		for _, x := range a {
			if m, ok := x.(M); ok {
				out = append(out, m)
			}
		}
		return out
	}
	return nil
}

func txHas(v any, key string) bool {
	switch a := v.(type) {
	case []string:
		for _, s := range a {
			if s == key {
				return true
			}
		}
	case []any:
		for _, s := range a {
			if x, ok := s.(string); ok && x == key {
				return true
			}
		}
	case M:
		_, ok := a[key]
		return ok
	}
	return false
}

func txDist(e M) float64 { return txGet(e, "_d", 999) }

// lookup float in any map keyed by id (python dict.get(id, def))
func txMapGet(mv any, key any, def float64) float64 {
	if mv == nil || key == nil {
		return def
	}
	r := reflect.ValueOf(mv)
	if r.Kind() != reflect.Map {
		return def
	}
	kv := reflect.ValueOf(key)
	kt := r.Type().Key()
	if kt.Kind() == reflect.Interface {
		// ok
	} else if kv.Type().ConvertibleTo(kt) && kv.Kind() == kt.Kind() {
		kv = kv.Convert(kt)
	} else {
		return def
	}
	val := r.MapIndex(kv)
	if !val.IsValid() {
		return def
	}
	return num(val.Interface())
}

func (b *Bot) tNow() float64 { return num(b.S["t"]) }

// ---- spot_vector ----

func (b *Bot) SpotVector(me, tgt M, slots M, weps []any, want, sx, sy float64) []float64 {
	w := b.W
	mp := txPos(me["pos"])
	x, y, z := mp[0], mp[1], mp[2]
	tp := txPos(tgt["pos"])
	tx, ty, tz := tp[0], tp[1], tp[2]
	mh := txGet(mm(me, "stats"), "muzzle_height", 3.6)
	tc := []float64{tx, ty, tz + 3.0}
	ready := true
	if pb("spot_duck") {
		wep := func(k string) M {
			iv, ok := slots[k]
			if !ok || iv == nil {
				return nil
			}
			i := int(num(iv))
			if i < 0 || i >= len(weps) {
				return nil
			}
			m, _ := weps[i].(M)
			return m
		}
		_, gHas := slots["gun"]
		_, laHas := slots["laser"]
		_, liHas := slots["lightning"]
		gOk := gHas && txGet(wep("gun"), "ammo", 1) > 0 && txGet(wep("gun"), "reload", 0) <= 0.25
		lOk := laHas && txGet(wep("laser"), "cooldown", 0) <= 0.3
		zOk := liHas && txDist(tgt) < 50 && mf(me, "energy") > 15
		ready = gOk || lOk || zOk
	}
	wantLos := 1.0
	if !ready {
		wantLos = -1.0
	}
	var others []M
	for _, e := range txMs(b.S["cur_enemies"]) {
		if e["id"] != tgt["id"] && txGet(e, "_d", 999) < 170 {
			others = append(others, e)
		}
	}
	if len(others) > 4 {
		others = others[:4]
	}
	var cube []float64
	for _, c := range txMs(b.S["cur_cubes"]) {
		cp := txPos(c["pos"])
		dc := d2(cp, []float64{x, y})
		if dc < 40 && (cube == nil || dc < cube[2]) {
			cube = []float64{cp[0], cp[1], dc}
		}
	}
	var bv []float64
	haveBest := false
	best := 0.0
	type cand struct{ dx, dy, r float64 }
	cands := []cand{{0, 0, 0}}
	for k := 0; k < 16; k++ {
		cands = append(cands, cand{math.Cos(float64(k) * math.Pi / 8), math.Sin(float64(k) * math.Pi / 8), 6.0})
	}
	for k := 0; k < 8; k++ {
		cands = append(cands, cand{math.Cos(float64(k)*math.Pi/4 + 0.39), math.Sin(float64(k)*math.Pi/4 + 0.39), 12.0})
	}
	for _, c := range cands {
		dx, dy, r := c.dx, c.dy, c.r
		cx, cy := x+dx*r, y+dy*r
		if r > 0 && (!w.Walkable(cx, cy) || !w.ClearLine([]float64{x, y}, []float64{cx, cy})) {
			continue
		}
		gz := z
		if r > 0 {
			gz = w.Ground(cx, cy)
		}
		s := -1.1*math.Abs(math.Hypot(tx-cx, ty-cy)-want) - r*0.4
		los := w.Los([]float64{cx, cy, gz + mh}, tc, 1.5)
		if los {
			s += pf("spot_los") * wantLos
		}
		body := []float64{cx, cy, gz + 3.0}
		for _, e := range others {
			ep := txPos(e["pos"])
			if w.Los([]float64{ep[0], ep[1], ep[2] + 5.0}, body, 1.5) {
				s -= pf("spot_expo")
			}
		}
		if cube != nil {
			s += 1.5 * (cube[2] - math.Hypot(cube[0]-cx, cube[1]-cy))
		}
		if wantLos > 0 {
			s += 9.0 * (dx*sx + dy*sy)
		}
		if r == 0 {
			s -= 8.0
			if wantLos < 0 && !los {
				s += 14.0
			}
		}
		if !haveBest || s > best {
			haveBest = true
			best, bv = s, []float64{dx, dy}
		}
	}
	if bv == nil {
		return nil
	}
	return []float64{bv[0], bv[1]}
}

// ---- fight_vector ----

func (b *Bot) FightVector(me, tgt M, slots M, weps []any, circle []float64, want float64) []float64 {
	t := b.tNow()
	mp := txPos(me["pos"])
	x, y := mp[0], mp[1]
	tp := txPos(tgt["pos"])
	tx, ty := tp[0], tp[1]
	d := d2([]float64{x, y}, []float64{tx, ty})
	rx, ry := (tx-x)/math.Max(d, 1e-6), (ty-y)/math.Max(d, 1e-6)
	px, py := -ry, rx
	strafeDir := num(b.S["strafe_dir"])
	strafeUntil := num(b.S["strafe_until"])
	aheadX, aheadY := x+px*strafeDir*7.0, y+py*strafeDir*7.0
	if t > strafeUntil || (t > strafeUntil-pf("strafe_max")+0.8 && !b.W.Walkable(aheadX, aheadY)) {
		strafeDir = -strafeDir
		strafeUntil = t + (pf("strafe_min") + rand.Float64()*(pf("strafe_max")-pf("strafe_min")))
		b.S["strafe_dir"] = strafeDir
		b.S["strafe_until"] = strafeUntil
	}
	if want >= 0 {
		// given
	} else if txHas(slots, "lightning") && mf(me, "energy") > 20 {
		want = pf("want_lightning")
	} else if txHas(slots, "laser") {
		want = pf("want_laser")
	} else {
		want = pf("want_gun")
	}
	if pb("aggressive") {
		thp := txGet(tgt, "hp", 1400) / math.Max(txGet(tgt, "max_hp", 1400), 1)
		myhp := mf(me, "hp") / math.Max(mf(me, "max_hp"), 1)
		crowd := 0
		if pb("press_multi_block") {
			for _, e := range txMs(b.S["cur_threats"]) {
				if e["id"] != tgt["id"] {
					crowd++
				}
			}
		}
		if (thp < pf("press_hp") || thp < myhp-pf("press_gap")) && crowd == 0 {
			want = math.Min(want, pf("press_r"))
		}
	}
	their := tgt["weapons"]
	if txHas(their, "lightning") && !txHas(slots, "lightning") {
		want = math.Max(want, 60.0)
	}
	if pf("laser_min_r") != 0 && txHas(their, "laser") && ms(tgt, "kind") == "mech" {
		thp := txGet(tgt, "hp", 1400) / math.Max(txGet(tgt, "max_hp", 1400), 1)
		onCd := (t - txMapGet(b.S["laser_fired"], tgt["id"], -99.0)) < pf("laser_cd_window")
		charging := false
		if c, ok := tgt["charging"]; ok && c != nil {
			if bb, ok := c.(bool); ok {
				charging = bb
			} else {
				charging = num(c) != 0
			}
		}
		if thp > pf("laser_min_finish") && (charging || !onCd) {
			want = math.Max(want, pf("laser_min_r"))
		}
	}
	if pb("standoff") && txHas(slots, "laser") && !txHas(their, "laser") && ms(tgt, "kind") == "mech" &&
		t >= num(b.S["shorted_until"]) {
		thp := txGet(tgt, "hp", 1400) / math.Max(txGet(tgt, "max_hp", 1400), 1)
		if thp > 0.2 {
			want = pf("standoff_r")
		} else {
			want = 120.0
		}
	}
	radial := math.Max(-1.0, math.Min(1.0, (d-want)/30.0))
	mvx := px*strafeDir*0.9 + rx*radial*0.7
	mvy := py*strafeDir*0.9 + ry*radial*0.7
	if pb("fight_spot") && ms(tgt, "kind") == "mech" {
		v := b.SpotVector(me, tgt, slots, weps, want, px*strafeDir, py*strafeDir)
		if v != nil {
			mvx, mvy = v[0], v[1]
		}
	} else if pf("fight_expo") > 0 && ms(tgt, "kind") == "mech" {
		var others []M
		for _, e := range txMs(b.S["cur_enemies"]) {
			if e["id"] != tgt["id"] && txGet(e, "_d", 999) < 170 {
				others = append(others, e)
			}
		}
		if len(others) > 0 {
			var bestV []float64
			haveBest := false
			bestS := 0.0
			for k := 0; k < 16; k++ {
				a := float64(k) * math.Pi / 8
				dx, dy := math.Cos(a), math.Sin(a)
				cx, cy := x+dx*7.0, y+dy*7.0
				if !b.W.Walkable(cx, cy) {
					continue
				}
				s := -1.1*math.Abs(d2([]float64{cx, cy}, []float64{tx, ty})-want) + 12.0*(dx*px*strafeDir+dy*py*strafeDir)
				gz := b.W.Ground(cx, cy)
				body := []float64{cx, cy, gz + 3.5}
				for _, e := range others {
					ep := txPos(e["pos"])
					if b.W.Los([]float64{ep[0], ep[1], ep[2] + 5.0}, body, 1.5) {
						s -= pf("fight_expo")
					}
				}
				if !haveBest || s > bestS {
					haveBest = true
					bestS, bestV = s, []float64{dx, dy}
				}
			}
			if bestV != nil {
				mvx, mvy = bestV[0]*1.1, bestV[1]*1.1
			}
		}
	}
	cx, cy, mr := circle[0], circle[1], circle[3]
	if math.Hypot(x-cx, y-cy) > mr {
		ax, ay := cx-x, cy-y
		n := math.Hypot(ax, ay)
		if n == 0 {
			n = 1
		}
		mvx += ax / n * 0.8
		mvy += ay / n * 0.8
	}
	return []float64{mvx, mvy}
}

// ---- power_position ----

func (b *Bot) txRoofBlack() map[int]float64 {
	rb, ok := b.S["roof_black"].(map[int]float64)
	if !ok {
		rb = map[int]float64{}
		b.S["roof_black"] = rb
	}
	return rb
}

func (b *Bot) PowerPosition(obs, me M, circle []float64, target M, enemies []M, t, hpFrac float64, muzzle []float64) M {
	if !(pf("pp_start") <= t && t < pf("pp_end")) {
		b.S["pp"] = nil
		return nil
	}
	w := b.W
	roofBlack := b.txRoofBlack()
	pp, _ := b.S["pp"].(M)
	mp := txPos(me["pos"])
	if pb("roof_safe") {
		recent2 := 0
		lastSeen := -99.0
		first := true
		if sa := b.S["seen_at"]; sa != nil {
			r := reflect.ValueOf(sa)
			if r.Kind() == reflect.Map {
				it := r.MapRange()
				for it.Next() {
					v := num(it.Value().Interface())
					if t-v < 5.0 {
						recent2++
					}
					if first || v > lastSeen {
						lastSeen = v
						first = false
					}
				}
			}
		}
		if first {
			lastSeen = -99.0
		}
		x_, y_, z_ := mp[0], mp[1], mp[2]
		onAny := w.RoofAt(x_, y_) >= 0 && z_ > w.Ground(x_, y_)+6.0
		_, hasSince := b.S["roof_since"].(float64)
		if onAny && !hasSince {
			b.S["roof_since"] = t
		}
		if !onAny {
			b.S["roof_since"] = nil
		}
		since, hasSince := b.S["roof_since"].(float64)
		if pp != nil && (recent2 >= 2 || (hasSince && t-since > 30.0)) {
			roofBlack[int(num(pp["roof"]))] = t + 60.0
			b.S["pp"] = nil
			return nil
		}
		if pp == nil && (hpFrac < 0.75 || t-lastSeen < 8.0) {
			return nil
		}
	}
	if len(w.Roofs) == 0 {
		return nil
	}
	x, y, z := mp[0], mp[1], mp[2]
	cx, cy, mr := circle[0], circle[1], circle[3]
	if pp != nil {
		k := int(num(pp["roof"]))
		rf := w.Roofs[k]
		bl, ok := roofBlack[k]
		if !ok {
			bl = -1
		}
		if math.Hypot(rf[3]-cx, rf[4]-cy) > math.Max(mr*0.95, 15.0) || bl > t {
			pp = nil
			b.S["pp"] = nil
		}
	}
	if pp == nil {
		bestK := -1
		bestSc := 0.0
		for _, rf := range w.Roofs {
			k := int(rf[0])
			bl, ok := roofBlack[k]
			if !ok {
				bl = -1
			}
			if bl > t {
				continue
			}
			if math.Hypot(rf[3]-cx, rf[4]-cy) > mr*0.8 {
				continue
			}
			dd := math.Hypot(rf[3]-x, rf[4]-y)
			var sc float64
			if pb("roof_safe") {
				if dd > 140.0 {
					continue
				}
				sc = dd + math.Abs(rf[2]-18.0)*3.0 - math.Min(rf[5], 3000)*0.01 + math.Hypot(rf[3]-cx, rf[4]-cy)*0.3
			} else {
				sc = dd + rf[2]*0.5 - math.Min(rf[5], 3000)*0.004
			}
			if bestK < 0 || sc < bestSc {
				bestSc, bestK = sc, k
			}
		}
		if bestK < 0 {
			return nil
		}
		k := bestK
		var cpBest []float64
		cpD := 0.0
		for _, row := range w.Climb {
			if int(row[0]) != k {
				continue
			}
			dd := math.Hypot(row[1]-x, row[2]-y)
			if cpBest == nil || dd < cpD {
				cpBest, cpD = row, dd
			}
		}
		if cpBest == nil {
			roofBlack[k] = t + 120
			return nil
		}
		pp = M{"roof": k, "state": "travel", "t0": t, "climb": []float64{cpBest[1], cpBest[2]}}
		b.S["pp"] = pp
	}
	k := int(num(pp["roof"]))
	rf := w.Roofs[k]
	top := rf[1]
	ctr := []float64{rf[3], rf[4]}
	onRoof := w.RoofAt(x, y) == k && z > top-1.5
	res := M{"move": []float64{0.0, 0.0}, "boost": false, "jump": false, "hold": false, "label": ""}
	grounded := true
	if g, ok := me["grounded"]; ok && g != nil {
		grounded = num(g) != 0
	}
	if onRoof {
		pp["state"] = "on"
		res["hold"] = true
		gi := -1
		wl := ma(me, "weapons")
		for i, wpv := range wl {
			if wp, ok := wpv.(M); ok && wp != nil && ms(wp, "kind") == "gun" {
				gi = i
			}
		}
		reloading := false
		if gi >= 0 {
			if wp, ok := wl[gi].(M); ok {
				reloading = txGet(wp, "reload", 0) > 0.3
			}
		}
		tgt := target
		if tgt == nil && len(enemies) > 0 {
			tgt = enemies[0]
		}
		var goal []float64
		if tgt != nil && hpFrac > 0.35 && !reloading {
			pk := b.RoofPeekPoint(me, k, top, tgt, t)
			if pk != nil {
				goal = pk
			} else {
				goal = ctr
			}
			res["label"] = "roof-peek"
		} else {
			goal = ctr
			res["label"] = "roof-hold"
		}
		vx, vy := goal[0]-x, goal[1]-y
		n := math.Hypot(vx, vy)
		if n > 1.5 {
			sp := 0.5
			if n > 6 {
				sp = 1.0
			}
			res["move"] = []float64{vx / n * sp, vy / n * sp}
		}
		return res
	}
	if ms(pp, "state") == "climb" {
		if t-num(pp["t0"]) > pf("pp_climb_s") {
			roofBlack[k] = t + 90
			b.S["pp"] = nil
			return nil
		}
		res["hold"] = true
		res["label"] = "climb"
		vx, vy := ctr[0]-x, ctr[1]-y
		n := math.Hypot(vx, vy)
		if n == 0 {
			n = 1.0
		}
		res["move"] = []float64{vx / n, vy / n}
		if grounded {
			res["jump"] = true
		}
		res["boost"] = z < top+1.5 || !grounded
		return res
	}
	cp, _ := pp["climb"].([]float64)
	dcp := math.Hypot(cp[0]-x, cp[1]-y)
	if dcp < 3.5 && mf(me, "energy") >= math.Min(95.0, 30.0+(top-z)*1.8) {
		pp["state"] = "climb"
		pp["t0"] = t
		res["hold"] = true
		res["label"] = "climb"
		res["jump"] = true
		return res
	}
	if t-num(pp["t0"]) > 60.0 {
		roofBlack[k] = t + 90
		b.S["pp"] = nil
		return nil
	}
	res["label"] = "to-roof"
	if dcp < 3.5 {
		res["label"] = "to-roof wait-energy"
		return res
	}
	res["move"] = b.GoTo(me, b.Local, cp, t)
	return res
}

// ---- roof_peek_point ----

func (b *Bot) RoofPeekPoint(me M, k int, top float64, tgt M, t float64) []float64 {
	if rp, ok := b.S["roof_peek"].([]any); ok && rp != nil && rp[2] == tgt["id"] && t < num(rp[3]) {
		return []float64{num(rp[0]), num(rp[1])}
	}
	w := b.W
	mp := txPos(me["pos"])
	x, y := mp[0], mp[1]
	mh := txGet(mm(me, "stats"), "muzzle_height", 7.0)
	tg := txPos(tgt["pos"])
	tp := []float64{tg[0], tg[1], tg[2] + 4.5}
	base := math.Atan2(tp[1]-y, tp[0]-x)
	tau := 2 * math.Pi
	var best []float64
	for _, ring := range []float64{0.0, 4.0, 8.0, 12.0, 18.0, 26.0} {
		n := 10
		if ring == 0 {
			n = 1
		}
		for kk := 0; kk < n; kk++ {
			a := base + float64(kk-5)*(tau/20)
			px, py := x+math.Cos(a)*ring, y+math.Sin(a)*ring
			if w.RoofAt(px, py) != k {
				continue
			}
			if w.Los([]float64{px, py, top + mh}, tp, 2.0) {
				best = []float64{px, py}
				break
			}
		}
		if best != nil {
			break
		}
	}
	if best == nil {
		b.S["roof_peek"] = nil
		return nil
	}
	b.S["roof_peek"] = []any{best[0], best[1], tgt["id"], t + 0.8}
	return best
}

// ---- roam_point ----

func (b *Bot) RoamPoint(obs M, circle []float64, t float64) []float64 {
	cx, cy, mr := circle[0], circle[1], circle[3]
	me := txPos(mm(obs, "self")["pos"])
	if rm, ok := b.S["_roam"].([]float64); ok && rm != nil && t < rm[2] && d2(rm, me) > 15.0 {
		return []float64{rm[0], rm[1]}
	}
	var best []float64
	for k := 0; k < 10; k++ {
		a := -math.Pi + rand.Float64()*(2*math.Pi)
		rr := (0.15 + rand.Float64()*(0.75-0.15)) * math.Max(mr, 20.0)
		px, py := cx+math.Cos(a)*rr, cy+math.Sin(a)*rr
		if !b.W.Walkable(px, py) {
			continue
		}
		dd := d2([]float64{px, py}, me)
		sc := math.Abs(dd - 120.0)
		if best == nil || sc < best[0] {
			best = []float64{sc, px, py}
		}
	}
	if best == nil {
		best = []float64{0, cx, cy}
	}
	b.S["_roam"] = []float64{best[1], best[2], t + 25.0}
	return []float64{best[1], best[2]}
}

// ---- peek_point ----

func (b *Bot) PeekPoint(me, tgt M, muzzle []float64, t float64) []float64 {
	if pk, ok := b.S["peek"].([]any); ok && pk != nil && pk[2] == tgt["id"] && t < num(pk[3]) {
		return []float64{num(pk[0]), num(pk[1])}
	}
	mp := txPos(me["pos"])
	x, y := mp[0], mp[1]
	w := b.W
	mh := txGet(mm(me, "stats"), "muzzle_height", 7.0)
	tg := txPos(tgt["pos"])
	tp := []float64{tg[0], tg[1], tg[2] + 4.5}
	var best []float64
	base := math.Atan2(tg[1]-y, tg[0]-x)
	tau := 2 * math.Pi
	for _, ring := range []float64{8.0, 16.0, 26.0, 38.0} {
		for k := 0; k < 12; k++ {
			a := base + float64(k-6)*(tau/12)
			px, py := x+math.Cos(a)*ring, y+math.Sin(a)*ring
			if !w.Walkable(px, py) {
				continue
			}
			pz := w.Ground(px, py) + mh
			if w.Los([]float64{px, py, pz}, tp, 2.0) {
				sc := ring + math.Abs(wrap(a-base))*4.0
				if best == nil || sc < best[0] {
					best = []float64{sc, px, py}
				}
			}
		}
		if best != nil {
			break
		}
	}
	if best == nil {
		b.S["peek"] = nil
		return nil
	}
	b.S["peek"] = []any{best[1], best[2], tgt["id"], t + 1.0}
	return []float64{best[1], best[2]}
}

// ---- cover_spot ----

func (b *Bot) CoverSpot(me M, threats []M, circle []float64) []float64 {
	mp := txPos(me["pos"])
	x, y := mp[0], mp[1]
	w := b.W
	cx, cy, r, mr := circle[0], circle[1], circle[2], circle[3]
	ths := append([]M(nil), threats...)
	sort.SliceStable(ths, func(i, j int) bool { return mf(ths[i], "_d") < mf(ths[j], "_d") })
	if len(ths) > 4 {
		ths = ths[:4]
	}
	hidden := func(px, py float64) bool {
		g := w.Ground(px, py)
		for _, e := range ths {
			ep := txPos(e["pos"])
			ez := []float64{ep[0], ep[1], ep[2] + 5.0}
			if w.Los(ez, []float64{px, py, g + 5.5}, 2.0) || w.Los(ez, []float64{px, py, g + 2.5}, 2.0) {
				return false
			}
		}
		return true
	}
	if hidden(x, y) {
		return []float64{x, y}
	}
	var best []float64
	bs := 1e18
	tau := 2 * math.Pi
	for _, ring := range []float64{5.0, 10.0, 16.0, 24.0, 34.0, 46.0, 60.0} {
		if ring > pf("cover_max") {
			break
		}
		for k := 0; k < 20; k++ {
			a := float64(k) * tau / 20
			px, py := x+math.Cos(a)*ring, y+math.Sin(a)*ring
			if !w.Walkable(px, py) || math.Hypot(px-cx, py-cy) > math.Max(r-5, mr) {
				continue
			}
			if math.Abs(w.Ground(px, py)-w.Ground(x, y)) > 3.0 {
				continue
			}
			sc := ring
			if !w.ClearLine([]float64{x, y}, []float64{px, py}) {
				sc = ring * 1.8
			}
			sum := 0.0
			for _, e := range ths {
				sum += d2([]float64{px, py}, txPos(e["pos"])) - mf(e, "_d")
			}
			sc -= 0.15 * sum
			if sc < bs && hidden(px, py) {
				bs, best = sc, []float64{px, py}
			}
		}
		if best != nil && ring >= 10.0 {
			break
		}
	}
	return best
}

// ---- retreat_vector ----

func (b *Bot) RetreatVector(me M, threats []M, circle []float64) []float64 {
	mp := txPos(me["pos"])
	x, y := mp[0], mp[1]
	w := b.W
	cx, cy, r, mr := circle[0], circle[1], circle[2], circle[3]
	best := -1e18
	var bv []float64
	tau := 2 * math.Pi
	th := threats
	if len(th) > 4 {
		th = th[:4]
	}
	for _, ring := range []float64{14.0, 28.0, 45.0} {
		for k := 0; k < 16; k++ {
			a := float64(k) * tau / 16
			px, py := x+math.Cos(a)*ring, y+math.Sin(a)*ring
			if !w.Walkable(px, py) {
				continue
			}
			pz := w.Ground(px, py) + 5.0
			hidden := 0
			for _, e := range th {
				ep := txPos(e["pos"])
				if !w.Los([]float64{ep[0], ep[1], ep[2] + 8.0}, []float64{px, py, pz}, 2.0) {
					hidden++
				}
			}
			away := 0.0
			for _, e := range th {
				away += d2([]float64{px, py}, txPos(e["pos"])) - mf(e, "_d")
			}
			inz := math.Hypot(px-cx, py-cy) <= math.Max(r-5, mr)
			sc := float64(hidden)*pf("retreat_hidden_w") + away*0.6 - ring*0.5
			if !inz {
				sc += -200
			}
			if !w.ClearLine([]float64{x, y}, []float64{px, py}) {
				sc -= 80
			}
			if sc > best {
				best, bv = sc, []float64{px, py}
			}
		}
	}
	if bv == nil {
		if len(threats) == 0 {
			return []float64{0, 0}
		}
		ep := txPos(threats[0]["pos"])
		ax, ay := x-ep[0], y-ep[1]
		n := math.Hypot(ax, ay)
		if n == 0 {
			n = 1
		}
		return []float64{ax / n, ay / n}
	}
	vx, vy := bv[0]-x, bv[1]-y
	n := math.Hypot(vx, vy)
	if n == 0 {
		n = 1
	}
	return []float64{vx / n, vy / n}
}

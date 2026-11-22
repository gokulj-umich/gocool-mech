// Gocool mech bot, Go port (v70): zone survival, laser-first loot, steady-strafe fights at the
// loadout's range, retreat when hurt and outnumbered, cube grabs, armor-first upgrades.
package main

import (
	"bufio"
	"encoding/json"
	"math"
	"math/rand"
	"os"
)

type M = map[string]any

func f(v any) float64 {
	switch x := v.(type) {
	case float64:
		return x
	case int:
		return float64(x)
	}
	return 0
}
func vec(v any) []float64 {
	a, _ := v.([]any)
	out := make([]float64, len(a))
	for i, e := range a {
		out[i] = f(e)
	}
	return out
}
func mp(v any) M { m, _ := v.(M); return m }
func arr(v any) []any { a, _ := v.([]any); return a }

var upg = []string{"nanites", "plating", "armor", "laser_damage", "laser_charge", "gun_damage", "gun_mag", "gun_rate",
	"lightning_efficiency", "lightning_range", "servos", "gun_velocity", "gyros", "lightning_chain", "reactor",
	"capacitor", "sensors", "jets", "mortar_reload", "mortar_velocity", "mortar_splash", "laser_pierce"}

type Bot struct {
	cfg        M
	strafeDir  float64
	strafeTill float64
	lastHit    float64
	recovering bool
	rng        *rand.Rand
}

func (b *Bot) gunSpeed() float64 {
	if w := mp(mp(b.cfg["weapons"])["gun"]); w != nil {
		if s := f(w["speed"]); s > 0 {
			return s
		}
	}
	return 240
}

func dist2(a, c []float64) float64 { return math.Hypot(a[0]-c[0], a[1]-c[1]) }

func (b *Bot) act(obs M) M {
	act := M{}
	me := mp(obs["self"])
	if me == nil {
		return act
	}
	t := f(obs["t"])
	pos := vec(me["pos"])
	if len(pos) < 3 {
		return act
	}
	hp, mhp := f(me["hp"]), math.Max(f(me["max_hp"]), 1)
	hpf := hp / mhp
	stats := mp(me["stats"])
	muzzleH := 7.0
	if stats != nil && f(stats["muzzle_height"]) > 0 {
		muzzleH = f(stats["muzzle_height"])
	}
	// upgrade offer
	if off := mp(obs["offer"]); off != nil {
		best, bi := 999, 0
		for i, c := range arr(off["choices"]) {
			id, _ := mp(c)["id"].(string)
			r := 50
			for k, u := range upg {
				if u == id {
					r = k
				}
			}
			if r < best {
				best, bi = r, i
			}
		}
		act["pick"] = bi
	}
	// events: last time we were hit
	for _, e := range arr(obs["events"]) {
		ev := mp(e)
		if s, _ := ev["type"].(string); s == "damaged" {
			if w, _ := ev["weapon"].(string); w != "zone" {
				b.lastHit = t
			}
		}
	}
	// weapons
	weps := arr(me["weapons"])
	slot := map[string]int{}
	for i, w := range weps {
		if k, _ := mp(w)["kind"].(string); k != "" {
			slot[k] = i
		}
	}
	// enemies
	myID := f(me["id"])
	type en struct {
		pos, vel []float64
		d, hp   float64
		id       float64
	}
	var enemies []en
	for _, m := range arr(obs["mechs"]) {
		e := mp(m)
		if k, _ := e["kind"].(string); k != "mech" || f(e["id"]) == myID {
			continue
		}
		p := vec(e["pos"])
		if len(p) < 3 {
			continue
		}
		h := f(e["hp"])
		if _, ok := e["hp"]; !ok {
			h = 1400
		}
		enemies = append(enemies, en{p, vec(e["vel"]), dist2(p, pos), h, f(e["id"])})
	}
	var tgt *en
	bestS := 1e18
	for i := range enemies {
		e := &enemies[i]
		if e.d > 300 {
			continue
		}
		s := e.d*0.5 + e.hp*0.08
		if s < bestS {
			bestS, tgt = s, e
		}
	}
	nearN := 0
	for _, e := range enemies {
		if e.d < 160 {
			nearN++
		}
	}
	// retreat state
	if (nearN >= 1 && hpf < 0.38) || (nearN >= 2 && hpf < 0.5) {
		if !(nearN <= 1 && hpf > 0.15) || nearN >= 2 {
			b.recovering = true
		}
	}
	if b.recovering && hpf > 0.8 {
		b.recovering = false
	}
	// zone
	zone := mp(obs["zone"])
	nc := vec(zone["next_center"])
	nr := f(zone["next_radius"])
	c := vec(zone["center"])
	r := f(zone["radius"])
	shrink := f(zone["shrink_start"])
	inZone, _ := me["in_zone"].(bool)
	var move = []float64{0, 0}
	boost := false
	toward := func(gx, gy float64) []float64 {
		dx, dy := gx-pos[0], gy-pos[1]
		n := math.Hypot(dx, dy)
		if n < 1e-6 {
			return []float64{0, 0}
		}
		return []float64{dx / n, dy / n}
	}
	urgent := false
	if len(nc) >= 2 {
		dn := math.Hypot(pos[0]-nc[0], pos[1]-nc[1])
		if dn > nr*0.85 && (shrink-t < dn/9.0+10 || t >= shrink) {
			urgent = true
		}
	}
	if !inZone && len(c) >= 2 {
		urgent = true
		nc = c
		_ = r
	}
	if t > b.strafeTill {
		b.strafeDir = -b.strafeDir
		if b.strafeDir == 0 {
			b.strafeDir = 1
		}
		b.strafeTill = t + 2 + b.rng.Float64()*2
	}
	switch {
	case urgent && len(nc) >= 2:
		move = toward(nc[0], nc[1])
		boost = true
	case b.recovering && tgt != nil:
		ax, ay := 0.0, 0.0
		for _, e := range enemies {
			if e.d < 200 {
				ax += (pos[0] - e.pos[0]) / math.Max(e.d, 1)
				ay += (pos[1] - e.pos[1]) / math.Max(e.d, 1)
			}
		}
		n := math.Hypot(ax, ay)
		if n > 0 {
			move = []float64{ax / n, ay / n}
		}
		boost = true
	case tgt != nil:
		want := 100.0
		if _, ok := slot["laser"]; ok {
			want = 120
		}
		if _, ok := slot["lightning"]; ok {
			want = 28
		}
		if tgt.hp < 0.38*1400 {
			want = math.Min(want, 45)
		}
		dx, dy := tgt.pos[0]-pos[0], tgt.pos[1]-pos[1]
		n := math.Max(math.Hypot(dx, dy), 1)
		rx, ry := dx/n, dy/n
		px, py := -ry, rx
		radial := math.Max(-1, math.Min(1, (tgt.d-want)/30))
		mx, my := px*b.strafeDir*0.9+rx*radial*0.7, py*b.strafeDir*0.9+ry*radial*0.7
		mn := math.Max(math.Hypot(mx, my), 1e-6)
		move = []float64{mx / mn, my / mn}
	default:
		// loot: cubes then weapons, else drift toward next circle
		bestD := 1e18
		var goal []float64
		for _, q := range arr(obs["cubes"]) {
			p := vec(mp(q)["pos"])
			if len(p) >= 2 {
				if d := dist2(p, pos); d < 110 && d < bestD {
					bestD, goal = d, p
				}
			}
		}
		if goal == nil {
			for _, q := range arr(obs["pickups"]) {
				pk := mp(q)
				k, _ := pk["kind"].(string)
				if _, has := slot[k]; has || (k != "laser" && k != "lightning") {
					continue
				}
				p := vec(pk["pos"])
				if len(p) >= 2 {
					if d := dist2(p, pos); d < 320 && d < bestD {
						bestD, goal = d, p
					}
				}
			}
		}
		if goal != nil {
			move = toward(goal[0], goal[1])
			boost = bestD > 30
		} else if len(nc) >= 2 && math.Hypot(pos[0]-nc[0], pos[1]-nc[1]) > nr*0.5 {
			move = toward(nc[0], nc[1])
		}
	}
	// take weapons we walk over (laser > lightning), drop slot 1 if full
	for _, q := range arr(obs["pickups"]) {
		pk := mp(q)
		k, _ := pk["kind"].(string)
		if k != "laser" && k != "lightning" {
			continue
		}
		if _, has := slot["laser"]; has {
			continue
		}
		if _, has := slot[k]; has {
			continue
		}
		p := vec(pk["pos"])
		if len(p) >= 2 && dist2(p, pos) < 4 {
			act["take"] = f(pk["id"])
			if len(weps) >= 2 {
				act["drop"] = 1
			}
			break
		}
	}
	// aim + fire
	fire := make([]bool, len(weps))
	if tgt != nil && !urgent {
		tz := tgt.pos[2] + 4.5
		mz := pos[2] + muzzleH
		dx, dy, dz := tgt.pos[0]-pos[0], tgt.pos[1]-pos[1], tz-mz
		d3 := math.Sqrt(dx*dx + dy*dy + dz*dz)
		tf := d3 / b.gunSpeed()
		if len(tgt.vel) >= 3 {
			dx += tgt.vel[0] * tf
			dy += tgt.vel[1] * tf
		}
		yaw := math.Atan2(dy, dx)
		pitch := math.Atan2(dz, math.Hypot(dx, dy))
		act["aim"] = []float64{yaw, pitch}
		myAim := vec(me["aim"])
		aligned := true
		if len(myAim) >= 1 {
			da := math.Mod(math.Abs(myAim[0]-yaw)+math.Pi, 2*math.Pi) - math.Pi
			aligned = math.Abs(da) < 0.08
		}
		for k, i := range slot {
			w := mp(weps[i])
			switch k {
			case "gun":
				fire[i] = aligned && f(w["ammo"]) > 0 && f(w["reload"]) <= 0 && tgt.d < 220
			case "laser":
				ch := f(w["charge"])
				full := 0.75
				if st := mp(w["stats"]); st != nil && f(st["charge"]) > 0 {
					full = f(st["charge"])
				}
				// hold to charge; release (false) once full and aligned
				fire[i] = f(w["cooldown"]) <= 0 && tgt.d < 270 && !(ch >= full-0.05 && aligned)
			case "lightning":
				fire[i] = tgt.d < 40 && f(me["energy"]) > 10
			case "mortar":
				fire[i] = false
			}
		}
		if gi, ok := slot["gun"]; ok {
			w := mp(weps[gi])
			if f(w["ammo"]) <= 5 && f(w["reload"]) == 0 && tgt.d > 150 {
				act["reload"] = true
			}
		}
	} else {
		// scan
		act["aim"] = []float64{math.Mod(t*0.8, 2*math.Pi) - math.Pi, 0}
	}
	act["fire"] = fire
	act["move"] = move
	act["boost"] = boost
	return act
}

func main() {
	in := bufio.NewReaderSize(os.Stdin, 1<<20)
	out := bufio.NewWriter(os.Stdout)
	lines := make(chan []byte, 64)
	go func() {
		for {
			l, err := in.ReadBytes('\n')
			if len(l) > 0 {
				lines <- l
			}
			if err != nil {
				close(lines)
				return
			}
		}
	}()
	var bot *Bot
	for {
		l, ok := <-lines
		if !ok {
			return
		}
		batch := [][]byte{l}
	drain:
		for {
			select {
			case x, ok2 := <-lines:
				if !ok2 {
					break drain
				}
				batch = append(batch, x)
			default:
				break drain
			}
		}
		var latest M
		for _, raw := range batch {
			var msg M
			if json.Unmarshal(raw, &msg) != nil {
				continue
			}
			switch msg["type"] {
			case "hello":
				bot = &Bot{cfg: mp(msg["config"]), strafeDir: 1, rng: rand.New(rand.NewSource(7))}
				b, _ := json.Marshal(M{"type": "ready", "name": "gocool"})
				out.Write(append(b, '\n'))
				out.Flush()
			case "obs":
				latest = msg
			case "end":
				return
			}
		}
		if latest == nil {
			continue
		}
		var act M
		func() {
			defer func() {
				if recover() != nil {
					act = M{}
				}
			}()
			if bot != nil {
				act = bot.act(latest)
			} else {
				act = M{}
			}
		}()
		act["tick"] = latest["tick"]
		b, err := json.Marshal(act)
		if err != nil {
			b, _ = json.Marshal(M{"tick": latest["tick"]})
		}
		out.Write(append(b, '\n'))
		out.Flush()
	}
}

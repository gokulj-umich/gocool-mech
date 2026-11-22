package main

import (
	"fmt"
	"math"
	"math/rand"
	"os"
	"runtime/debug"
	"time"
)

func logf(a ...any) { fmt.Fprintln(os.Stderr, a...) }

// Bot mirrors Python class Bot: every `self.x` lives in S["x"].
type Bot struct {
	S     M
	W     *World
	Local *Planner // self.local_plan
	Zone  *Planner // self.zone_plan
	Cfg   M
	Hello M
	Rng   *rand.Rand
}

func getNum(m M, k string, def float64) float64 {
	if v, ok := m[k]; ok && v != nil {
		return num(v)
	}
	return def
}

func NewBot(hello M) *Bot {
	b := &Bot{S: M{}, Hello: hello}
	b.Cfg = mm(hello, "config")
	seed := int64(1)
	if v, ok := hello["seed"]; ok {
		seed = int64(num(v))
	}
	b.Rng = rand.New(rand.NewSource(seed))
	b.W = NewWorld()
	b.Zone = NewPlanner(b.W)
	b.Local = NewPlanner(b.W)
	s := b.S
	s["cfg"] = b.Cfg
	s["w"] = b.W
	s["zone_plan"] = b.Zone
	s["local_plan"] = b.Local
	s["local_goal"] = nil
	s["strafe_dir"] = 1.0
	s["strafe_until"] = 0.0
	s["last_pos"] = []any{}
	s["stuck_until"] = 0.0
	s["unstick_dir"] = nil
	s["threat_bearing"] = nil
	s["threat_t"] = -99.0
	s["last_damage_t"] = -99.0
	s["hits_by"] = []any{}
	s["hit_from"] = M{}
	s["regen_on"] = false
	s["gang_until"] = -99.0
	s["cur_threats"] = []any{}
	s["cur_enemies"] = []any{}
	s["cur_cubes"] = []any{}
	s["scan_phase"] = b.Rng.Float64() * TAU
	s["hide_goal"] = nil
	s["hide_key"] = nil
	s["mode"] = "start"
	s["shorted_until"] = 0.0
	s["t"] = 0.0
	s["enemy_seen_t"] = -99.0
	s["ignore_crates"] = M{}
	s["ok"] = true
	s["known_pk"] = M{}
	s["scan_start"] = nil
	s["peek"] = nil
	s["heard"] = nil
	s["tracks"] = M{}
	s["laser_fired"] = M{}
	s["was_charging"] = M{}
	s["pp"] = nil
	s["my_kills"] = 0.0
	s["g_stick_id"] = nil
	s["g_stick_t"] = -99.0
	s["cover_t"] = -99.0
	s["cover_spot_"] = nil
	s["mem_crates"] = M{}
	s["mem_cubes"] = M{}
	s["mem_npcs"] = M{}
	s["loot_id"] = nil
	s["loot_since"] = 0.0
	s["loot_bad"] = M{}
	s["last_kill_t"] = -99.0
	s["hit_ids"] = M{} // set: keys are ids formatted with fmt.Sprint
	s["recovering"] = false
	s["cur_tid"] = nil
	s["seen_at"] = M{}
	s["survival"] = false
	s["roof_since"] = nil
	s["ffa"] = false
	s["roof_black"] = M{}
	s["roof_peek"] = nil
	return b
}

// ZoneGoal returns (goal or nil, circle {cx,cy,r,margin_r}).
func (b *Bot) ZoneGoal(obs M) ([]float64, []float64) {
	z := mm(obs, "zone")
	me := vec(mm(obs, "self")["pos"])
	t := mf(obs, "t")
	var cx, cy, r float64
	if t >= mf(z, "shrink_end")-0.01 && mf(z, "next_radius") >= mf(z, "radius")-0.5 {
		c := vec(z["center"])
		cx, cy = c[0], c[1]
		r = mf(z, "radius")
	} else {
		c := vec(z["next_center"])
		cx, cy = c[0], c[1]
		r = mf(z, "next_radius")
	}
	var marginR float64
	if r <= 30 {
		marginR = math.Max(0.0, r*0.35)
	} else if r <= 80 {
		marginR = r * 0.5
	} else {
		marginR = math.Max(r*0.55, r-60.0)
	}
	dx, dy := me[0]-cx, me[1]-cy
	dist := math.Hypot(dx, dy)
	circle := []float64{cx, cy, r, marginR}
	if dist <= marginR {
		return nil, circle
	}
	s := marginR / math.Max(dist, 1e-6)
	return []float64{cx + dx*s, cy + dy*s}, circle
}

// PickHide: a walkable cell near a tall wall, inside the safe circle, near where we are.
func (b *Bot) PickHide(obs M, circle []float64) []float64 {
	cx, cy, r, mr := circle[0], circle[1], circle[2], circle[3]
	me := vec(mm(obs, "self")["pos"])
	key := [3]float64{math.RoundToEven(cx), math.RoundToEven(cy), math.RoundToEven(r)}
	if k, ok := b.S["hide_key"].([3]float64); ok && k == key {
		if hg, ok := b.S["hide_goal"].([]float64); ok && hg != nil {
			return hg
		}
	}
	set := func(g []float64) []float64 {
		b.S["hide_goal"] = g
		b.S["hide_key"] = key
		return g
	}
	w := b.W
	dx, dy := me[0]-cx, me[1]-cy
	dist := math.Hypot(dx, dy)
	s := math.Min(1.0, mr/math.Max(dist, 1e-6))
	px, py := cx+dx*s, cy+dy*s
	rad := 40.0
	i0, j0 := w.Cell(px-rad, py-rad)
	i1, j1 := w.Cell(px+rad, py+rad)
	if i1 < i0 || j1 < j0 {
		return set([]float64{px, py})
	}
	edge := pb("edge_hide")
	best := 1e9
	found := false
	bk := [2]float64{}
	for j := j0; j <= j1; j++ {
		for i := i0; i <= i1; i++ {
			X := w.X0 + 2.0*float64(i)
			Y := w.Y0 + 2.0*float64(j)
			if !w.Walk[j*w.W+i] {
				continue
			}
			if math.Hypot(X-cx, Y-cy) > mr {
				continue
			}
			sc := math.Hypot(X-px, Y-py)*0.25 + math.Min(math.Max(float64(w.Cover[j*w.W+i])-3.5, 0), 30)*1.5
			if edge {
				sc += math.Abs(math.Hypot(X-cx, Y-cy)-mr*0.85) * 0.15
			}
			if !found || sc < best {
				best, found, bk = sc, true, [2]float64{X, Y}
			}
		}
	}
	if !found {
		return set([]float64{px, py})
	}
	return set([]float64{bk[0], bk[1]})
}

// Steer picks a walkable heading close to the desired (unnormalised) vector.
func (b *Bot) Steer(me M, desired []float64, avoid [][]float64) []float64 {
	pos := vec(me["pos"])
	x, y := pos[0], pos[1]
	mag := math.Hypot(desired[0], desired[1])
	if mag < 1e-6 {
		return []float64{0.0, 0.0}
	}
	dx, dy := desired[0]/mag, desired[1]/mag
	w := b.W
	hereOK := w.Walkable(x, y)
	best, bh := -9.0, [2]float64{dx, dy}
	base := math.Atan2(dy, dx)
	for k := 0; k < 16; k++ {
		kk := k
		if k > 8 {
			kk = k - 16
		}
		a := base + float64(kk)*(TAU/16)
		hx, hy := math.Cos(a), math.Sin(a)
		sc := hx*dx + hy*dy
		if hereOK {
			if !w.Walkable(x+hx*3.0, y+hy*3.0) {
				sc -= 3.0
			} else if !w.Walkable(x+hx*6.0, y+hy*6.0) {
				sc -= 0.6
			}
		}
		if sc > best {
			best, bh = sc, [2]float64{hx, hy}
		}
	}
	m := math.Min(1.0, mag)
	return []float64{bh[0] * m, bh[1] * m}
}

// GoTo: desired move vector towards goal using the planner field (straight if not ready).
func (b *Bot) GoTo(me M, pl *Planner, goal []float64, t float64) []float64 {
	pos := vec(me["pos"])
	x, y := pos[0], pos[1]
	gi, gj := b.W.Cell(goal[0], goal[1])
	pl.SetGoal(gi, gj)
	si, sj := b.W.Cell(x, y)
	pl.Search(si, sj, 0.012)
	wp := pl.Waypoint(x, y, 8)
	if wp == nil {
		wp = goal
	}
	vx, vy := wp[0]-x, wp[1]-y
	n := math.Hypot(vx, vy)
	if n < 1e-6 {
		return []float64{0.0, 0.0}
	}
	return []float64{vx / n, vy / n}
}

// Lead returns {yaw, pitch, hd, px, py, pz}.
func (b *Bot) Lead(me, tgt M, speed, scale float64) []float64 {
	pos := vec(me["pos"])
	mz := pos[2] + getNum(mm(me, "stats"), "muzzle_height", 7.0)
	mx, my := pos[0], pos[1]
	tsize := 0.39
	if ms(tgt, "kind") == "mech" {
		tsize = 1.0 + 0.05*getNum(tgt, "cubes", 0)
	}
	tp := vec(tgt["pos"])
	tx, ty, tz := tp[0], tp[1], tp[2]+4.5*tsize
	v := vec(tgt["vel"])
	vx, vy, vz := v[0], v[1], v[2]
	if math.Abs(vz) < 1.0 {
		vz = 0.0
	}
	t := 0.0
	var px, py, pz float64
	for i := 0; i < 3; i++ {
		px, py, pz = tx+vx*t, ty+vy*t, tz+vz*t*0.5
		dist := math.Sqrt((px-mx)*(px-mx) + (py-my)*(py-my) + (pz-mz)*(pz-mz))
		if speed > 0 {
			t = dist / speed
		} else {
			t = 0.0
		}
	}
	px, py, pz = tx+vx*t, ty+vy*t, tz+vz*t*0.5
	hd := math.Hypot(px-mx, py-my)
	return []float64{math.Atan2(py-my, px-mx), math.Atan2(pz-mz, math.Max(hd, 0.1)), hd, px, py, pz}
}

// MortarAim returns {yaw, theta, D} or nil when out of range.
func (b *Bot) MortarAim(me, tgt M, w M) []float64 {
	v := getNum(mm(w, "stats"), "speed", 72.0)
	g := getNum(mm(mm(b.Cfg, "weapons"), "mortar"), "gravity", 22.0)
	pos := vec(me["pos"])
	mx, my := pos[0], pos[1]
	mz := pos[2] + getNum(mm(me, "stats"), "height", 9.0) + 0.5
	tp := vec(tgt["pos"])
	tx, ty, tz := tp[0], tp[1], tp[2]+1.0
	tv := vec(tgt["vel"])
	vx, vy := tv[0], tv[1]
	T := 1.0
	var px, py, D, th float64
	for i := 0; i < 3; i++ {
		px, py = tx+vx*T*0.6, ty+vy*T*0.6
		D = math.Hypot(px-mx, py-my)
		dz := tz - mz
		disc := math.Pow(v, 4) - g*(g*D*D+2*dz*v*v)
		if disc < 0 {
			return nil
		}
		th = math.Atan((v*v - math.Sqrt(disc)) / (g * math.Max(D, 1.0)))
		T = D / math.Max(v*math.Cos(th), 1.0)
	}
	return []float64{math.Atan2(py-my, px-mx), th, D}
}

// Act wraps actInner; on panic falls back to walking at the zone (never stand still).
func (b *Bot) Act(obs M) (act M) {
	t0 := time.Now()
	func() {
		defer func() {
			if e := recover(); e != nil {
				logf("act error", obs["tick"], e, string(debug.Stack()))
				me := mm(obs, "self")
				z := mm(obs, "zone")
				c := vec(z["next_center"])
				p := vec(me["pos"])
				yaw := math.Atan2(c[1]-p[1], c[0]-p[0])
				act = M{"move": []float64{math.Cos(yaw), math.Sin(yaw)}, "aim": []float64{yaw, 0.0}, "boost": !mb(me, "in_zone")}
			}
		}()
		act = b.actInner(obs)
	}()
	if dt := float64(time.Since(t0).Microseconds()) / 1000; dt > 30 {
		logf(fmt.Sprintf("slow tick %v: %.1f ms", obs["tick"], dt))
	}
	return act
}

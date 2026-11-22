package main

import (
	"bufio"
	"encoding/json"
	"math"
	"math/rand"
	"os"
	"time"
)

type WStats struct {
	Damage    float64 `json:"damage"`
	Speed     float64 `json:"speed"`
	Range     float64 `json:"range"`
	Charge    float64 `json:"charge"`
	HoldS     float64 `json:"hold_s"`
	MinCharge float64 `json:"min_charge"`
	MinFrac   float64 `json:"min_frac"`
	ConeDeg   float64 `json:"cone_deg"`
	Energy    float64 `json:"energy"`
}
type Weapon struct {
	Kind     string  `json:"kind"`
	Cooldown float64 `json:"cooldown"`
	Ammo     float64 `json:"ammo"`
	Reload   float64 `json:"reload"`
	Charge   float64 `json:"charge"`
	Stats    WStats  `json:"stats"`
}
type SelfStats struct {
	MuzzleHeight float64 `json:"muzzle_height"`
}
type Self struct {
	ID      int        `json:"id"`
	Pos     [3]float64 `json:"pos"`
	Vel     [3]float64 `json:"vel"`
	Aim     [2]float64 `json:"aim"`
	HP      float64    `json:"hp"`
	MaxHP   float64    `json:"max_hp"`
	Energy  float64    `json:"energy"`
	Cubes   int        `json:"cubes"`
	Weapons []*Weapon  `json:"weapons"`
	Stats   SelfStats  `json:"stats"`
}
type Mech struct {
	ID       int        `json:"id"`
	Kind     string     `json:"kind"`
	Pos      [3]float64 `json:"pos"`
	Vel      [3]float64 `json:"vel"`
	Aim      [2]float64 `json:"aim"`
	HP       float64    `json:"hp"`
	Cubes    int        `json:"cubes"`
	Charging bool       `json:"charging"`
	MaxHP    float64    `json:"max_hp"`
	Weapons  []string   `json:"weapons"`
}
type Proj struct {
	Kind string     `json:"kind"`
	Pos  [3]float64 `json:"pos"`
	Vel  [3]float64 `json:"vel"`
}
type Item struct {
	ID   int        `json:"id"`
	Kind string     `json:"kind"`
	Pos  [3]float64 `json:"pos"`
}
type Event struct {
	Type   string `json:"type"`
	Effect string `json:"effect"`
	By     int    `json:"by"`
}
type Zone struct {
	Center      [2]float64 `json:"center"`
	Radius      float64    `json:"radius"`
	NextCenter  [2]float64 `json:"next_center"`
	NextRadius  float64    `json:"next_radius"`
	ShrinkStart float64    `json:"shrink_start"`
}
type Choice struct {
	ID string `json:"id"`
}
type Offer struct {
	Choices []Choice `json:"choices"`
}
type Obs struct {
	Type    string  `json:"type"`
	Tick    int     `json:"tick"`
	T       float64 `json:"t"`
	Self    Self    `json:"self"`
	Mechs   []Mech  `json:"mechs"`
	Projs   []Proj  `json:"projectiles"`
	Cubes   []Item  `json:"cubes"`
	Pickups []Item  `json:"pickups"`
	Events  []Event `json:"events"`
	Zone    *Zone   `json:"zone"`
	Offer   *Offer  `json:"offer"`
	Sounds  []Sound `json:"sounds"`
}

type Sound struct {
	Kind    string  `json:"kind"`
	Bearing float64 `json:"bearing"`
	Dist    string  `json:"dist"`
}

type ghostT struct {
	x, y, t float64
}

type Action struct {
	Tick   int        `json:"tick"`
	Move   [2]float64 `json:"move"`
	Boost  bool       `json:"boost"`
	Aim    [2]float64 `json:"aim"`
	Fire   [2]bool    `json:"fire"`
	Reload bool       `json:"reload,omitempty"`
	Pick   *int       `json:"pick,omitempty"`
	Take   *int       `json:"take,omitempty"`
	Drop   *int       `json:"drop,omitempty"`
}

var upg = []string{"nanites", "plating", "armor", "laser_damage", "laser_charge", "gun_damage", "gun_mag", "gun_rate",
	"lightning_efficiency", "lightning_range", "servos", "gun_velocity", "gyros", "lightning_chain", "reactor",
	"capacitor", "sensors", "jets"}

func wrank(k string) float64 {
	switch k {
	case "laser":
		return 4.0
	case "lightning":
		return 2.5
	case "mortar":
		return 1.0
	}
	return 0
}

func wrap(a float64) float64 {
	return math.Mod(math.Mod(a+math.Pi, 2*math.Pi)+2*math.Pi, 2*math.Pi) - math.Pi
}
func hyp(a, b float64) float64 { return math.Hypot(a, b) }

type Bot struct {
	strafe      float64
	strafeUntil float64
	shorted     float64
	lastCharge  float64
	nav         *Nav
	path        [][2]float64
	pathAt      float64
	pathGx      float64
	pathGy      float64
	retreating  bool
	hitFrom     map[int]float64
	curTid      int
	ghosts      map[int]ghostT
	heardB      float64
	heardD      float64
	heardT      float64
	roamX       float64
	roamY       float64
	roamUntil   float64
	roamSet     bool
}

// retreatDir: best of 16 headings: far from threats, inside zone, walkable.
func (b *Bot) retreatDir(x, y float64, thr []*Mech, zcx, zcy, zr float64) (float64, float64) {
	n := b.nav
	bs, bx, by := -1e18, 0.0, 0.0
	for k := 0; k < 16; k++ {
		a := float64(k) * math.Pi / 8
		dx, dy := math.Cos(a), math.Sin(a)
		sc := 0.0
		for _, d := range []float64{15, 35} {
			px, py := x+dx*d, y+dy*d
			for _, m := range thr {
				sc += math.Min(hyp(px-m.Pos[0], py-m.Pos[1]), 200) * (d / 35)
			}
			if o := hyp(px-zcx, py-zcy) - (zr - 20); o > 0 {
				sc -= o * 3
			}
			if n != nil && n.ok {
				i, j := n.cell(px, py)
				if !n.free(i, j) {
					sc -= 150
				}
			}
		}
		if sc > bs {
			bs, bx, by = sc, dx, dy
		}
	}
	return bx, by
}

// steer returns a unit direction toward (gx,gy) via A* waypoints; straight line fallback.
func (b *Bot) steer(x, y, gx, gy, t float64) (float64, float64) {
	if b.nav == nil || !b.nav.ok {
		return norm(gx-x, gy-y)
	}
	if hyp(gx-b.pathGx, gy-b.pathGy) > 15 || t-b.pathAt > 1.0 || len(b.path) == 0 {
		b.path = b.nav.path(x, y, gx, gy, 15*time.Millisecond)
		b.pathAt, b.pathGx, b.pathGy = t, gx, gy
	}
	for _, w := range b.path {
		if hyp(w[0]-x, w[1]-y) > 5 {
			return norm(w[0]-x, w[1]-y)
		}
	}
	return norm(gx-x, gy-y)
}

func (b *Bot) lead(me *Self, tgt *Mech, speed float64) (yaw, pitch, hd float64) {
	mz := me.Pos[2] + me.Stats.MuzzleHeight
	if me.Stats.MuzzleHeight == 0 {
		mz = me.Pos[2] + 7
	}
	tz := tgt.Pos[2] + 4.5
	if tgt.Kind != "mech" {
		tz = tgt.Pos[2] + 1.75
	}
	vx, vy, vz := tgt.Vel[0], tgt.Vel[1], tgt.Vel[2]
	if math.Abs(vz) < 1 {
		vz = 0
	}
	t := 0.0
	for i := 0; i < 3; i++ {
		px, py, pz := tgt.Pos[0]+vx*t, tgt.Pos[1]+vy*t, tz+vz*t*0.5
		d := math.Sqrt(math.Pow(px-me.Pos[0], 2) + math.Pow(py-me.Pos[1], 2) + math.Pow(pz-mz, 2))
		if speed > 0 {
			t = d / speed
		} else {
			t = 0
		}
	}
	px, py, pz := tgt.Pos[0]+vx*t, tgt.Pos[1]+vy*t, tz+vz*t*0.5
	hd = hyp(px-me.Pos[0], py-me.Pos[1])
	return math.Atan2(py-me.Pos[1], px-me.Pos[0]), math.Atan2(pz-mz, math.Max(hd, 0.1)), hd
}

func aligned(me *Self, yaw, pitch, dist, tol float64) bool {
	t := math.Atan2(tol, math.Max(dist, 1)) + 0.004
	return math.Abs(wrap(me.Aim[0]-yaw)) <= t && math.Abs(me.Aim[1]-pitch) <= t*1.5+0.01
}

func norm(x, y float64) (float64, float64) {
	n := hyp(x, y)
	if n < 1e-9 {
		return 0, 0
	}
	return x / n, y / n
}

func (b *Bot) act(o *Obs) Action {
	me := &o.Self
	t := o.T
	a := Action{Tick: o.Tick, Aim: me.Aim}
	// events
	for _, e := range o.Events {
		if e.Type == "effect" && e.Effect == "short" {
			b.shorted = t + 2.0
		}
		if e.Type == "damaged" && e.By != 0 {
			b.hitFrom[e.By] = t
		}
	}
	if b.ghosts == nil {
		b.ghosts = map[int]ghostT{}
	}
	for i := range o.Mechs {
		m := &o.Mechs[i]
		if m.Kind == "mech" && m.HP > 0 {
			b.ghosts[m.ID] = ghostT{m.Pos[0], m.Pos[1], t}
		}
	}
	for _, sn := range o.Sounds {
		switch sn.Kind {
		case "gun", "laser", "laser_charge", "lightning", "step", "boost", "reload", "jet", "mortar":
		default:
			continue
		}
		if sn.Dist != "near" && sn.Dist != "mid" {
			continue
		}
		dm := 35.0
		if sn.Dist == "mid" {
			dm = 105.0
		}
		if b.heardT == 0 || dm <= b.heardD || t-b.heardT > 2.0 {
			b.heardB, b.heardD, b.heardT = sn.Bearing, dm, t
		}
	}
	// upgrade
	if o.Offer != nil && len(o.Offer.Choices) > 0 {
		bi, best := 0, 999
		for i, c := range o.Offer.Choices {
			r := 50
			for j, u := range upg {
				if u == c.ID {
					r = j
				}
			}
			if r < best {
				best, bi = r, i
			}
		}
		a.Pick = &bi
	}
	// weapon slots
	gi, la, li := -1, -1, -1
	for i, w := range me.Weapons {
		if w == nil || i > 1 {
			continue
		}
		switch w.Kind {
		case "gun":
			gi = i
		case "laser":
			la = i
		case "lightning":
			li = i
		}
	}
	hpf := me.HP / math.Max(me.MaxHP, 1)

	// target: threat-aware scoring
	x, y := me.Pos[0], me.Pos[1]
	var tgt *Mech
	td := 1e9
	for i := range o.Mechs {
		m := &o.Mechs[i]
		d := hyp(m.Pos[0]-x, m.Pos[1]-y)
		if d > 300 {
			continue
		}
		if m.Kind != "mech" && (d > 110 || hpf < 0.5) {
			continue
		}
		sc := d*0.5 + m.HP*0.08
		if !b.nav.los(me.Pos[0], me.Pos[1], me.Pos[2]+7, m.Pos[0], m.Pos[1], m.Pos[2]+4.5) {
			sc += 120
		}
		rx, ry := x-m.Pos[0], y-m.Pos[1]
		closing := (m.Vel[0]*rx + m.Vel[1]*ry) / math.Max(d, 1)
		bear := math.Atan2(ry, rx)
		hf, hit := b.hitFrom[m.ID]
		threat := hit && t-hf < 6.0 ||
			math.Abs(wrap(m.Aim[0]-bear)) < 0.22 && (closing > 1 || d < 120) || d < 45
		if m.Kind == "mech" {
			if threat {
				sc -= 90
			} else {
				sc += 45
			}
		} else {
			sc += 60
		}
		if sc < td {
			td, tgt = sc, m
		}
	}
	if tgt != nil {
		b.curTid = tgt.ID
	}
	moveX, moveY := 0.0, 0.0
	// zone center / safety
	zcx, zcy, zr := x, y, 1e9
	if o.Zone != nil {
		z := o.Zone
		zcx, zcy, zr = z.Center[0], z.Center[1], z.Radius
		if t >= z.ShrinkStart-20 && z.NextRadius > 0 {
			zcx, zcy = z.NextCenter[0], z.NextCenter[1]
			zr = z.NextRadius
		}
	}
	zd := hyp(zcx-x, zcy-y)
	outside := zd > zr-25

	if tgt != nil {
		clear := b.nav.los(x, y, me.Pos[2]+7, tgt.Pos[0], tgt.Pos[1], tgt.Pos[2]+4.5)
		d := hyp(tgt.Pos[0]-x, tgt.Pos[1]-y)
		// v63 fight_vector port (effective P settings)
		rx, ry := norm(tgt.Pos[0]-x, tgt.Pos[1]-y)
		px, py := -ry, rx
		walk := func(wx, wy float64) bool {
			if b.nav == nil || !b.nav.ok {
				return true
			}
			i, j := b.nav.cell(wx, wy)
			return b.nav.free(i, j)
		}
		aheadX, aheadY := x+px*b.strafe*7.0, y+py*b.strafe*7.0
		if t > b.strafeUntil || (t > b.strafeUntil-4.0+0.8 && !walk(aheadX, aheadY)) {
			b.strafe = -b.strafe
			if b.strafe == 0 {
				b.strafe = 1
			}
			b.strafeUntil = t + 2.0 + rand.Float64()*2.0
		}
		tmax := tgt.MaxHP
		if tmax <= 0 {
			tmax = 1400
		}
		thp := tgt.HP / math.Max(tmax, 1)
		var want float64
		if li >= 0 && me.Energy > 20 {
			want = 28
		} else if la >= 0 {
			want = 120
		} else {
			want = 100
		}
		if thp < 0.38 || thp < hpf-0.12 { // crowd == 0 (press_multi_block False)
			want = math.Min(want, 45)
		}
		theirLight, theirLaser := false, false
		for _, k := range tgt.Weapons {
			if k == "lightning" {
				theirLight = true
			}
			if k == "laser" {
				theirLaser = true
			}
		}
		if theirLight && li < 0 {
			want = math.Max(want, 60)
		}
		if la >= 0 && !theirLaser && tgt.Kind == "mech" && t >= b.shorted {
			if thp > 0.2 {
				want = 180
			} else {
				want = 120
			}
		}
		radial := math.Max(-1, math.Min(1, (d-want)/30.0))
		ax, ay := rx, ry
		if radial > 0 && b.nav != nil && b.nav.ok && !b.nav.lineClear(x, y, tgt.Pos[0], tgt.Pos[1]) {
			ax, ay = b.steer(x, y, tgt.Pos[0], tgt.Pos[1], t)
		}
		moveX = px*b.strafe*0.9 + ax*radial*0.7
		moveY = py*b.strafe*0.9 + ay*radial*0.7
		if tgt.Kind == "mech" && b.nav != nil && b.nav.ok {
			var others []*Mech
			for k := range o.Mechs {
				e := &o.Mechs[k]
				if e.Kind == "mech" && e.ID != tgt.ID && e.ID != me.ID && hyp(e.Pos[0]-x, e.Pos[1]-y) < 170 {
					others = append(others, e)
				}
			}
			if len(others) > 0 {
				found := false
				bestS := 0.0
				bvx, bvy := 0.0, 0.0
				for k := 0; k < 16; k++ {
					ang := float64(k) * math.Pi / 8
					dx, dy := math.Cos(ang), math.Sin(ang)
					cx, cy := x+dx*7.0, y+dy*7.0
					if !walk(cx, cy) {
						continue
					}
					sc := -1.1*math.Abs(hyp(cx-tgt.Pos[0], cy-tgt.Pos[1])-want) + 12.0*(dx*px*b.strafe+dy*py*b.strafe)
					gz := b.nav.ground(cx, cy)
					for _, e := range others {
						if b.nav.los(e.Pos[0], e.Pos[1], e.Pos[2]+5.0, cx, cy, gz+3.5) {
							sc -= 30.0
						}
					}
					if !found || sc > bestS {
						found, bestS, bvx, bvy = true, sc, dx, dy
					}
				}
				if found {
					moveX, moveY = bvx*1.1, bvy*1.1
				}
			}
		}

		// cube-grab: visible cube close by, move to it while still aiming/firing
		if hpf > 0.3 {
			cd := 1e9
			var cx, cy float64
			for i := range o.Cubes {
				c := &o.Cubes[i]
				dd := hyp(c.Pos[0]-x, c.Pos[1]-y)
				if dd < 100 && dd < cd && math.Abs(c.Pos[2]-me.Pos[2]) < 6 && hyp(c.Pos[0]-zcx, c.Pos[1]-zcy) < zr-10 {
					cd, cx, cy = dd, c.Pos[0], c.Pos[1]
				}
			}
			if cd < 100 {
				moveX, moveY = b.steer(x, y, cx, cy, t)
				if cd > 12 && me.Energy > 30 {
					a.Boost = true
				}
			}
		}

		// shoot
		spd := 252.0
		rng := 230.0
		if gi >= 0 {
			gw := me.Weapons[gi]
			if gw.Stats.Speed > 0 {
				spd = gw.Stats.Speed
			}
			if gw.Stats.Range > 0 {
				rng = gw.Stats.Range
			}
		}
		laserAim := false
		set := false
		if li >= 0 {
			lw := me.Weapons[li]
			lr := lw.Stats.Range
			if lr == 0 {
				lr = 45
			}
			if d < lr*0.95 && tgt.Kind == "mech" || d < lr*0.95 {
				yaw := math.Atan2(tgt.Pos[1]-y, tgt.Pos[0]-x)
				a.Aim = [2]float64{yaw, 0}
				set = true
				cone := lw.Stats.ConeDeg
				if cone == 0 {
					cone = 30
				}
				if math.Abs(wrap(me.Aim[0]-yaw)) < cone*0.4*math.Pi/180 && lw.Cooldown <= 0.1 && me.Energy >= 3 && clear {
					a.Fire[li] = true
				}
			}
		}
		if la >= 0 && !a.Fire[max0(li)] || la >= 0 && li < 0 {
			lw := me.Weapons[la]
			lr := lw.Stats.Range
			if lr == 0 {
				lr = 280
			}
			full := lw.Stats.Charge
			if full == 0 {
				full = 0.75
			}
			hold := lw.Stats.HoldS
			if hold == 0 {
				hold = 2
			}
			minc := lw.Stats.MinCharge
			if minc == 0 {
				minc = 0.15
			}
			minf := lw.Stats.MinFrac
			if minf == 0 {
				minf = 0.35
			}
			ly, lp, lhd := b.lead(me, tgt, 1e9)
			ch := lw.Charge
			if d < lr*0.97 && t >= b.shorted && (li < 0 || d > 30) && clear {
				if lw.Cooldown <= 0.15 {
					lethal := false
					if tgt.Kind == "mech" && ch >= minc && ch < full-0.12 {
						dmg := lw.Stats.Damage * (minf + (1-minf)*ch/full)
						dmg *= 1 + 0.04*math.Min(float64(me.Cubes), 10)
						lethal = tgt.HP > 0 && dmg*0.7 >= tgt.HP
					}
					if lethal {
						laserAim = true
						a.Aim = [2]float64{ly, lp}
						set = true
						a.Fire[la] = !aligned(me, ly, lp, lhd, 1.2)
					} else if ch < full-0.12 {
						a.Fire[la] = true
					} else {
						laserAim = true
						a.Aim = [2]float64{ly, lp}
						set = true
						if aligned(me, ly, lp, lhd, 1.4) {
							a.Fire[la] = false
						} else {
							a.Fire[la] = ch < full+hold-0.25
						}
					}
				}
			} else if ch > 0.2 {
				a.Fire[la] = false // let go (out of range / shorted)
			}
		}
		if gi >= 0 {
			gw := me.Weapons[gi]
			yaw, pitch, hd := b.lead(me, tgt, spd)
			if !set || (!laserAim && li < 0) {
				a.Aim = [2]float64{yaw, pitch}
				set = true
			}
			if clear && hd < rng*0.97 && gw.Reload <= 0.1 && (gw.Ammo > 0 || gw.Reload > 0) {
				tol := 2.2
				if tgt.Kind != "mech" {
					tol = 1.2
				}
				if aligned(me, yaw, pitch, hd, tol) {
					a.Fire[gi] = true
				}
			}
			if gw.Ammo <= 0 && gw.Reload <= 0 {
				a.Reload = true
			}
		}
		if !set {
			a.Aim = [2]float64{math.Atan2(tgt.Pos[1]-y, tgt.Pos[0]-x), 0}
		}
	} else {
		// no target: loot / zone / roam
		bestD := 1e9
		var tx, ty float64
		found := false
		for i := range o.Pickups {
			p := &o.Pickups[i]
			d := hyp(p.Pos[0]-x, p.Pos[1]-y)
			if (p.Kind == "laser" && la < 0 || p.Kind == "lightning" && li < 0 && la >= 0 || p.Kind == "laser" && li >= 0 && la < 0) && d < 110 && d < bestD {
				bestD, tx, ty, found = d, p.Pos[0], p.Pos[1], true
			}
		}
		if !found {
			for i := range o.Cubes {
				p := &o.Cubes[i]
				d := hyp(p.Pos[0]-x, p.Pos[1]-y)
				if d < 100 && d < bestD {
					bestD, tx, ty, found = d, p.Pos[0], p.Pos[1], true
				}
			}
		}
		if found && !outside {
			moveX, moveY = norm(tx-x, ty-y)
		} else {
			moveX, moveY = b.steer(x, y, zcx, zcy, t)
			if zd < zr*0.35 {
				moveX, moveY = 0, 0
			}
			if hpf > 0.6 && !outside {
				inZone := func(px, py float64) bool { return hyp(px-zcx, py-zcy) < zr-10 }
				done := false
				// (1) ghost tracks, most recent first
				var bg *ghostT
				for id, g := range b.ghosts {
					g := g
					if t-g.t > 12 {
						delete(b.ghosts, id)
						continue
					}
					if hyp(g.x-x, g.y-y) < 8 {
						delete(b.ghosts, id)
						continue
					}
					if inZone(g.x, g.y) && (bg == nil || g.t > bg.t) {
						bg = &g
					}
				}
				if bg != nil {
					moveX, moveY = b.steer(x, y, bg.x, bg.y, t)
					done = true
				}
				// (2) heard sound
				if !done && b.heardT > 0 && t-b.heardT < 3.0 {
					gx, gy := x+math.Cos(b.heardB)*b.heardD, y+math.Sin(b.heardB)*b.heardD
					if inZone(gx, gy) {
						moveX, moveY = b.steer(x, y, gx, gy, t)
						done = true
					}
				}
				// (3) roam inside next zone circle
				if !done {
					if !b.roamSet || t >= b.roamUntil || hyp(b.roamX-x, b.roamY-y) < 15 {
						bs, bx, by := 1e18, zcx, zcy
						for k := 0; k < 10; k++ {
							ang := (rand.Float64()*2 - 1) * math.Pi
							rr := (0.15 + 0.6*rand.Float64()) * math.Max(zr, 20)
							px, py := zcx+math.Cos(ang)*rr, zcy+math.Sin(ang)*rr
							if b.nav != nil && b.nav.ok {
								i, j := b.nav.cell(px, py)
								if !b.nav.free(i, j) {
									continue
								}
							}
							if sc := math.Abs(hyp(px-x, py-y) - 120); sc < bs {
								bs, bx, by = sc, px, py
							}
						}
						b.roamX, b.roamY, b.roamUntil, b.roamSet = bx, by, t+25, true
					}
					moveX, moveY = b.steer(x, y, b.roamX, b.roamY, t)
				}
			}
		}
		// idle sweep
		a.Aim = [2]float64{wrap(me.Aim[0] + 0.5), 0}
		if gi >= 0 {
			gw := me.Weapons[gi]
			if gw.Ammo < 6 && gw.Reload <= 0 {
				a.Reload = true
			}
		}
	}
	// pickup take (WEAPON_RANK, laser first)
	cur1 := ""
	if len(me.Weapons) > 1 && me.Weapons[1] != nil {
		cur1 = me.Weapons[1].Kind
	}
	for i := range o.Pickups {
		p := &o.Pickups[i]
		if p.Kind == "gun" || p.Kind == "laser" && la >= 0 || p.Kind == "lightning" && li >= 0 || p.Kind == "mortar" && false {
			continue
		}
		if p.Kind != "laser" && p.Kind != "lightning" {
			continue
		}
		if cur1 != "" && wrank(p.Kind) <= wrank(cur1)+0.1 {
			continue
		}
		if hyp(p.Pos[0]-x, p.Pos[1]-y) < 3.5 && math.Abs(p.Pos[2]-me.Pos[2]) < 3.5 {
			if cur1 == "laser" && me.Weapons[1].Charge > 0 {
				continue
			}
			id := p.ID
			a.Take = &id
			if cur1 != "" {
				one := 1
				a.Drop = &one
			}
		}
	}
	// zone override
	if outside {
		zx, zy := b.steer(x, y, zcx, zcy, t)
		moveX, moveY = moveX*0.3+zx, moveY*0.3+zy
		if me.Energy > 40 {
			a.Boost = true
		}
	}
	// dodge charging lasers aimed at us
	for i := range o.Mechs {
		e := &o.Mechs[i]
		if !e.Charging {
			continue
		}
		d := hyp(x-e.Pos[0], y-e.Pos[1])
		if d > 290 {
			continue
		}
		bear := math.Atan2(y-e.Pos[1], x-e.Pos[0])
		if math.Abs(wrap(e.Aim[0]-bear)) < math.Atan2(6, math.Max(d, 1))+0.03 {
			s := b.strafe
			if s == 0 {
				s = 1
			}
			moveX, moveY = -math.Sin(bear)*s, math.Cos(bear)*s
			if me.Energy > 30 {
				a.Boost = true
			}
		}
	}
	// incoming projectiles: simple perpendicular sidestep
	for _, p := range o.Projs {
		rx, ry, rz := x-p.Pos[0], y-p.Pos[1], me.Pos[2]+4.5-p.Pos[2]
		vv := p.Vel[0]*p.Vel[0] + p.Vel[1]*p.Vel[1] + p.Vel[2]*p.Vel[2]
		if vv < 1 || p.Kind == "mortar" {
			continue
		}
		tc := (rx*p.Vel[0] + ry*p.Vel[1] + rz*p.Vel[2]) / vv
		if tc < 0 || tc > 0.8 {
			continue
		}
		mx, my := p.Pos[0]+p.Vel[0]*tc-x, p.Pos[1]+p.Vel[1]*tc-y
		if hyp(mx, my) < 4 {
			vx, vy := norm(p.Vel[0], p.Vel[1])
			s := b.strafe
			if s == 0 {
				s = 1
			}
			moveX, moveY = -vy*s, vx*s
		}
	}
	// laser grab within 90 m (target present or not)
	if la < 0 && hpf > 0.3 && !outside {
		gd, gx, gy := 90.0, 0.0, 0.0
		gf := false
		for i := range o.Pickups {
			p := &o.Pickups[i]
			d := hyp(p.Pos[0]-x, p.Pos[1]-y)
			if p.Kind == "laser" && d < gd && math.Abs(p.Pos[2]-me.Pos[2]) < 6 {
				gd, gx, gy, gf = d, p.Pos[0], p.Pos[1], true
			}
		}
		if gf {
			moveX, moveY = b.steer(x, y, gx, gy, t)
			if gd > 12 && me.Energy > 30 {
				a.Boost = true
			}
		}
	}
	// retreat rule: <38% w/ threat or <50% w/ 2+; recover to 80%
	var thr []*Mech
	near40 := false
	for i := range o.Mechs {
		m := &o.Mechs[i]
		if m.Kind != "mech" || m.HP <= 0 {
			continue
		}
		d := hyp(m.Pos[0]-x, m.Pos[1]-y)
		if d < 160 {
			thr = append(thr, m)
		}
		if d < 40 {
			near40 = true
		}
	}
	if hpf >= 0.8 || len(thr) == 0 && hpf >= 0.5 {
		b.retreating = false
	} else if len(thr) > 0 && (hpf < 0.38 || hpf < 0.5 && len(thr) >= 2) {
		b.retreating = true
	}
	if b.retreating && !near40 && len(thr) > 0 {
		moveX, moveY = b.retreatDir(x, y, thr, zcx, zcy, zr)
		if outside && me.Energy > 40 {
			a.Boost = true
		}
	}
	mx, my := norm(moveX, moveY)
	a.Move = [2]float64{mx, my}
	// slow while charging laser (server does it); keep move as is
	return a
}

func max0(i int) int {
	if i < 0 {
		return 0
	}
	return i
}

func main() {
	rd := bufio.NewReaderSize(os.Stdin, 1<<20)
	out := bufio.NewWriter(os.Stdout)
	lines := make(chan []byte, 256)
	go func() {
		for {
			l, err := rd.ReadBytes('\n')
			if len(l) > 0 {
				lines <- l
			}
			if err != nil {
				close(lines)
				return
			}
		}
	}()
	send := func(v any) {
		bs, _ := json.Marshal(v)
		out.Write(bs)
		out.WriteByte('\n')
		out.Flush()
	}
	bot := &Bot{strafe: 1, hitFrom: map[int]float64{}}
	for l := range lines {
		// skip to newest obs
	drain:
		for {
			select {
			case n, ok := <-lines:
				if !ok {
					break drain
				}
				var h struct {
					Type string `json:"type"`
				}
				json.Unmarshal(n, &h)
				if h.Type == "end" {
					return
				}
				l = n
			default:
				break drain
			}
		}
		var h struct {
			Type string `json:"type"`
		}
		if json.Unmarshal(l, &h) != nil {
			continue
		}
		switch h.Type {
		case "hello":
			var hh struct {
				Map struct {
					Dir string `json:"dir"`
				} `json:"map"`
			}
			if json.Unmarshal(l, &hh) == nil && hh.Map.Dir != "" {
				bot.nav = loadNav(hh.Map.Dir)
			}
			send(map[string]string{"type": "ready", "name": "gocool-go-F"})
		case "obs":
			var o Obs
			if json.Unmarshal(l, &o) != nil {
				continue
			}
			send(bot.act(&o))
		case "end":
			return
		}
	}
}

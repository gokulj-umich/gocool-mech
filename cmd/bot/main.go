package main

import (
	"bufio"
	"encoding/json"
	"math"
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
	Crates  []Item  `json:"crates"`
	Events  []Event `json:"events"`
	Zone    *Zone   `json:"zone"`
	Offer   *Offer  `json:"offer"`
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
}

var upg = []string{"nanites", "plating", "armor", "laser_damage", "laser_charge", "gun_damage", "gun_mag", "gun_rate",
	"lightning_efficiency", "lightning_range", "servos", "gun_velocity", "gyros", "lightning_chain", "reactor",
	"capacitor", "sensors", "jets"}

func wrap(a float64) float64 {
	return math.Mod(math.Mod(a+math.Pi, 2*math.Pi)+2*math.Pi, 2*math.Pi) - math.Pi
}
func hyp(a, b float64) float64 { return math.Hypot(a, b) }

type Bot struct {
	strafe      float64
	strafeUntil float64
	shorted     float64
	lastCharge  float64

	nav     *Nav
	path    [][2]float64
	pathG   [2]float64
	pathT   float64
	mem     map[int]*memItem
	lootID  int
	lootT   float64
	badLoot map[int]float64
}

type memItem struct {
	kind string
	pos  [3]float64
}

// steer returns a unit direction toward (gx,gy) following A* waypoints; straight line fallback.
func (b *Bot) steer(x, y, gx, gy, t float64) (float64, float64) {
	if b.nav == nil || !b.nav.ok {
		return norm(gx-x, gy-y)
	}
	if hyp(gx-x, gy-y) < 6 {
		return norm(gx-x, gy-y)
	}
	if b.path == nil || t-b.pathT > 0.7 || hyp(b.pathG[0]-gx, b.pathG[1]-gy) > 8 {
		b.path = b.nav.path(x, y, gx, gy, 12*time.Millisecond)
		b.pathG = [2]float64{gx, gy}
		b.pathT = t
	}
	for i, w := range b.path {
		if hyp(w[0]-x, w[1]-y) > 4 {
			// look ahead a few cells for smoother motion
			j := i + 2
			if j >= len(b.path) {
				j = len(b.path) - 1
			}
			w = b.path[j]
			if hyp(w[0]-x, w[1]-y) < 1 {
				w = b.path[i]
			}
			return norm(w[0]-x, w[1]-y)
		}
	}
	return norm(gx-x, gy-y)
}

func (b *Bot) updateMem(o *Obs, x, y float64) {
	if b.mem == nil {
		b.mem = map[int]*memItem{}
		b.badLoot = map[int]float64{}
	}
	cur := map[int]bool{}
	add := func(l []Item, kind string) {
		for _, it := range l {
			k := kind
			if kind == "pk" {
				k = it.Kind
			}
			b.mem[it.ID] = &memItem{k, it.Pos}
			cur[it.ID] = true
		}
	}
	add(o.Cubes, "cube")
	add(o.Crates, "crate")
	add(o.Pickups, "pk")
	for id, m := range b.mem {
		if !cur[id] && hyp(m.pos[0]-x, m.pos[1]-y) < 22 {
			delete(b.mem, id)
		}
	}
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

	// target: nearest mech, else near NPC
	var tgt *Mech
	td := 1e9
	for i := range o.Mechs {
		m := &o.Mechs[i]
		d := hyp(m.Pos[0]-me.Pos[0], m.Pos[1]-me.Pos[1])
		pri := d
		if m.Kind != "mech" {
			if d > 110 || hpf < 0.5 {
				continue
			}
			pri = d + 120
		}
		if m.HP > 0 && m.HP < 500 {
			pri -= 40
		}
		if pri < td {
			td, tgt = pri, m
		}
	}
	x, y := me.Pos[0], me.Pos[1]
	moveX, moveY := 0.0, 0.0
	shootCrate := false
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
		d := hyp(tgt.Pos[0]-x, tgt.Pos[1]-y)
		want := 100.0
		if la >= 0 {
			want = 120
		}
		if tgt.Kind != "mech" {
			want = 55
		}
		if t > b.strafeUntil {
			if b.strafe == 0 || int(t*7)%3 != 0 {
				b.strafe = -b.strafe
			}
			if b.strafe == 0 {
				b.strafe = 1
			}
			b.strafeUntil = t + 2 + float64(int(t*13)%3)
		}
		rx, ry := norm(tgt.Pos[0]-x, tgt.Pos[1]-y)
		radial := 0.0
		if d > want+15 {
			radial = 1
		} else if d < want-25 {
			radial = -1
		}
		if hpf < 0.4 {
			radial = -1
		}
		moveX = rx*radial + -ry*b.strafe*0.9
		moveY = ry*radial + rx*b.strafe*0.9

		if hpf > 0.3 {
			b.updateMem(o, x, y)
			cd, cx, cy := 100.0, 0.0, 0.0
			for i := range o.Cubes {
				c := &o.Cubes[i]
				dd := hyp(c.Pos[0]-x, c.Pos[1]-y)
				if dd < cd && hyp(c.Pos[0]-zcx, c.Pos[1]-zcy) < zr-25 {
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
				if math.Abs(wrap(me.Aim[0]-yaw)) < cone*0.4*math.Pi/180 && lw.Cooldown <= 0.1 && me.Energy >= 3 {
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
			if d < lr*0.97 && t >= b.shorted && (li < 0 || d > 30) {
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
			if hd < rng*0.97 && gw.Reload <= 0.1 && (gw.Ammo > 0 || gw.Reload > 0) {
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
		// no target: loot memory / zone / roam
		b.updateMem(o, x, y)
		bestU := 0.0
		var tx, ty float64
		bestID, bestKind, bestD := -1, "", 0.0
		for id, m := range b.mem {
			if b.badLoot[id] > t {
				continue
			}
			d := hyp(m.pos[0]-x, m.pos[1]-y)
			if hyp(m.pos[0]-zcx, m.pos[1]-zcy) > zr-25 {
				continue
			}
			v := 0.0
			switch m.kind {
			case "cube":
				v = 110
			case "crate":
				v = 105
			case "laser":
				if la < 0 {
					v = 400
					if d > 320 {
						continue
					}
				}
			case "lightning":
				if li < 0 && la >= 0 {
					v = 150
					if d > 110 {
						continue
					}
				}
			}
			if v == 0 || (m.kind != "laser" && d > 160) || (hpf <= 0.45 && d > 35 && m.kind != "laser") {
				continue
			}
			u := v / (d + 18)
			if id == b.lootID {
				u *= 1.35
			}
			if u > bestU {
				bestU, tx, ty, bestID, bestKind, bestD = u, m.pos[0], m.pos[1], id, m.kind, d
			}
		}
		if bestID >= 0 {
			if bestID != b.lootID {
				b.lootID, b.lootT = bestID, t
			} else if t-b.lootT > 25 {
				b.badLoot[bestID] = t + 60
				b.lootID = -1
				bestID = -1
			}
		} else {
			b.lootID = -1
		}
		if bestID >= 0 && !outside {
			if bestKind == "crate" && bestD < 40 && gi >= 0 {
				fake := &Mech{Kind: "crate", Pos: b.mem[bestID].pos}
				fake.Pos[2] += 0.5
				spd := 252.0
				if me.Weapons[gi].Stats.Speed > 0 {
					spd = me.Weapons[gi].Stats.Speed
				}
				yaw, pitch, hd := b.lead(me, fake, spd)
				a.Aim = [2]float64{yaw, pitch}
				gw := me.Weapons[gi]
				if gw.Reload <= 0.1 && gw.Ammo > 0 && aligned(me, yaw, pitch, hd, 2.0) {
					a.Fire[gi] = true
				}
				if bestD > 28 {
					moveX, moveY = b.steer(x, y, tx, ty, t)
				}
				shootCrate = true
			} else {
				moveX, moveY = b.steer(x, y, tx, ty, t)
			}
		} else {
			moveX, moveY = b.steer(x, y, zcx, zcy, t)
			if zd < zr*0.35 {
				moveX, moveY = 0, 0
			}
		}
		// idle sweep
		if !shootCrate {
			a.Aim = [2]float64{wrap(me.Aim[0] + 0.5), 0}
		}
		if gi >= 0 {
			gw := me.Weapons[gi]
			if gw.Ammo < 6 && gw.Reload <= 0 {
				a.Reload = true
			}
		}
	}
	// pickup take
	for i := range o.Pickups {
		p := &o.Pickups[i]
		if hyp(p.Pos[0]-x, p.Pos[1]-y) < 3.5 && math.Abs(p.Pos[2]-me.Pos[2]) < 3.5 &&
			(p.Kind == "laser" && la < 0 || p.Kind == "lightning" && li < 0 || p.Kind == "mortar" && false) &&
			(me.Weapons == nil || len(me.Weapons) < 2 || me.Weapons[1] == nil) {
			id := p.ID
			a.Take = &id
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
	bot := &Bot{strafe: 1}
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
			json.Unmarshal(l, &hh)
			if hh.Map.Dir != "" {
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

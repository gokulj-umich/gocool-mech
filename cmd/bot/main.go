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
	By     int    `json:"by"`
	Target int    `json:"target"`
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
	hitIDs      map[int]bool
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
		if e.Type == "hit" && e.Target != 0 {
			b.hitIDs[e.Target] = true
		}
		if e.Type == "damaged" && e.By != 0 {
			b.hitFrom[e.By] = t
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
	gi, la, li, mo := -1, -1, -1, -1
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
		case "mortar":
			mo = i
		}
	}
	hpf := me.HP / math.Max(me.MaxHP, 1)

	// target: threat-aware scoring
	x, y := me.Pos[0], me.Pos[1]
	var tgt *Mech
	td := 1e9
	myMax := math.Max(me.MaxHP, 1400)
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
		d := hyp(tgt.Pos[0]-x, tgt.Pos[1]-y)
		want := 100.0
		if la >= 0 {
			want = 120
		} else if li >= 0 && gi < 0 {
			want = 28
		}
		if tgt.Kind != "mech" {
			want = 55
		} else {
			thp := tgt.HP / myMax
			crowd := 0
			for k := range o.Mechs {
				if o.Mechs[k].ID != tgt.ID && o.Mechs[k].Kind == "mech" && hyp(o.Mechs[k].Pos[0]-x, o.Mechs[k].Pos[1]-y) < 150 {
					crowd++
				}
			}
			if (thp < 0.38 || thp < hpf-0.15) && crowd == 0 {
				want = math.Min(want, 45)
			}
		}
		if t > b.strafeUntil {
			b.strafe = -b.strafe
			if b.strafe == 0 {
				b.strafe = 1
			}
			b.strafeUntil = t + 2 + 2*(t*13.7-math.Floor(t*13.7))
		}
		rx, ry := norm(tgt.Pos[0]-x, tgt.Pos[1]-y)
		ax, ay := rx, ry
		radial := 0.0
		if d > want+15 {
			radial = 1
		} else if d < want-25 {
			radial = -1
		}
		if hpf < 0.4 {
			radial = -1
		}
		if radial > 0 && b.nav != nil && b.nav.ok && !b.nav.lineClear(x, y, tgt.Pos[0], tgt.Pos[1]) {
			ax, ay = b.steer(x, y, tgt.Pos[0], tgt.Pos[1], t)
		}
		moveX = ax*radial + -ry*b.strafe*0.9
		moveY = ay*radial + rx*b.strafe*0.9

		a.Aim, a.Fire = b.shoot(me, tgt, gi, la, li, mo, t)
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
		}
		// idle sweep
		a.Aim = [2]float64{wrap(me.Aim[0] + 0.5), 0}
		if gi >= 0 {
			gw := me.Weapons[gi]
			if gw.Ammo <= 5 && gw.Reload <= 0 {
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
	bot := &Bot{strafe: 1, hitFrom: map[int]float64{}, hitIDs: map[int]bool{}}
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

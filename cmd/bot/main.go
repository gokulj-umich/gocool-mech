package main

import (
	"bufio"
	"encoding/json"
	"fmt"
	"math"
	"os"
)

type V3 [3]float64

type Prism struct {
	Kind       string
	Base, Top  float64
	Ring       []float64
	minx, miny float64
	maxx, maxy float64
}

var prisms []Prism

func loadMap(dir string) {
	b, err := os.ReadFile(dir + "/v2/world.json")
	if err != nil {
		return
	}
	var w struct {
		Prisms []struct {
			Kind string    `json:"kind"`
			Base float64   `json:"base"`
			Top  float64   `json:"top"`
			Ring []float64 `json:"ring"`
		} `json:"prisms"`
	}
	if json.Unmarshal(b, &w) != nil {
		return
	}
	for _, p := range w.Prisms {
		if len(p.Ring) < 6 {
			continue
		}
		q := Prism{Kind: p.Kind, Base: p.Base, Top: p.Top, Ring: p.Ring, minx: 1e18, miny: 1e18, maxx: -1e18, maxy: -1e18}
		for i := 0; i+1 < len(p.Ring); i += 2 {
			q.minx = math.Min(q.minx, p.Ring[i])
			q.maxx = math.Max(q.maxx, p.Ring[i])
			q.miny = math.Min(q.miny, p.Ring[i+1])
			q.maxy = math.Max(q.maxy, p.Ring[i+1])
		}
		prisms = append(prisms, q)
	}
}

func segX(ax, ay, bx, by, cx, cy, dx, dy float64) bool {
	d1x, d1y := bx-ax, by-ay
	d2x, d2y := dx-cx, dy-cy
	den := d1x*d2y - d1y*d2x
	if math.Abs(den) < 1e-9 {
		return false
	}
	t := ((cx-ax)*d2y - (cy-ay)*d2x) / den
	u := ((cx-ax)*d1y - (cy-ay)*d1x) / den
	return t >= 0 && t <= 1 && u >= 0 && u <= 1
}

func inRing(r []float64, x, y float64) bool {
	in := false
	n := len(r) / 2
	for i, j := 0, n-1; i < n; j, i = i, i+1 {
		xi, yi, xj, yj := r[2*i], r[2*i+1], r[2*j], r[2*j+1]
		if (yi > y) != (yj > y) && x < (xj-xi)*(y-yi)/(yj-yi)+xi {
			in = !in
		}
	}
	return in
}

// clear returns true if no solid blocks the segment a->b (eye-level heights).
func clear(a, b V3) bool {
	lx0, lx1 := math.Min(a[0], b[0]), math.Max(a[0], b[0])
	ly0, ly1 := math.Min(a[1], b[1]), math.Max(a[1], b[1])
	for i := range prisms {
		p := &prisms[i]
		if p.maxx < lx0 || p.minx > lx1 || p.maxy < ly0 || p.miny > ly1 {
			continue
		}
		zl := (a[2] + b[2]) / 2
		if zl < p.Base-0.5 || zl > p.Top {
			continue
		}
		if inRing(p.Ring, a[0], a[1]) || inRing(p.Ring, b[0], b[1]) {
			return false
		}
		n := len(p.Ring) / 2
		for k, j := 0, n-1; k < n; j, k = k, k+1 {
			if segX(a[0], a[1], b[0], b[1], p.Ring[2*j], p.Ring[2*j+1], p.Ring[2*k], p.Ring[2*k+1]) {
				return false
			}
		}
	}
	return true
}

type Stats struct {
	Range float64 `json:"range"`
	Speed float64 `json:"speed"`
}
type Weapon struct {
	Kind     string  `json:"kind"`
	Cooldown float64 `json:"cooldown"`
	Ammo     float64 `json:"ammo"`
	Reload   float64 `json:"reload"`
	Stats    Stats   `json:"stats"`
}
type Self struct {
	Pos     V3         `json:"pos"`
	Vel     V3         `json:"vel"`
	Aim     [2]float64 `json:"aim"`
	HP      float64    `json:"hp"`
	MaxHP   float64    `json:"max_hp"`
	Weapons []*Weapon  `json:"weapons"`
}
type Mech struct {
	ID   int     `json:"id"`
	Kind string  `json:"kind"`
	Pos  V3      `json:"pos"`
	Vel  V3      `json:"vel"`
	HP   float64 `json:"hp"`
}
type Zone struct {
	Center      [2]float64 `json:"center"`
	Radius      float64    `json:"radius"`
	NextCenter  [2]float64 `json:"next_center"`
	NextRadius  float64    `json:"next_radius"`
	ShrinkStart float64    `json:"shrink_start"`
	ShrinkEnd   float64    `json:"shrink_end"`
}
type Obs struct {
	Type  string  `json:"type"`
	Tick  int     `json:"tick"`
	T     float64 `json:"t"`
	Self  Self    `json:"self"`
	Mechs []Mech  `json:"mechs"`
	Zone  *Zone   `json:"zone"`
	Offer *struct {
		Choices []struct {
			ID string `json:"id"`
		} `json:"choices"`
	} `json:"offer"`
	Cubes []struct {
		Pos V3 `json:"pos"`
	} `json:"cubes"`
}

type Hello struct {
	Map struct {
		Dir string `json:"dir"`
	} `json:"map"`
}

type Act struct {
	Tick   int        `json:"tick"`
	Move   [2]float64 `json:"move"`
	Boost  bool       `json:"boost"`
	Aim    [2]float64 `json:"aim"`
	Fire   [2]bool    `json:"fire"`
	Reload bool       `json:"reload,omitempty"`
	Pick   *int       `json:"pick,omitempty"`
}

func eye(p V3) V3 { return V3{p[0], p[1], p[2] + 8} }
func ctr(p V3) V3 { return V3{p[0], p[1], p[2] + 4.5} }

func zoneCircle(z *Zone, t float64) (cx, cy, r float64) {
	if z == nil {
		return 0, 0, 1e9
	}
	if t <= z.ShrinkStart {
		return z.Center[0], z.Center[1], z.Radius
	}
	if t >= z.ShrinkEnd || z.ShrinkEnd <= z.ShrinkStart {
		return z.NextCenter[0], z.NextCenter[1], z.NextRadius
	}
	f := (t - z.ShrinkStart) / (z.ShrinkEnd - z.ShrinkStart)
	return z.Center[0] + (z.NextCenter[0]-z.Center[0])*f, z.Center[1] + (z.NextCenter[1]-z.Center[1])*f, z.Radius + (z.NextRadius-z.Radius)*f
}

func decide(o *Obs) Act {
	s := &o.Self
	a := Act{Tick: o.Tick, Aim: s.Aim}
	if o.Offer != nil {
		pk := 0
		for i, c := range o.Offer.Choices {
			if c.ID == "armor" {
				pk = i
			}
		}
		a.Pick = &pk
	}
	me := s.Pos
	gunSpeed, gunRange := 252.0, 230.0
	if len(s.Weapons) > 0 && s.Weapons[0] != nil {
		w := s.Weapons[0]
		if w.Stats.Speed > 0 {
			gunSpeed = w.Stats.Speed
		}
		if w.Stats.Range > 0 {
			gunRange = w.Stats.Range
		}
		if w.Ammo <= 0 && w.Reload <= 0 {
			a.Reload = true
		}
	}
	hpf := s.HP / math.Max(s.MaxHP, 1)

	// enemies
	var tgt *Mech
	best := 1e18
	near := 0
	var threats []*Mech
	for i := range o.Mechs {
		m := &o.Mechs[i]
		if m.Kind != "mech" {
			continue
		}
		d := math.Hypot(m.Pos[0]-me[0], m.Pos[1]-me[1])
		if d < 130 {
			near++
			threats = append(threats, m)
		}
		sc := d
		if clear(eye(me), ctr(m.Pos)) {
			sc -= 1000
		}
		if sc < best {
			best, tgt = sc, m
		}
	}
	retreat := tgt != nil && (hpf < 0.38 || near >= 3)

	// movement
	mx, my := 0.0, 0.0
	if tgt != nil {
		dx, dy := tgt.Pos[0]-me[0], tgt.Pos[1]-me[1]
		d := math.Hypot(dx, dy) + 1e-6
		ux, uy := dx/d, dy/d
		phase := math.Floor(o.T / 3.0)
		sg := 1.0
		if int(phase)%2 == 1 {
			sg = -1
		}
		if retreat {
			bestSc := -1e18
			for k := 0; k < 16; k++ {
				ang := float64(k) * math.Pi / 8
				cx, cy := math.Cos(ang), math.Sin(ang)
				p := V3{me[0] + cx*30, me[1] + cy*30, me[2]}
				sc := 0.0
				for _, th := range threats {
					if !clear(eye(p), ctr(th.Pos)) {
						sc += 100
					}
					sc += math.Hypot(th.Pos[0]-p[0], th.Pos[1]-p[1]) * 0.2
				}
				if !clear(eye(me), eye(p)) {
					sc -= 20 // path crosses a wall
				}
				if sc > bestSc {
					bestSc, mx, my = sc, cx, cy
				}
			}
			a.Boost = true
		} else if d > 70 {
			mx, my = ux+(-uy)*sg*0.4, uy+ux*sg*0.4
		} else if d < 35 {
			mx, my = -ux+(-uy)*sg*0.6, -uy+ux*sg*0.6
		} else {
			mx, my = -uy*sg, ux*sg
		}
	} else if len(o.Cubes) > 0 {
		c := o.Cubes[0].Pos
		mx, my = c[0]-me[0], c[1]-me[1]
	}
	// zone override
	if o.Zone != nil {
		cx, cy, r := zoneCircle(o.Zone, o.T)
		if o.T >= o.Zone.ShrinkStart-12 {
			cx, cy, r = o.Zone.NextCenter[0], o.Zone.NextCenter[1], o.Zone.NextRadius
		}
		dz := math.Hypot(me[0]-cx, me[1]-cy)
		if dz > r*0.8 {
			ux, uy := cx-me[0], cy-me[1]
			l := math.Hypot(ux, uy) + 1e-6
			w := 1.0
			if dz > r {
				w = 3
			}
			n := math.Hypot(mx, my) + 1e-6
			mx, my = mx/n+w*ux/l, my/n+w*uy/l
		}
	}
	if l := math.Hypot(mx, my); l > 1e-6 {
		a.Move = [2]float64{mx / l, my / l}
	}

	// aim and fire
	if tgt != nil {
		dx, dy := tgt.Pos[0]-me[0], tgt.Pos[1]-me[1]
		d := math.Hypot(dx, dy)
		t := d / gunSpeed
		for i := 0; i < 2; i++ {
			px, py := tgt.Pos[0]+tgt.Vel[0]*t, tgt.Pos[1]+tgt.Vel[1]*t
			t = math.Hypot(px-me[0], py-me[1]) / gunSpeed
		}
		px, py := tgt.Pos[0]+tgt.Vel[0]*t, tgt.Pos[1]+tgt.Vel[1]*t
		pz := tgt.Pos[2] + 4.5
		hd := math.Hypot(px-me[0], py-me[1])
		a.Aim = [2]float64{math.Atan2(py-me[1], px-me[0]), math.Atan2(pz-(me[2]+7), hd)}
		yawErr := math.Abs(math.Remainder(a.Aim[0]-s.Aim[0], 2*math.Pi))
		if d < gunRange && yawErr < 0.12 && clear(V3{me[0], me[1], me[2] + 7}, V3{px, py, pz}) {
			a.Fire[0] = true
		}
	}
	return a
}

func main() {
	rd := bufio.NewReaderSize(os.Stdin, 1<<20)
	out := bufio.NewWriter(os.Stdout)
	lines := make(chan []byte, 1)
	go func() {
		for {
			l, err := rd.ReadBytes('\n')
			if len(l) > 0 {
				c := append([]byte(nil), l...)
				select {
				case lines <- c:
				default:
					select {
					case <-lines:
					default:
					}
					select {
					case lines <- c:
					default:
					}
				}
			}
			if err != nil {
				close(lines)
				return
			}
		}
	}()
	for l := range lines {
		var h struct {
			Type string `json:"type"`
		}
		if json.Unmarshal(l, &h) != nil {
			continue
		}
		switch h.Type {
		case "hello":
			var he Hello
			json.Unmarshal(l, &he)
			dir := he.Map.Dir
			if dir == "" {
				dir = "/arena"
			}
			loadMap(dir)
			fmt.Fprintln(out, `{"type":"ready","name":"go-D-los"}`)
			out.Flush()
		case "obs":
			var o Obs
			if json.Unmarshal(l, &o) != nil {
				continue
			}
			b, _ := json.Marshal(decide(&o))
			out.Write(b)
			out.WriteByte('\n')
			out.Flush()
		case "end":
			return
		}
	}
}

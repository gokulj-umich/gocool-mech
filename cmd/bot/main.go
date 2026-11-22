// Minimal Go port of the Mech Royale bot: zone survival, lead aim, fire, target choice, low-HP retreat.
package main

import (
	"bufio"
	"encoding/json"
	"math"
	"os"
)

type V3 = [3]float64

type Weapon struct {
	Kind     string  `json:"kind"`
	Cooldown float64 `json:"cooldown"`
	Ammo     int     `json:"ammo"`
	Reload   float64 `json:"reload"`
	Stats    struct {
		Speed float64 `json:"speed"`
		Range float64 `json:"range"`
	} `json:"stats"`
}
type Mech struct {
	ID    int        `json:"id"`
	Kind  string     `json:"kind"`
	Pos   V3         `json:"pos"`
	Vel   V3         `json:"vel"`
	Aim   [2]float64 `json:"aim"`
	HP    float64    `json:"hp"`
	MaxHP float64    `json:"max_hp"`
}
type Self struct {
	Mech
	Energy  float64   `json:"energy"`
	Weapons []*Weapon `json:"weapons"`
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
		Choices []json.RawMessage `json:"choices"`
	} `json:"offer"`
}

type Action struct {
	Tick   int        `json:"tick"`
	Move   [2]float64 `json:"move"`
	Boost  bool       `json:"boost"`
	Jump   bool       `json:"jump"`
	Aim    [2]float64 `json:"aim"`
	Fire   [2]bool    `json:"fire"`
	Reload bool       `json:"reload"`
	Pick   *int       `json:"pick,omitempty"`
}

func norm(x, y float64) (float64, float64) {
	l := math.Hypot(x, y)
	if l < 1e-9 {
		return 0, 0
	}
	return x / l, y / l
}

func decide(o *Obs) Action {
	a := Action{Tick: o.Tick}
	me := o.Self
	a.Aim = me.Aim
	if o.Offer != nil && len(o.Offer.Choices) > 0 {
		z := 0
		a.Pick = &z
	}
	hpf := 1.0
	if me.MaxHP > 0 {
		hpf = me.HP / me.MaxHP
	}
	var gun *Weapon
	for _, w := range me.Weapons {
		if w != nil && w.Kind == "gun" {
			gun = w
		}
	}
	// target: nearest visible mech within 300 m, preferring low HP
	var tgt *Mech
	best := math.Inf(1)
	for i := range o.Mechs {
		m := &o.Mechs[i]
		if m.Kind != "mech" {
			continue
		}
		d := math.Hypot(m.Pos[0]-me.Pos[0], m.Pos[1]-me.Pos[1])
		if d > 300 {
			continue
		}
		s := d * (0.5 + 0.5*m.HP/math.Max(m.MaxHP, 1))
		if s < best {
			best, tgt = s, m
		}
	}
	// aim + fire
	var dist float64
	if tgt != nil {
		speed := 252.0
		rng := 230.0
		if gun != nil {
			if gun.Stats.Speed > 0 {
				speed = gun.Stats.Speed
			}
			if gun.Stats.Range > 0 {
				rng = gun.Stats.Range
			}
		}
		rx, ry := tgt.Pos[0]-me.Pos[0], tgt.Pos[1]-me.Pos[1]
		rz := tgt.Pos[2] + 4.5 - (me.Pos[2] + 7)
		tt := math.Sqrt(rx*rx+ry*ry+rz*rz) / speed
		for i := 0; i < 3; i++ {
			px, py, pz := rx+(tgt.Vel[0]-me.Vel[0]*0)*tt, ry+tgt.Vel[1]*tt, rz+tgt.Vel[2]*tt
			tt = math.Sqrt(px*px+py*py+pz*pz) / speed
		}
		px, py, pz := rx+tgt.Vel[0]*tt, ry+tgt.Vel[1]*tt, rz+tgt.Vel[2]*tt
		a.Aim = [2]float64{math.Atan2(py, px), math.Atan2(pz, math.Hypot(px, py))}
		dist = math.Hypot(rx, ry)
		d := math.Mod(a.Aim[0]-me.Aim[0]+3*math.Pi, 2*math.Pi) - math.Pi
		if dist < rng && math.Abs(d) < 0.15 && gun != nil && gun.Reload <= 0 {
			a.Fire[0] = true
		}
	}
	if gun != nil && gun.Ammo == 0 && gun.Reload <= 0 {
		a.Reload = true
	}
	// movement
	mx, my := 0.0, 0.0
	if tgt != nil {
		ux, uy := norm(tgt.Pos[0]-me.Pos[0], tgt.Pos[1]-me.Pos[1])
		sgn := 1.0
		if int(o.T/3)%2 == 1 {
			sgn = -1
		}
		mx, my = -uy*sgn, ux*sgn
		if hpf < 0.45 {
			mx, my = -ux, -uy
		} else if dist > 100 {
			mx, my = ux*0.8-uy*sgn*0.3, uy*0.8+ux*sgn*0.3
		} else if dist < 40 {
			mx, my = -ux*0.6-uy*sgn*0.6, -uy*0.6+ux*sgn*0.6
		}
	}
	// zone overrides
	if z := o.Zone; z != nil {
		cx, cy, r := z.Center[0], z.Center[1], z.Radius
		if o.T >= z.ShrinkStart-15 {
			cx, cy = z.NextCenter[0], z.NextCenter[1]
			r = math.Min(r, z.NextRadius)
		}
		dx, dy := cx-me.Pos[0], cy-me.Pos[1]
		d := math.Hypot(dx, dy)
		if d > r*0.8 {
			ux, uy := dx/d, dy/d
			if d > r*0.95 || tgt == nil {
				mx, my = ux, uy
			} else {
				mx, my = mx*0.4+ux, my*0.4+uy
			}
			a.Boost = me.Energy > 25 && d > r
		}
	}
	mx, my = norm(mx, my)
	a.Move = [2]float64{mx, my}
	return a
}

func main() {
	in := make(chan []byte, 4096)
	go func() {
		rd := bufio.NewReaderSize(os.Stdin, 1<<20)
		for {
			l, err := rd.ReadBytes('\n')
			if len(l) > 0 {
				in <- l
			}
			if err != nil {
				close(in)
				return
			}
		}
	}()
	out := bufio.NewWriter(os.Stdout)
	enc := json.NewEncoder(out)
	for line := range in {
		// skip to newest queued line
		for len(in) > 0 {
			var t struct {
				Type string `json:"type"`
			}
			json.Unmarshal(line, &t)
			if t.Type != "obs" {
				break
			}
			line = <-in
		}
		var t struct {
			Type string `json:"type"`
		}
		if json.Unmarshal(line, &t) != nil {
			continue
		}
		switch t.Type {
		case "hello":
			enc.Encode(map[string]string{"type": "ready", "name": "gocool-go-A"})
			out.Flush()
		case "obs":
			var o Obs
			if json.Unmarshal(line, &o) != nil {
				continue
			}
			enc.Encode(decide(&o))
			out.Flush()
		case "end":
			return
		}
	}
}

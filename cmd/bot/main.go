package main

import (
	"bufio"
	"encoding/json"
	"math"
	"os"
)

type Weapon struct {
	Kind  string `json:"kind"`
	Ammo  int    `json:"ammo"`
	Stats struct {
		Speed float64 `json:"speed"`
		Range float64 `json:"range"`
	} `json:"stats"`
}
type Mech struct {
	ID    int       `json:"id"`
	Kind  string    `json:"kind"`
	Pos   []float64 `json:"pos"`
	Vel   []float64 `json:"vel"`
	HP    float64   `json:"hp"`
	MaxHP float64   `json:"max_hp"`
}
type Obs struct {
	Type string `json:"type"`
	Tick int    `json:"tick"`
	Self struct {
		Pos     []float64 `json:"pos"`
		Weapons []*Weapon `json:"weapons"`
	} `json:"self"`
	Mechs   []Mech `json:"mechs"`
	Pickups []struct {
		ID   int       `json:"id"`
		Pos  []float64 `json:"pos"`
	} `json:"pickups"`
	Zone struct {
		Center []float64 `json:"center"`
		Radius float64   `json:"radius"`
	} `json:"zone"`
	Offer interface{} `json:"offer"`
}

func main() {
	rd := bufio.NewReaderSize(os.Stdin, 1<<20)
	out := bufio.NewWriter(os.Stdout)
	send := func(v interface{}) {
		b, _ := json.Marshal(v)
		out.Write(b)
		out.WriteByte('\n')
		out.Flush()
	}
	next := func() []byte {
		l, err := rd.ReadBytes('\n')
		if err != nil && len(l) == 0 {
			return nil
		}
		return l
	}
	for {
		line := next()
		if line == nil {
			return
		}
		var o Obs
		if json.Unmarshal(line, &o) != nil {
			continue
		}
		switch o.Type {
		case "hello":
			send(map[string]interface{}{"type": "ready", "name": "go-E"})
			continue
		case "end":
			return
		case "obs":
		default:
			continue
		}
		// skip to newest queued obs
		for rd.Buffered() > 0 {
			l := next()
			if l == nil {
				break
			}
			var o2 Obs
			if json.Unmarshal(l, &o2) == nil {
				if o2.Type == "end" {
					return
				}
				if o2.Type == "obs" {
					o = o2
				}
			}
		}
		send(act(&o))
	}
}

func act(o *Obs) map[string]interface{} {
	a := map[string]interface{}{"tick": o.Tick}
	if len(o.Self.Pos) < 3 {
		return a
	}
	sp := o.Self.Pos
	// target scoring: close + wounded first, mechs before npcs
	var best *Mech
	bs := math.Inf(1)
	for i := range o.Mechs {
		m := &o.Mechs[i]
		if len(m.Pos) < 3 {
			continue
		}
		d := math.Hypot(m.Pos[0]-sp[0], m.Pos[1]-sp[1])
		s := d
		if m.MaxHP > 0 {
			s -= (1 - m.HP/m.MaxHP) * 80
		}
		if m.Kind == "npc" {
			s += 60
		}
		if s < bs {
			bs, best = s, m
		}
	}
	// movement: toward zone center, stop near it; strafe when engaged
	mx, my := 0.0, 0.0
	if len(o.Zone.Center) == 2 {
		dx, dy := o.Zone.Center[0]-sp[0], o.Zone.Center[1]-sp[1]
		d := math.Hypot(dx, dy)
		if d > o.Zone.Radius*0.3 && d > 1 {
			mx, my = dx/d, dy/d
		}
	}
	if best != nil {
		dx, dy := best.Pos[0]-sp[0], best.Pos[1]-sp[1]
		d := math.Hypot(dx, dy)
		if d > 1 {
			// strafe perpendicular, tick-alternating
			s := 1.0
			if (o.Tick/40)%2 == 0 {
				s = -1
			}
			mx = mx*0.5 + (-dy/d)*s*0.7
			my = my*0.5 + (dx/d)*s*0.7
		}
	}
	a["move"] = []float64{mx, my}
	// weapons
	speed, rng := 252.0, 230.0
	if len(o.Self.Weapons) > 0 && o.Self.Weapons[0] != nil {
		if o.Self.Weapons[0].Stats.Speed > 0 {
			speed = o.Self.Weapons[0].Stats.Speed
		}
		if o.Self.Weapons[0].Stats.Range > 0 {
			rng = o.Self.Weapons[0].Stats.Range
		}
	}
	if best != nil {
		tx, ty, tz := best.Pos[0], best.Pos[1], best.Pos[2]+4.5
		vx, vy := 0.0, 0.0
		if len(best.Vel) >= 2 {
			vx, vy = best.Vel[0], best.Vel[1]
		}
		px, py := tx, ty
		for i := 0; i < 3; i++ {
			d := math.Sqrt((px-sp[0])*(px-sp[0]) + (py-sp[1])*(py-sp[1]) + (tz-sp[2]-7)*(tz-sp[2]-7))
			t := d / speed
			px, py = tx+vx*t, ty+vy*t
		}
		h := math.Hypot(px-sp[0], py-sp[1])
		yaw := math.Atan2(py-sp[1], px-sp[0])
		pitch := math.Atan2(tz-sp[2]-7, h)
		a["aim"] = []float64{yaw, pitch}
		in := h < rng
		a["fire"] = []bool{in, in && h < 270}
	}
	// pickups: take nearest within 4 m
	for _, p := range o.Pickups {
		if len(p.Pos) >= 3 && math.Hypot(p.Pos[0]-sp[0], p.Pos[1]-sp[1]) < 3.5 && math.Abs(p.Pos[2]-sp[2]) < 3.5 {
			if len(o.Self.Weapons) > 1 && o.Self.Weapons[1] == nil {
				a["take"] = p.ID
			}
			break
		}
	}
	if o.Offer != nil {
		a["pick"] = 0
	}
	return a
}

package main

import (
	"bufio"
	"encoding/json"
	"fmt"
	"math"
	"os"
)

type Weapon struct {
	Kind   string  `json:"kind"`
	Charge float64 `json:"charge"`
	Cd     float64 `json:"cooldown"`
	Ammo   int     `json:"ammo"`
	Stats  struct {
		Speed float64 `json:"speed"`
		Range float64 `json:"range"`
	} `json:"stats"`
}
type Self struct {
	Pos     [3]float64 `json:"pos"`
	Vel     [3]float64 `json:"vel"`
	Hp      float64    `json:"hp"`
	MaxHp   float64    `json:"max_hp"`
	Weapons []*Weapon  `json:"weapons"`
}
type Mech struct {
	Id   int        `json:"id"`
	Kind string     `json:"kind"`
	Pos  [3]float64 `json:"pos"`
	Vel  [3]float64 `json:"vel"`
}
type Item struct {
	Id   int        `json:"id"`
	Kind string     `json:"kind"`
	Pos  [3]float64 `json:"pos"`
}
type Zone struct {
	Center      [2]float64 `json:"center"`
	Radius      float64    `json:"radius"`
	NextCenter  [2]float64 `json:"next_center"`
	NextRadius  float64    `json:"next_radius"`
	ShrinkStart float64    `json:"shrink_start"`
}
type Offer struct {
	Choices []struct {
		Id string `json:"id"`
	} `json:"choices"`
}
type Obs struct {
	Type    string  `json:"type"`
	Tick    int     `json:"tick"`
	T       float64 `json:"t"`
	Self    Self    `json:"self"`
	Mechs   []Mech  `json:"mechs"`
	Cubes   []Item  `json:"cubes"`
	Pickups []Item  `json:"pickups"`
	Zone    *Zone   `json:"zone"`
	Offer   *Offer  `json:"offer"`
}

var upg = []string{"nanites", "plating", "armor", "laser_damage", "laser_charge", "gun_damage", "gun_mag", "gun_rate",
	"lightning_efficiency", "lightning_range", "servos", "gun_velocity", "gyros", "lightning_chain", "reactor",
	"capacitor", "sensors", "jets", "mortar_reload", "mortar_velocity", "mortar_splash", "laser_pierce"}

func dist(a, b [3]float64) float64 { return math.Hypot(a[0]-b[0], a[1]-b[1]) }

func toward(from, to [3]float64) [2]float64 {
	d := dist(from, to)
	if d < 1e-6 {
		return [2]float64{}
	}
	return [2]float64{(to[0] - from[0]) / d, (to[1] - from[1]) / d}
}

func main() {
	rd := bufio.NewReaderSize(os.Stdin, 1<<22)
	out := bufio.NewWriter(os.Stdout)
	var strafe float64 = 1
	lastFlip := 0.0
	for {
		line, err := rd.ReadBytes('\n')
		if len(line) == 0 && err != nil {
			return
		}
		var o Obs
		if json.Unmarshal(line, &o) != nil {
			continue
		}
		switch o.Type {
		case "hello":
			fmt.Fprintln(out, `{"type":"ready","name":"go-C"}`)
			out.Flush()
			continue
		case "end":
			return
		case "obs":
		default:
			continue
		}
		// skip to newest queued obs
		for rd.Buffered() > 0 {
			l2, _ := rd.ReadBytes('\n')
			var o2 Obs
			if json.Unmarshal(l2, &o2) == nil {
				if o2.Type == "end" {
					return
				}
				if o2.Type == "obs" {
					o = o2
				}
			}
		}
		if o.T-lastFlip > 2.0 {
			strafe = -strafe
			lastFlip = o.T
		}
		act := decide(&o, strafe)
		b, _ := json.Marshal(act)
		out.Write(b)
		out.WriteByte('\n')
		out.Flush()
	}
}

func decide(o *Obs, strafe float64) map[string]interface{} {
	me := o.Self
	act := map[string]interface{}{"tick": o.Tick}
	if o.Offer != nil && len(o.Offer.Choices) > 0 {
		best, bi := 999, 0
		for i, c := range o.Offer.Choices {
			r := 50
			for j, u := range upg {
				if u == c.Id {
					r = j
				}
			}
			if r < best {
				best, bi = r, i
			}
		}
		act["pick"] = bi
	}
	// weapon slots
	slot := map[string]int{}
	empty := -1
	for i, w := range me.Weapons {
		if w == nil {
			if empty < 0 {
				empty = i
			}
		} else {
			slot[w.Kind] = i
		}
	}
	// nearest enemy mech
	var tgt *Mech
	td := 1e9
	for i := range o.Mechs {
		m := &o.Mechs[i]
		if m.Kind != "mech" {
			continue
		}
		if d := dist(me.Pos, m.Pos); d < td {
			td, tgt = d, m
		}
	}
	move := [2]float64{}
	hasMove := false
	fire := []bool{false, false}
	// loot: laser / lightning pickups
	if empty >= 0 {
		var bp *Item
		bd := 70.0
		for i := range o.Pickups {
			p := &o.Pickups[i]
			if p.Kind != "laser" && p.Kind != "lightning" {
				continue
			}
			if _, have := slot[p.Kind]; have {
				continue
			}
			if d := dist(me.Pos, p.Pos); d < bd && math.Abs(p.Pos[2]-me.Pos[2]) < 30 {
				bd, bp = d, p
			}
		}
		if bp != nil && (tgt == nil || td > 40) {
			move, hasMove = toward(me.Pos, bp.Pos), true
			if bd < 3.5 && math.Abs(bp.Pos[2]-me.Pos[2]) < 3.5 {
				act["take"] = bp.Id
			}
		}
	}
	// cubes
	if !hasMove && (tgt == nil || td > 60) {
		var bc *Item
		bd := 80.0
		for i := range o.Cubes {
			if d := dist(me.Pos, o.Cubes[i].Pos); d < bd {
				bd, bc = d, &o.Cubes[i]
			}
		}
		if bc != nil {
			move, hasMove = toward(me.Pos, bc.Pos), true
		}
	}
	// aim + fire
	if tgt != nil {
		gi, hg := slot["gun"]
		if !hg {
			gi = 0
		}
		speed := 252.0
		if w := me.Weapons; len(w) > gi && w[gi] != nil && w[gi].Stats.Speed > 0 {
			speed = w[gi].Stats.Speed
		}
		// lead target
		rel := [3]float64{tgt.Pos[0] - me.Pos[0], tgt.Pos[1] - me.Pos[1], tgt.Pos[2] + 4.5 - (me.Pos[2] + 7)}
		tt := math.Sqrt(rel[0]*rel[0]+rel[1]*rel[1]+rel[2]*rel[2]) / speed
		for k := 0; k < 2; k++ {
			px := rel[0] + (tgt.Vel[0]-me.Vel[0])*tt
			py := rel[1] + (tgt.Vel[1]-me.Vel[1])*tt
			pz := rel[2] + tgt.Vel[2]*tt
			tt = math.Sqrt(px*px+py*py+pz*pz) / speed
		}
		px := rel[0] + (tgt.Vel[0]-me.Vel[0])*tt
		py := rel[1] + (tgt.Vel[1]-me.Vel[1])*tt
		pz := rel[2] + tgt.Vel[2]*tt
		yaw := math.Atan2(py, px)
		pitch := math.Atan2(pz, math.Hypot(px, py))
		act["aim"] = [2]float64{yaw, pitch}
		if td < 230 {
			fire[gi] = true
		}
		if li, ok := slot["laser"]; ok && td < 270 {
			fire[li] = me.Weapons[li].Charge < 0.7
		}
		if li, ok := slot["lightning"]; ok && td < 42 {
			fire[li] = true
		}
		if !hasMove {
			// approach to ~70 m, strafe
			t := toward(me.Pos, tgt.Pos)
			s := [2]float64{-t[1] * strafe, t[0] * strafe}
			switch {
			case td > 90:
				move = [2]float64{t[0]*0.8 + s[0]*0.4, t[1]*0.8 + s[1]*0.4}
			case td < 40:
				move = [2]float64{-t[0]*0.5 + s[0], -t[1]*0.5 + s[1]}
			default:
				move = s
			}
			hasMove = true
		}
	}
	// zone
	if z := o.Zone; z != nil {
		c := z.Center
		r := z.Radius
		if o.T >= z.ShrinkStart {
			c, r = z.NextCenter, z.NextRadius
		}
		px := [3]float64{c[0], c[1], 0}
		d := dist(me.Pos, px)
		if d > r*0.8 || (!hasMove && d > 10) {
			move, hasMove = toward(me.Pos, px), true
		}
		if d > r {
			act["boost"] = true
		}
	}
	if hasMove {
		act["move"] = move
	}
	act["fire"] = fire
	return act
}

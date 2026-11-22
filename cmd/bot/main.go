package main

import (
	"bufio"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"time"
)

type Mech struct {
	ID    int       `json:"id"`
	Kind  string    `json:"kind"`
	Pos   []float64 `json:"pos"`
	Vel   []float64 `json:"vel"`
	Aim   []float64 `json:"aim"`
	HP    float64   `json:"hp"`
	MaxHP float64   `json:"max_hp"`
}
type Weapon struct {
	Kind     string  `json:"kind"`
	Cooldown float64 `json:"cooldown"`
	Ammo     int     `json:"ammo"`
	Reload   float64 `json:"reload"`
}
type Self struct {
	ID      int       `json:"id"`
	Pos     []float64 `json:"pos"`
	Vel     []float64 `json:"vel"`
	Aim     []float64 `json:"aim"`
	HP      float64   `json:"hp"`
	MaxHP   float64   `json:"max_hp"`
	Weapons []*Weapon `json:"weapons"`
}
type Obs struct {
	Type    string `json:"type"`
	Tick    int    `json:"tick"`
	Self    Self   `json:"self"`
	Mechs   []Mech `json:"mechs"`
	Pickups []struct {
		ID   int       `json:"id"`
		Kind string    `json:"kind"`
		Pos  []float64 `json:"pos"`
	} `json:"pickups"`
	Cubes []struct {
		Pos []float64 `json:"pos"`
	} `json:"cubes"`
	Zone struct {
		Center      []float64 `json:"center"`
		Radius      float64   `json:"radius"`
		NextCenter  []float64 `json:"next_center"`
		NextRadius  float64   `json:"next_radius"`
		ShrinkStart float64   `json:"shrink_start"`
	} `json:"zone"`
	T     float64 `json:"t"`
	Offer *struct {
		Choices []struct {
			ID string `json:"id"`
		} `json:"choices"`
	} `json:"offer"`
	Events []struct {
		Type string `json:"type"`
	} `json:"events"`
}
type Hello struct {
	Type string `json:"type"`
	Map  struct {
		Dir string `json:"dir"`
	} `json:"map"`
}

var nav = &Nav{}
var path [][2]float64
var pathGoal [2]float64
var pathTick = -100
var lastPos [2]float64
var stuck int
var sideSign = 1.0

func wrap(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a < -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// steer returns a unit move vector toward goal following A* path.
func steer(o *Obs, gx, gy float64) [2]float64 {
	px, py := o.Self.Pos[0], o.Self.Pos[1]
	if !nav.ok || nav.lineClear(px, py, gx, gy) && math.Hypot(gx-px, gy-py) < 30 {
		d := math.Hypot(gx-px, gy-py)
		if d < 1e-6 {
			return [2]float64{}
		}
		return [2]float64{(gx - px) / d, (gy - py) / d}
	}
	if o.Tick-pathTick >= 6 || math.Hypot(gx-pathGoal[0], gy-pathGoal[1]) > 8 || len(path) == 0 {
		path = nav.path(px, py, gx, gy, 12*time.Millisecond)
		pathGoal = [2]float64{gx, gy}
		pathTick = o.Tick
	}
	// furthest path point in line of sight within 40 m
	tx, ty := gx, gy
	if len(path) > 0 {
		best := -1
		for i := 0; i < len(path) && i < 60; i++ {
			if math.Hypot(path[i][0]-px, path[i][1]-py) > 40 {
				break
			}
			if nav.lineClear(px, py, path[i][0], path[i][1]) {
				best = i
			}
		}
		if best < 0 {
			best = 0
			for i := range path {
				if math.Hypot(path[i][0]-px, path[i][1]-py) > 3 {
					best = i
					break
				}
			}
		}
		tx, ty = path[best][0], path[best][1]
	}
	d := math.Hypot(tx-px, ty-py)
	if d < 1e-6 {
		return [2]float64{}
	}
	return [2]float64{(tx - px) / d, (ty - py) / d}
}

func act(o *Obs) map[string]interface{} {
	me := o.Self
	px, py, pz := me.Pos[0], me.Pos[1], me.Pos[2]
	a := map[string]interface{}{"tick": o.Tick}
	// stuck detection
	if math.Hypot(px-lastPos[0], py-lastPos[1]) < 0.15 {
		stuck++
	} else {
		stuck = 0
	}
	lastPos = [2]float64{px, py}

	// nearest enemy mech
	var tgt *Mech
	td := 1e9
	for i := range o.Mechs {
		m := &o.Mechs[i]
		if m.Kind != "mech" || m.ID == me.ID {
			continue
		}
		d := math.Hypot(m.Pos[0]-px, m.Pos[1]-py)
		if d < td {
			td, tgt = d, m
		}
	}
	// zone goal
	z := o.Zone
	gx, gy := z.Center[0], z.Center[1]
	if z.NextRadius > 0 && o.T > z.ShrinkStart-20 {
		gx, gy = z.NextCenter[0], z.NextCenter[1]
	}
	zr := z.Radius
	if z.NextRadius > 0 && o.T > z.ShrinkStart-20 {
		zr = z.NextRadius
	}
	dz := math.Hypot(px-gx, py-gy)
	outside := dz > zr*0.8
	goalX, goalY := gx, gy
	hold := !outside
	mode := "zone"
	if tgt != nil && td < 230 && !outside {
		mode = "fight"
		hold = false
		if td > 60 {
			goalX, goalY = tgt.Pos[0], tgt.Pos[1]
		} else {
			// strafe around target
			ang := math.Atan2(py-tgt.Pos[1], px-tgt.Pos[0]) + sideSign*0.5
			goalX, goalY = tgt.Pos[0]+45*math.Cos(ang), tgt.Pos[1]+45*math.Sin(ang)
			if o.Tick%40 == 0 {
				sideSign = -sideSign
			}
		}
	} else if !outside {
		// loot: nearest pickup within 70 m
		bd := 70.0
		for _, pk := range o.Pickups {
			d := math.Hypot(pk.Pos[0]-px, pk.Pos[1]-py)
			if d < bd {
				bd, goalX, goalY, hold = d, pk.Pos[0], pk.Pos[1], false
			}
			if d < 3.5 {
				a["take"] = pk.ID
			}
		}
		for _, c := range o.Cubes {
			d := math.Hypot(c.Pos[0]-px, c.Pos[1]-py)
			if d < bd {
				bd, goalX, goalY, hold = d, c.Pos[0], c.Pos[1], false
			}
		}
	}
	mv := [2]float64{}
	if !hold {
		mv = steer(o, goalX, goalY)
		if stuck > 8 { // wiggle off a snag
			ang := math.Atan2(mv[1], mv[0]) + sideSign*1.2
			mv = [2]float64{math.Cos(ang), math.Sin(ang)}
			if stuck > 20 {
				a["jump"] = true
				a["boost"] = true
			}
			path = nil
		}
	}
	a["move"] = mv
	_ = mode

	// aim / fire
	fire := []bool{false, false}
	if tgt != nil && td < 240 {
		tz := tgt.Pos[2] + 4.5
		mz := pz + 7
		tx, ty := tgt.Pos[0], tgt.Pos[1]
		vx, vy := 0.0, 0.0
		if len(tgt.Vel) >= 2 {
			vx, vy = tgt.Vel[0]-me.Vel[0]*0, tgt.Vel[1]
		}
		t := 0.0
		for k := 0; k < 3; k++ {
			dx, dy, dzz := tx+vx*t-px, ty+vy*t-py, tz-mz
			t = math.Sqrt(dx*dx+dy*dy+dzz*dzz) / 252.0
		}
		dx, dy := tx+vx*t-px, ty+vy*t-py
		yaw := math.Atan2(dy, dx)
		pitch := math.Atan2(tz-mz, math.Hypot(dx, dy))
		a["aim"] = []float64{yaw, pitch}
		cur := me.Aim[0]
		if math.Abs(wrap(yaw-cur)) < 0.25 {
			fire[0] = len(me.Weapons) > 0 && me.Weapons[0] != nil && me.Weapons[0].Ammo > 0
			if len(me.Weapons) > 1 && me.Weapons[1] != nil && me.Weapons[1].Kind == "lightning" && td < 40 {
				fire[1] = true
			}
		}
		if len(me.Weapons) > 0 && me.Weapons[0] != nil && me.Weapons[0].Ammo == 0 && me.Weapons[0].Reload == 0 {
			a["reload"] = true
		}
	} else if len(mv) == 2 && (mv[0] != 0 || mv[1] != 0) {
		a["aim"] = []float64{math.Atan2(mv[1], mv[0]), 0.0}
	}
	a["fire"] = fire
	if o.Offer != nil && len(o.Offer.Choices) > 0 {
		pick := 0
		for i, c := range o.Offer.Choices {
			if c.ID == "armor" {
				pick = i
			}
		}
		a["pick"] = pick
	}
	return a
}

func main() {
	in := make(chan []byte, 64)
	go func() {
		r := bufio.NewReaderSize(os.Stdin, 1<<20)
		for {
			l, err := r.ReadBytes('\n')
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
	send := func(v interface{}) {
		b, _ := json.Marshal(v)
		out.Write(b)
		out.WriteByte('\n')
		out.Flush()
	}
	for l := range in {
		// skip to newest queued
		for len(in) > 0 {
			var peek struct{ Type string }
			json.Unmarshal(l, &peek)
			if peek.Type != "obs" {
				break
			}
			l = <-in
		}
		var h struct{ Type string }
		if json.Unmarshal(l, &h) != nil {
			continue
		}
		switch h.Type {
		case "hello":
			var hl Hello
			json.Unmarshal(l, &hl)
			dir := hl.Map.Dir
			if dir == "" {
				dir = "/arena"
			}
			nav = loadNav(dir)
			fmt.Fprintln(os.Stderr, "nav ok:", nav.ok)
			send(map[string]interface{}{"type": "ready", "name": "gocool-go-G"})
		case "obs":
			var o Obs
			if json.Unmarshal(l, &o) != nil || len(o.Self.Pos) < 3 {
				continue
			}
			func() {
				defer func() {
					if r := recover(); r != nil {
						fmt.Fprintln(os.Stderr, "panic", r)
						send(map[string]interface{}{"tick": o.Tick})
					}
				}()
				send(act(&o))
			}()
		case "end":
			return
		}
	}
}

package main

import (
	"bufio"
	"encoding/json"
	"math"
	"os"
	"time"
)

type Weapon struct {
	Kind     string  `json:"kind"`
	Cooldown float64 `json:"cooldown"`
	Ammo     int     `json:"ammo"`
	Reload   float64 `json:"reload"`
	Charge   float64 `json:"charge"`
}
type Self struct {
	Pos      [3]float64 `json:"pos"`
	Vel      [3]float64 `json:"vel"`
	Aim      [2]float64 `json:"aim"`
	HP       float64    `json:"hp"`
	MaxHP    float64    `json:"max_hp"`
	InZone   bool       `json:"in_zone"`
	Weapons  []*Weapon  `json:"weapons"`
	Grounded bool       `json:"grounded"`
}
type Actor struct {
	ID   int        `json:"id"`
	Kind string     `json:"kind"`
	Pos  [3]float64 `json:"pos"`
	Vel  [3]float64 `json:"vel"`
	HP   float64    `json:"hp"`
}
type Pickup struct {
	ID   int        `json:"id"`
	Kind string     `json:"kind"`
	Pos  [3]float64 `json:"pos"`
}
type Cube struct {
	ID  int        `json:"id"`
	Pos [3]float64 `json:"pos"`
}
type Zone struct {
	Center [2]float64 `json:"center"`
	Radius float64    `json:"radius"`
}
type Offer struct {
	Choices []struct {
		ID string `json:"id"`
	} `json:"choices"`
}
type Msg struct {
	Type    string   `json:"type"`
	Tick    int      `json:"tick"`
	T       float64  `json:"t"`
	Self    Self     `json:"self"`
	Mechs   []Actor  `json:"mechs"`
	Pickups []Pickup `json:"pickups"`
	Cubes   []Cube   `json:"cubes"`
	Zone    Zone     `json:"zone"`
	Offer   *Offer   `json:"offer"`
}

var (
	strafeDir  = 1.0
	strafeNext = 0.0
	rng        = uint64(88172645463325252)
)

func rnd() float64 {
	rng ^= rng << 13
	rng ^= rng >> 7
	rng ^= rng << 17
	return float64(rng%1000000) / 1000000
}

func has(s *Self, kind string) int {
	for i, w := range s.Weapons {
		if w != nil && w.Kind == kind {
			return i
		}
	}
	return -1
}

func decide(m *Msg) map[string]interface{} {
	s := &m.Self
	act := map[string]interface{}{"tick": m.Tick}
	fire := []bool{false, false}
	px, py := s.Pos[0], s.Pos[1]
	if m.Offer != nil && len(m.Offer.Choices) > 0 {
		pick := 0
		for i, c := range m.Offer.Choices {
			if c.ID == "armor" {
				pick = i
			}
		}
		act["pick"] = pick
	}
	// weapon pickups
	for _, p := range m.Pickups {
		d := math.Hypot(p.Pos[0]-px, p.Pos[1]-py)
		if d < 3.5 && has(s, p.Kind) < 0 && (p.Kind == "laser" || p.Kind == "lightning") {
			act["take"] = p.ID
			if len(s.Weapons) >= 2 && s.Weapons[0] != nil && s.Weapons[1] != nil {
				act["drop"] = 1
			}
			break
		}
	}
	// nearest visible enemy mech
	var tgt *Actor
	best := 1e9
	for i := range m.Mechs {
		a := &m.Mechs[i]
		if a.Kind != "mech" {
			continue
		}
		d := math.Hypot(a.Pos[0]-px, a.Pos[1]-py)
		if d < best {
			best, tgt = d, a
		}
	}
	mx, my := 0.0, 0.0
	aimYaw, aimPitch := s.Aim[0], s.Aim[1]
	// zone pressure
	zdx, zdy := m.Zone.Center[0]-px, m.Zone.Center[1]-py
	zd := math.Hypot(zdx, zdy)
	outside := m.Zone.Radius > 0 && zd > m.Zone.Radius*0.85
	lasI, ligI := has(s, "laser"), has(s, "lightning")
	if tgt != nil {
		pref := 100.0
		if lasI >= 0 {
			pref = 120
		} else if ligI >= 0 {
			pref = 28
		}
		if m.T >= strafeNext {
			strafeDir = -strafeDir
			strafeNext = m.T + 2 + 2*rnd()
		}
		dx, dy := tgt.Pos[0]-px, tgt.Pos[1]-py
		d := math.Hypot(dx, dy)
		ux, uy := dx/d, dy/d
		rad := 0.0
		if d > pref+10 {
			rad = 1
		} else if d < pref-10 {
			rad = -1
		}
		mx = ux*rad + -uy*strafeDir*0.9
		my = uy*rad + ux*strafeDir*0.9
		// lead
		const bs = 252.0
		t := d / bs
		for k := 0; k < 3; k++ {
			tx := tgt.Pos[0] + tgt.Vel[0]*t - px
			ty := tgt.Pos[1] + tgt.Vel[1]*t - py
			t = math.Hypot(tx, ty) / bs
		}
		tx := tgt.Pos[0] + tgt.Vel[0]*t - px
		ty := tgt.Pos[1] + tgt.Vel[1]*t - py
		aimYaw = math.Atan2(ty, tx)
		aimPitch = math.Atan2(tgt.Pos[2]+4.5-(s.Pos[2]+7), math.Hypot(tx, ty))
		diff := math.Abs(math.Remainder(aimYaw-s.Aim[0], 2*math.Pi))
		onT := diff < 0.08
		if w := s.Weapons[0]; w != nil && w.Cooldown <= 0.02 && w.Reload <= 0 && w.Ammo > 0 && d < 225 && onT {
			fire[0] = true
		}
		if lasI >= 0 {
			w := s.Weapons[lasI]
			if d < 270 && w.Cooldown <= 0.02 {
				if w.Charge < 0.72 {
					fire[lasI] = true
				} else if onT {
					fire[lasI] = false // release
				} else {
					fire[lasI] = true
				}
			}
		}
		if ligI >= 0 && d < 40 && s.Weapons[ligI].Cooldown <= 0.02 {
			fire[ligI] = true
		}
	} else if len(m.Cubes) > 0 {
		c := m.Cubes[0]
		mx, my = c.Pos[0]-px, c.Pos[1]-py
	} else {
		// head toward zone center loosely
		mx, my = zdx, zdy
	}
	if outside {
		mx, my = zdx/zd+mx*0.3, zdy/zd+my*0.3
		act["boost"] = true
	}
	n := math.Hypot(mx, my)
	if n > 1 {
		mx, my = mx/n, my/n
	}
	act["move"] = []float64{mx, my}
	act["aim"] = []float64{aimYaw, aimPitch}
	act["fire"] = fire
	return act
}

func main() {
	in := bufio.NewReaderSize(os.Stdin, 1<<20)
	out := bufio.NewWriter(os.Stdout)
	enc := json.NewEncoder(out)
	var pending []byte
	_ = time.Now
	for {
		line, err := in.ReadBytes('\n')
		if len(line) > 0 {
			pending = line
		}
		// skip to newest queued
		for in.Buffered() > 0 {
			l2, e2 := in.ReadBytes('\n')
			if len(l2) > 0 {
				var h struct {
					Type string `json:"type"`
				}
				json.Unmarshal(l2, &h)
				if h.Type == "obs" || h.Type == "end" {
					pending = l2
				}
			}
			if e2 != nil {
				break
			}
		}
		if len(pending) > 0 {
			var m Msg
			if json.Unmarshal(pending, &m) == nil {
				switch m.Type {
				case "hello":
					enc.Encode(map[string]string{"type": "ready", "name": "gocool-go-B"})
					out.Flush()
				case "obs":
					if len(m.Self.Weapons) == 0 {
						m.Self.Weapons = []*Weapon{nil, nil}
					}
					for len(m.Self.Weapons) < 2 {
						m.Self.Weapons = append(m.Self.Weapons, nil)
					}
					enc.Encode(decide(&m))
					out.Flush()
				case "end":
					return
				}
			}
			pending = nil
		}
		if err != nil {
			return
		}
	}
}

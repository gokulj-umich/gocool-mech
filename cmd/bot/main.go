// Minimal Go port of the Gocool Mech Royale bot (zone goal, lead aim, gun/laser/lightning fire,
// laser pickup, armor upgrades, dodge, retreat). Navmap planner is stubbed (straight-line steering).
package main

import (
	"bufio"
	"encoding/json"
	"math"
	"os"
)

type V3 = [3]float64

type Weapon struct {
	Kind     string             `json:"kind"`
	Cooldown float64            `json:"cooldown"`
	Ammo     float64            `json:"ammo"`
	Reload   float64            `json:"reload"`
	Stats    map[string]float64 `json:"stats"`
}
type Self struct {
	ID      int                `json:"id"`
	Pos     V3                 `json:"pos"`
	Vel     V3                 `json:"vel"`
	Aim     [2]float64         `json:"aim"`
	HP      float64            `json:"hp"`
	MaxHP   float64            `json:"max_hp"`
	Energy  float64            `json:"energy"`
	Weapons []*Weapon          `json:"weapons"`
	Stats   map[string]float64 `json:"stats"`
}
type Mech struct {
	ID    int     `json:"id"`
	Kind  string  `json:"kind"`
	Pos   V3      `json:"pos"`
	Vel   V3      `json:"vel"`
	HP    float64 `json:"hp"`
	Cubes float64 `json:"cubes"`
}
type Pickup struct {
	ID   int    `json:"id"`
	Kind string `json:"kind"`
	Pos  V3     `json:"pos"`
}
type Zone struct {
	Center     [2]float64 `json:"center"`
	Radius     float64    `json:"radius"`
	NextCenter [2]float64 `json:"next_center"`
	NextRadius float64    `json:"next_radius"`
	ShrinkEnd  float64    `json:"shrink_end"`
}
type Choice struct {
	ID string `json:"id"`
}
type Offer struct {
	Choices []Choice `json:"choices"`
}
type Obs struct {
	Type    string   `json:"type"`
	Tick    int      `json:"tick"`
	T       float64  `json:"t"`
	Self    Self     `json:"self"`
	Mechs   []Mech   `json:"mechs"`
	Pickups []Pickup `json:"pickups"`
	Cubes   []struct {
		Pos V3 `json:"pos"`
	} `json:"cubes"`
	Zone  Zone   `json:"zone"`
	Offer *Offer `json:"offer"`
}

var upgArmor = []string{"plating", "armor", "laser_damage", "laser_charge", "nanites", "gun_damage", "gun_mag", "gun_rate", "lightning_efficiency", "lightning_range", "servos", "gun_velocity", "gyros", "lightning_chain", "reactor", "capacitor", "sensors", "jets"}

type Bot struct {
	strafeDir float64
	lastFlip  float64
	taken     map[int]bool
}

func wrap(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a < -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// zoneGoal mirrors Bot.zone_goal: returns goal (ok=false if inside the safe margin).
func zoneGoal(o *Obs) (gx, gy float64, ok bool) {
	z := o.Zone
	cx, cy, r := z.NextCenter[0], z.NextCenter[1], z.NextRadius
	if o.T >= z.ShrinkEnd-0.01 && z.NextRadius >= z.Radius-0.5 {
		cx, cy, r = z.Center[0], z.Center[1], z.Radius
	}
	var m float64
	switch {
	case r <= 30:
		m = r * 0.35
	case r <= 80:
		m = r * 0.5
	default:
		m = math.Max(r*0.55, r-60)
	}
	dx, dy := o.Self.Pos[0]-cx, o.Self.Pos[1]-cy
	d := math.Hypot(dx, dy)
	if d <= m {
		return 0, 0, false
	}
	s := m / math.Max(d, 1e-6)
	return cx + dx*s, cy + dy*s, true
}

// lead mirrors Bot.lead: yaw, pitch, horizontal dist.
func lead(me *Self, t *Mech, speed float64) (yaw, pitch, hd float64) {
	mh := me.Stats["muzzle_height"]
	if mh == 0 {
		mh = 7
	}
	mz := me.Pos[2] + mh
	mx, my := me.Pos[0], me.Pos[1]
	size := 1.0
	if t.Kind == "mech" {
		size += 0.05 * t.Cubes
	} else {
		size = 0.39
	}
	tx, ty, tz := t.Pos[0], t.Pos[1], t.Pos[2]+4.5*size
	vx, vy, vz := t.Vel[0], t.Vel[1], t.Vel[2]
	if math.Abs(vz) < 1 {
		vz = 0
	}
	tt := 0.0
	var px, py, pz float64
	for i := 0; i < 4; i++ {
		px, py, pz = tx+vx*tt, ty+vy*tt, tz+vz*tt*0.5
		d := math.Sqrt((px-mx)*(px-mx) + (py-my)*(py-my) + (pz-mz)*(pz-mz))
		if speed > 0 {
			tt = d / speed
		}
	}
	hd = math.Hypot(px-mx, py-my)
	return math.Atan2(py-my, px-mx), math.Atan2(pz-mz, math.Max(hd, 0.1)), hd
}

func (b *Bot) act(o *Obs) map[string]any {
	me := &o.Self
	act := map[string]any{"tick": o.Tick}
	if o.Offer != nil {
		best, bi := 999, 0
		for i, c := range o.Offer.Choices {
			r := 50
			for j, u := range upgArmor {
				if u == c.ID {
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
	gunSlot, laserSlot, ltSlot := -1, -1, -1
	for i, w := range me.Weapons {
		if w == nil {
			continue
		}
		switch w.Kind {
		case "gun":
			gunSlot = i
		case "laser":
			laserSlot = i
		case "lightning":
			ltSlot = i
		}
	}
	// pick up a laser if one is near and a slot is free (or replace slot 1)
	for _, p := range o.Pickups {
		if p.Kind != "laser" || laserSlot >= 0 || b.taken[p.ID] {
			continue
		}
		d := math.Hypot(p.Pos[0]-me.Pos[0], p.Pos[1]-me.Pos[1])
		if d < 3.5 {
			act["take"] = p.ID
			b.taken[p.ID] = true
			if len(me.Weapons) > 1 && me.Weapons[1] != nil {
				act["drop"] = 1
			}
		}
	}
	// nearest enemy mech
	var tgt *Mech
	bd := 1e9
	for i := range o.Mechs {
		m := &o.Mechs[i]
		if m.Kind != "mech" || m.ID == me.ID {
			continue
		}
		d := math.Hypot(m.Pos[0]-me.Pos[0], m.Pos[1]-me.Pos[1])
		if d < bd {
			bd, tgt = d, m
		}
	}
	hpFrac := me.HP / math.Max(me.MaxHP, 1)
	mvx, mvy := 0.0, 0.0
	aimYaw, aimPitch := me.Aim[0], me.Aim[1]
	fire := [2]bool{}

	gx, gy, outside := zoneGoal(o)
	if outside {
		mvx, mvy = gx-me.Pos[0], gy-me.Pos[1]
	}
	if tgt != nil {
		speed := 252.0
		if gunSlot >= 0 && me.Weapons[gunSlot].Stats["speed"] > 0 {
			speed = me.Weapons[gunSlot].Stats["speed"]
		}
		yaw, pitch, hd := lead(me, tgt, speed)
		aimYaw, aimPitch = yaw, pitch
		err := math.Abs(wrap(yaw - me.Aim[0]))
		if gunSlot >= 0 && hd < 230 && err < 0.12 && me.Weapons[gunSlot].Ammo > 0 {
			fire[gunSlot] = true
		}
		if ltSlot >= 0 && hd < 42 && err < 0.4 && me.Energy > 10 {
			fire[ltSlot] = true
		}
		if laserSlot >= 0 && hd < 270 && err < 0.03 && me.Weapons[laserSlot].Cooldown <= 0 {
			fire[laserSlot] = true
		}
		if !outside {
			if o.T-b.lastFlip > 1.5 {
				b.strafeDir = -b.strafeDir
				b.lastFlip = o.T
			}
			if hpFrac < 0.4 { // retreat
				mvx, mvy = me.Pos[0]-tgt.Pos[0], me.Pos[1]-tgt.Pos[1]
			} else {
				ux, uy := (tgt.Pos[0]-me.Pos[0])/math.Max(hd, 1), (tgt.Pos[1]-me.Pos[1])/math.Max(hd, 1)
				fwd := 0.0
				if hd > 60 {
					fwd = 0.6
				} else if hd < 30 {
					fwd = -0.5
				}
				mvx, mvy = ux*fwd-uy*b.strafeDir, uy*fwd+ux*b.strafeDir
			}
		}
	} else if !outside {
		// roam: nearest cube, else drift toward zone center
		bc := 1e9
		for _, c := range o.Cubes {
			d := math.Hypot(c.Pos[0]-me.Pos[0], c.Pos[1]-me.Pos[1])
			if d < bc {
				bc = d
				mvx, mvy = c.Pos[0]-me.Pos[0], c.Pos[1]-me.Pos[1]
			}
		}
		if bc > 1e8 {
			mvx, mvy = o.Zone.NextCenter[0]-me.Pos[0], o.Zone.NextCenter[1]-me.Pos[1]
			if math.Hypot(mvx, mvy) < 40 {
				mvx, mvy = 0, 0
			}
		}
	}
	if n := math.Hypot(mvx, mvy); n > 1e-6 {
		mvx, mvy = mvx/n, mvy/n
	}
	act["move"] = [2]float64{mvx, mvy}
	act["aim"] = [2]float64{aimYaw, aimPitch}
	act["fire"] = fire
	return act
}

func main() {
	in := bufio.NewReaderSize(os.Stdin, 1<<20)
	out := bufio.NewWriter(os.Stdout)
	send := func(v any) {
		bs, _ := json.Marshal(v)
		out.Write(bs)
		out.WriteByte('\n')
		out.Flush()
	}
	b := &Bot{strafeDir: 1, taken: map[int]bool{}}
	var pending []byte
	lines := make(chan []byte, 64)
	go func() {
		for {
			l, err := in.ReadBytes('\n')
			if len(l) > 0 {
				lines <- l
			}
			if err != nil {
				close(lines)
				return
			}
		}
	}()
	_ = pending
	for l := range lines {
		// skip to newest queued line
		for len(lines) > 0 {
			var peek struct {
				Type string `json:"type"`
			}
			json.Unmarshal(l, &peek)
			if peek.Type != "obs" {
				break
			}
			l = <-lines
		}
		var head struct {
			Type string `json:"type"`
		}
		if json.Unmarshal(l, &head) != nil {
			continue
		}
		switch head.Type {
		case "hello":
			send(map[string]any{"type": "ready", "name": "gocool-go"})
		case "obs":
			var o Obs
			if json.Unmarshal(l, &o) != nil {
				continue
			}
			send(b.act(&o))
		case "end":
			return
		}
	}
}

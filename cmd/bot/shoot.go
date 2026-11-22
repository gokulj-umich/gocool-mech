package main

import "math"

// Exact port of bot.py shoot / lead / mortar_aim / aligned (v63-p4).

func (b *Bot) lead(me *Self, tgt *Mech, speed float64) (yaw, pitch, hd float64) {
	mh := me.Stats.MuzzleHeight
	if mh == 0 {
		mh = 7.0
	}
	mz := me.Pos[2] + mh
	mx, my := me.Pos[0], me.Pos[1]
	tsize := 0.39
	if tgt.Kind == "mech" {
		tsize = 1.0 + 0.05*float64(tgt.Cubes)
	}
	tx, ty, tz := tgt.Pos[0], tgt.Pos[1], tgt.Pos[2]+4.5*tsize
	vx, vy, vz := tgt.Vel[0], tgt.Vel[1], tgt.Vel[2]
	if math.Abs(vz) < 1.0 {
		vz = 0
	}
	t := 0.0
	var px, py, pz float64
	for i := 0; i < 3; i++ {
		px, py, pz = tx+vx*t, ty+vy*t, tz+vz*t*0.5
		dist := math.Sqrt((px-mx)*(px-mx) + (py-my)*(py-my) + (pz-mz)*(pz-mz))
		if speed > 0 {
			t = dist / speed
		} else {
			t = 0
		}
	}
	px, py, pz = tx+vx*t, ty+vy*t, tz+vz*t*0.5
	hd = math.Hypot(px-mx, py-my)
	return math.Atan2(py-my, px-mx), math.Atan2(pz-mz, math.Max(hd, 0.1)), hd
}

func aligned(me *Self, yaw, pitch, dist, tol float64) bool {
	t := math.Atan2(tol, math.Max(dist, 1.0)) + 0.004
	return math.Abs(wrap(me.Aim[0]-yaw)) <= t && math.Abs(me.Aim[1]-pitch) <= t*1.5+0.01
}

func (b *Bot) mortarAim(me *Self, tgt *Mech, w *Weapon) (yaw, pitch, D float64, ok bool) {
	v := w.Stats.Speed
	if v == 0 {
		v = 72.0
	}
	g := 22.0
	mx, my := me.Pos[0], me.Pos[1]
	mz := me.Pos[2] + 9.0 + 0.5
	tx, ty, tz := tgt.Pos[0], tgt.Pos[1], tgt.Pos[2]+1.0
	vx, vy := tgt.Vel[0], tgt.Vel[1]
	T := 1.0
	var px, py, th float64
	for i := 0; i < 3; i++ {
		px, py = tx+vx*T*0.6, ty+vy*T*0.6
		D = math.Hypot(px-mx, py-my)
		dz := tz - mz
		disc := math.Pow(v, 4) - g*(g*D*D+2*dz*v*v)
		if disc < 0 {
			return 0, 0, 0, false
		}
		th = math.Atan((v*v - math.Sqrt(disc)) / (g * math.Max(D, 1.0)))
		T = D / math.Max(v*math.Cos(th), 1.0)
	}
	return math.Atan2(py-my, px-mx), th, D, true
}

// shoot returns aim and fire; slots are weapon indexes (-1 = none).
func (b *Bot) shoot(me *Self, tgt *Mech, gi, la, li, mo int, t float64) ([2]float64, [2]bool) {
	var fire [2]bool
	npc := tgt.Kind != "mech"
	d := hyp(tgt.Pos[0]-me.Pos[0], tgt.Pos[1]-me.Pos[1])
	tzo := 4.5
	if npc {
		tzo = 1.75
	}
	mh := me.Stats.MuzzleHeight
	if mh == 0 {
		mh = 7.0
	}
	vis := b.nav.los(me.Pos[0], me.Pos[1], me.Pos[2]+mh, tgt.Pos[0], tgt.Pos[1], tgt.Pos[2]+tzo)
	var aim [2]float64
	haveAim := false
	if li >= 0 {
		lw := me.Weapons[li]
		rng := lw.Stats.Range
		if rng == 0 {
			rng = 45
		}
		if d < rng*0.95 && vis {
			yaw := math.Atan2(tgt.Pos[1]-me.Pos[1], tgt.Pos[0]-me.Pos[0])
			aim = [2]float64{yaw, 0}
			haveAim = true
			cone := lw.Stats.ConeDeg
			if cone == 0 {
				cone = 30
			}
			en := lw.Stats.Energy
			if en == 0 {
				en = 3
			}
			if math.Abs(wrap(me.Aim[0]-yaw)) < cone*0.4*math.Pi/180 && lw.Cooldown <= 0.1 && me.Energy >= en {
				fire[li] = true
			}
		}
	}
	laserAimNow := false
	if la >= 0 && !(li >= 0 && fire[li]) {
		lw := me.Weapons[la]
		rng := lw.Stats.Range
		if rng == 0 {
			rng = 280
		}
		charge := lw.Charge
		full := lw.Stats.Charge
		if full == 0 {
			full = 0.75
		}
		hold := lw.Stats.HoldS
		if hold == 0 {
			hold = 2.0
		}
		minc := lw.Stats.MinCharge
		if minc == 0 {
			minc = 0.15
		}
		minf := lw.Stats.MinFrac
		if minf == 0 {
			minf = 0.35
		}
		dmg0 := lw.Stats.Damage
		if dmg0 == 0 {
			dmg0 = 315
		}
		ly, lp, lhd := b.lead(me, tgt, 1e9)
		if d < rng*0.97 && vis && t >= b.shorted && (li < 0 || d > 30) {
			if lw.Cooldown <= 0.15 {
				lethal := false
				if tgt.Kind == "mech" && charge >= minc && charge < full-0.12 {
					dmg := dmg0 * (minf + (1-minf)*charge/full)
					dmg *= 1 + 0.04*math.Min(float64(me.Cubes), 10)
					lethal = dmg*0.7 >= tgt.HP
				}
				if lethal {
					laserAimNow = true
					aim = [2]float64{ly, lp}
					haveAim = true
					fire[la] = !aligned(me, ly, lp, lhd, 1.2)
				} else if charge < full-0.12 {
					fire[la] = true
				} else {
					laserAimNow = true
					aim = [2]float64{ly, lp}
					haveAim = true
					if aligned(me, ly, lp, lhd, 1.4) {
						fire[la] = false
					} else {
						fire[la] = charge < full+hold-0.25
					}
				}
			}
		} else if charge > 0.2 {
			if vis || charge > full+hold-0.3 {
				fire[la] = false
			} else {
				fire[la] = true
			}
		} else if !vis && d < 160 && lw.Cooldown <= 0.15 && t >= b.shorted && charge < full {
			fire[la] = true
			aim = [2]float64{ly, lp}
			haveAim = true
		}
	}
	if mo >= 0 && !npc {
		mw := me.Weapons[mo]
		if d > 28 && d < 220 && mw.Cooldown <= 0.1 {
			if yaw, pitch, D, ok := b.mortarAim(me, tgt, mw); ok && (!haveAim || !vis) {
				aim = [2]float64{yaw, pitch}
				haveAim = true
				if aligned(me, yaw, pitch, D, 2.0) {
					fire[mo] = true
				}
			}
		}
	}
	if gi >= 0 {
		gw := me.Weapons[gi]
		spd := gw.Stats.Speed
		if spd == 0 {
			spd = 252
		}
		rng := gw.Stats.Range
		if rng == 0 {
			rng = 230
		}
		yaw, pitch, hd := b.lead(me, tgt, spd)
		if !haveAim || (!laserAimNow && li < 0) {
			aim = [2]float64{yaw, pitch}
			haveAim = true
		}
		if vis && hd < rng*0.97 && (gw.Ammo > 0 || (gw.Reload > 0 && gw.Reload <= 0.1)) && gw.Reload <= 0.1 {
			tol := 2.2
			if npc {
				tol = 1.2
			}
			if aligned(me, yaw, pitch, hd, tol) {
				fire[gi] = true
			}
		}
		if fire[gi] && la >= 0 && tgt.Kind == "mech" && !b.hitIDs[tgt.ID] &&
			me.Weapons[la].Cooldown <= 0.15 && t >= b.shorted {
			b2me := math.Atan2(me.Pos[1]-tgt.Pos[1], me.Pos[0]-tgt.Pos[0])
			if math.Abs(wrap(tgt.Aim[0]-b2me)) > math.Pi/2 {
				fire[gi] = false
			}
		}
	}
	if !haveAim {
		aim = [2]float64{math.Atan2(tgt.Pos[1]-me.Pos[1], tgt.Pos[0]-me.Pos[0]), 0}
	}
	return aim, fire
}

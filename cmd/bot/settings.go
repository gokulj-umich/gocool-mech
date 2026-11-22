package main

// Module-level settings of bot.py: P defaults + every P.update in order, upgrade orders, weapon rank.

const TAU = 2 * 3.14159265358979323846

var upgradePriority = []string{"plating", "gun_mag", "gun_damage", "laser_damage", "nanites", "armor", "laser_charge", "gun_rate",
	"lightning_efficiency", "servos", "gun_velocity", "sensors", "lightning_range", "gyros",
	"lightning_chain", "reactor", "capacitor", "jets", "mortar_reload", "mortar_velocity",
	"mortar_splash", "laser_pierce"}

var upgArmor []string

var weaponRank map[string]float64

func initSettings() {
	P["engage_r"] = 240.0
	P["passive"] = false
	P["close_r"] = 100.0
	P["strafe_min"] = 3.0
	P["strafe_max"] = 5.0
	P["want_gun"] = 80.0
	P["want_laser"] = 100.0
	P["want_lightning"] = 28.0
	P["retreat_hp"] = 0.22
	P["retreat_hp_multi"] = 0.38
	P["loot_r"] = 110.0
	P["npc_hp"] = 0.45
	P["crates"] = true
	P["laser_first"] = true
	P["lightning_top"] = false
	P["reload_cover"] = false
	P["hide_shuffle"] = false
	P["pacifist"] = false
	P["stealth"] = false
	P["stealth_fight_r"] = 50.0
	P["power_pos"] = false
	P["pp_start"] = 150.0
	P["pp_end"] = 330.0
	P["pp_climb_s"] = 7.0
	P["assassin"] = true
	P["mode_detect"] = true
	P["backstab_laser_first"] = true
	P["smart_targets"] = true
	P["gang_escape"] = false
	P["gang_n"] = 2.0
	P["gang_window"] = 3.0
	P["gang_hold"] = 4.0
	P["converge_n"] = 3.0
	P["recover_hp"] = 0.6
	P["endgame_t"] = 330.0
	P["commit"] = true
	P["heat_max"] = 2.0
	P["ghost_max_age"] = 12.0
	P["roof_safe"] = false
	P["standoff"] = true
	P["standoff_r"] = 180.0
	P["laser_min_r"] = 0.0
	P["laser_min_finish"] = 0.15
	P["laser_cd_window"] = 1.3
	P["powerup_until"] = 100.0
	P["powerup_r"] = 150.0
	P["roam_until"] = 300.0
	P["roam_hp"] = 0.6
	P["aggressive"] = true
	P["hunt_hidden"] = true
	P["hold_angles"] = false
	P["ghost_hunt"] = true
	P["edge_hide"] = false
	P["cube_r"] = 110.0
	P["crate_r"] = 110.0
	P["npc_r"] = 160.0
	P["npc_stand"] = 62.0
	P["laser_hunt_r"] = 320.0
	P["open_scan_s"] = 3.0
	P["drops"] = true
	P["drop_r"] = 200.0
	P["heal_to"] = 0.6
	P["drop_heal"] = true

	P["press_hp"] = 0.45
	P["press_gap"] = 0.25
	P["press_r"] = 45.0
	P["forget_dead"] = false
	P["farm_hp_gate"] = false
	P["drone_avoid"] = false
	P["drone_avoid_hp"] = 0.8
	P["press_multi_block"] = false
	P["fight_boost"] = true
	P["retreat_hidden_w"] = 60.0
	P["gang_fire"] = true

	P["laser_grab"] = true
	P["laser_grab_r"] = 90.0
	P["recover_hp"] = 0.0
	P["endgame_t"] = 9999.0
	P["retreat_hp_multi"] = 0.25
	P["drone_avoid"] = true
	P["forget_dead"] = true
	P["farm_hp_gate"] = true

	P["quiet"] = false
	P["quiet_scale"] = 0.4
	P["heal_first"] = false
	P["sit_heal"] = false
	P["cube_fight_r"] = 0.0
	P["cube_fight_hp"] = 0.35
	P["upg"] = "base"
	P["lose_rule"] = false
	P["lethal_laser"] = false
	P["fight_expo"] = 0.0
	P["cover_retreat"] = false
	P["cover_max"] = 60.0
	P["cornered_fight"] = true
	P["avoid_r"] = 0.0
	P["avoid_w"] = 1.2
	P["avoid_loot"] = 60.0
	P["stick"] = 0.0
	P["stick_hurt"] = 0.0
	P["solo_duel"] = false
	P["solo_hp"] = 0.25
	P["blind_fire"] = false
	P["retreat_holdfire"] = false
	P["g_outgun"] = false
	P["g_swamp"] = false
	P["g_target"] = false
	P["econ"] = false
	P["econ_r"] = 260.0
	P["econ_npc_hp"] = 0.6
	P["econ_boost"] = false
	P["econ_give_up"] = 30.0
	P["econ_v_cube"] = 110.0
	P["econ_v_crate"] = 105.0
	P["econ_v_npc"] = 170.0
	P["threat_pri"] = 0.0
	P["pace_s"] = 0.0
	P["pace_hp"] = 0.9
	P["reactive"] = false
	P["react_r"] = 60.0
	P["regen"] = false
	P["regen_hp"] = 0.8
	P["regen_to"] = 0.95
	P["regen_fin"] = 0.3
	P["fight_spot"] = false
	P["spot_expo"] = 30.0
	P["spot_los"] = 45.0
	P["spot_duck"] = true
	P["idle_still"] = false
	P["idle_still_hp"] = 0.0

	P["retreat_hp"] = 0.38
	P["recover_hp"] = 0.8
	P["lethal_laser"] = true
	P["upg"] = "armor"

	P["econ"] = true
	P["cube_fight_r"] = 70.0
	P["cube_fight_hp"] = 0.3
	P["threat_pri"] = 90.0

	P["retreat_hp_multi"] = 0.5
	P["lose_rule"] = false
	P["fight_expo"] = 30.0
	P["strafe_min"] = 3.0
	P["strafe_max"] = 5.0
	P["want_gun"] = 100.0
	P["engage_r"] = 300.0
	P["smart_targets"] = false
	P["commit"] = false
	P["laser_grab_r"] = 60.0

	P["solo_duel"] = true
	P["solo_hp"] = 0.25
	P["recover_hp"] = 0.8
	P["retreat_hp"] = 0.45
	P["avoid_r"] = 80.0
	P["want_laser"] = 130.0

	P["laser_hunt_r"] = 320.0
	P["want_laser"] = 120.0
	P["gang_escape"] = false
	P["lose_rule"] = false
	P["solo_hp"] = 0.15
	P["press_gap"] = 0.12
	P["avoid_r"] = 0.0
	P["cube_fight_hp"] = 0.3
	P["strafe_min"] = 2.0
	P["strafe_max"] = 4.0
	P["press_hp"] = 0.38
	P["econ_npc_hp"] = 0.5
	P["cube_fight_r"] = 100.0
	P["want_gun"] = 100.0

	base := []string{"plating", "armor", "laser_damage", "laser_charge", "nanites", "gun_damage", "gun_mag", "gun_rate",
		"lightning_efficiency", "lightning_range", "servos", "gun_velocity", "gyros", "lightning_chain", "reactor",
		"capacitor", "sensors", "jets", "mortar_reload", "mortar_velocity", "mortar_splash", "laser_pierce"}
	front := []string{"nanites", "plating", "armor", "laser_damage", "laser_charge"}
	upgArmor = append([]string{}, front...)
	for _, u := range base {
		skip := false
		for _, f := range front {
			if u == f {
				skip = true
			}
		}
		if !skip {
			upgArmor = append(upgArmor, u)
		}
	}
	weaponRank = map[string]float64{"lightning": 3.0, "laser": 2.6, "mortar": 1.0, "gun": 0.0}
	if pb("laser_first") {
		weaponRank["laser"] = 4.0
		weaponRank["lightning"] = 2.5
	}
	if pb("lightning_top") {
		weaponRank["lightning"] = 3.5
	}
}

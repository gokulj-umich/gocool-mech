// Shared conventions for the 1:1 port of bot.py (our own code).
// Python dicts stay dynamic: obs / hello / per-mech dicts are map[string]any (decoded JSON).
// Bot state: Python `self.x` -> b.S["x"] (any); World/Planner are real structs in world.go.
// Settings: Python P["k"] -> pf("k") float, pb("k") bool, ps("k") string. Defaults+P.update live in settings.go.
package main

import "math"

type M = map[string]any

var P = map[string]any{}

func pf(k string) float64 { return num(P[k]) }
func pb(k string) bool    { v, _ := P[k].(bool); return v }
func ps(k string) string  { v, _ := P[k].(string); return v }

// num converts JSON/python-ish values to float64 (bool -> 0/1, nil -> 0)
func num(v any) float64 {
	switch x := v.(type) {
	case float64:
		return x
	case int:
		return float64(x)
	case bool:
		if x {
			return 1
		}
		return 0
	}
	return 0
}
func mf(m M, k string) float64  { return num(m[k]) }
func mm(m M, k string) M        { v, _ := m[k].(M); return v }
func ma(m M, k string) []any    { v, _ := m[k].([]any); return v }
func ms(m M, k string) string   { v, _ := m[k].(string); return v }
func mb(m M, k string) bool     { v, _ := m[k].(bool); return v }
func vec(v any) []float64 {
	a, _ := v.([]any)
	out := make([]float64, len(a))
	for i, x := range a {
		out[i] = num(x)
	}
	return out
}
func wrap(a float64) float64 { return math.Mod(math.Mod(a+math.Pi, 2*math.Pi)+2*math.Pi, 2*math.Pi) - math.Pi }
func d2(a, b []float64) float64 { return math.Hypot(a[0]-b[0], a[1]-b[1]) }

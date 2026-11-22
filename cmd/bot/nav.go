package main

import (
	"container/heap"
	"encoding/binary"
	"encoding/json"
	"math"
	"os"
	"time"
)

const cs = 2.0 // nav cell size, m

type Nav struct {
	ok       bool
	x0, y0   float64
	W, H     int
	block    []bool
	nx, ny   int
	tx0, ty0 float64
	terr     []int16
}

type prism struct {
	Base, Top float64
	Ring      []float64
	Holes     [][]float64
}

func pip(poly []float64, x, y float64) bool {
	in := false
	n := len(poly) / 2
	for i, j := 0, n-1; i < n; j, i = i, i+1 {
		xi, yi, xj, yj := poly[2*i], poly[2*i+1], poly[2*j], poly[2*j+1]
		if (yi > y) != (yj > y) && x < (xj-xi)*(y-yi)/(yj-yi)+xi {
			in = !in
		}
	}
	return in
}

func (n *Nav) ground(x, y float64) float64 {
	if n.terr == nil {
		return 0
	}
	i := int(math.Round(x - n.tx0))
	j := int(math.Round(y - n.ty0))
	if i < 0 {
		i = 0
	}
	if j < 0 {
		j = 0
	}
	if i >= n.nx {
		i = n.nx - 1
	}
	if j >= n.ny {
		j = n.ny - 1
	}
	return float64(n.terr[j*n.nx+i]) * 0.01
}

func loadNav(dir string) *Nav {
	n := &Nav{}
	defer func() { recover() }()
	b, err := os.ReadFile(dir + "/v2/world.json")
	if err != nil {
		return n
	}
	var w struct {
		Bounds  struct{ MinX, MaxX, MinY, MaxY float64 }
		Terrain struct {
			MinX, MinY float64
			Nx, Ny     int
		}
		Prisms []struct {
			Base, Top float64
			Ring      []float64
			Holes     [][]float64
		}
	}
	if json.Unmarshal(b, &w) != nil {
		return n
	}
	tb, err := os.ReadFile(dir + "/v2/terrain.i16")
	if err == nil && len(tb) >= w.Terrain.Nx*w.Terrain.Ny*2 {
		n.nx, n.ny, n.tx0, n.ty0 = w.Terrain.Nx, w.Terrain.Ny, w.Terrain.MinX, w.Terrain.MinY
		n.terr = make([]int16, n.nx*n.ny)
		for i := range n.terr {
			n.terr[i] = int16(binary.LittleEndian.Uint16(tb[2*i:]))
		}
	}
	n.x0, n.y0 = w.Bounds.MinX, w.Bounds.MinY
	n.W = int((w.Bounds.MaxX-w.Bounds.MinX)/cs) + 1
	n.H = int((w.Bounds.MaxY-w.Bounds.MinY)/cs) + 1
	raw := make([]bool, n.W*n.H)
	for _, p := range w.Prisms {
		minx, maxx, miny, maxy := 1e9, -1e9, 1e9, -1e9
		for k := 0; k+1 < len(p.Ring); k += 2 {
			minx, maxx = math.Min(minx, p.Ring[k]), math.Max(maxx, p.Ring[k])
			miny, maxy = math.Min(miny, p.Ring[k+1]), math.Max(maxy, p.Ring[k+1])
		}
		i0, i1 := int((minx-n.x0)/cs)-1, int((maxx-n.x0)/cs)+1
		j0, j1 := int((miny-n.y0)/cs)-1, int((maxy-n.y0)/cs)+1
		for j := max(j0, 0); j <= min(j1, n.H-1); j++ {
			for i := max(i0, 0); i <= min(i1, n.W-1); i++ {
				x, y := n.x0+cs*float64(i), n.y0+cs*float64(j)
				if !pip(p.Ring, x, y) {
					continue
				}
				hole := false
				for _, h := range p.Holes {
					if pip(h, x, y) {
						hole = true
						break
					}
				}
				if hole {
					continue
				}
				g := n.ground(x, y)
				if p.Base < g+9.0 && p.Top > g+1.2 {
					raw[j*n.W+i] = true
				}
			}
		}
	}
	// slope + map edge
	for j := 0; j < n.H; j++ {
		for i := 0; i < n.W; i++ {
			x, y := n.x0+cs*float64(i), n.y0+cs*float64(j)
			g := n.ground(x, y)
			s := math.Max(math.Abs(n.ground(x+cs, y)-g), math.Abs(n.ground(x, y+cs)-g)) / cs
			if s > 1.0 {
				raw[j*n.W+i] = true
			}
		}
	}
	// dilate by 1 cell (mech radius 2.5 m)
	n.block = make([]bool, n.W*n.H)
	for j := 0; j < n.H; j++ {
		for i := 0; i < n.W; i++ {
			if !raw[j*n.W+i] {
				continue
			}
			for dj := -1; dj <= 1; dj++ {
				for di := -1; di <= 1; di++ {
					ii, jj := i+di, j+dj
					if ii >= 0 && jj >= 0 && ii < n.W && jj < n.H {
						n.block[jj*n.W+ii] = true
					}
				}
			}
		}
	}
	n.ok = true
	return n
}

func (n *Nav) cell(x, y float64) (int, int) {
	i := int(math.Round((x - n.x0) / cs))
	j := int(math.Round((y - n.y0) / cs))
	return min(max(i, 0), n.W-1), min(max(j, 0), n.H-1)
}

func (n *Nav) free(i, j int) bool {
	return i >= 0 && j >= 0 && i < n.W && j < n.H && !n.block[j*n.W+i]
}

// nearest free cell by ring search
func (n *Nav) nearFree(i, j int) (int, int) {
	for r := 0; r < 30; r++ {
		for dj := -r; dj <= r; dj++ {
			for di := -r; di <= r; di++ {
				if max(abs(di), abs(dj)) == r && n.free(i+di, j+dj) {
					return i + di, j + dj
				}
			}
		}
	}
	return i, j
}

func abs(a int) int {
	if a < 0 {
		return -a
	}
	return a
}

func (n *Nav) lineClear(ax, ay, bx, by float64) bool {
	d := math.Hypot(bx-ax, by-ay)
	k := int(d/1.0) + 1
	for s := 1; s <= k; s++ {
		t := float64(s) / float64(k)
		i, j := n.cell(ax+(bx-ax)*t, ay+(by-ay)*t)
		if !n.free(i, j) {
			return false
		}
	}
	return true
}

type item struct {
	f float64
	k int
}
type pq []item

func (p pq) Len() int            { return len(p) }
func (p pq) Less(a, b int) bool  { return p[a].f < p[b].f }
func (p pq) Swap(a, b int)       { p[a], p[b] = p[b], p[a] }
func (p *pq) Push(x interface{}) { *p = append(*p, x.(item)) }
func (p *pq) Pop() interface{} {
	o := *p
	x := o[len(o)-1]
	*p = o[:len(o)-1]
	return x
}

var nb = [8][3]float64{{1, 0, 1}, {-1, 0, 1}, {0, 1, 1}, {0, -1, 1}, {1, 1, 1.4142}, {1, -1, 1.4142}, {-1, 1, 1.4142}, {-1, -1, 1.4142}}

// path returns waypoints (world coords) from s to g, time capped; best partial if cap hit.
func (n *Nav) path(sx, sy, gx, gy float64, budget time.Duration) [][2]float64 {
	if !n.ok {
		return [][2]float64{{gx, gy}}
	}
	si, sj := n.cell(sx, sy)
	gi, gj := n.cell(gx, gy)
	si, sj = n.nearFree(si, sj)
	gi, gj = n.nearFree(gi, gj)
	N := n.W * n.H
	g := make([]float32, N)
	for i := range g {
		g[i] = 1e18
	}
	par := make([]int32, N)
	closed := make([]bool, N)
	h := func(i, j int) float64 {
		dx, dy := math.Abs(float64(i-gi)), math.Abs(float64(j-gj))
		return math.Max(dx, dy) + 0.4142*math.Min(dx, dy)
	}
	sk := sj*n.W + si
	g[sk] = 0
	par[sk] = -1
	op := &pq{{h(si, sj), sk}}
	end := time.Now().Add(budget)
	best, bh := sk, h(si, sj)
	cnt := 0
	for op.Len() > 0 {
		cnt++
		if cnt&255 == 0 && time.Now().After(end) {
			break
		}
		it := heap.Pop(op).(item)
		k := it.k
		if closed[k] {
			continue
		}
		closed[k] = true
		i, j := k%n.W, k/n.W
		if hv := h(i, j); hv < bh {
			bh, best = hv, k
		}
		if i == gi && j == gj {
			best = k
			break
		}
		for _, d := range nb {
			ii, jj := i+int(d[0]), j+int(d[1])
			if !n.free(ii, jj) {
				continue
			}
			if d[0] != 0 && d[1] != 0 && (!n.free(ii, j) || !n.free(i, jj)) {
				continue
			}
			kk := jj*n.W + ii
			if closed[kk] {
				continue
			}
			ng := g[k] + float32(d[2])
			if ng < g[kk] {
				g[kk] = ng
				par[kk] = int32(k)
				heap.Push(op, item{float64(ng) + h(ii, jj), kk})
			}
		}
	}
	var rev [][2]float64
	for k := best; k != -1; k = int(par[k]) {
		rev = append(rev, [2]float64{n.x0 + cs*float64(k%n.W), n.y0 + cs*float64(k/n.W)})
	}
	out := make([][2]float64, 0, len(rev))
	for i := len(rev) - 1; i >= 0; i-- {
		out = append(out, rev[i])
	}
	return out
}

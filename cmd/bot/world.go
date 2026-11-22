package main

import (
	"archive/zip"
	"container/heap"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
	"time"
)

// ---------------------------------------------------------------- npz reader

type npyArr struct {
	shape []int
	f64   []float64 // all numeric data widened to float64 (exact for f2/f4/i2/b1/f8)
}

var npyShapeRe = regexp.MustCompile(`'shape':\s*\(([^)]*)\)`)
var npyDescrRe = regexp.MustCompile(`'descr':\s*'([^']*)'`)

func float16To64(h uint16) float64 {
	s := 1.0
	if h&0x8000 != 0 {
		s = -1.0
	}
	e := int((h >> 10) & 0x1f)
	m := float64(h & 0x3ff)
	switch e {
	case 0:
		return s * m * math.Ldexp(1, -24)
	case 31:
		if m == 0 {
			return s * math.Inf(1)
		}
		return math.NaN()
	}
	return s * (1 + m/1024) * math.Ldexp(1, e-15)
}

func parseNpy(b []byte) (*npyArr, error) {
	if len(b) < 10 || string(b[:6]) != "\x93NUMPY" {
		return nil, fmt.Errorf("bad npy magic")
	}
	var hl, off int
	if b[6] == 1 {
		hl = int(binary.LittleEndian.Uint16(b[8:10]))
		off = 10
	} else {
		hl = int(binary.LittleEndian.Uint32(b[8:12]))
		off = 12
	}
	hdr := string(b[off : off+hl])
	data := b[off+hl:]
	dm := npyDescrRe.FindStringSubmatch(hdr)
	sm := npyShapeRe.FindStringSubmatch(hdr)
	if dm == nil || sm == nil {
		return nil, fmt.Errorf("bad npy header %q", hdr)
	}
	var shape []int
	n := 1
	for _, p := range strings.Split(sm[1], ",") {
		p = strings.TrimSpace(p)
		if p == "" {
			continue
		}
		v, _ := strconv.Atoi(p)
		shape = append(shape, v)
		n *= v
	}
	out := make([]float64, n)
	switch dm[1] {
	case "<f8":
		for i := range out {
			out[i] = math.Float64frombits(binary.LittleEndian.Uint64(data[i*8:]))
		}
	case "<f4":
		for i := range out {
			out[i] = float64(math.Float32frombits(binary.LittleEndian.Uint32(data[i*4:])))
		}
	case "<f2":
		for i := range out {
			out[i] = float16To64(binary.LittleEndian.Uint16(data[i*2:]))
		}
	case "<i2":
		for i := range out {
			out[i] = float64(int16(binary.LittleEndian.Uint16(data[i*2:])))
		}
	case "|b1", "|u1":
		for i := range out {
			out[i] = float64(data[i])
		}
	default:
		return nil, fmt.Errorf("unsupported dtype %s", dm[1])
	}
	return &npyArr{shape: shape, f64: out}, nil
}

func loadNpz(path string) (map[string]*npyArr, error) {
	zr, err := zip.OpenReader(path)
	if err != nil {
		return nil, err
	}
	defer zr.Close()
	res := map[string]*npyArr{}
	for _, f := range zr.File {
		rc, err := f.Open()
		if err != nil {
			return nil, err
		}
		b, err := io.ReadAll(rc)
		rc.Close()
		if err != nil {
			return nil, err
		}
		a, err := parseNpy(b)
		if err != nil {
			return nil, fmt.Errorf("%s: %v", f.Name, err)
		}
		res[strings.TrimSuffix(f.Name, ".npy")] = a
	}
	return res, nil
}

// navmapPath finds navmap.npz: $NAVMAP, next to the executable, cwd, /app.
func navmapPath() string {
	var c []string
	if p := os.Getenv("NAVMAP"); p != "" {
		c = append(c, p)
	}
	if ex, err := os.Executable(); err == nil {
		c = append(c, filepath.Join(filepath.Dir(ex), "navmap.npz"))
	}
	c = append(c, "navmap.npz", "/app/navmap.npz", "../../navmap.npz")
	for _, p := range c {
		if _, err := os.Stat(p); err == nil {
			return p
		}
	}
	return "navmap.npz"
}

// ---------------------------------------------------------------- World

type World struct {
	X0, Y0 float64
	Nx, Ny int
	Terr   []float64 // ny*nx
	Lo, Hi []float64 // ny*nx
	Walk   []bool    // H*W (walk_l)
	Surf   []float64 // H*W (surf_l)
	Cover  []float64
	Near   []int // 2*H*W
	Roof2  []int
	Roofs  [][]float64 // (n,6) rows
	Climb  [][]float64 // (n,3) rows
	H, W   int
}

func NewWorld() *World {
	d, err := loadNpz(navmapPath())
	if err != nil {
		panic(err)
	}
	w := &World{}
	meta := d["meta"].f64
	w.X0, w.Y0 = meta[0], meta[1]
	w.Nx, w.Ny = int(meta[2]), int(meta[3])
	w.Terr = d["terrain"].f64
	w.Lo = d["lo"].f64
	w.Hi = d["hi"].f64
	w.H, w.W = d["walk2"].shape[0], d["walk2"].shape[1]
	w.Walk = make([]bool, len(d["walk2"].f64))
	for i, v := range d["walk2"].f64 {
		w.Walk[i] = v != 0
	}
	w.Surf = d["surf2"].f64
	w.Cover = d["cover2"].f64
	w.Near = make([]int, len(d["near2"].f64))
	for i, v := range d["near2"].f64 {
		w.Near[i] = int(v)
	}
	if r, ok := d["roof2"]; ok {
		w.Roof2 = make([]int, len(r.f64))
		for i, v := range r.f64 {
			w.Roof2[i] = int(v)
		}
	} else {
		w.Roof2 = make([]int, w.H*w.W)
		for i := range w.Roof2 {
			w.Roof2[i] = -1
		}
	}
	if r, ok := d["roofs"]; ok {
		w.Roofs = rows(r.f64, 6)
	}
	if r, ok := d["climb"]; ok {
		w.Climb = rows(r.f64, 3)
	}
	return w
}

func clampi(v, lo, hi int) int {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

// Cell: 2 m nav grid
func (w *World) Cell(x, y float64) (int, int) {
	i := int(math.RoundToEven((x - w.X0) / 2.0))
	j := int(math.RoundToEven((y - w.Y0) / 2.0))
	return clampi(i, 0, w.W-1), clampi(j, 0, w.H-1)
}

func (w *World) Cpos(i, j int) (float64, float64) {
	return w.X0 + 2.0*float64(i), w.Y0 + 2.0*float64(j)
}

func (w *World) Walkable(x, y float64) bool {
	i, j := w.Cell(x, y)
	return w.Walk[j*w.W+i]
}

func (w *World) RoofAt(x, y float64) int {
	i, j := w.Cell(x, y)
	return w.Roof2[j*w.W+i]
}

// NearestWalk returns (i, j) of the nearest walkable cell.
func (w *World) NearestWalk(i, j int) (int, int) {
	hw := w.H * w.W
	return w.Near[hw+j*w.W+i], w.Near[j*w.W+i]
}

func (w *World) Ground(x, y float64) float64 {
	i := clampi(int(math.RoundToEven(x-w.X0)), 0, w.Nx-1)
	j := clampi(int(math.RoundToEven(y-w.Y0)), 0, w.Ny-1)
	return w.Terr[j*w.Nx+i]
}

// Los: true if the straight 3D segment a->b is not blocked by solids or terrain. (python default step=1.5)
func (w *World) Los(a, b []float64, step float64) bool {
	dx, dy, dz := b[0]-a[0], b[1]-a[1], b[2]-a[2]
	L := math.Sqrt(dx*dx + dy*dy + dz*dz)
	n := int(L / step)
	if n < 2 {
		return true
	}
	// np.linspace(0,1,n+1)[1:-1]: t_k = k*(1/n)
	st := 1.0 / float64(n)
	for k := 1; k < n; k++ {
		t := float64(k) * st
		x := a[0] + dx*t
		y := a[1] + dy*t
		z := a[2] + dz*t
		i := clampi(int(math.RoundToEven(x-w.X0)), 0, w.Nx-1)
		j := clampi(int(math.RoundToEven(y-w.Y0)), 0, w.Ny-1)
		p := j*w.Nx + i
		if z < w.Terr[p]-0.3 || (z >= w.Lo[p] && z <= w.Hi[p]) {
			return false
		}
	}
	return true
}

// ClearLine: walkable straight line on the nav grid between 2D points.
func (w *World) ClearLine(a, b []float64) bool {
	L := d2(a, b)
	n := int(L / 1.5)
	if n < 2 {
		n = 2
	}
	for k := 1; k <= n; k++ {
		t := float64(k) / float64(n)
		if !w.Walkable(a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t) {
			return false
		}
	}
	return true
}

// ---------------------------------------------------------------- Planner

type nb struct {
	di, dj int
	c      float64
}

var NB = [8]nb{{1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
	{1, 1, 1.4142}, {1, -1, 1.4142}, {-1, 1, 1.4142}, {-1, -1, 1.4142}}

type item struct {
	f, g float64
	k    int
}

type pq []item

func (h pq) Len() int { return len(h) }
func (h pq) Less(a, b int) bool {
	x, y := h[a], h[b]
	if x.f != y.f {
		return x.f < y.f
	}
	if x.g != y.g {
		return x.g < y.g
	}
	return x.k < y.k
}
func (h pq) Swap(a, b int) { h[a], h[b] = h[b], h[a] }
func (h *pq) Push(x any)   { *h = append(*h, x.(item)) }
func (h *pq) Pop() any {
	o := *h
	n := len(o)
	x := o[n-1]
	*h = o[:n-1]
	return x
}

// Planner: reverse A* from a goal; settled g values form a distance-to-goal field.
type Planner struct {
	w      *World
	Goal   int // -1 = None
	G      map[int]float64
	Closed map[int]bool
	open   pq
	Target int // -1 = None
}

func NewPlanner(w *World) *Planner {
	return &Planner{w: w, Goal: -1, G: map[int]float64{}, Closed: map[int]bool{}, Target: -1}
}

func (p *Planner) SetGoal(gi, gj int) {
	w := p.w
	if !w.Walk[gj*w.W+gi] {
		gi, gj = w.NearestWalk(gi, gj)
	}
	k := gj*w.W + gi
	if p.Goal == k {
		return
	}
	p.Goal = k
	p.G = map[int]float64{k: 0.0}
	p.Closed = map[int]bool{}
	p.open = pq{{0.0, 0.0, k}}
	p.Target = -1
}

func (p *Planner) H(k, tk int) float64 {
	W := p.w.W
	dx := k%W - tk%W
	if dx < 0 {
		dx = -dx
	}
	dy := k/W - tk/W
	if dy < 0 {
		dy = -dy
	}
	mx, mn := dx, dy
	if dy > dx {
		mx, mn = dy, dx
	}
	return float64(mx) + 0.4142*float64(mn)
}

func (p *Planner) Search(si, sj int, budgetS float64) bool {
	w := p.w
	if p.Goal < 0 {
		return false
	}
	if !w.Walk[sj*w.W+si] {
		si, sj = w.NearestWalk(si, sj)
	}
	sk := sj*w.W + si
	if p.Closed[sk] {
		return true
	}
	if p.Target != sk {
		p.Target = sk
		no := make(pq, 0, len(p.open))
		for _, it := range p.open {
			if !p.Closed[it.k] {
				g := p.G[it.k]
				no = append(no, item{g + p.H(it.k, sk), g, it.k})
			}
		}
		p.open = no
		heap.Init(&p.open)
	}
	tEnd := time.Now().Add(time.Duration(budgetS * float64(time.Second)))
	W, H := w.W, w.H
	walk, surf := w.Walk, w.Surf
	g, closed := p.G, p.Closed
	n := 0
	for p.open.Len() > 0 {
		n++
		if n&127 == 0 && time.Now().After(tEnd) {
			return false
		}
		it := heap.Pop(&p.open).(item)
		gk, k := it.g, it.k
		if closed[k] {
			continue
		}
		closed[k] = true
		if k == sk {
			return true
		}
		i, j := k%W, k/W
		hk := surf[k]
		for _, e := range NB {
			ii, jj := i+e.di, j+e.dj
			if ii < 0 || jj < 0 || ii >= W || jj >= H {
				continue
			}
			kk := jj*W + ii
			if !walk[kk] || closed[kk] {
				continue
			}
			if e.di != 0 && e.dj != 0 && !(walk[j*W+ii] && walk[jj*W+i]) {
				continue
			}
			if math.Abs(surf[kk]-hk) > 2.3 {
				continue
			}
			ng := gk + e.c
			old, ok := g[kk]
			if !ok {
				old = 1e18
			}
			if ng < old {
				g[kk] = ng
				heap.Push(&p.open, item{ng + p.H(kk, sk), ng, kk})
			}
		}
	}
	return false
}

// Waypoint follows the descent chain from our cell; returns a point to steer at (nil = None).
func (p *Planner) Waypoint(x, y float64, look int) []float64 {
	w := p.w
	i, j := w.Cell(x, y)
	if !w.Walk[j*w.W+i] {
		i, j = w.NearestWalk(i, j)
		cx, cy := w.Cpos(i, j)
		return []float64{cx, cy}
	}
	k := j*w.W + i
	if !p.Closed[k] {
		return nil
	}
	g := p.G
	W := w.W
	var chain []int
	for n := 0; n < look; n++ {
		if gv, ok := g[k]; !ok || gv <= 0.0 {
			if ok {
				break
			}
			// python g.get(k, 1e18) -> 1e18 > 0, continues
		}
		ci, cj := k%W, k/W
		best, bk, found := g[k], 0, false
		for _, e := range NB {
			kk := (cj+e.dj)*W + (ci + e.di)
			gv, ok := g[kk]
			if ok && p.Closed[kk] && gv < best {
				best, bk, found = gv, kk, true
			}
		}
		if !found {
			break
		}
		k = bk
		chain = append(chain, k)
	}
	if len(chain) == 0 {
		cx, cy := w.Cpos(p.Goal%W, p.Goal/W)
		return []float64{cx, cy}
	}
	for n := len(chain) - 1; n >= 0; n-- {
		k := chain[n]
		cx, cy := w.Cpos(k%W, k/W)
		pt := []float64{cx, cy}
		if w.ClearLine([]float64{x, y}, pt) {
			return pt
		}
	}
	k = chain[0]
	cx, cy := w.Cpos(k%W, k/W)
	return []float64{cx, cy}
}

func rows(a []float64, n int) [][]float64 {
	out := make([][]float64, 0, len(a)/n)
	for i := 0; i+n <= len(a); i += n {
		out = append(out, a[i:i+n])
	}
	return out
}

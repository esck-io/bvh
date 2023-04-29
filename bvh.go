package bvh

import (
	"math"
	"sort"
)

type Position [3]float64
type Direction [3]float64

type AABB struct {
	Upper Position `json:"upper"`
	Lower Position `json:"lower"`
}

type Ray struct {
	Position  Position
	Direction Direction
}

type LineSegment struct {
	From Position
	To   Position
}

type Bounded interface {
	Bounds() AABB
	Centroid() Position
}

type BVH[V Bounded] struct {
	LeftBVH   *BVH[V]     `json:"leftbvh"`
	RightBVH  *BVH[V]     `json:"rightbvh"`
	LeftV     *V          `json:"leftv"`
	RightV    *V          `json:"rightv"`
	LeftLeaf  *bvhLeaf[V] `json:"leftleaf"`
	RightLeaf *bvhLeaf[V] `json:"rightleaf"`
	B         AABB        `json:"bounds"`
	C         Position    `json:"centroid"`
}

type Intersecter interface {
	Intersects(b AABB) bool
}

type bvhLeaf[V Bounded] struct {
	Primitives []V      `json:"v"`
	B          AABB     `json:"bounds"`
	C          Position `json:"centroid"`
}

func (bvh *bvhLeaf[V]) Bounds() AABB {
	return bvh.B
}

func (bvh *bvhLeaf[V]) Centroid() Position {
	return bvh.C
}

func (l *bvhLeaf[V]) Contents(out []V) []V {
	return append(out, l.Primitives...)
}

func (l *bvhLeaf[V]) Query(q Intersecter, out []V) []V {

	for _, v := range l.Primitives {
		if q.Intersects(v.Bounds()) {
			out = append(out, v)
		}
	}

	return out
}

func (b *AABB) Center() Position {

	center := [3]float64{}
	center[0] = b.Lower[0] + (b.Upper[0]-b.Lower[0])/2.0
	center[1] = b.Lower[1] + (b.Upper[1]-b.Lower[1])/2.0
	center[2] = b.Lower[2] + (b.Upper[2]-b.Lower[2])/2.0
	return center
}

func (b *AABB) SurfaceArea() float64 {
	dims := [3]float64{}
	dims[0] = b.Upper[0] - b.Lower[0]
	dims[1] = b.Upper[1] - b.Lower[1]
	dims[2] = b.Upper[2] - b.Lower[2]

	return 2 * (dims[0]*dims[1] + dims[0]*dims[2] + dims[1]*dims[2])
}

func (b *AABB) Intersects(other *AABB) bool {
	for i := 0; i < 3; i++ {
		if b.Upper[i] < other.Lower[i] || b.Lower[i] > other.Upper[i] {
			return false
		}
	}
	return true
}

func (b *AABB) grow(other AABB) {
	if other.Upper[0] > b.Upper[0] {
		b.Upper[0] = other.Upper[0]
	}
	if other.Upper[1] > b.Upper[1] {
		b.Upper[1] = other.Upper[1]
	}
	if other.Upper[2] > b.Upper[2] {
		b.Upper[2] = other.Upper[2]
	}
	if other.Lower[0] < b.Lower[0] {
		b.Lower[0] = other.Lower[0]
	}
	if other.Lower[1] < b.Lower[1] {
		b.Lower[1] = other.Lower[1]
	}
	if other.Lower[2] < b.Lower[2] {
		b.Lower[2] = other.Lower[2]
	}
}

func (r *Ray) Intersects(b AABB) bool {
	tmin := -math.MaxFloat64
	tmax := math.MaxFloat64

	for i := 0; i < 3; i++ {
		if math.Abs(r.Direction[i]) < 1e-8 {
			if r.Position[i] < b.Lower[i] || r.Position[i] > b.Upper[i] {
				return false
			}
		} else {
			invD := 1.0 / r.Direction[i]
			t1 := (b.Lower[i] - r.Position[i]) * invD
			t2 := (b.Upper[i] - r.Position[i]) * invD

			if t1 > t2 {
				t1, t2 = t2, t1
			}

			tmin = math.Max(tmin, t1)
			tmax = math.Min(tmax, t2)

			if tmin > tmax {
				return false
			}
		}
	}

	return true
}

func (ls *LineSegment) Intersects(b AABB) bool {
	tmin := -math.MaxFloat64
	tmax := math.MaxFloat64

	for i := 0; i < 3; i++ {
		direction := ls.To[i] - ls.From[i]

		if math.Abs(direction) < 1e-8 {
			if ls.From[i] < b.Lower[i] || ls.From[i] > b.Upper[i] {
				return false
			}
		} else {
			invD := 1.0 / direction
			t1 := (b.Lower[i] - ls.From[i]) * invD
			t2 := (b.Upper[i] - ls.From[i]) * invD

			if t1 > t2 {
				t1, t2 = t2, t1
			}

			tmin = math.Max(tmin, t1)
			tmax = math.Min(tmax, t2)

			if tmin > tmax || tmax < 0 || tmin > 1 {
				return false
			}
		}
	}

	return true
}

func (bvh *BVH[V]) Bounds() AABB {
	return bvh.B
}

func (bvh *BVH[V]) Centroid() Position {
	return bvh.C
}

func (bvh *BVH[V]) Contents(out []V) []V {
	if bvh.LeftV != nil {
		out = append(out, *bvh.LeftV)
	}
	if bvh.RightV != nil {
		out = append(out, *bvh.RightV)
	}
	if bvh.LeftBVH != nil {
		out = bvh.LeftBVH.Contents(out)
	}
	if bvh.RightBVH != nil {
		out = bvh.RightBVH.Contents(out)
	}
	if bvh.LeftLeaf != nil {
		out = bvh.LeftLeaf.Contents(out)
	}
	if bvh.RightLeaf != nil {
		out = bvh.RightLeaf.Contents(out)
	}

	return out
}

func (bvh *BVH[V]) Query(q Intersecter, out []V) []V {

	if bvh.LeftV != nil && q.Intersects((*bvh.LeftV).Bounds()) {
		out = append(out, *bvh.LeftV)
	}
	if bvh.RightV != nil && q.Intersects((*bvh.RightV).Bounds()) {
		out = append(out, *bvh.RightV)
	}
	if bvh.LeftBVH != nil && q.Intersects(bvh.LeftBVH.B) {
		out = bvh.LeftBVH.Query(q, out)
	}
	if bvh.RightBVH != nil && q.Intersects(bvh.RightBVH.B) {
		out = bvh.RightBVH.Query(q, out)
	}
	if bvh.LeftLeaf != nil && q.Intersects(bvh.LeftLeaf.B) {
		out = bvh.LeftLeaf.Query(q, out)
	}
	if bvh.RightLeaf != nil && q.Intersects(bvh.RightLeaf.B) {
		out = bvh.RightLeaf.Query(q, out)
	}

	return out
}

type precomputed struct {
	centroid Position
	bounds   AABB
}

func Build[V Bounded](primitives []V, depth int) *BVH[V] {
	pre := make([]precomputed, len(primitives))

	for i, v := range primitives {
		pre[i].bounds = v.Bounds()
		pre[i].centroid = v.Centroid()
	}

	out, _ := buildPrecomputed(pre, primitives)

	return out
}

func buildPrecomputed[V Bounded](pre []precomputed, primitives []V) (*BVH[V], bool) {

	out := &BVH[V]{}

	if len(pre) == 2 {
		out.LeftV = &primitives[0]
		out.RightV = &primitives[1]
		out.B = pre[0].bounds
		out.B.grow(pre[1].bounds)
		out.C = out.B.Center()
		return out, true
	}

	bvhBounds := pre[0].bounds
	for _, p := range pre {
		bvhBounds.grow(p.bounds)
	}
	bvhCost := bvhBounds.SurfaceArea() * float64(len(pre))

	var bestLeftCount int
	var bestRightCount int
	var bestAxis int
	bestCost := math.Inf(1)

	for axis := 0; axis < 3; axis++ {
		step := (bvhBounds.Upper[axis] - bvhBounds.Lower[axis]) / 4.0

		for s := bvhBounds.Lower[axis]; s < bvhBounds.Upper[axis]; s += step {
			leftCount := 0
			rightCount := 0
			var leftBounds AABB
			var rightBounds AABB
			var zeroBounds AABB

			for _, p := range pre {
				if p.centroid[axis] < s {
					leftCount++
					if leftBounds == zeroBounds {
						leftBounds = p.bounds
					}
					leftBounds.grow(p.bounds)
				} else {
					rightCount++
					if rightBounds == zeroBounds {
						rightBounds = p.bounds
					}
					rightBounds.grow(p.bounds)
				}
			}

			if leftCount == 0 || rightCount == 0 {
				continue
			}

			cost := float64(leftCount)*leftBounds.SurfaceArea() + float64(rightCount)*rightBounds.SurfaceArea()

			if cost < bestCost {
				bestCost = cost
				bestAxis = axis
				bestLeftCount = leftCount
				bestRightCount = rightCount
			}
		}
	}

	if bestCost >= bvhCost || bestLeftCount == 0 || bestRightCount == 0 {
		return nil, false
	}

	sortFunc := func(i, j int) bool {
		return pre[i].centroid[bestAxis] < pre[j].centroid[bestAxis]
	}

	sort.Slice(pre, sortFunc)
	sort.Slice(primitives, sortFunc)

	leftPre := pre[:bestLeftCount]
	rightPre := pre[bestLeftCount:]
	leftPrimitives := primitives[:bestLeftCount]
	rightPrimitives := primitives[bestLeftCount:]

	if len(leftPre) == 1 {
		out.LeftV = &leftPrimitives[0]
	} else {
		leftTree, ok := buildPrecomputed(leftPre, leftPrimitives)
		if ok {
			out.LeftBVH = leftTree
		} else {
			out.LeftLeaf = buildLeaf(leftPre, leftPrimitives)
		}

	}

	if len(rightPre) == 1 {
		out.RightV = &rightPrimitives[0]
	} else {
		rightTree, ok := buildPrecomputed(rightPre, rightPrimitives)
		if ok {
			out.RightBVH = rightTree
		} else {
			out.RightLeaf = buildLeaf(rightPre, rightPrimitives)
		}
	}

	out.B = bvhBounds
	out.C = out.B.Center()

	return out, true
}

func buildLeaf[V Bounded](pre []precomputed, primitives []V) *bvhLeaf[V] {
	out := &bvhLeaf[V]{}
	out.Primitives = primitives

	out.B = pre[0].bounds
	for _, p := range pre {
		out.B.grow(p.bounds)
	}

	out.C = out.B.Center()

	return out
}

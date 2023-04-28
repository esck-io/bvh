package bvh

import (
	"math"
	"sort"
)

type Position [3]float64
type Direction [3]float64

type Bounds struct {
	Upper Position
	Lower Position
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
	Bounds() Bounds
	Centroid() Position
}

type BVH[V Bounded] struct {
	left     Bounded
	right    Bounded
	bounds   Bounds
	centroid Position
}

type Intersecter interface {
	Intersects(b Bounds) bool
}

type bvhLeaf[V Bounded] struct {
	primitives []V
	bounds     Bounds
	centroid   Position
}

func (bvh *bvhLeaf[V]) Bounds() Bounds {
	return bvh.bounds
}

func (bvh *bvhLeaf[V]) Centroid() Position {
	return bvh.centroid
}

func (l *bvhLeaf[V]) Contents() []V {
	return l.primitives
}

func (l *bvhLeaf[V]) Query(q Intersecter, out []V) []V {

	for _, v := range l.primitives {
		if q.Intersects(v.Bounds()) {
			out = append(out, v)
		}
	}

	return out
}

func (b *Bounds) Center() Position {

	center := [3]float64{}
	center[0] = b.Lower[0] + (b.Upper[0]-b.Lower[0])/2.0
	center[1] = b.Lower[1] + (b.Upper[1]-b.Lower[1])/2.0
	center[2] = b.Lower[2] + (b.Upper[2]-b.Lower[2])/2.0
	return center
}

func (b *Bounds) SurfaceArea() float64 {
	dims := [3]float64{}
	dims[0] = b.Upper[0] - b.Lower[0]
	dims[1] = b.Upper[1] - b.Lower[1]
	dims[2] = b.Upper[2] - b.Lower[2]

	return 2 * (dims[0]*dims[1] + dims[0]*dims[2] + dims[1]*dims[2])
}

func (b *Bounds) Intersects(other *Bounds) bool {
	for i := 0; i < 3; i++ {
		if b.Upper[i] < other.Lower[i] || b.Lower[i] > other.Upper[i] {
			return false
		}
	}
	return true
}

func (b *Bounds) grow(other Bounds) {
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

func (r *Ray) Intersects(b Bounds) bool {
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

func (ls *LineSegment) Intersects(b Bounds) bool {
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

func (bvh *BVH[V]) Bounds() Bounds {
	return bvh.bounds
}

func (bvh *BVH[V]) Centroid() Position {
	return bvh.centroid
}

func (bvh *BVH[V]) Contents() []V {
	out := []V{}

	leftV, ok := bvh.left.(V)
	if ok {
		out = append(out, leftV)
	}
	rightV, ok := bvh.right.(V)
	if ok {
		out = append(out, rightV)
	}

	leftBVH, ok := bvh.left.(*BVH[V])
	if ok {
		out = append(out, leftBVH.Contents()...)
	}
	rightBVH, ok := bvh.right.(*BVH[V])
	if ok {
		out = append(out, rightBVH.Contents()...)
	}

	leftLeaf, ok := bvh.left.(*bvhLeaf[V])
	if ok {
		out = append(out, leftLeaf.Contents()...)
	}
	rightLeaf, ok := bvh.right.(*bvhLeaf[V])
	if ok {
		out = append(out, rightLeaf.Contents()...)
	}

	return out
}

func (bvh *BVH[V]) Query(q Intersecter, out []V) []V {

	leftIntersects := q.Intersects(bvh.left.Bounds())
	rightIntersects := q.Intersects(bvh.right.Bounds())

	leftV, ok := bvh.left.(V)
	if ok && leftIntersects {
		out = append(out, leftV)
	}
	rightV, ok := bvh.right.(V)
	if ok && rightIntersects {
		out = append(out, rightV)
	}

	leftBVH, ok := bvh.left.(*BVH[V])
	if ok && leftIntersects {
		out = leftBVH.Query(q, out)
	}
	rightBVH, ok := bvh.right.(*BVH[V])
	if ok && rightIntersects {
		out = rightBVH.Query(q, out)
	}

	leftLeaf, ok := bvh.left.(*bvhLeaf[V])
	if ok && leftIntersects {
		out = leftLeaf.Query(q, out)
	}
	rightLeaf, ok := bvh.right.(*bvhLeaf[V])
	if ok && rightIntersects {
		out = rightLeaf.Query(q, out)
	}

	return out
}

type precomputed struct {
	centroid Position
	bounds   Bounds
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
		out.left = primitives[0]
		out.right = primitives[1]
		out.bounds = pre[0].bounds
		out.bounds.grow(pre[1].bounds)
		out.centroid = out.bounds.Center()
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
			var leftBounds Bounds
			var rightBounds Bounds
			var zeroBounds Bounds

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
		out.left = leftPrimitives[0]
	} else {
		leftTree, ok := buildPrecomputed(leftPre, leftPrimitives)
		if ok {
			out.left = leftTree
		} else {
			out.left = buildLeaf(leftPre, leftPrimitives)
		}

	}

	if len(rightPre) == 1 {
		out.right = rightPrimitives[0]
	} else {
		rightTree, ok := buildPrecomputed(rightPre, rightPrimitives)
		if ok {
			out.right = rightTree
		} else {
			out.right = buildLeaf(rightPre, rightPrimitives)
		}
	}

	out.bounds = bvhBounds
	out.centroid = out.bounds.Center()

	return out, true
}

func buildLeaf[V Bounded](pre []precomputed, primitives []V) *bvhLeaf[V] {
	out := &bvhLeaf[V]{}
	out.primitives = primitives

	out.bounds = pre[0].bounds
	for _, p := range pre {
		out.bounds.grow(p.bounds)
	}

	out.centroid = out.bounds.Center()

	return out
}

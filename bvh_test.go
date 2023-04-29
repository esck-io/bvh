package bvh_test

import (
	"math/rand"
	"reflect"
	"testing"

	"github.com/esck-io/bvh"
)

const BVH_DEPTH = 10

type TestPrimitive struct {
	id     int
	bounds bvh.AABB
}

func (p TestPrimitive) Bounds() bvh.AABB {
	return p.bounds
}

func (p TestPrimitive) Centroid() bvh.Position {
	return p.bounds.Center()
}

func TestBVHQuery(t *testing.T) {
	primitives := []TestPrimitive{
		{
			id: 1,
			bounds: bvh.AABB{
				Lower: bvh.Position{-1, -1, -1},
				Upper: bvh.Position{1, 1, 1},
			},
		},
		{
			id: 2,
			bounds: bvh.AABB{
				Lower: bvh.Position{2, 2, 2},
				Upper: bvh.Position{4, 4, 4},
			},
		},
		{
			id: 3,
			bounds: bvh.AABB{
				Lower: bvh.Position{5, 5, 5},
				Upper: bvh.Position{7, 7, 7},
			},
		},
	}

	tree := bvh.Build(primitives, BVH_DEPTH)

	tests := []struct {
		name     string
		query    bvh.Intersecter
		expected []int
	}{
		{
			name: "Ray intersecting primitive 1",
			query: &bvh.Ray{
				Position:  bvh.Position{-2, 0, 0},
				Direction: bvh.Direction{1, 0, 0},
			},
			expected: []int{1},
		},
		{
			name: "Ray intersecting primitive 2",
			query: &bvh.Ray{
				Position:  bvh.Position{1.5, 3, 3},
				Direction: bvh.Direction{1, 0, 0},
			},
			expected: []int{2},
		},
		{
			name: "Ray intersecting primitives 1 and 2",
			query: &bvh.Ray{
				Position:  bvh.Position{-2, 0, 0},
				Direction: bvh.Direction{1, 0.5, 0.5},
			},
			expected: []int{1, 2},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			results := []TestPrimitive{}
			results = tree.Query(tt.query, results)
			resultIDs := make([]int, len(results))

			for i, result := range results {
				resultIDs[i] = result.id
			}

			if !reflect.DeepEqual(tt.expected, resultIDs) {
				t.Errorf("Expected %v, got %v", tt.expected, resultIDs)
			}
		})
	}
}

func TestLineSegmentIntersects(t *testing.T) {
	b := bvh.AABB{
		Upper: bvh.Position{3, 3, 3},
		Lower: bvh.Position{1, 1, 1},
	}

	testCases := []struct {
		name        string
		lineSegment bvh.LineSegment
		expected    bool
	}{
		{
			name: "Intersection",
			lineSegment: bvh.LineSegment{
				From: bvh.Position{0, 2, 2},
				To:   bvh.Position{4, 2, 2},
			},
			expected: true,
		},
		{
			name: "No Intersection",
			lineSegment: bvh.LineSegment{
				From: bvh.Position{0, 0, 2},
				To:   bvh.Position{0, 4, 2},
			},
			expected: false,
		},
		{
			name: "Touching",
			lineSegment: bvh.LineSegment{
				From: bvh.Position{1, 1, 1},
				To:   bvh.Position{1, 1, 4},
			},
			expected: true,
		},
		{
			name: "LongerThanBounds",
			lineSegment: bvh.LineSegment{
				From: bvh.Position{0, 0, 0},
				To:   bvh.Position{4, 4, 4},
			},
			expected: true,
		},
		{
			name: "LongerThanBoundsReversed",
			lineSegment: bvh.LineSegment{
				From: bvh.Position{4, 4, 4},
				To:   bvh.Position{0, 0, 0},
			},
			expected: true,
		},
		{
			name: "Contains",
			lineSegment: bvh.LineSegment{
				From: bvh.Position{1.5, 1.5, 1.5},
				To:   bvh.Position{2, 2, 2},
			},
			expected: true,
		},
		{
			name: "EndsBeforeBox",
			lineSegment: bvh.LineSegment{
				From: bvh.Position{-1, 2, 2},
				To:   bvh.Position{0.5, 2, 2},
			},
			expected: false,
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			result := tc.lineSegment.Intersects(b)
			if result != tc.expected {
				t.Errorf("expected %v, got %v", tc.expected, result)
			}
		})
	}
}

func generateRandomPrimitives(numPrimitives int) []TestPrimitive {
	primitives := make([]TestPrimitive, numPrimitives)

	for i := 0; i < numPrimitives; i++ {
		x := rand.Float64() * 100
		y := rand.Float64() * 100
		z := rand.Float64() * 100
		size := rand.Float64() * 5

		primitives[i] = TestPrimitive{
			id: i,
			bounds: bvh.AABB{
				Lower: bvh.Position{x - size, y - size, z - size},
				Upper: bvh.Position{x + size, y + size, z + size},
			},
		}
	}

	return primitives
}

func BenchmarkBVHQuery(b *testing.B) {
	numPrimitives := 100000
	primitives := generateRandomPrimitives(numPrimitives)
	tree := bvh.Build(primitives, BVH_DEPTH)

	query := &bvh.LineSegment{
		From: bvh.Position{0, 0, 0},
		To:   bvh.Position{10, 10, 10},
	}

	out := []TestPrimitive{}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_ = tree.Query(query, out)
	}
}

func BenchmarkNaiveQuery(b *testing.B) {
	numPrimitives := 100000
	primitives := generateRandomPrimitives(numPrimitives)

	query := &bvh.LineSegment{
		From: bvh.Position{0, 0, 0},
		To:   bvh.Position{10, 10, 10},
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		intersected := []TestPrimitive{}
		for _, p := range primitives {
			if query.Intersects(p.Bounds()) {
				intersected = append(intersected, p)
			}
		}
	}
}

func BenchmarkBuild(b *testing.B) {
	numPrimitives := 100000
	primitives := generateRandomPrimitives(numPrimitives)
	b.ResetTimer()

	for i := 0; i < b.N; i++ {
		_ = bvh.Build(primitives, BVH_DEPTH)
	}
}

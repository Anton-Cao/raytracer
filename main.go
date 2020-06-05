package main

import (
	"fmt"
	"image"
	"image/color"
	"image/png"
	"math"
	"math/rand"
	"os"
	"sync"
	"time"
)

const (
	tolerance = 0.001
)

type point struct {
	x, y, z float64
}

type direction struct {
	dx, dy, dz float64
}

func (d direction) scale(factor float64) direction {
	return direction{factor * d.dx, factor * d.dy, factor * d.dz}
}

func (d direction) flip() direction {
	return d.scale(-1)
}

func (d direction) add(d2 direction) direction {
	return direction{d.dx + d2.dx, d.dy + d2.dy, d.dz + d2.dz}
}

func (p point) to(other point) direction {
	return direction{other.x - p.x, other.y - p.y, other.z - p.z}
}

func norm2(d direction) float64 {
	return d.dx*d.dx + d.dy*d.dy + d.dz*d.dz
}

func norm(d direction) float64 {
	return math.Sqrt(norm2(d))
}

func (d *direction) normalize() {
	n := norm(*d)
	d.dx /= n
	d.dy /= n
	d.dz /= n
}

func dot(d1 direction, d2 direction) float64 {
	return d1.dx*d2.dx + d1.dy*d2.dy + d1.dz*d2.dz
}

// rotate v 180 degrees around normal
func rotateAround(v direction, normal direction) direction {
	normal.normalize()
	dotprod := normal.dx*v.dx + normal.dy*v.dy + normal.dz*v.dz
	diff := direction{v.dx - dotprod*normal.dx, v.dy - dotprod*normal.dy, v.dz - dotprod*normal.dz}
	rot := direction{v.dx - 2*diff.dx, v.dy - 2*diff.dy, v.dz - 2*diff.dz}
	return rot
}

type ray struct {
	src point
	dir direction
}

func (r *ray) scale(factor float64) point {
	return point{
		x: r.src.x + factor*r.dir.dx,
		y: r.src.y + factor*r.dir.dy,
		z: r.src.z + factor*r.dir.dz,
	}
}

func getRay(src point, dst point) ray {
	return ray{src: src, dir: src.to(dst)}
}

func (r ray) flip() ray {
	return getRay(r.src, r.scale(-1))
}

func (r ray) shiftBy(d direction) ray {
	return ray{r.src, r.dir.add(d)}
}

type material struct {
	luminous  bool // emits light
	color     color.Color
	roughness float64 // degree of scattering between 0-1
}

// object that rays can interact with
type object interface {
	intersect(ray) (intersection point, intersects bool)
	reflect(ray, point) (outgoing []ray, weights []float64, color color.Color)
}

type ball struct {
	center point
	radius float64
	mat    material
}

func (b *ball) intersect(r ray) (point, bool) {
	// find intersection using quadratic formula
	xx := r.src.x - b.center.x
	yy := r.src.y - b.center.y
	zz := r.src.z - b.center.z
	dx := r.dir.dx
	dy := r.dir.dy
	dz := r.dir.dz
	aa := dx*dx + dy*dy + dz*dz
	bb := 2.0 * (xx*dx + yy*dy + zz*dz)
	cc := xx*xx + yy*yy + zz*zz - b.radius*b.radius
	radical := bb*bb - 4.0*aa*cc
	if radical < 0 { // imaginary solutions
		return point{}, false
	}
	factor := (-bb - math.Sqrt(radical)) / (2.0 * aa)
	if factor < tolerance { // inside ball already
		return point{}, false
	}
	return r.scale(factor), true
}

func (b *ball) reflect(r ray, intersection point) ([]ray, []float64, color.Color) {
	outgoing := make([]ray, 0)
	weights := make([]float64, 0)
	if !b.mat.luminous {
		normal := getRay(intersection, b.center).flip()
		outgoingDir := rotateAround(r.flip().dir, normal.dir)
		outgoingRay := ray{src: intersection, dir: outgoingDir}
		outgoing = append(outgoing, outgoingRay)
		weights = append(weights, 1.0)
		if b.mat.roughness > 0 {
			// generate a random sphere of vectors around outgoingRay
			if b.mat.roughness > 1 {
				b.mat.roughness = 1
			}
			// find two vectors orthogonal to outgoing dir
			var n1dir direction
			var n2dirTemp direction
			if outgoingDir.dx != 0 {
				n1dir = direction{
					-(outgoingDir.dy + outgoingDir.dz) / outgoingDir.dx,
					1,
					1}
				n2dirTemp = direction{
					-(outgoingDir.dy + 2*outgoingDir.dz) / outgoingDir.dx,
					1,
					2}
			} else if outgoingDir.dy != 0 {
				n1dir = direction{
					1,
					-(outgoingDir.dx + outgoingDir.dz) / outgoingDir.dy,
					1}
				n2dirTemp = direction{
					2,
					-(2*outgoingDir.dx + outgoingDir.dz) / outgoingDir.dy,
					1}
			} else { // outgoingDir.dz must be non-zero
				n1dir = direction{
					1,
					1,
					-(outgoingDir.dx + outgoingDir.dy) / outgoingDir.dz}
				n2dirTemp = direction{
					1,
					2,
					-(outgoingDir.dx + 2*outgoingDir.dy) / outgoingDir.dz}
			}
			n1dir.normalize()
			// make n2dirTemp orthogonal to n1dir
			parallelComp := dot(n1dir, n2dirTemp)
			n2dir := direction{
				n2dirTemp.dx - parallelComp*n1dir.dx,
				n2dirTemp.dy - parallelComp*n1dir.dy,
				n2dirTemp.dz - parallelComp*n1dir.dz,
			}
			n2dir.normalize()
			for radius := 0.01; radius <= 5*b.mat.roughness; radius += 0.05 {
				for angle := 0; angle < 360; angle += 5 {
					radians := float64(angle) * (math.Pi / 180)
					n1s := math.Cos(radians)
					n2s := math.Sin(radians)
					shiftDir := direction{
						(radius * norm(outgoingDir)) * (n1s*n1dir.dx + n2s*n2dir.dx),
						(radius * norm(outgoingDir)) * (n1s*n1dir.dy + n2s*n2dir.dy),
						(radius * norm(outgoingDir)) * (n1s*n1dir.dz + n2s*n2dir.dz),
					}
					newDir := direction{
						outgoingDir.dx + shiftDir.dx,
						outgoingDir.dy + shiftDir.dy,
						outgoingDir.dz + shiftDir.dz,
					}
					outgoing = append(outgoing, ray{intersection, newDir})
					weights = append(weights, (1.0 / 72))
				}
			}
			// if math.Abs(dot(n2dir, outgoingDir)) > tolerance {
			// 	fmt.Printf("%v*%v=%v not orthogonal!\n", n2dir, outgoingDir, dot(n2dir, outgoingDir))
			// }
			// if dot(n1dir, outgoingDir) > tolerance {
			// 	fmt.Printf("%v*%v=%v not orthogonal!\n", n1dir, outgoingDir, dot(n1dir, outgoingDir))
			// }
			// if dot(n1dir, n2dir) > tolerance {
			// 	fmt.Printf("%v*%v=%v not orthogonal!\n", n1dir, n2dir, dot(n1dir, n2dir))
			// }
			/*
				samples := 2000
				for i := 0; i < samples; i++ {
					deflectionRadius2 := rand.Float64() / b.mat.roughness * norm2(outgoingDir)
					randomDir := direction{
						dx: rand.Float64(),
						dy: rand.Float64(),
						dz: rand.Float64(),
					}
					randomNorm2 := norm2(randomDir)
					randomDir = randomDir.scale(deflectionRadius2 / randomNorm2)
					outgoing = append(outgoing, outgoingRay.shiftBy(randomDir))
				}
			*/
		}
	}
	return outgoing, weights, b.mat.color
}

type scene struct {
	objects    []object
	background color.Color
}

func (s *scene) getColor(r ray) color.Color {
	minDist := -1.0
	minI := -1
	var ipnt point
	for i, o := range s.objects {
		if intersection, intersects := o.intersect(r); intersects {
			dist := norm(r.src.to(intersection))
			if minDist < 0 || dist < minDist {
				minDist = dist
				minI = i
				ipnt = intersection
			}
		}
	}
	if minI != -1 {
		collidesWith := s.objects[minI]
		outgoing, weights, cc := collidesWith.reflect(r, ipnt)
		if len(outgoing) == 0 { // luminous
			return cc
		}
		totalWeight := 0.0
		r, g, b, a := cc.RGBA()
		finalR := 0.0
		finalG := 0.0
		finalB := 0.0
		for i := 0; i < len(outgoing); i++ {
			outgoingRay := outgoing[i]
			weight := weights[i]
			totalWeight += weight
			nc := s.getColor(outgoingRay)
			nr, ng, nb, na := nc.RGBA()
			finalR += float64(nr) / float64(na) * float64(r) * weight
			finalG += float64(ng) / float64(na) * float64(g) * weight
			finalB += float64(nb) / float64(na) * float64(b) * weight
		}
		finalR /= totalWeight
		finalG /= totalWeight
		finalB /= totalWeight
		return color.RGBA{
			uint8(finalR / float64(a) * 255),
			uint8(finalG / float64(a) * 255),
			uint8(finalB / float64(a) * 255),
			255,
		}
	}
	return s.background
}

func (s *scene) render(output string) {
	origin := point{0, 0, 0}
	width := 2000
	height := 1000
	img := image.NewRGBA(image.Rectangle{image.Point{0, 0}, image.Point{width, height}})
	imgLock := sync.Mutex{}
	imgGroup := sync.WaitGroup{}
	xMin := -0.5
	xMax := 0.5
	yMin := -0.25
	yMax := 0.25
	z := 1.0

	granularity := 200 // should divide width and height
	for row := 0; row < height; row += granularity {
		for col := 0; col < width; col += granularity {
			imgGroup.Add(1)
			go func(initr int, initc int) {
				for r := initr; r < initr+granularity; r++ {
					for c := initc; c < initc+granularity; c++ {
						x := xMin + float64(c)*(xMax-xMin)/float64(width)
						y := yMin + float64(r)*(yMax-yMin)/float64(height)
						ray := getRay(origin, point{x, y, z})
						color := s.getColor(ray)
						imgLock.Lock()
						img.Set(c, r, color)
						imgLock.Unlock()
					}
				}
				imgGroup.Done()
			}(row, col)
		}
	}
	imgGroup.Wait()
	f, _ := os.Create(output)
	png.Encode(f, img)
}

func main() {
	fmt.Println("Rendering...")
	rand.Seed(42)
	s := scene{
		objects: []object{
			// &ball{center: point{2, 2, 25}, radius: 1, mat: material{
			// 	luminous:  false,
			// 	color:     color.RGBA{255, 0, 0, 255},
			// 	roughness: 0},
			// },
			&ball{center: point{-10, 0, 50}, radius: 1, mat: material{
				luminous:  false,
				color:     color.RGBA{0, 255, 0, 255},
				roughness: 1},
			},
			&ball{center: point{30, 0, 40}, radius: 30, mat: material{
				luminous:  true,
				color:     color.RGBA{255, 255, 255, 255},
				roughness: 0},
			},
		},
		background: color.RGBA{32, 32, 32, 255}}
	start := time.Now()
	s.render("image.png")
	elapsed := time.Now().Sub(start)
	fmt.Printf("Time elapsed: %v\n", elapsed)
}

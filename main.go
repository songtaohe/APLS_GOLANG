package main 

import (
	"fmt"
	"encoding/json"
	"os"
	"io/ioutil"
	"math"
	"strconv"
	"sort"
	"github.com/dhconnelly/rtreego"
)

type graph struct{
	// base info
	Nodes	[][2]float64 
	Edges	[][2]int

	// aux info 
	loc2index map[string]int  
	neighbors map[int]map[int]bool
}

func GPSDistance(p1 [2]float64, p2 [2]float64) float64 {
	a := (p1[0] - p2[0]) * 111111.0 
	b := (p1[1] - p2[1]) * 111111.0 * math.Cos(p1[0]/360.0 *  2.0 * math.Pi)

	return math.Sqrt(a*a + b*b)
}

type gpsnode struct {
	nid int 
	loc [2]float64
}

var tol = 0.000001

func (s *gpsnode) Bounds() *rtreego.Rect {
  // define the bounds of s to be a rectangle centered at s.location
  // with side lengths 2 * tol:

  return rtreego.Point{s.loc[0], s.loc[1]}.ToRect(tol)
}



func loc2key(loc [2]float64) string {
	return fmt.Sprintf("%.7f_%.7f", loc[0], loc[1])
}

func LoadGraphFromJson(filename string) *graph{
	g := new(graph)

	g.loc2index = make(map[string]int)
	g.neighbors = make(map[int]map[int]bool)

	jsonFile, _ := os.Open(filename)
	defer jsonFile.Close()

	byteValue, err := ioutil.ReadAll(jsonFile)

	if err != nil {
        fmt.Println(err)
    }

	var rawresult []interface{}
	json.Unmarshal([]byte(byteValue), &rawresult)

	nodes := rawresult[0].([]interface{})
	edges := rawresult[1].([]interface{})

	for ind, node := range nodes {
		loc := [2]float64{node.([]interface{})[0].(float64), node.([]interface{})[1].(float64)}
		g.Nodes = append(g.Nodes, loc)

		sk := loc2key(loc)

		if _,ok := g.loc2index[sk]; ok {
			fmt.Println("Warning: duplicated location", sk)
		} else {
			g.loc2index[sk] = ind  
		}

	}

	for _, edge := range edges {
		link := [2]int{edge.([]interface{})[0].(int), edge.([]interface{})[1].(int)}
		g.Edges = append(g.Edges, link)
	}	

	return g 
}	

func (g *graph) addEdge(loc1 [2]float64, loc2 [2]float64) {
	sk1 := loc2key(loc1)
	sk2 := loc2key(loc2)

	var nid1,nid2 int 

	if _, ok := g.loc2index[sk1]; ok {
		nid1 = g.loc2index[sk1]
	} else {
		nid1 = len(g.Nodes)
		g.Nodes = append(g.Nodes, loc1)
	}

	if _, ok := g.loc2index[sk2]; ok {
		nid2 = g.loc2index[sk2]
	} else {
		nid2 = len(g.Nodes)
		g.Nodes = append(g.Nodes, loc2)
	}

	g.Edges = append(g.Edges, [2]int{nid1, nid2})

	if _, ok := g.neighbors[nid1]; ok {
		g.neighbors[nid1][nid2] = true
	} else {
		g.neighbors[nid1] = make(map[int]bool)
		g.neighbors[nid1][nid2] = true
	}

	if _, ok := g.neighbors[nid2]; ok {
		g.neighbors[nid2][nid1] = true
	} else {
		g.neighbors[nid2] = make(map[int]bool)
		g.neighbors[nid2][nid1] = true
	}


}

func GraphDensify(g *graph) *graph {
	ng := new(graph)

	for _, edge := range g.Edges {
		n1 := edge[0]
		n2 := edge[1]

		d := GPSDistance(g.Nodes[n1], g.Nodes[n2])

		if d > 0.000003 {
			n := int(d/0.000002)+1
			for i :=0; i<n;i++ {
				alpha1 := float64(i)/float64(n)
				alpha2 := float64(i+1)/float64(n)

				if i == 0 {
					loc1 := g.Nodes[n1]
					loc2 := [2]float64{g.Nodes[n1][0] * (1-alpha2) + g.Nodes[n2][0] * alpha2,  g.Nodes[n1][1] * (1-alpha2) + g.Nodes[n2][1] * alpha2}
					
					ng.addEdge(loc1,loc2)
				} else if i == n-1 {
					loc1 := [2]float64{g.Nodes[n1][0] * (1-alpha1) + g.Nodes[n2][0] * alpha1,  g.Nodes[n1][1] * (1-alpha1) + g.Nodes[n2][1] * alpha1}
					loc2 := g.Nodes[n2]

					ng.addEdge(loc1,loc2)
				} else {
					loc1 := [2]float64{g.Nodes[n1][0] * (1-alpha1) + g.Nodes[n2][0] * alpha1,  g.Nodes[n1][1] * (1-alpha1) + g.Nodes[n2][1] * alpha1}
					loc2 := [2]float64{g.Nodes[n1][0] * (1-alpha2) + g.Nodes[n2][0] * alpha2,  g.Nodes[n1][1] * (1-alpha2) + g.Nodes[n2][1] * alpha2}
					
					ng.addEdge(loc1,loc2)
				}
			}
		}
	}

	return ng 
}


func apls_one_way(graph_gt *graph, graph_prop *graph) float64 {

	// sample control point on graph 
	visited := make(map[int]bool)

	control_point_gt := make(map[int]int)

	for nid, _ := range graph_gt.Nodes {
		if len(graph_gt.neighbors[nid]) != 2 {
			for next_nid, _ := range graph_gt.neighbors[nid] {
				if _,ok := visited[next_nid];ok {
					continue 
				}
				var chain []int 

				chain = append(chain, nid)
				chain = append(chain, next_nid)

				last_nid := nid
				current_nid := next_nid 

				for len(graph_gt.neighbors[current_nid]) == 2 {
					var s int = 0 
					for k,_ := range graph_gt.neighbors[current_nid] {
						s = s + k 
					}

					current_nid, last_nid= s - last_nid, current_nid

					chain = append(chain, current_nid)
				}

				if len(chain) > 15 {
					n := int(float64(len(chain)) / 15.0) + 1

					for i := 1; i < n; i ++ {
						idx = int(float64(len(chain)) * float64(i)/float64(n))

						control_point_gt[idx] = -1
					}
				}

				for _, cnid := range chain {
					visited[cnid] = true
				}

			}

			control_point_gt[nid] = -1
		}
	}

	fmt.Println("ground truth map control points: ", len(control_point_gt))

	// snap to proposal map 
	// - create index 
	rt := rtreego.NewTree(2, 25, 50)

	for nid, loc := range graph_prop.Nodes {
		var gNode gpsnode

		gNode.nid = nid 
		gNode.loc = loc 

		rt.Insert(&gNode)
	}

	var matched_point int = 0 

	for nid1, _ := range control_point_gt {
		q := rtreego.Point{graph_gt.Nodes[nid1][0], graph_gt.Nodes[nid1][1]}
		results := rt.NearestNeighbors(1, q)

		if GPSDistance(results[0].loc, graph_gt.Nodes[nid1]) < 15.0 {
			control_point_gt[nid1] = results[0].nid
			matched_point += 1
		}
	}

	fmt.Println("snapped to proposal graph, matched nodes:", matched_point)

	
}





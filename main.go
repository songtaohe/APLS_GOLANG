package main 

import (
	"fmt"
	"encoding/json"
	"os"
	"io/ioutil"
	"math"
	//"strconv"
	//"sort"
	"github.com/dhconnelly/rtreego"
	"container/heap"

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
	fmt.Println("loading graphs", filename)
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

	fmt.Println("loading nodes", len(nodes))
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

	fmt.Println("loading edges", len(edges))
	for _, edge := range edges {
		link := [2]int{int(edge.([]interface{})[0].(float64)), int(edge.([]interface{})[1].(float64))}
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
		//fmt.Println("never hit?")
	} else {
		nid1 = len(g.Nodes)
		g.Nodes = append(g.Nodes, loc1)
		g.loc2index[sk1] = nid1
	}

	if _, ok := g.loc2index[sk2]; ok {
		nid2 = g.loc2index[sk2]
	} else {
		nid2 = len(g.Nodes)
		g.Nodes = append(g.Nodes, loc2)
		g.loc2index[sk2] = nid2
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
	ng.loc2index = make(map[string]int)
	ng.neighbors = make(map[int]map[int]bool)


	for _, edge := range g.Edges {
		n1 := edge[0]
		n2 := edge[1]

		d := GPSDistance(g.Nodes[n1], g.Nodes[n2])

		if d > 3.0 {

			n := int(d/2.0)+1
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

	fmt.Println("densify graph ", len(g.Nodes), len(ng.Nodes))
	return ng 
}


func apls_one_way(graph_gt *graph, graph_prop *graph) float64 {

	// sample control point on graph 
	visited := make(map[int]bool)

	control_point_gt := make(map[int]int)

	for nid, _ := range graph_gt.Nodes {
		if len(graph_gt.neighbors[nid]) != 2 {
			//fmt.Println("len(graph_gt.neighbors[nid])", nid, graph_gt.neighbors[nid])
			for next_nid, _ := range graph_gt.neighbors[nid] {
				if _,ok := visited[next_nid];ok {
					continue 
				}
				var chain []int 

				chain = append(chain, nid)
				chain = append(chain, next_nid)

				last_nid := nid
				current_nid := next_nid 

				//fmt.Println("inloop")
				for len(graph_gt.neighbors[current_nid]) == 2 {
					var s int = 0 
					for k,_ := range graph_gt.neighbors[current_nid] {
						s = s + k 
					}

					current_nid, last_nid= s - last_nid, current_nid

					chain = append(chain, current_nid)
				}
				//fmt.Println("outloop")

				if len(chain) > 37 { // 50 meters
					n := int(float64(len(chain)) / 25.0) + 1

					for i := 1; i < n; i ++ {
						idx := int(float64(len(chain)) * float64(i)/float64(n))

						control_point_gt[chain[idx]] = -1
					}
				}

				for _, cnid := range chain {
					visited[cnid] = true
				}
			}

			//control_point_gt[nid] = -1
		} else {

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

		if GPSDistance(results[0].(*gpsnode).loc, graph_gt.Nodes[nid1]) < 15.0 {
			control_point_gt[nid1] = results[0].(*gpsnode).nid
			matched_point += 1
		}
	}

	fmt.Println("snapped to proposal graph, matched nodes:", matched_point)


	fmt.Println("Finding shortest paths")


	var cc float64 = 0.0 
	var sum float64 = 0.0 
	var pair_num int = 0 

	var control_point_prop_list []int 
	control_point_prop_map := make(map[int]bool)
	var control_point_gt_list []int 

	for cp1_gt, cp1_prop := range  control_point_gt {
		control_point_gt_list = append(control_point_gt_list, cp1_gt)
		if _, ok := control_point_prop_map[cp1_prop]; ok {

		} else {
			control_point_prop_map[cp1_prop] = true
			control_point_prop_list = append(control_point_prop_list, cp1_prop)
		}
	}

	shortest_paths_gt := make(map[int]map[int]float64)
	shortest_paths_prop := make(map[int]map[int]float64)

	var counter = 0 
	for _, cp_prop := range control_point_prop_list {
		shortest_paths_prop[cp_prop] = graph_prop.ShortestPaths(cp_prop, control_point_prop_list)
		counter += 1 

		if counter % 100 == 0 {
			fmt.Println("computing prop graph shortest paths ", counter, len(control_point_prop_list))
		}
	}

	counter = 0

	for _, cp_gt := range  control_point_gt_list {
		shortest_paths_gt[cp_gt] = graph_gt.ShortestPaths(cp_gt, control_point_gt_list )
		counter += 1

		if counter % 100 == 0 {
			fmt.Println("computing gt graph shortest paths ", counter, len(control_point_gt_list))
		}
	}


	for cp1_gt, cp1_prop := range  control_point_gt {
		for cp2_gt, cp2_prop := range  control_point_gt {
			if cp2_gt <= cp1_gt {
				continue
			}

			pair_num += 1 

			d1 := shortest_paths_gt[cp1_gt][cp2_gt]


			if d1 > 10.0 {
				d2 := shortest_paths_prop[cp1_prop][cp2_prop]

				if d2 < 0 {
					d2 = 0 
				}

				s := math.Abs(d1 - d2) / d1
				if s > 1.0 {
					s = 1.0
				}

				cc += 1.0
				sum += s 

			}

			// if int(cc) % 1000 == 0 {
			// 	fmt.Println(int(cc), "current apls:", 1.0 - sum/cc, "progress:", float64(pair_num) / float64(len(control_point_gt) * len(control_point_gt)/2) * 100.0)
			// }

		}
	}


	return 1.0 - sum/cc
}


type NodeItem struct {
	nid 		int 
	distance	int 
	index 		int 
}

type PriorityQueue []*NodeItem

func (pq PriorityQueue) Len() int { return len(pq) }

func (pq PriorityQueue) Less(i, j int) bool {
	
	return pq[i].distance < pq[j].distance
}

func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}	

func (pq *PriorityQueue) Push(x interface{}) {
	n := len(*pq)
	item := x.(*NodeItem)
	item.index = n
	*pq = append(*pq, item)
}

func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	old[n-1] = nil
	*pq = old[0 : n-1]
	return item
}

func (pq *PriorityQueue) update(item *NodeItem, distance int) {
	item.distance = distance
	heap.Fix(pq, item.index)
}


func (g *graph) ShortestPath(nid1 int, nid2 int) float64 {
	mindistance := make(map[int]int)

	for nid, _ := range g.Nodes {
		mindistance[nid] = 100000000000
	}

	mindistance[nid1] = 0

	queuemap := make(map[int]*NodeItem)
	pq := make(PriorityQueue, 1)


	nodeitem := NodeItem{nid: nid1, distance: 0}
	queuemap[nid1] = &nodeitem
	pq[0] = &nodeitem

	heap.Init(&pq)

	var result int = -1

	for pq.Len() > 0 {
		cur_node_item := heap.Pop(&pq).(*NodeItem)
		delete(queuemap, cur_node_item.nid)

		if cur_node_item.nid == nid2 {
			result = cur_node_item.distance
		}

		for next_nid, _ := range g.neighbors[cur_node_item.nid] {
			d := int(GPSDistance(g.Nodes[cur_node_item.nid], g.Nodes[next_nid]) * 100.0)

			if d + mindistance[cur_node_item.nid] < mindistance[next_nid] {
				mindistance[next_nid] = d + mindistance[cur_node_item.nid]

				if v, ok := queuemap[next_nid]; ok {
					pq.update(v, mindistance[next_nid])
				} else {
					nodeitem := NodeItem{nid: next_nid, distance: mindistance[next_nid]}
					heap.Push(&pq, &nodeitem)
					queuemap[next_nid] = &nodeitem
				}
			}
		}
	}

	return float64(result)/100.0
}

func (g *graph) ShortestPaths(nid1 int, nid2 []int) map[int]float64 {

	result := make(map[int]float64)
	for _, v := range nid2 {
		result[v] = -1.0
	}

	mindistance := make(map[int]int)

	for nid, _ := range g.Nodes {
		mindistance[nid] = 100000000000
	}

	mindistance[nid1] = 0

	queuemap := make(map[int]*NodeItem)
	pq := make(PriorityQueue, 1)


	nodeitem := NodeItem{nid: nid1, distance: 0}
	queuemap[nid1] = &nodeitem
	pq[0] = &nodeitem

	heap.Init(&pq)
	for pq.Len() > 0 {
		cur_node_item := heap.Pop(&pq).(*NodeItem)
		delete(queuemap, cur_node_item.nid)

		if _, ok := result[cur_node_item.nid]; ok {
			result[cur_node_item.nid] = float64(cur_node_item.distance) / 100.0
		}
		

		for next_nid, _ := range g.neighbors[cur_node_item.nid] {
			d := int(GPSDistance(g.Nodes[cur_node_item.nid], g.Nodes[next_nid]) * 100.0)

			if d + mindistance[cur_node_item.nid] < mindistance[next_nid] {
				mindistance[next_nid] = d + mindistance[cur_node_item.nid]

				if v, ok := queuemap[next_nid]; ok {
					pq.update(v, mindistance[next_nid])
				} else {
					nodeitem := NodeItem{nid: next_nid, distance: mindistance[next_nid]}
					heap.Push(&pq, &nodeitem)
					queuemap[next_nid] = &nodeitem
				}
			}
		}
	}

	return result 
}

func apls(graph_gt *graph, graph_prop *graph) {
	apls_gt := apls_one_way(graph_gt, graph_prop)
	apls_prop := apls_one_way(graph_prop, graph_gt)

	fmt.Println(apls_gt, apls_prop, (apls_gt+apls_prop)/2.0)

	d1 := []byte(fmt.Sprintf("%f %f %f\n", apls_gt, apls_prop, (apls_gt+apls_prop)/2.0))
    _ = ioutil.WriteFile(os.Args[3], d1, 0644)
    
}

func main() {
	graph_gt := LoadGraphFromJson(os.Args[1])
	graph_prop := LoadGraphFromJson(os.Args[2])

	graph_gt_dense := GraphDensify(graph_gt)
	graph_prop_dense := GraphDensify(graph_prop)

	apls(graph_gt_dense, graph_prop_dense)



}





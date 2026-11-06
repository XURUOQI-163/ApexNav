#include <plan_env/frontier_semantic_memory.h>
#include <plan_env/frontier_ids.h>
#include <cassert>
#include <cmath>
#include <iostream>
using namespace apexnav_planner;
bool near(double a,double b) { return std::abs(a-b)<1e-8; }
int main() {
  // F67 -> F90 -> F92: direct and transitive adjacency must form one group.
  std::vector<std::vector<std::pair<int,int>>> fragments={
    {{733,662},{734,662}},{{735,662},{736,662}},{{737,662},{738,663}},{{742,662}}};
  auto groups=adjacentFrontierGroups(fragments);
  assert((groups==std::vector<std::vector<int>>{{0,1,2},{3}}));
  std::reverse(fragments.begin(),fragments.end());
  groups=adjacentFrontierGroups(fragments);
  assert((groups==std::vector<std::vector<int>>{{0},{1,2,3}}));
  assert((adjacentFrontierGroups({{{0,0}},{{1,1}}})==std::vector<std::vector<int>>{{0,1}}));
  assert((adjacentFrontierGroups({{{0,0}},{{2,0}}})==std::vector<std::vector<int>>{{0},{1}}));
  assert((adjacentFrontierGroups({{{0,0},{0,0}},{{0,0}}})==std::vector<std::vector<int>>{{0,1}}));
  assert(adjacentFrontierGroups({}).empty());
  FrontierSemanticMemory m;
  std::vector<FrontierCellGeometry> cells{{10,1,0,1.,0.},{11,2,0,2.,0.}};
  m.sync({{1,1.5,0,true,{10,11}}},cells);
  const auto a=m.cellId(10),b=m.cellId(11);
  m.fuse({{a,1.},{b,.25}},10,.2);
  m.fuse({{a,.5},{b,1.}},20,.8);
  assert(near(m.cells().at(a).value,.4));
  assert(near(m.cells().at(b).value,.68));
  assert(near(m.nodes().at(1).value,.68)); // max AFTER per-cell fusion, not fused node max
  m.fuse({{a,1.},{b,1.}},20,.99); // same frame cannot count twice
  assert(near(m.cells().at(a).value,.4));
  m.sync({{2,1,0,true,{10}},{3,2,0,false,{11}}},cells);
  assert(near(m.nodes().at(2).value,.4) && near(m.nodes().at(3).value,.68));
  const auto evidence_a=m.cellAt(10).evidence,evidence_b=m.cellAt(11).evidence;
  m.sync({{4,1.5,0,true,{10,11}}},cells);
  assert(m.cellId(10)==a && m.cellId(11)==b);
  assert(m.cellAt(10).evidence==evidence_a && m.cellAt(11).evidence==evidence_b);
  assert(m.nodes().size()==1 && m.nodes().at(4).scored_cells==2);
  assert(near(m.nodes().at(4).value,.68)); // cluster IDs don't own evidence
  m.sync({{4,2,0,true,{11}}},{cells[1]});
  assert(!m.cells().count(a));
  m.sync({{4,1.5,0,true,{10,11}}},cells);
  assert(m.cellId(10)!=a);
  m.fuse({{a,1.}},30,.9); // removed cell token can't update reborn coordinate
  assert(m.cells().at(m.cellId(10)).confidence==0.);
  m.sync({},{});
  assert(m.nodes().empty() && m.cells().empty());
  m.sync({{1,1.5,0,true,{10,11}}},cells);
  auto x=m.cellId(10),y=m.cellId(11);
  m.fuse({{x,.5}},20,.8);m.fuse({{x,1.}},10,.2); // replay timestamp order
  assert(near(m.cells().at(x).value,.4));
  m.fuse({{y,1.}},30,-.1);
  m.sync({{9,2,0,true,{11}}},{cells[1]});
  assert(near(m.nodes().at(9).value,-.1)); // no zero bias for valid negative scores
  for (int t=31;t<90;++t) m.fuse({{y,1.}},t,.3);
  assert(m.cells().at(y).evidence.size()<=20);
  m.fuse({{y,1.}},30,.9); // folded evidence still cannot count twice
  assert(near(m.cells().at(y).value,.3));
  std::cout<<"per-cell fusion, derived maximum, lifetime and bounded evidence passed\n";
}

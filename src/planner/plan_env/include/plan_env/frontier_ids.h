#ifndef PLAN_ENV_FRONTIER_IDS_H
#define PLAN_ENV_FRONTIER_IDS_H
#include <algorithm>
#include <map>
#include <set>
#include <vector>
#include <utility>

namespace apexnav_planner {
// Connected groups of frontier clusters using their actual grid members.
// Match expandFrontier's eight-neighbour connectivity; never merge by centroid
// distance or across missing boundary cells. Handles transitive chains/overlap.
inline std::vector<std::vector<int>> adjacentFrontierGroups(
    const std::vector<std::vector<std::pair<int,int>>>& cells) {
  std::vector<int> parent(cells.size());
  for(size_t i=0;i<parent.size();++i)parent[i]=int(i);
  auto root=[&](int i) {
    while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}
    return i;
  };
  std::map<std::pair<int,int>,int> owner;
  for(size_t i=0;i<cells.size();++i)for(auto [x,y]:cells[i]) {
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
      auto it=owner.find({x+dx,y+dy});if(it==owner.end())continue;
      int a=root(int(i)),b=root(it->second);
      if(a!=b)parent[std::max(a,b)]=std::min(a,b);
    }
    owner.emplace(std::make_pair(x,y),int(i));
  }
  std::map<int,std::vector<int>> components;
  for(size_t i=0;i<cells.size();++i)components[root(int(i))].push_back(int(i));
  std::vector<std::vector<int>> out;
  for(auto& entry:components)out.push_back(std::move(entry.second));
  return out;
}

// Assign strongest overlaps first so a small early split fragment cannot steal
// the identity and accumulated evidence of the dominant continuation.
inline std::vector<int> matchFrontierIds(const std::vector<std::map<int,int>>& overlaps,
                                        int& next_id) {
  struct Match { int candidate, previous, count; };
  std::vector<Match> matches;
  for (size_t i=0; i<overlaps.size(); ++i)
    for (const auto& old : overlaps[i])
      if (old.second > 0) matches.push_back({static_cast<int>(i),old.first,old.second});
  std::sort(matches.begin(),matches.end(),[](const Match& a,const Match& b) {
    if (a.count != b.count) return a.count > b.count;
    if (a.previous != b.previous) return a.previous < b.previous;
    return a.candidate < b.candidate;
  });
  std::vector<int> ids(overlaps.size(),-1);
  std::set<int> used;
  for (const auto& match : matches)
    if (ids[match.candidate]<0 && !used.count(match.previous)) {
      ids[match.candidate]=match.previous;
      used.insert(match.previous);
    }
  for (auto& id : ids) if (id<0) id=next_id++;
  return ids;
}
}  // namespace apexnav_planner
#endif

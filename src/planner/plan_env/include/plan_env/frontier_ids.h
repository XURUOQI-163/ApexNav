#ifndef PLAN_ENV_FRONTIER_IDS_H
#define PLAN_ENV_FRONTIER_IDS_H
#include <algorithm>
#include <map>
#include <set>
#include <vector>

namespace apexnav_planner {
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

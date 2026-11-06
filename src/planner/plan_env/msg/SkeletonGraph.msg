std_msgs/Header header
nav_msgs/MapMetaData map_info
uint64 revision
SkeletonNode[] nodes
SkeletonEdge[] edges
uint32 skeleton_pixels
uint32 pruned_branches
uint32 changed_tiles
float64 sampling_ms
float64 build_ms
string thinning_backend
uint32 removed_deadends
# Legacy wire compatibility only; always zero. Distance-based merging was removed.
uint32 merged_junctions
SkeletonFrontierLink[] frontier_links
bool geometry_rebuilt
float64 frontier_link_ms

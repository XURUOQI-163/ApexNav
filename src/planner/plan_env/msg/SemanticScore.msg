# Header of the ORIGINAL RGB frame, not the inference completion time.
std_msgs/Header header
# clip or blip2. Both supply raw cosine/ITC similarity, never ITM probability.
string backend
float64 score
# Cell evidence selected from the original 20-frame RGB/scan/pose window.
uint64[] cell_ids
float64[] weights

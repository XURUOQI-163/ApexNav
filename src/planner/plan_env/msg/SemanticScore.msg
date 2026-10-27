# Header of the ORIGINAL RGB frame, not the inference completion time.
std_msgs/Header header
# clip or blip2. Both supply raw cosine/ITC similarity, never ITM probability.
string backend
float64 score

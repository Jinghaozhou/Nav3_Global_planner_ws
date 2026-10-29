std_msgs/Header header
string map_path
string map_signature
float32 center_x
float32 center_y
float32 resolution
float32 slice_h0
float32 slice_dh
float32 cost_barrier
uint32 size_x
uint32 size_y
int32[] source_layer_indices
# Row-major [retained layer, x, y], including zero-cost cells.
float32[] costs

Header header
int16 id
int16 layer
int16 connected_id
string state
float32 x_v # For red and blue nodes
float32 f_r # For robot nodes
float32 g_t # For target nodes
float32 t_r # For red nodes in the first layer (S*(r))

float32 z_r # For red nodes in the recursive algorithm
float32 z_b # For blue nodes in the recursive algorithm
float32 s_b # For blue nodes in the phase 3 of the recursive algorithm
float32 z_v # For red and blue nodes in the last of the phase 3

float64[] edge_weight

int16 loc_deg # Number of degress on the corresponding node
int16[] loc_neighbor_id # ID of local neighbors
int16[] loc_layer # Layer for the local neighbors
int16[] loc_connected_id
string[] loc_state
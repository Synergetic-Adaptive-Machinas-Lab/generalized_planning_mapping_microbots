# Message containing all detected microbots
std_msgs/Header header
int32 total_count                           # Total number of microbots detected
MicrobotPosition[] microbots                # Array of individual microbot positions

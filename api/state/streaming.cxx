
//////
//
// Includes
//

// Local includes
#include "streaming.h"



//////
//
// Class implementations
//

////
// on_tube_vis

std::mutex command_stream::mtx;
std::queue<std::shared_ptr<command>> command_stream::queue;
std::condition_variable command_stream::cv;

#include "frame.hpp"

uint64_t Frame::next_id_ = 0;

Frame::Ptr Frame::create()
{
    auto frame = std::make_shared<Frame>();
    frame->id = next_id_++;
    return frame;
}

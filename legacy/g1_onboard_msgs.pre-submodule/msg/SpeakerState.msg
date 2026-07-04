# SpeakerState — speaker_node playback state. Emit on every push/pop transition.
# STT Provider on PC mutes its input while `playing == true` (echo cancellation).

std_msgs/Header header

bool   playing
uint32 current_chunk_id      # 0 when idle
uint8  queue_depth           # pending playback chunks (max < 256, capped by max_queue_depth)

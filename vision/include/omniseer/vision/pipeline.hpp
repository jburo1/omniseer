#pragma once
namespace omniseer::vision
{
  (producer thread)
      :

        auto frame = cap.dequeue();

  if (!pool.acquire_write(idx))
  {
    frame.requeue();
    continue;
  }
  (drop policy)

      rga.transform(frame.desc, pool.buffer_at(idx));

  pool.publish_ready(idx);

  frame.requeue();
} // namespace omniseer::vision

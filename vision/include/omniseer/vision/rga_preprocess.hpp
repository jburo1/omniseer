#pragma once
namespace omniseer::vision
{
  const Takes FrameDescriptor & src and ImageBuffer &dst

                                                Does resize /
                                            convert /
                                            letterbox

                                                No V4L2 knowledge,
      no ioctls
}

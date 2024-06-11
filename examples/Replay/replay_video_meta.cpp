#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/Display.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"

int main() {
    dai::Pipeline pipeline(true);

    auto replay = pipeline.create<dai::node::ReplayVideo>();
    auto cam = pipeline.create<dai::node::Camera>();
    auto display = pipeline.create<dai::node::Display>();

    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);

    replay->setReplayVideoFile("video.mp4");
    replay->setReplayMetadataFile("video.mcap");
    replay->setFps(1);
    replay->setOutFrameType(dai::ImgFrame::Type::YUV420p);

    replay->out.link(cam->mockIsp);
    cam->video.link(display->input);

    pipeline.start();

    std::this_thread::sleep_for(std::chrono::seconds(10));

    pipeline.stop();
}

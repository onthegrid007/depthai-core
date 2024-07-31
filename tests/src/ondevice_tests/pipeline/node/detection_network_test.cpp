#include <catch2/catch_all.hpp>
#include <depthai/openvino/OpenVINO.hpp>

#include "depthai/depthai.hpp"

TEST_CASE("DetectionNetwork can load BLOB properly") {
    const std::string archivePath = BLOB_ARCHIVE_PATH;

    // Create pipeline
    dai::Pipeline p;
    auto nn = p.create<dai::node::DetectionNetwork>();

    // No classes at the beginning
    REQUIRE(!nn->getClasses().has_value());

    // Load NNArchive
    dai::NNArchive nnArchive(archivePath);
    REQUIRE_NOTHROW(nn->setNNArchive(nnArchive));

    // Network loads classes
    REQUIRE(nn->getClasses().has_value());

    // Classes match
    auto loadedClasses = nn->getClasses().value();
    auto archiveClasses = nnArchive.getConfig().getConfigV1()->model.heads->at(0).metadata.classes.value();
    REQUIRE(loadedClasses == archiveClasses);

    // Throws if number of shaves is specified
    REQUIRE_THROWS(nn->setNNArchive(nnArchive, 6));
}

TEST_CASE("DetectionNetwork can load SUPERBLOB properly") {
    const std::string archivePath = SUPERBLOB_ARCHIVE_PATH;

    // Create pipeline
    dai::Pipeline p;
    auto nn = p.create<dai::node::DetectionNetwork>();

    // Load NNArchive
    dai::NNArchive nnArchive(archivePath);
    REQUIRE_NOTHROW(nn->setNNArchive(nnArchive));

    // Does not throw if number of shaves is specified
    REQUIRE_NOTHROW(nn->setNNArchive(nnArchive, 6));
}

TEST_CASE("DetectionNetwork throws when passed the OTHER NNArchive type") {
    const std::string archivePath = ONNX_ARCHIVE_PATH;

    // Create pipeline
    dai::Pipeline p;
    auto nn = p.create<dai::node::DetectionNetwork>();

    // Load NNArchive
    dai::NNArchive nnArchive(archivePath);
    REQUIRE_THROWS(nn->setNNArchive(nnArchive));
    REQUIRE_THROWS(nn->setNNArchive(nnArchive, 6));
}

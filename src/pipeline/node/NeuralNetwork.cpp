#include "depthai/pipeline/node/NeuralNetwork.hpp"

#include <stdexcept>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

std::optional<OpenVINO::Version> NeuralNetwork::getRequiredOpenVINOVersion() {
    return networkOpenvinoVersion;
}

// Specify local filesystem path to load the blob (which gets loaded at loadAssets)
void NeuralNetwork::setBlobPath(const dai::Path& path) {
    setBlob(OpenVINO::Blob(path));
}

void NeuralNetwork::setBlob(const dai::Path& path) {
    setBlobPath(path);
}

void NeuralNetwork::setBlob(OpenVINO::Blob blob) {
    if(device) {
        if(blob.device == OpenVINO::Device::VPUX && device->getPlatform() != Platform::RVC3) {
            throw std::runtime_error(fmt::format("Loaded model is for RVC3, but the device is {}", device->getPlatformAsString()));
        }
        if(blob.device == OpenVINO::Device::VPU && device->getPlatform() != Platform::RVC2) {
            throw std::runtime_error(fmt::format("Loaded model is for RVC2, but the device is {}", device->getPlatformAsString()));
        }
    }
    networkOpenvinoVersion = blob.version;
    auto asset = assetManager.set("__blob", std::move(blob.data));
    properties.blobUri = asset->getRelativeUri();
    properties.blobSize = static_cast<uint32_t>(asset->data.size());
    properties.modelSource = Properties::ModelSource::BLOB;
}

void NeuralNetwork::setModelPath(const dai::Path& modelPath) {
    // Check if the modelPath is a blob
    if(modelPath.string().find(".blob") != std::string::npos) {
        setBlobPath(modelPath);
        return;
    }

    // Generic case
    auto modelAsset = assetManager.set("__model", modelPath);
    properties.modelUri = modelAsset->getRelativeUri();
    properties.modelSource = Properties::ModelSource::CUSTOM_MODEL;
}

void NeuralNetwork::setNumPoolFrames(int numFrames) {
    properties.numFrames = numFrames;
}

void NeuralNetwork::setNumInferenceThreads(int numThreads) {
    properties.numThreads = numThreads;
}

void NeuralNetwork::setNumNCEPerInferenceThread(int numNCEPerThread) {
    properties.numNCEPerThread = numNCEPerThread;
}

void NeuralNetwork::setNumShavesPerInferenceThread(int numShavesPerThread) {
    properties.numShavesPerThread = numShavesPerThread;
}

void NeuralNetwork::setBackend(std::string backend) {
    properties.backend = backend;
}

void NeuralNetwork::setBackendProperties(std::map<std::string, std::string> props) {
    properties.backendProperties = props;
}

int NeuralNetwork::getNumInferenceThreads() {
    return properties.numThreads;
}

}  // namespace node
}  // namespace dai

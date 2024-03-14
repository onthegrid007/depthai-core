#include "depthai/pipeline/DeviceNode.hpp"

// std
#include "spdlog/fmt/fmt.h"

// project
#include "depthai/pipeline/Pipeline.hpp"

namespace dai {

DeviceNode::DeviceNode(std::unique_ptr<Properties> props, bool conf) : propertiesHolder(std::move(props)) {
    configureMode = conf;
}

DeviceNode::DeviceNode(const std::shared_ptr<Device>& device, std::unique_ptr<Properties> props, bool conf) : device(device), propertiesHolder(std::move(props)) {
    configureMode = conf;
}

Properties& DeviceNode::getProperties() {
    return *propertiesHolder;
}

void DeviceNode::run() {
    // auto pipeline = getParentPipeline();
    // if(pipeline.getDefaultDevice() != nullptr) {
    //     auto dev = pipeline.getDefaultDevice();

    //     auto outputQueueNames = dev->getOutputQueueNames();
    //     auto inputQueueNames = dev->getInputQueueNames();

    //     for(auto output : getOutputRefs()) {
    //         auto xlinkName = fmt::format("__x_{}_{}", output->getParent().id, output->toString());
    //         printf("checking output: %s\n", xlinkName.c_str());

    //         if(std::find(outputQueueNames.begin(), outputQueueNames.end(), xlinkName) != outputQueueNames.end()) {
    //             auto thisNode = pipeline.getNode(id);

    //             dev->getOutputQueue(xlinkName)->addCallback([thisNode, output](std::string, std::shared_ptr<ADatatype> msg) { output->send(msg); });
    //         }
    //     }
    //     for(auto input : getInputRefs()) {
    //         auto xlinkName = fmt::format("__x_{}_{}", input->getParent().id, input->toString());
    //         printf("checking input: %s\n", xlinkName.c_str());
    //         if(std::find(inputQueueNames.begin(), inputQueueNames.end(), xlinkName) != inputQueueNames.end()) {
    //             input->queue->addCallback([dev, xlinkName](std::string, std::shared_ptr<ADatatype> msg) { dev->getInputQueue(xlinkName)->send(msg); });
    //         }
    //     }
    // }
}

}  // namespace dai

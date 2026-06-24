#include <malloc.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "eys3d_camera/camera_node.hpp"

int main(int argc, char** argv) {
    // Keep large per-frame allocations (image and point cloud, several
    // MB each) in glibc's heap so resizes hit the cached free list
    // instead of an mmap / munmap round trip on every publish.
    //   M_MMAP_MAX       = 0   — allocate from the heap, not mmap
    //   M_TRIM_THRESHOLD = -1  — never return heap segments to the OS
    mallopt(M_MMAP_MAX, 0);
    mallopt(M_TRIM_THRESHOLD, -1);

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    // Intra-process delivery is opt-in via the component container
    // path; the standalone executable runs a single node and keeps
    // IPC off.
    options.use_intra_process_comms(false);
    auto node = std::make_shared<eys3d_camera::CameraNode>(options);

    rclcpp::spin(node);
    // Drop the node here so its destructor (thread join, device close,
    // handle release) runs before main() returns.
    node.reset();
    rclcpp::shutdown();
    return 0;
}

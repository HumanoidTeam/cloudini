// SPDX-License-Identifier: Apache-2.0

// Python bindings of the ROS message conversions of cloudini_lib. They work on CDR-serialized messages
// (rclpy.serialization), the same path as the topic converter, so no ROS library is linked here.

#include <pybind11/pybind11.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/ros_msg_utils.hpp"

namespace py = pybind11;

namespace {

Cloudini::ConstBufferView view_of(const std::string& serialized) {
  return Cloudini::ConstBufferView(reinterpret_cast<const uint8_t*>(serialized.data()), serialized.size());
}

py::bytes bytes_of(const std::vector<uint8_t>& buffer) {
  return py::bytes(reinterpret_cast<const char*>(buffer.data()), buffer.size());
}

py::bytes encode_pointcloud2(const std::string& pointcloud2_cdr, float resolution, int encoding_version) {
  if (encoding_version < Cloudini::kMinEncodingVersion || encoding_version > Cloudini::kMaxEncodingVersion) {
    throw std::invalid_argument(
        "encoding_version must be " + std::to_string(Cloudini::kMinEncodingVersion) + " to " +
        std::to_string(Cloudini::kMaxEncodingVersion) + " (got " + std::to_string(encoding_version) + ")");
  }
  std::vector<uint8_t> compressed_cdr;
  {
    py::gil_scoped_release release;
    cloudini_ros::RosPointCloud2 pc_info = cloudini_ros::getDeserializedPointCloudMessage(view_of(pointcloud2_cdr));
    cloudini_ros::applyResolutionProfile(cloudini_ros::ResolutionProfile{}, pc_info.fields, resolution);
    Cloudini::EncodingInfo encoding_info = cloudini_ros::toEncodingInfo(pc_info);
    encoding_info.version = static_cast<uint8_t>(encoding_version);
    cloudini_ros::convertPointCloud2ToCompressedCloud(pc_info, encoding_info, compressed_cdr);
  }
  return bytes_of(compressed_cdr);
}

py::bytes decode_compressed_pointcloud2(const std::string& compressed_cdr) {
  std::vector<uint8_t> pointcloud2_cdr;
  {
    py::gil_scoped_release release;
    const cloudini_ros::RosPointCloud2 pc_info =
        cloudini_ros::getDeserializedPointCloudMessage(view_of(compressed_cdr));
    cloudini_ros::convertCompressedCloudToPointCloud2(pc_info, pointcloud2_cdr);
  }
  return bytes_of(pointcloud2_cdr);
}

}  // namespace

PYBIND11_MODULE(_cloudini, module) {
  module.doc() = "Cloudini point cloud compression for CDR-serialized ROS messages";
  module.attr("ENCODING_VERSION") = Cloudini::kEncodingVersion;
  module.def(
      "encode_pointcloud2", &encode_pointcloud2, py::arg("pointcloud2_cdr"), py::arg("resolution"),
      py::arg("encoding_version") = static_cast<int>(Cloudini::kEncodingVersion),
      "Compress a CDR-serialized sensor_msgs/msg/PointCloud2 into a CDR-serialized\n"
      "point_cloud_interfaces/msg/CompressedPointCloud2. FLOAT32 fields are quantized to `resolution`\n"
      "(maximum error resolution / 2); packed colour fields (rgb, rgba, ...) stay lossless.");
  module.def(
      "decode_compressed_pointcloud2", &decode_compressed_pointcloud2, py::arg("compressed_cdr"),
      "Decompress a CDR-serialized point_cloud_interfaces/msg/CompressedPointCloud2 into a CDR-serialized\n"
      "sensor_msgs/msg/PointCloud2.");
}

// SPDX-License-Identifier: Apache-2.0

// Python bindings of the ROS message conversions of cloudini_lib. They work on CDR-serialized messages
// (rclpy.serialization), the same path as the topic converter, so no ROS library is linked here.

#include <pybind11/pybind11.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/ros_msg_utils.hpp"

namespace py = pybind11;

namespace {

// A view of the bytes object, no copy: valid while the caller holds the object.
Cloudini::ConstBufferView view_of(const py::bytes& serialized) {
  char* data = nullptr;
  Py_ssize_t size = 0;
  if (PyBytes_AsStringAndSize(serialized.ptr(), &data, &size) != 0) {
    throw py::error_already_set();
  }
  return Cloudini::ConstBufferView(reinterpret_cast<const uint8_t*>(data), static_cast<size_t>(size));
}

py::bytes bytes_of(const std::vector<uint8_t>& buffer) {
  return py::bytes(reinterpret_cast<const char*>(buffer.data()), buffer.size());
}

constexpr size_t kAxes = 3;
constexpr std::array<const char*, kAxes> kAxisNames{"x", "y", "z"};
// Cells per axis at most 2^kMaxCellBits, so that the three axes and the non-finite flag fit a 64-bit key.
constexpr int kMaxCellBits = 21;
constexpr int kRadixBits = 8;
constexpr size_t kRadixBuckets = size_t{1} << kRadixBits;
constexpr uint64_t kRadixMask = kRadixBuckets - 1;

using AxisOffsets = std::array<uint32_t, kAxes>;
using AxisValues = std::array<double, kAxes>;

// Offsets of the FLOAT32 x, y and z fields; none when the cloud does not have all three inside a point.
std::optional<AxisOffsets> xyz_offsets(const cloudini_ros::RosPointCloud2& cloud) {
  AxisOffsets offsets{};
  for (size_t axis = 0; axis < kAxes; ++axis) {
    bool found = false;
    for (const Cloudini::PointField& field : cloud.fields) {
      if (field.name == kAxisNames[axis] && field.type == Cloudini::FieldType::FLOAT32 &&
          field.offset + sizeof(float) <= cloud.point_step) {
        offsets[axis] = field.offset;
        found = true;
        break;
      }
    }
    if (!found) {
      return std::nullopt;
    }
  }
  return offsets;
}

// x, y, z of a point; false when one is not finite.
bool read_xyz(const uint8_t* point, const AxisOffsets& offsets, AxisValues& xyz) {
  for (size_t axis = 0; axis < kAxes; ++axis) {
    float value = 0.0F;
    std::memcpy(&value, point + offsets[axis], sizeof(value));
    if (!std::isfinite(value)) {
      return false;
    }
    xyz[axis] = value;
  }
  return true;
}

struct KeyedPoint {
  uint64_t key;
  uint32_t index;
};

// Sorts `points` by key (stable LSD radix sort over the low `key_bits` bits). Keys move with their indices, so
// every pass reads in order.
void radix_sort(std::vector<KeyedPoint>& points, int key_bits) {
  std::vector<KeyedPoint> sorted(points.size());
  for (int shift = 0; shift < key_bits; shift += kRadixBits) {
    std::array<size_t, kRadixBuckets + 1> bucket_start{};
    for (const KeyedPoint& point : points) {
      ++bucket_start[((point.key >> shift) & kRadixMask) + 1];
    }
    std::partial_sum(bucket_start.begin(), bucket_start.end(), bucket_start.begin());
    for (const KeyedPoint& point : points) {
      sorted[bucket_start[(point.key >> shift) & kRadixMask]++] = point;
    }
    points.swap(sorted);
  }
}

// The points ordered by cell of `resolution` (z, then y, then x), non-finite points last: points near in space
// become near in the buffer, which the encoder needs to compress well. Empty (keep the order) when the cloud has
// no FLOAT32 x, y, z or spans more than 2^kMaxCellBits cells on an axis.
std::vector<uint8_t> points_in_cell_order(const cloudini_ros::RosPointCloud2& cloud, double resolution) {
  const std::optional<AxisOffsets> offsets = xyz_offsets(cloud);
  if (!offsets || cloud.point_step == 0 || !(resolution > 0.0)) {
    return {};
  }
  const size_t step = cloud.point_step;
  const size_t count = cloud.data.size() / step;
  AxisValues minimum;
  AxisValues maximum;
  minimum.fill(std::numeric_limits<double>::infinity());
  maximum.fill(-std::numeric_limits<double>::infinity());
  AxisValues xyz{};
  for (size_t point = 0; point < count; ++point) {
    if (read_xyz(cloud.data.data() + point * step, *offsets, xyz)) {
      for (size_t axis = 0; axis < kAxes; ++axis) {
        minimum[axis] = std::min(minimum[axis], xyz[axis]);
        maximum[axis] = std::max(maximum[axis], xyz[axis]);
      }
    }
  }
  std::array<int, kAxes> cell_bits{};
  for (size_t axis = 0; axis < kAxes; ++axis) {
    const double span_cells = maximum[axis] > minimum[axis] ? (maximum[axis] - minimum[axis]) / resolution : 0.0;
    if (span_cells >= static_cast<double>(uint64_t{1} << kMaxCellBits)) {
      return {};
    }
    cell_bits[axis] = std::bit_width(static_cast<uint64_t>(span_cells));
  }
  const int y_shift = cell_bits[0];
  const int z_shift = y_shift + cell_bits[1];
  const int non_finite_shift = z_shift + cell_bits[2];
  const double inverse_resolution = 1.0 / resolution;
  std::vector<KeyedPoint> points(count);
  for (size_t point = 0; point < count; ++point) {
    points[point].index = static_cast<uint32_t>(point);
    if (!read_xyz(cloud.data.data() + point * step, *offsets, xyz)) {
      points[point].key = uint64_t{1} << non_finite_shift;
      continue;
    }
    std::array<uint64_t, kAxes> cell{};
    for (size_t axis = 0; axis < kAxes; ++axis) {
      cell[axis] = static_cast<uint64_t>((xyz[axis] - minimum[axis]) * inverse_resolution);
    }
    points[point].key = (cell[2] << z_shift) | (cell[1] << y_shift) | cell[0];
  }
  radix_sort(points, non_finite_shift + 1);
  std::vector<uint8_t> reordered(count * step);
  for (size_t point = 0; point < count; ++point) {
    std::memcpy(reordered.data() + point * step, cloud.data.data() + size_t{points[point].index} * step, step);
  }
  return reordered;
}

struct EncodeOptions {
  float resolution = 0.0F;
  uint8_t encoding_version = Cloudini::kEncodingVersion;
  Cloudini::CompressionOption compression = Cloudini::CompressionOption::ZSTD;
  bool use_threads = true;
  bool reorder = false;
};

EncodeOptions make_options(
    float resolution, int encoding_version, const std::string& compression, bool use_threads, bool reorder) {
  if (encoding_version < Cloudini::kMinEncodingVersion || encoding_version > Cloudini::kMaxEncodingVersion) {
    throw std::invalid_argument(
        "encoding_version must be " + std::to_string(Cloudini::kMinEncodingVersion) + " to " +
        std::to_string(Cloudini::kMaxEncodingVersion) + " (got " + std::to_string(encoding_version) + ")");
  }
  return EncodeOptions{
      resolution, static_cast<uint8_t>(encoding_version), Cloudini::CompressionOptionFromString(compression),
      use_threads, reorder};
}

// Without the GIL: `pointcloud2_cdr` must stay alive. `encoder_cache` is null for a one-off encoder.
std::vector<uint8_t> encode_cdr(
    Cloudini::ConstBufferView pointcloud2_cdr, const EncodeOptions& options,
    Cloudini::PointcloudEncoderCache* encoder_cache) {
  cloudini_ros::RosPointCloud2 pc_info = cloudini_ros::getDeserializedPointCloudMessage(pointcloud2_cdr);
  std::vector<uint8_t> reordered;
  if (options.reorder) {
    if (pc_info.height != 1) {
      throw std::invalid_argument("reorder needs an unorganized cloud (height 1)");
    }
    reordered = points_in_cell_order(pc_info, options.resolution);
    if (!reordered.empty()) {
      pc_info.data = Cloudini::ConstBufferView(reordered.data(), reordered.size());
    }
  }
  cloudini_ros::applyResolutionProfile(cloudini_ros::ResolutionProfile{}, pc_info.fields, options.resolution);
  Cloudini::EncodingInfo encoding_info = cloudini_ros::toEncodingInfo(pc_info);
  encoding_info.version = options.encoding_version;
  encoding_info.compression_opt = options.compression;
  encoding_info.use_threads = options.use_threads;
  std::vector<uint8_t> compressed_cdr;
  cloudini_ros::convertPointCloud2ToCompressedCloud(pc_info, encoding_info, compressed_cdr, encoder_cache);
  return compressed_cdr;
}

py::bytes encode_pointcloud2(
    const py::bytes& pointcloud2_cdr, float resolution, int encoding_version, const std::string& compression,
    bool use_threads, bool reorder) {
  const EncodeOptions options = make_options(resolution, encoding_version, compression, use_threads, reorder);
  const Cloudini::ConstBufferView input = view_of(pointcloud2_cdr);
  std::vector<uint8_t> compressed_cdr;
  {
    py::gil_scoped_release release;
    compressed_cdr = encode_cdr(input, options, nullptr);
  }
  return bytes_of(compressed_cdr);
}

// Encodes a stream of clouds (e.g. one topic) with one Cloudini encoder, which V6 needs to be fast: the encoder
// keeps the lag, predictor and mask kind it chose instead of probing them again for every cloud.
class PointCloud2Encoder {
 public:
  explicit PointCloud2Encoder(const EncodeOptions& options) : options_(options) {}

  py::bytes encode(const py::bytes& pointcloud2_cdr) {
    const Cloudini::ConstBufferView input = view_of(pointcloud2_cdr);
    std::vector<uint8_t> compressed_cdr;
    {
      py::gil_scoped_release release;
      const std::lock_guard<std::mutex> lock(mutex_);
      compressed_cdr = encode_cdr(input, options_, &encoder_cache_);
    }
    return bytes_of(compressed_cdr);
  }

 private:
  const EncodeOptions options_;
  Cloudini::PointcloudEncoderCache encoder_cache_;
  std::mutex mutex_;
};

py::bytes decode_compressed_pointcloud2(const py::bytes& compressed_cdr) {
  const Cloudini::ConstBufferView input = view_of(compressed_cdr);
  std::vector<uint8_t> pointcloud2_cdr;
  {
    py::gil_scoped_release release;
    const cloudini_ros::RosPointCloud2 pc_info = cloudini_ros::getDeserializedPointCloudMessage(input);
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
      py::arg("encoding_version") = static_cast<int>(Cloudini::kEncodingVersion), py::arg("compression") = "ZSTD",
      py::arg("use_threads") = true, py::arg("reorder") = false,
      "Compress a CDR-serialized sensor_msgs/msg/PointCloud2 into a CDR-serialized\n"
      "point_cloud_interfaces/msg/CompressedPointCloud2. FLOAT32 fields are quantized to `resolution`\n"
      "(maximum error resolution / 2); packed colour fields (rgb, rgba, ...) stay lossless.\n"
      "`compression` is the second stage: \"ZSTD\" (smallest), \"LZ4\" (faster) or \"NONE\" (fastest, largest).\n"
      "`use_threads` runs the second stage in a worker thread (less latency, more CPU time).\n"
      "`reorder` sorts the points by cell of `resolution` (z, then y, then x) first, which compresses clouds in\n"
      "arbitrary order (e.g. voxel maps) much better; the decoded cloud has the same points in that order.\n"
      "Unorganized clouds (height 1) only.\n"
      "For a stream of clouds, PointCloud2Encoder is faster.");
  py::class_<PointCloud2Encoder>(
      module, "PointCloud2Encoder",
      "encode_pointcloud2 for a stream of clouds (e.g. one topic): one Cloudini encoder for all, which keeps\n"
      "what it learned of the clouds and codes the next ones faster. Same options as encode_pointcloud2.")
      .def(
          py::init([](float resolution, int encoding_version, const std::string& compression, bool use_threads,
                      bool reorder) {
            return std::make_unique<PointCloud2Encoder>(
                make_options(resolution, encoding_version, compression, use_threads, reorder));
          }),
          py::arg("resolution"), py::arg("encoding_version") = static_cast<int>(Cloudini::kEncodingVersion),
          py::arg("compression") = "ZSTD", py::arg("use_threads") = true, py::arg("reorder") = false)
      .def(
          "encode", &PointCloud2Encoder::encode, py::arg("pointcloud2_cdr"),
          "Compress a CDR-serialized sensor_msgs/msg/PointCloud2 into a CDR-serialized\n"
          "point_cloud_interfaces/msg/CompressedPointCloud2.");
  module.def(
      "decode_compressed_pointcloud2", &decode_compressed_pointcloud2, py::arg("compressed_cdr"),
      "Decompress a CDR-serialized point_cloud_interfaces/msg/CompressedPointCloud2 into a CDR-serialized\n"
      "sensor_msgs/msg/PointCloud2.");
}

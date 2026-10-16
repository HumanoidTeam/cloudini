# cloudini (Python bindings)

Native Python bindings of `cloudini_lib` (pybind11, built with scikit-build-core). They convert
CDR-serialized ROS messages, as the topic converter does, so they need no ROS library at build time.

```python
from rclpy.serialization import serialize_message
import cloudini

compressed_cdr = cloudini.encode_pointcloud2(serialize_message(cloud), resolution=0.001)
compressed_publisher.publish(compressed_cdr)  # a point_cloud_interfaces/msg/CompressedPointCloud2 publisher
```

For a stream of clouds (one topic), keep one `PointCloud2Encoder`: it reuses the Cloudini encoder, so V6 does not
probe every cloud again (~15 % faster).

```python
encoder = cloudini.PointCloud2Encoder(resolution=0.025, use_threads=False, reorder=True)
compressed_publisher.publish(encoder.encode(serialize_message(cloud)))
```

Options of both:
- `compression`: second stage, `"ZSTD"` (default, smallest), `"LZ4"` (faster) or `"NONE"`.
- `use_threads`: run the second stage in a worker thread (default). Less latency, more CPU time.
- `reorder`: sort the points by cell of `resolution` (z, then y, then x) before encoding (radix sort, C++). Clouds in
  arbitrary order, e.g. voxel maps from a hash table, become about half the size. The decoded cloud has the points
  in that order. Unorganized clouds only.

Install: `pip install ./cloudini_python` (zstd and lz4 from the system or the conda environment when found,
downloaded otherwise).

A plain CMake build (no scikit-build) writes the module into `cloudini/`, which is then importable as is. CMake uses
`cloudini_lib/` next to `CMakeLists.txt` when a self-contained copy of the package has it, else `../cloudini_lib`.

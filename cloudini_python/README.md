# cloudini (Python bindings)

Native Python bindings of `cloudini_lib` (pybind11, built with scikit-build-core). They convert
CDR-serialized ROS messages, as the topic converter does, so they need no ROS library at build time.

```python
from rclpy.serialization import serialize_message
import cloudini

compressed_cdr = cloudini.encode_pointcloud2(serialize_message(cloud), resolution=0.001)
compressed_publisher.publish(compressed_cdr)  # a point_cloud_interfaces/msg/CompressedPointCloud2 publisher
```

Install: `pip install ./cloudini_python` (zstd and lz4 from the system or the conda environment when found,
downloaded otherwise).

"""Cloudini point cloud compression for CDR-serialized ROS messages.

Serialize and deserialize the messages with ``rclpy.serialization``; a publisher can publish the
returned bytes as they are.
"""

from cloudini._cloudini import ENCODING_VERSION, decode_compressed_pointcloud2, encode_pointcloud2

__all__ = ["ENCODING_VERSION", "decode_compressed_pointcloud2", "encode_pointcloud2"]

// Foxy-compatible rosbag2 writer for original CDR bytes. No ROS subscriptions or motor APIs.
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rcutils/allocator.h>
#include <rcutils/types/uint8_array.h>
#include <unistd.h>
#include <cerrno>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

static bool read_exact(void *buffer, size_t length, bool allow_eof = false) {
  auto *bytes = static_cast<unsigned char *>(buffer);
  size_t offset = 0;
  while (offset < length) {
    auto n = ::read(STDIN_FILENO, bytes + offset, length - offset);
    if (n < 0 && errno == EINTR) continue;
    if (n < 0) throw std::runtime_error("stdin read failed");
    if (n == 0) {
      if (offset == 0 && allow_eof) return false;
      throw std::runtime_error("truncated CDR input frame");
    }
    offset += static_cast<size_t>(n);
  }
  return true;
}
static uint32_t u32(const unsigned char *p) {
  return uint32_t(p[0]) | uint32_t(p[1])<<8 | uint32_t(p[2])<<16 | uint32_t(p[3])<<24;
}
static uint64_t u64(const unsigned char *p) {
  return uint64_t(u32(p)) | uint64_t(u32(p+4))<<32;
}
int main(int argc, char **argv) {
  if (argc != 2) { std::cerr << "usage: xrcam_cdr_bag_writer OUTPUT_URI\n"; return 2; }
  try {
    // Destruction closes SQLite and writes metadata.yaml. EOF also occurs if the parent dies.
    rosbag2_cpp::Writer writer(std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
    rosbag2_cpp::StorageOptions options;
    options.uri = argv[1]; options.storage_id = "sqlite3";
    options.max_bagfile_size = 256ULL * 1024 * 1024;
    options.max_cache_size = 0;
    writer.open(options, {"cdr", "cdr"});
    std::unordered_map<std::string, std::string> topics;
    std::cout << "READY" << std::endl;
    unsigned char header[20];
    while (read_exact(header, sizeof(header), true)) {
      auto topic_n = u32(header), type_n = u32(header+4), payload_n = u32(header+8);
      auto timestamp = u64(header+12);
      if (!topic_n || topic_n > 4096 || !type_n || type_n > 4096 || payload_n > 64*1024*1024)
        throw std::runtime_error("invalid CDR frame lengths");
      std::string topic(topic_n, '\0'), type(type_n, '\0');
      read_exact(&topic[0], topic_n); read_exact(&type[0], type_n);
      if (topics.count(topic) && topics[topic] != type) throw std::runtime_error("topic type changed");
      if (!topics.count(topic)) {
        writer.create_topic({topic, type, "cdr", ""}); topics[topic] = type;
      }
      auto data = std::shared_ptr<rcutils_uint8_array_t>(
        new rcutils_uint8_array_t(rcutils_get_zero_initialized_uint8_array()),
        [](rcutils_uint8_array_t *p) { auto ignored = rcutils_uint8_array_fini(p); (void)ignored; delete p; });
      auto allocator = rcutils_get_default_allocator();
      if (rcutils_uint8_array_init(data.get(), payload_n, &allocator) != RCUTILS_RET_OK)
        throw std::runtime_error("CDR allocation failed");
      read_exact(data->buffer, payload_n); data->buffer_length = payload_n;
      auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      message->topic_name = topic; message->time_stamp = static_cast<int64_t>(timestamp);
      message->serialized_data = data; writer.write(message);
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "cdr_bag_writer: " << e.what() << std::endl; return 2;
  }
}

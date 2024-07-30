#include <boost/interprocess/managed_shared_memory.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <semaphore.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <vector>

int counter = 0;
uint64_t total_latency = 0;

namespace bip = boost::interprocess;

struct Header {
    uint32_t seq;
    uint64_t stamp_sec;
    uint64_t stamp_nsec;
    char frame_id[255];
};

struct PointCloud2 {
    Header header;
    uint32_t height;
    uint32_t width;
    bool is_bigendian;
    bool is_dense;
    uint32_t point_step;
    uint32_t row_step;
    std::vector<uint8_t> data;
    std::vector<Header> fields;  // Using Header as a placeholder for PointField
};

const char* PUB_SEMAPHORE_NAME = "/pointcloud_pub_semaphore";
const char* SUB_SEMAPHORE_NAME = "/pointcloud_sub_semaphore";

int main() {
    // Open semaphore
    sem_t* pub_sem = sem_open(PUB_SEMAPHORE_NAME, 0);
    if (pub_sem == SEM_FAILED) {
        std::cerr << "Failed to open semaphore" << std::endl;
        return 1;
    }

    sem_t* sub_sem = sem_open(SUB_SEMAPHORE_NAME, 0);
    if (sub_sem == SEM_FAILED) {
        std::cerr << "Failed to open semaphore" << std::endl;
        return 1;
    }

    // Open shared memory
    bip::managed_shared_memory segment(bip::open_only, "SharedMemory");

    // Find the PointCloud2 in shared memory
    PointCloud2* cloud = segment.find<PointCloud2>("PointCloud2").first;

    if (!cloud) {
        std::cerr << "PointCloud2 not found in shared memory!" << std::endl;
        return 1;
    }

    while (true) {
        // Wait for semaphore
        sem_wait(pub_sem);

        // Read the timestamp from shared memory
        auto now = std::chrono::system_clock::now();
        auto receive_time = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

        uint64_t sent_time = (cloud->header.stamp_sec * 1000000000) + (cloud->header.stamp_nsec);
        uint64_t latency_ns = receive_time - sent_time;

        // total_latency += std::chrono::nanoseconds(static_cast<long long>(latency_ns));
        total_latency += latency_ns;
        counter++;
        if (counter == 1000) {
            uint64_t avg_latency = total_latency / 1000;
            std::cout << "Average latency: " << avg_latency << " ns" << std::endl;

            std::cout << "Average latency: " << (avg_latency / 1000000.0f) << " ms" << std::endl;
            counter = 0;
            total_latency = 0;
        } 

        std::cout << "Latency: " << latency_ns << " us" << std::endl;

        // Post semaphore
        sem_post(sub_sem);
    }

    // Close semaphore
    sem_close(pub_sem);
    sem_close(sub_sem);

    return 0;
}

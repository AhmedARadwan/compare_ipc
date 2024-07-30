#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <iostream>
#include <random>
#include <chrono>
#include <thread>
#include <cstring>
#include <semaphore.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <vector>

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

void createPointCloud(PointCloud2& cloud) {
    cloud.height = 1;
    cloud.width = 1000000;
    cloud.is_dense = false;
    cloud.is_bigendian = false;
    cloud.point_step = 12;  // 3 fields * 4 bytes/field (float32)
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step * cloud.height);

    std::mt19937 gen;
    std::uniform_real_distribution<float> dist(-10.0, 10.0);

    for (size_t i = 0; i < cloud.width; ++i) {
        float x = dist(gen);
        float y = dist(gen);
        float z = dist(gen);
        memcpy(&cloud.data[i * cloud.point_step + 0], &x, sizeof(float));
        memcpy(&cloud.data[i * cloud.point_step + 4], &y, sizeof(float));
        memcpy(&cloud.data[i * cloud.point_step + 8], &z, sizeof(float));
    }
}

int main() {
    sem_unlink(PUB_SEMAPHORE_NAME);
    sem_unlink(SUB_SEMAPHORE_NAME);
    // Create or open semaphore
    sem_t* pub_sem = sem_open(PUB_SEMAPHORE_NAME, O_CREAT | O_EXCL, S_IRUSR | S_IWUSR, 1);
    if (pub_sem == SEM_FAILED) {
        std::cerr << "Failed to create pub semaphore" << std::endl;
        return 1;
    }

    sem_t* sub_sem = sem_open(SUB_SEMAPHORE_NAME, O_CREAT | O_EXCL, S_IRUSR | S_IWUSR, 1);
    if (sub_sem == SEM_FAILED) {
        std::cerr << "Failed to create sub semaphore" << std::endl;
        return 1;
    }

    // Remove shared memory on construction and destruction
    struct shm_remove {
        shm_remove() { bip::shared_memory_object::remove("SharedMemory"); }
        ~shm_remove() { bip::shared_memory_object::remove("SharedMemory"); }
    } remover;

    // Create shared memory
    bip::managed_shared_memory segment(bip::create_only, "SharedMemory", 65536);

    // Initialize the PointCloud2 in shared memory
    PointCloud2* cloud = segment.construct<PointCloud2>("PointCloud2")();

    createPointCloud(*cloud);

    uint32_t seq = 0;

    while (true) {
        // Wait for semaphore
        sem_wait(sub_sem);

        // Update header
        cloud->header.seq = seq++;
        auto now = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        cloud->header.stamp_sec = duration;
        cloud->header.stamp_nsec = (std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() % 1000000000);
        std::strcpy(cloud->header.frame_id, "base_link");

        // Post semaphore
        sem_post(pub_sem);
        std::cout << "Publishing PointCloud2 message with seq: " << cloud->header.seq << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Close and unlink semaphore
    sem_close(pub_sem);
    sem_close(sub_sem);
    sem_unlink(PUB_SEMAPHORE_NAME);
    sem_unlink(SUB_SEMAPHORE_NAME);

    return 0;
}

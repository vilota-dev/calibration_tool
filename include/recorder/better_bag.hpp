#define private public
#include <rosbag/bag.h>
#include <rosbag/view.h>
#undef private
#include <rigtorp/SPSCQueue.h>
#include <sensor_msgs/Image.h>
#include <spdlog/spdlog.h>

#include <mutex>

namespace vk {
    struct WriteData {
        std::string stream_name;
        ros::Time ts;
        sensor_msgs::ImagePtr img;
    };

    /**
     * @brief This is a thread-safe wrapper class around rosbag::Bag.
     */
    class BetterBag {
    public:
        BetterBag() : write_queue(1024), m_stop(false){ }
        ~BetterBag() {
            m_stop = true;
            processing_thread.join();
            if (m_bag.isOpen()) {
                m_bag.close();
            }
        };

        template <typename... Args>
        void openWrite(Args&&... args) {
            this->m_bag.openWrite(std::forward<Args>(args)...);
            this->processing_thread = std::thread(&BetterBag::process_writes, this);
        }

        void close() {
            spdlog::trace("BetterBag::close");
            // Need to ensure that the processing thread is finished writing
            // Here, the write_queue may not be fully empty after closing.
            m_stop = true;
            processing_thread.join();
            spdlog::debug("Processing thread successfully joined");
            this->m_bag.close();
        }

        void write(const std::string& stream_name, const ros::Time& ts, const sensor_msgs::ImagePtr& img) {
            WriteData temp;
            temp.stream_name = stream_name;
            temp.ts = ts;
            temp.img = img;
            this->write_queue.push(std::move(temp));
            spdlog::debug("Pushed to write queue, size: {}", write_queue.size());
        }

    private:
        void process_writes() {
            spdlog::debug("Starting write thread for bag file: {}", m_bag.getFileName());
            WriteData data;
            while(!m_stop) {
                if (!write_queue.empty()) {
                    data = *write_queue.front();
                    write_queue.pop();

                    spdlog::trace("performing write");
                    m_bag.write(data.stream_name, data.ts, data.img);
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
            spdlog::debug("Stopping write thread for bag file: {}", m_bag.getFileName());
        }

        rigtorp::SPSCQueue<WriteData> write_queue;
        std::thread processing_thread;
        rosbag::Bag m_bag;

        std::atomic<bool> m_stop;
    };
} // namespace vk
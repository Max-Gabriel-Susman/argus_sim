#ifndef REPLAY_SERVER_HPP
    #define REPLAY_SERVER_HPP

    #include <atomic>
    #include <cstdint>
    #include <mutex>
    #include <string>
    #include <thread>
    #include <vector>

    namespace argus_sim
    {
        class ReplaySource
        {
        public:
            static ReplaySource from_file(const std::string &path, uint16_t channel_count);
            static ReplaySource synthetic(size_t n_samples, uint16_t channel_count);

            ReplaySource() = default;
            ~ReplaySource();

            ReplaySource(const ReplaySource &) = delete;
            ReplaySource & operator = (const ReplaySource &) = delete;
            ReplaySource(ReplaySource && other) noexcept;
            ReplaySource & operator = (ReplaySource && other) noexcept;

            const uint16_t *data() const { return data_;}
            size_t sample_count() const { return n_samples_; }
            uint16_t channel_count() const { return n_channels_; }
            bool valid() const { return data_ != nullptr && n_samples_ > 0; }
            const uint16_t *sample(size_t index) const
            {
                return data_ + index * static_cast<size_t>(n_channels_);
            }
        
        private:
            void release();
            std::vector<uint16_t> owned_;
            void *map_ {nullptr};
            size_t map_bytes_ {0};
            const uint16_t *data_ {nullptr};
            size_t n_samples_ {0};
            uint16_t n_channels_ {0};
        };

        struct ReplayStats
        {
            uint64_t requests_served {0};
            uint64_t retransmits_served {0};
            uint64_t chunks_sent {0};
            uint64_t bytes_sent {0};
            uint64_t bad_packets {0};
            uint64_t wraps {0};
            uint64_t last_offset {0};
            std::string last_peer;
        };

        class ReplayServer
        {
        public:
            struct Config
            {
                std::string bind_addr {"0.0.0.0"};
                uint16_t port {0};
                bool loop {true};
                uint16_t max_samples_per_request {0};
            };

            ReplayServer(const ReplaySource &source, Config config);
            ~ReplayServer();

            ReplayServer(const ReplayServer &) = delete;
            ReplayServer &operator = (const ReplayServer &) = delete;

            void start();
            void stop();

            ReplayStats stats() const;

            uint16_t samples_per_chunk() const { return samples_per_chunk_; }
        
        private:
            void run();
            void handle(const uint8_t *data, size_t len, const void *peer, size_t peer_len);
            
            const ReplaySource &source_;
            Config config_;
            uint16_t samples_per_chunk_ {0};
            size_t row_bytes_ {0};

            int sock_ {-1};
            std::thread thread_;
            std::atomic<bool> running_ {false};

            mutable std::mutex stats_mutex_;
            ReplayStats stats_;

            std::vector<uint8_t> tx_bufer_;
        };
    } // namespace argus_sim

#endif /* REPLAY_SERVER_HPP */
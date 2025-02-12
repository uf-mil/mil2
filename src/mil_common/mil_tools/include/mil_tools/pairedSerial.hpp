#include <mil_tools/ringBuffer.hpp>

#include <pthread.h>

#include <string>

namespace mil_tools
{
    class PairedSerial
    {
        public:
        PairedSerial();
        ~PairedSerial();

        int open(std::string& slave1Name, std::string& slave2Name);
        void close();

        private:

        int master1Fd_;
        int master2Fd_;
        pthread_t workThread_;

        static void* workThreadFunc_(void* arg);
    };
}
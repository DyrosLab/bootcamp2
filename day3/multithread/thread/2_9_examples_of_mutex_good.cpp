#include <thread>
#include <mutex>

struct DataStorage
{
    int a;
};

class RobotController
{
public:
    // readCallback() runs in another thread.
    void readCallback(const std::shared_ptr<DataStorage> data_storage)
    {
        std::lock_guard<std::mutex> lg(mutex_);
        input_data_ = *data_storage;
    }

    // processSomethingNew() takes a lot of time.
    void processSomethingNew(DataStorage & data)
    {
        // Heavy computations ...
    }

    // compute() runs in another thread.
    void compute()
    {
        {
            std::lock_guard<std::mutex> lg(mutex_);
            compute_data_ = input_data_;
        }
        processSomethingNew(compute_data_);
    }
    

private:
    std::mutex mutex_;
    DataStorage input_data_;
    DataStorage compute_data_;
};


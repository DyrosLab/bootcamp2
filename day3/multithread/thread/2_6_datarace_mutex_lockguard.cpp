#include <iostream>
#include <thread>
#include <vector>
#include <mutex>

int global_data {1000};
std::mutex global_mutex;

void compute()
{
    for (int i=0; i<1000000; ++i)
    {
        std::lock_guard<std::mutex> lock(global_mutex);
        global_data += 1;
    }
}

int main()
{
    const int num_thread = 4;
    std::vector<std::thread> threads;
    
    for (int i=0; i<num_thread; ++i)
    {
        threads.push_back(std::thread(compute));
    }
    for (auto & thread : threads)
    {
        thread.join();
    }
    std::cout << "global_data: " 
              <<  global_data << std::endl;

    return 0;
}



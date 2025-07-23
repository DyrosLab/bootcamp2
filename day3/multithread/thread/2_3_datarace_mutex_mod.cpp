#include <iostream>
#include <thread>
#include <vector>
#include <mutex>

std::mutex global_mutex;
int global_data = 1000;

void compute()
{
    for (int i=0; i<1000000; ++i)
    {
        global_mutex.lock();
        global_data += 1;
        global_mutex.unlock();
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



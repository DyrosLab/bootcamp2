#include <iostream>
#include <thread>
#include <vector>
#include <mutex>

class SuhanPark
{
public:
    void foo(int inc = 1)
    {
        for (int i=0; i<1000000; i++)
        {
            std::lock_guard<std::mutex> lg(mutex_);
            number_ += inc;
        }
    }
    void printNumber()
    {
        std::cout << "The number is " << number_ << std::endl;
    }

private:
    int number_ {0};
    std::mutex mutex_;    
};

int main()
{
    const int num_threads = 8;
    SuhanPark suhan_park;
    // basic call
    suhan_park.foo();

    std::vector<std::thread> threads;
    // call by other threads
    for (int i=0; i<num_threads; ++i)
    {
        threads.push_back(std::thread(&SuhanPark::foo, &suhan_park, 5));
    }
    for (auto & thread : threads)
    {
        thread.join();
    }

    suhan_park.printNumber();

    return 0;
}


#include <iostream>
#include <thread>

void hello()
{
    std::cout << "Hello?";
}
int main()
{
    // basic call
    hello();

    // call by another thread
    std::thread t1(hello);
    t1.join();

    return 0;
}



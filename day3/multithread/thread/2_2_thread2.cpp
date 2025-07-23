#include <iostream>
#include <thread>

void hello()
{
    std::cout << "Hello?";
}

void dyros()
{
    std::cout << " dyros" << std::endl;
}

int main()
{
    // basic call
    hello();
    dyros();

    // call by another thread
    std::thread t1(hello);
    std::thread t2(dyros);
    t1.join();
    t2.join();

    return 0;
}


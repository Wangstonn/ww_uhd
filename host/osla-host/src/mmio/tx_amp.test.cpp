#include <iostream>
#include <cmath>

int main()
{
    uint16_t a = (std::pow(2,15)-1);
    uint64_t b = 0x8000003400000000+a;
    
    std::cout << std::hex << a << std::endl;
    std::cout << std::hex << b << std::endl;
    

    return 0;
}

/// ping function for ubuntu.

#if 0

#include <iostream>
#include <stdio.h>
#include <time.h>
#include <string.h>

int main(int argc,char *argv[])
{
    if (system( "ping -c1 -s1 -w1 192.168.2.113") ) {
        printf("internet connx failed \n");
    }
    else {
        printf("internet connx OK ! :) \n");
    }
}

#endif

#include <iostream>
#include <stdio.h>
#include <time.h>
#include <string.h>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
static const std::string slash="\\";
#else
static const std::string slash="/";
#endif

int main()
{
    std::cout << "slash is: " << slash << std::endl;

    return 1;
}
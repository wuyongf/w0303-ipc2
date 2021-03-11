/// ping function for ubuntu.

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
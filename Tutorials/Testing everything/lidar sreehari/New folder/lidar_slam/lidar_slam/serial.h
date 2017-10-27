#ifndef SERIAL_H
#define SERIAL_H

#include <windows.h>
#include <string>


// must be 2^x-1
#define RX_FIFO_SIZE 32767

class Serial
{
  public:
    Serial();
    ~Serial();
    bool begin(char *port, int baud);
    int available();
    int read();
    int peek();
    void flush();
    size_t write(const uint8_t c);
    /*int read(std::string &s);
    int write(std::string &s);*/
    void end();
    void appendRxFifo(unsigned char c);
    HANDLE hPort;
    HANDLE hThread;
    unsigned char rx_fifo[RX_FIFO_SIZE];
    unsigned int rx_fifo_start;
    unsigned int rx_fifo_end;
};

#endif


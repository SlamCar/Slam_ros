#pragma once

#include <iostream>
#include <inttypes.h>
#include <vector>
#include <deque>
#include <queue>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>

typedef std::vector<uint8_t> Buffer;

class Transport 
{
public:
	virtual bool init()=0;
    virtual void setTimeout(int t)=0;
    virtual bool isTimeout()=0;
	
	virtual Buffer read() = 0;
	virtual void write(Buffer &data) = 0;
};

/* TRANSPORT_BASE_H_ */
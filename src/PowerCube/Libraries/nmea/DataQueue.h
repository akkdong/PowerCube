// DataQueue.h
//

#ifndef __DATA_QUEUE_H__
#define __DATA_QUEUE_H__

#define MAX_NMEA_PARSER_BUFFER      (1024)


/////////////////////////////////////////////////////////////////////////////
// class DataQueue

class DataQueue
{
public:
	DataQueue();

public:
	void				push(int ch);
	int					pop();

	int					front() { return mFront; }

	int					isFull() 	{ return ((mFront + 1) % MAX_NMEA_PARSER_BUFFER) == mTail; }
	bool				isEmpty()	{ return mHead == mTail; }	

	int					get(int index);
	int					copy(char* dst, int startPos, int count);

	void				acceptReserve();
	void				rejectReserve();

	void				reset();

	// debugging stubs
	void				dumpReserve();

protected:
	char				mBuffer[MAX_NMEA_PARSER_BUFFER];
	volatile int		mHead;
	volatile int		mTail;
	volatile int		mFront;
};


#endif // __DATA_QUEUE_H__

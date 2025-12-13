/*
 * LineBuffer.h
 *
 *
 */

#ifndef LINEBUFFER_H_
#define LINEBUFFER_H_


//
// LineBuffer
//
//

class LineBuffer
{
public:
	LineBuffer(char *buf, int size);

public:
	int push(int ch);
	void reset();

	bool hasCompleteLine() { return m_hasLine; }
	int getLength() { return m_pos; }

	operator const char *() { return m_buf; }

protected:
	char *m_buf;
	int m_max;
	int m_pos;
	bool m_hasLine;
};


#endif /* LINEBUFFER_H_ */

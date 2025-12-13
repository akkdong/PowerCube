/*
 * LineBuffer.cpp
 *
 *
 */

#include "LineBuffer.h"

#include <stddef.h>
#include <stdint.h>



//
// LineBuffer
//
//

LineBuffer::LineBuffer(char *buf, int size)
	: m_buf(buf)
	, m_max(size)
	, m_pos(0)
	, m_hasLine(false)
{
	reset();
}


int LineBuffer::push(int ch)
{
	if (ch == '\r' || ch == '\n')
	{
		if (!m_hasLine)
		{
			m_hasLine = true;
		}
		else
		{
			m_buf[0] = 0;
			m_pos = 0;
		}
	}
	else if (m_pos == m_max - 1)
	{
		return -1;
	}
	else
	{
		m_buf[m_pos] = ch;
		++m_pos;
		m_buf[m_pos] = 0;
		m_hasLine = false;
	}

	return ch;
}

void LineBuffer::reset()
{
	m_buf[0] = 0;
	m_pos = 0;
	m_hasLine = false;
}


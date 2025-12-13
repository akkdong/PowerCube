// VarioSentence.h
//

#ifndef __VARIOSENTENCE_H__
#define __VARIOSENTENCE_H__

#include "Digit.h"


/////////////////////////////////////////////////////////////////////////////
//

enum VarioSentenceType
{
	VSENTENCE_LK8,
	VSENTENCE_LXNAV
};




/////////////////////////////////////////////////////////////////////////////
// interface IVarioSentence

class IVarioSentence
{
public:
	virtual void		begin(float height, float vel, float temp, float bat) = 0;
	virtual void		begin(float height, float vel, float temp, float prs, float bat) = 0;
	
	virtual int			available() = 0;
	virtual int			read() = 0;
};


/////////////////////////////////////////////////////////////////////////////
// class LK8Sentence

class LK8Sentence : public IVarioSentence
{
public:
	virtual void		begin(float height, float vel, float temp, float bat);
	virtual void		begin(float height, float vel, float temp, float prs, float bat);
	
	virtual int			available();
	virtual int			read();
	
private:
	float				altimeter;
	float				vario;
	float				temperature;
	float				voltage;
	
	Digit 				valueDigit;
	HexDigit 			parityDigit;
	
	unsigned char		tagPos;
	unsigned char		parity;
	
	static const char *	LK8Tag;
};


/////////////////////////////////////////////////////////////////////////////
// class LxNavSentence

class LxNavSentence : public IVarioSentence
{
public:
	virtual void		begin(float height, float vel, float temp, float bat);
	virtual void		begin(float height, float vel, float temp, float prs, float bat);
	
	virtual int			available();
	virtual int			read();
	
private:
	float				vario;
	
	Digit 				valueDigit;
	HexDigit 			parityDigit;
	
	unsigned char		tagPos;
	unsigned char		parity;
	
	static const char *	LxNavTag;
};


/////////////////////////////////////////////////////////////////////////////
// class VarioSentence

class VarioSentence
{
public:
	VarioSentence(VarioSentenceType type);
	
public:
	void				begin(float height, float vel, float temp, float bat);
	void				begin(float height, float vel, float temp, float prs, float bat);
	
	int					available();	
	int					read();
	
	/*
	void				use(VarioSentenceType type)	{ sentenceType = type; }
	*/
	
	int					checkInterval();
	
private:
	IVarioSentence *	varioSentence;

	VarioSentenceType	sentenceType;
	unsigned long		lastTick;

	const uint32_t		maxInterval = 500;
};

#endif // __VARIOSENTENCE_H__

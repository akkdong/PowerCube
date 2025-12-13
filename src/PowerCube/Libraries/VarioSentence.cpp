// VarioSentence.cpp
//

#include <stdint.h>
//#include "config.h"
#include "utils.h"

#include "VarioSentence.h"


/////////////////////////////////////////////////////////////////////////////
//
	
LK8Sentence			LK8;
LxNavSentence		LxNav;


/////////////////////////////////////////////////////////////////////////////
// class VarioSentence

VarioSentence::VarioSentence(VarioSentenceType type) : sentenceType(type)
{
	varioSentence = 0;
	switch (type)
	{
	case VSENTENCE_LK8:
		varioSentence = (IVarioSentence *)&LK8;
		break;
	case VSENTENCE_LXNAV:
		varioSentence = (IVarioSentence *)&LxNav;
		break;
	}
				
	lastTick = millis();
}
	
void VarioSentence::begin(float height, float vel, float temp, float bat)
{
	varioSentence->begin(height, vel, temp, bat);
}

void VarioSentence::begin(float height, float vel, float temp, float prs, float bat)
{
	varioSentence->begin(height, vel, temp, prs, bat);
}
	
int VarioSentence::available()
{
	return varioSentence->available();
}
	
int VarioSentence::read()
{
	return varioSentence->read();
}

int VarioSentence::checkInterval()
{
	uint32_t tick = millis();
	if (! varioSentence->available() && (tick - lastTick) > maxInterval)
	{
		lastTick = tick;

		return true;
	}
	
	return false;
}

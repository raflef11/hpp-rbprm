#include "hpp/rbprm/Timer.h"

#include <time.h>

Timer::~Timer()
{
	// NOTHING
}

Timer::Timer()
{
	resetted_ = true;
	running_ = false;
	beg_ = 0;
	end_ = 0;
}


void Timer::Start(t_time initTime) {
	if(!running_)
	{
		if(resetted_)
			beg_ = (t_time) clock() - initTime;
		else
			beg_ -= end_ - (t_time) clock();
		running_ = true;
		resetted_ = false;
	}
}


void Timer::Stop() {
	if(running_)
	{
		end_ = (t_time) clock();
		running_ = false;
	}
}

void Timer::Reset(t_time initTime) {
	bool wereRunning = running_;
	if(wereRunning)
		Stop();
	resetted_ = true;
	beg_ = 0;
	end_ = 0;
	if(wereRunning)
		Start(initTime);
}


bool Timer::IsRunning() {
	return running_;
}


Timer::t_time Timer::GetTime() {
	if(running_)
        return ((t_time) clock() - beg_) / CLOCKS_PER_SEC;
	else
        return (end_ - beg_)/CLOCKS_PER_SEC;
}


bool Timer::IsOver(Timer::t_time milliseconds) {
	return milliseconds >= GetTime();
}

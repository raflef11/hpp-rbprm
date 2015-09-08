
#ifndef _CLASS_TIMER
#define _CLASS_TIMER


class Timer {

public:
	typedef double t_time;

public:
	  Timer();
	 ~Timer();

public:
	void    Start(t_time initTime = 0);
	void    Stop();
	void    Reset(t_time initTime = 0);
	bool    IsRunning();
	t_time	GetTime();
	bool    IsOver(t_time milliseconds);

private:
	bool    resetted_;
	bool    running_;
	t_time	beg_;
	t_time  end_;
}; // Timer

#endif //_CLASS_TIMER
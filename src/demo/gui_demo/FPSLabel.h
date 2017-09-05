#ifndef DEMO_GUI_DEMO_FPS_LABEL_H
#define DEMO_GUI_DEMO_FPS_LABEL_H

class Label;

class FPSLabel
{
public:
	FPSLabel(Label& label);

	void frameRendered();

	double getFPS() const { return fps_; }
private:
	double getWallTime();
	double getCPUTime();

	Label* label_;
	double current_time_;

	double frames_rendered_;
	double fps_;
};

#endif

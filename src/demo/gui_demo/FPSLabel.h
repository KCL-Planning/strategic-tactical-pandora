#ifndef DEMO_GUI_DEMO_FPS_LABEL_H
#define DEMO_GUI_DEMO_FPS_LABEL_H

namespace DreadedPE
{
	class Label;
};

class FPSLabel
{
public:
	FPSLabel(DreadedPE::Label& label);

	void frameRendered();

	double getFPS() const { return fps_; }
private:
	double getWallTime();
	double getCPUTime();

	DreadedPE::Label* label_;
	double current_time_;

	double frames_rendered_;
	double fps_;
};

#endif

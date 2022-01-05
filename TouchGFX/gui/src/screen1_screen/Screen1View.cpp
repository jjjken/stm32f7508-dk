#include <gui/screen1_screen/Screen1View.hpp>
#include <videos/VideoDatabase.hpp>

Screen1View::Screen1View()
{
	  video1.setPosition(112, 30, 240, 135);
    video1.setRepeat(true);
    video1.setVideoData(video_SampleVideo1_240x135_bin_start, video_SampleVideo1_240x135_bin_length);
}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}

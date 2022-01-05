#include <gui/subscreen_screen/SubScreenView.hpp>
#include "Serial.hpp"

SubScreenView::SubScreenView()
{

}

void SubScreenView::setupScreen()
{
    SubScreenViewBase::setupScreen();
}

void SubScreenView::tearDownScreen()
{
    SubScreenViewBase::tearDownScreen();
}

void SubScreenView::TestButtonCallback(){

	serial1.println("Clicked");
}

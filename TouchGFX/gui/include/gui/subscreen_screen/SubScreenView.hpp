#ifndef SUBSCREENVIEW_HPP
#define SUBSCREENVIEW_HPP

#include <gui_generated/subscreen_screen/SubScreenViewBase.hpp>
#include <gui/subscreen_screen/SubScreenPresenter.hpp>

class SubScreenView : public SubScreenViewBase
{
public:
    SubScreenView();
    virtual ~SubScreenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // SUBSCREENVIEW_HPP

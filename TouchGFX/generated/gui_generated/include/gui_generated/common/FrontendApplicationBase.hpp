/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef FRONTENDAPPLICATIONBASE_HPP
#define FRONTENDAPPLICATIONBASE_HPP

#include <mvp/MVPApplication.hpp>
#include <gui/model/Model.hpp>

class FrontendHeap;

class FrontendApplicationBase : public touchgfx::MVPApplication
{
public:
    FrontendApplicationBase(Model& m, FrontendHeap& heap);
    virtual ~FrontendApplicationBase() { }

    // Main
    void gotoMainScreenNoTransition();

    // SubScreen
    void gotoSubScreenScreenNoTransition();

    // Screen1
    void gotoScreen1ScreenNoTransition();

protected:
    touchgfx::Callback<FrontendApplicationBase> transitionCallback;
    FrontendHeap& frontendHeap;
    Model& model;

    // Main
    void gotoMainScreenNoTransitionImpl();

    // SubScreen
    void gotoSubScreenScreenNoTransitionImpl();

    // Screen1
    void gotoScreen1ScreenNoTransitionImpl();
};

#endif // FRONTENDAPPLICATIONBASE_HPP

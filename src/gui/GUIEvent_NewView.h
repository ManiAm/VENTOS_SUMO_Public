/****************************************************************************/
/// @file    GUIEvent_NewView.h
/// @author  Mani Amoozadeh
/// @date    June 2017
///
///
// Event send when a new view should be created
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2002-2017 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef GUIEvent_NewView_h
#define GUIEvent_NewView_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <iostream>
#include <utils/gui/events/GUIEvent.h>


// ===========================================================================
// class declarations
// ===========================================================================
class GUISUMOAbstractView;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class  GUIEvent_NewView
 *
 * Throw to GUIApplicationWindow from GUIRunThread to trigger creating a new view
 */
class GUIEvent_NewView : public GUIEvent {
public:
    /// constructor
    GUIEvent_NewView(GUISUMOAbstractView* view): GUIEvent(EVENT_OPENVIEW), myView(view) {

    }

    /// destructor
    ~GUIEvent_NewView() { }

public:
    /// @brief the view to save
    GUISUMOAbstractView* const myView;

    /// @brief the name of the file to save to
    const std::string myFile;

private:
    /// @brief Invalidated assignment operator
    GUIEvent_NewView& operator=(const GUIEvent_NewView& s);
};


#endif

/****************************************************************************/


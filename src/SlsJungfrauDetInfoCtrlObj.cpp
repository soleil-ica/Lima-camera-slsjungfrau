//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2018
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################

/*************************************************************************/
/*! 
 *  \file   SlsJungfrauDetInfoCtrlObj.h
 *  \brief  SlsJungfrau detector info class implementation 
 *  \author Cedric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#include "SlsJungfrauCamera.h"
#include "SlsJungfrauDetInfoCtrlObj.h"
#include <limits>

using namespace lima;
using namespace lima::SlsJungfrau;

/*******************************************************************
 * \brief constructor
 *******************************************************************/
DetInfoCtrlObj::DetInfoCtrlObj(Camera& cam) :    m_cam(cam)
{
    DEB_CONSTRUCTOR();
}

/*******************************************************************
 * \brief destructor
 *******************************************************************/
DetInfoCtrlObj::~DetInfoCtrlObj()
{
    DEB_DESTRUCTOR();
}

//==================================================================
// image size management
//==================================================================
/*******************************************************************
 * \brief gets the maximum image size
 *******************************************************************/
void DetInfoCtrlObj::getMaxImageSize(Size & size)
{
    DEB_MEMBER_FUNCT();

    // gets the max image size
    size = Size(m_cam.getMaxWidth(), m_cam.getMaxHeight());

    DEB_RETURN() << DEB_VAR1(size);
}

/*******************************************************************
 * \brief gets the detector image size
 *******************************************************************/
void DetInfoCtrlObj::getDetectorImageSize(Size& size)
{
    DEB_MEMBER_FUNCT();

    // gets the max image size of the detector
    size = Size(m_cam.getWidth(), m_cam.getHeight());

    DEB_RETURN() << DEB_VAR1(size);
}

//==================================================================
// current image type management
//==================================================================
/*******************************************************************
 * \brief gets the default image type
 *******************************************************************/
void DetInfoCtrlObj::getDefImageType(lima::ImageType & image_type)
{
    DEB_MEMBER_FUNCT();
    image_type = m_cam.getDefImageType();
    DEB_RETURN() << DEB_VAR1(image_type);
}

/*******************************************************************
 * \brief gets the current image type
 *******************************************************************/
void DetInfoCtrlObj::getCurrImageType(lima::ImageType & image_type)
{
    DEB_MEMBER_FUNCT();
    image_type = m_cam.getImageType();
    DEB_RETURN() << DEB_VAR1(image_type);
}

/*******************************************************************
 * \brief sets the current image type
 *******************************************************************/
void DetInfoCtrlObj::setCurrImageType(lima::ImageType image_type)
{
    DEB_MEMBER_FUNCT();
    m_cam.setImageType(image_type);
}

//==================================================================
// pixel size management
//==================================================================
/*******************************************************************
 * \brief gets the pixel size
 *******************************************************************/
void DetInfoCtrlObj::getPixelSize(double & x_size, double & y_size)
{
    DEB_MEMBER_FUNCT();
    m_cam.getPixelSize(x_size, y_size);
    DEB_RETURN() << DEB_VAR2(x_size, y_size);
}

//==================================================================
// detector info management
//==================================================================
/*******************************************************************
 * \brief gets the detector type
 *******************************************************************/
void DetInfoCtrlObj::getDetectorType(std::string & type)
{
    DEB_MEMBER_FUNCT();
    type = m_cam.getDetectorType();
    DEB_RETURN() << type;
}

/*******************************************************************
 * \brief gets the detector model
 *******************************************************************/
void DetInfoCtrlObj::getDetectorModel(std::string & model)
{
    DEB_MEMBER_FUNCT();
    model = m_cam.getDetectorModel();
    DEB_RETURN() << model;
}

//==================================================================
// maximum image size callback management
//==================================================================
/*******************************************************************
 * \brief registers maximum image size callback
 *******************************************************************/
void DetInfoCtrlObj::registerMaxImageSizeCallback(HwMaxImageSizeCallback & cb)
{
	m_cam.registerMaxImageSizeCallback(cb);
}

/*******************************************************************
 * \brief unregisters maximum image size callback
 *******************************************************************/
void DetInfoCtrlObj::unregisterMaxImageSizeCallback(HwMaxImageSizeCallback & cb)
{
	m_cam.unregisterMaxImageSizeCallback(cb);
}

/*************************************************************************/
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
 *  \brief  SlsJungfrau detector info class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#ifndef SLSJUNGFRAUDETINFOCTRLOBJ_H
#define SLSJUNGFRAUDETINFOCTRLOBJ_H

#include "lima/Debug.h"
#include "SlsJungfrauCompatibility.h"
#include "lima/HwDetInfoCtrlObj.h"

namespace lima 
{
    namespace SlsJungfrau 
    {
        class Camera;

        /***********************************************************************
         * \class DetInfoCtrlObj
         * \brief Control object providing SlsJungfrau detector info interface
         ***********************************************************************/

        class LIBSLSJUNGFRAU_API DetInfoCtrlObj: public HwDetInfoCtrlObj 
        {
            DEB_CLASS_NAMESPC(DebModCamera, "DetInfoCtrlObj", "SlsJungfrau");

        public:
            // constructor
            DetInfoCtrlObj(Camera & cam);

            // destructor
            virtual ~DetInfoCtrlObj();

            //==================================================================
            // image size management
            //==================================================================
            // gets the maximum image size
            virtual void getMaxImageSize(Size & max_image_size);
            
            // gets the detector image size
            virtual void getDetectorImageSize(Size & det_image_size);

            // gets the default image type
            virtual void getDefImageType(lima::ImageType & def_image_type);

            //==================================================================
            // current image type management
            //==================================================================
            // gets the current image type
            virtual void getCurrImageType(lima::ImageType & curr_image_type);

            // sets the current image type
            virtual void setCurrImageType(lima::ImageType curr_image_type);

            //==================================================================
            // pixel size management
            //==================================================================
            // gets the pixel size
            virtual void getPixelSize(double & x_size, double & y_size);

            //==================================================================
            // detector info management
            //==================================================================
            // gets the detector type
            virtual void getDetectorType(std::string & det_type);

            // gets the detector model
            virtual void getDetectorModel(std::string & det_model);

            //==================================================================
            // maximum image size callback management
            //==================================================================
            // registers maximum image size callback
            virtual void registerMaxImageSizeCallback(HwMaxImageSizeCallback & cb);
            
            // unregisters maximum image size callback
            virtual void unregisterMaxImageSizeCallback(HwMaxImageSizeCallback & cb);

        private:
            Camera & m_cam;
        };
    } // namespace SlsJungfrau
} // namespace lima

#endif // SLSJUNGFRAUDETINFOCTRLOBJ_H

/*************************************************************************/
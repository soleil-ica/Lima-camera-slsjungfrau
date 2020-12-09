//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2011
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
//
// SlsJungfrauImageSize.h
// Created on: August 05, 2020
// Author: Cedric Castel

#ifndef SLSJUNGFRAUIMAGESIZE_H_
#define SLSJUNGFRAUIMAGESIZE_H_

private:
    class MaxImageSizeCallbackGen: public HwMaxImageSizeCallbackGen
    {
    public:
        MaxImageSizeCallbackGen() 
        { 
            m_mis_cb_act = false; 
        }

        void updateImageFormat(const Size & size, ImageType image_type)
        { 
            if(m_mis_cb_act)
                maxImageSizeChanged(size, image_type);
        }
        
    protected:
        virtual void setMaxImageSizeCallbackActive(bool cb_active)
        { 
            m_mis_cb_act = cb_active; 
        }

    private:
        bool m_mis_cb_act;
    };

    MaxImageSizeCallbackGen m_mis_cb_gen;

public:
    void registerMaxImageSizeCallback(HwMaxImageSizeCallback& cb)
    {
        m_mis_cb_gen.registerMaxImageSizeCallback(cb);
    }

    void unregisterMaxImageSizeCallback(HwMaxImageSizeCallback& cb)
    {
        m_mis_cb_gen.unregisterMaxImageSizeCallback(cb);
    }

    void updateImageFormat()
    {
        DEB_MEMBER_FUNCT();

        ImageType image_depth;
        Size      image_size ;

        // specific part from the camera plugin [START]
        getDetectorImageSize(image_size);
    	getImageType(image_depth);
        // specific part from the camera plugin [END]

	    DEB_TRACE() << "Camera::updateImageFormat() " << image_size.getWidth () << ", " 
                                                      << image_size.getHeight() << ", " 
                                                      << image_depth;

        m_mis_cb_gen.updateImageFormat(image_size, image_depth);
    }

#endif // SLSJUNGFRAUIMAGESIZE_H_

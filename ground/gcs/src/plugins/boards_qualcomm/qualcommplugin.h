/**
 ******************************************************************************
 *
 * @file       qualcommplugin.h
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013
 * @author     Kenn Sebesta, Copyright (C) 2015
 *
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup Boards_Qualcomm Qualcomm boards support plugin
 * @{
 * @brief Plugin to support boards by Qualcomm
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#ifndef QUALCOMMPLUGIN_H
#define QUALCOMMPLUGIN_H

#include <extensionsystem/iplugin.h>

class QualcommPlugin : public ExtensionSystem::IPlugin
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "TauLabs.plugins.Qualcomm" FILE "Qualcomm.json")
    
public:
   QualcommPlugin();
   ~QualcommPlugin();

   void extensionsInitialized();
   bool initialize(const QStringList & arguments, QString * errorString);
   void shutdown();

};

#endif // QUALCOMMPLUGIN_H

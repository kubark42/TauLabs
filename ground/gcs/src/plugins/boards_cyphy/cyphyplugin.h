/**
 ******************************************************************************
 *
 * @file       cyphyplugin.h
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013
 *
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup Boards_CyPhy CyPhy Works boards support Plugin
 * @{
 * @brief Plugin to support boards by CyPhy Works
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
#ifndef CYPHYPLUGIN_H
#define CYPHYPLUGIN_H

#include <extensionsystem/iplugin.h>

class CyPhyPlugin : public ExtensionSystem::IPlugin
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "TauLabs.plugins.CyPhy" FILE "CyPhy.json")
    
public:
   CyPhyPlugin();
   ~CyPhyPlugin();

   void extensionsInitialized();
   bool initialize(const QStringList & arguments, QString * errorString);
   void shutdown();

};

#endif // CYPHYPLUGIN_H

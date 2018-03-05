/*ckwg +29
 * Copyright 2016-2017 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name Kitware, Inc. nor the names of any contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "MainWindow.h"
#include "tools/AbstractTool.h"
#include "VideoImport.h"

#include <maptk/version.h>

#include <kwiversys/SystemTools.hxx>
#include <vital/plugin_loader/plugin_manager.h>

#include <qtCliArgs.h>
#include <qtStlUtil.h>
#include <qtUtil.h>

#include <QApplication>
#include <QMetaType>
#include <QtCore/QDir>

#include <memory>

//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{
  // Set application information
  QApplication::setApplicationName("MAP-Tk TeleSculptor");
  QApplication::setOrganizationName("Kitware");
  QApplication::setOrganizationDomain("kitware.com");
  QApplication::setApplicationVersion(MAPTK_VERSION);

  // Register meta types
  qRegisterMetaType<std::shared_ptr<ToolData>>();
  qRegisterMetaType<std::shared_ptr<VideoData>>();

  // Set up command line options
  qtCliArgs args(argc, argv);
  qtCliOptions nargs;

  nargs.add("files", "List of files to open", qtCliOption::NamedList);
  args.addNamedArguments(nargs);

  // Parse arguments
  args.parseOrDie();

  // Create application instance and set icon
  QApplication app(args.qtArgc(), args.qtArgv());
  qtUtil::setApplicationIcon("TeleSculptor");

  // Load KWIVER plugins
  auto const exeDir = QDir(QApplication::applicationDirPath());
  auto const rel_path = stdString(exeDir.absoluteFilePath(".."));
  auto & vpm = kwiver::vital::plugin_manager::instance();
  vpm.add_search_path(rel_path + "/lib/modules");
  vpm.add_search_path(rel_path + "/lib/sprokit");
  vpm.load_all_plugins();

  // Tell PROJ where to find its data files
  std::string rel_proj_path = rel_path + "/share/proj";
  if ( kwiversys::SystemTools::FileExists(rel_proj_path) &&
       kwiversys::SystemTools::FileIsDirectory(rel_proj_path) )
  {
    kwiversys::SystemTools::PutEnv("PROJ_LIB="+rel_proj_path);
  }

  // Create and show main window
  MainWindow window;
  window.show();
  window.openFiles(args.values("files"));

  // Hand off to event loop
  return app.exec();
}

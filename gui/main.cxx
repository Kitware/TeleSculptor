/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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

#include <iostream>

#include <maptk/version.h>

#include <vital/algorithm_plugin_manager.h>

#include <qtCliArgs.h>
#include <qtStlUtil.h>
#include <qtUtil.h>

#include <QApplication>
#include <QtCore/QDir>

#include <kwiversys/CommandLineArguments.hxx>

//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{
  // Set application information
  QApplication::setApplicationName("MapGUI");
  QApplication::setOrganizationName("Kitware");
  QApplication::setOrganizationDomain("kitware.com");
  QApplication::setApplicationVersion(MAPTK_VERSION);

  // Set up command line options
  qtCliArgs args(argc, argv);
  qtCliOptions nargs;

  nargs.add("files", "List of files to open", qtCliOption::NamedList);
  args.addNamedArguments(nargs);

  // Parse arguments
  args.parseOrDie();

  // Create application instance and set icon
  QApplication app(args.qtArgc(), args.qtArgv());
  qtUtil::setApplicationIcon("mapgui");

  bool help;
  std::string openFilePath;

  typedef kwiversys::CommandLineArguments argT;

  argT arg;

  arg.Initialize(argc, argv);

  arg.AddBooleanArgument("--help", &help, "Display usage information");
  arg.AddBooleanArgument("-h", &help, "Display usage information");
  arg.AddArgument("--input", argT::SPACE_ARGUMENT,
                  &openFilePath, "File to open at launch");
  arg.AddArgument("-i", argT::SPACE_ARGUMENT,
                  &openFilePath, "File to open at launch");

  if(!arg.Parse() || help)
  {
    std::cout << "USAGE:" << std::endl;
    std::cout << arg.GetHelp() << std::endl;
    return EXIT_FAILURE;
  }

  // Load Vital/MAP-Tk plugins
  auto const exeDir = QDir(QApplication::applicationDirPath());
  auto const rel_path = stdString(exeDir.absoluteFilePath("..")) + "/lib/maptk";
  kwiver::vital::algorithm_plugin_manager::instance().add_search_path(rel_path);
  kwiver::vital::algorithm_plugin_manager::instance().register_plugins();

  // Create and show main window
  MainWindow window;

  if (!openFilePath.empty())
  {
    window.openFile(QString::fromStdString(openFilePath));
  }

  window.show();
  window.openFiles(args.values("files"));

  // Hand off to event loop
  return app.exec();
}

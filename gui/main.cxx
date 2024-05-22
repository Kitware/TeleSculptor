// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "MainWindow.h"
#include "tools/AbstractTool.h"
#include "VideoImport.h"

#include <maptk/version.h>

#include <kwiversys/SystemTools.hxx>
#include <vital/plugin_management/plugin_manager.h>

#include <qtCliArgs.h>
#include <qtStlUtil.h>
#include <qtUtil.h>

#include <QApplication>
#include <QDir>
#include <QMetaType>
#include <QSurfaceFormat>

#include <memory>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkOpenGLRenderWindow.h>

//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{
  // Set the default surface format for the OpenGL view
  vtkOpenGLRenderWindow::SetGlobalMaximumNumberOfMultiSamples(0);
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
  vtkObject::GlobalWarningDisplayOff();

  // Set application information
  QApplication::setApplicationName("TeleSculptor");
  QApplication::setOrganizationName("Kitware");
  QApplication::setOrganizationDomain("kitware.com");
  QApplication::setApplicationVersion(TELESCULPTOR_VERSION);

  // Tell Qt to fully use High-DPI scaling, as the partial implementation we
  // get otherwise is worse (and doesn't use our high-resolution icons)
  QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

  // Register meta types
  using map_metadata_t = kwiver::vital::metadata_map::map_metadata_t;
  qRegisterMetaType<std::shared_ptr<ToolData>>();
  qRegisterMetaType<std::shared_ptr<map_metadata_t>>();

  // Set up command line options
  qtCliArgs args(argc, argv);
  qtCliOptions options;

  options.add("project <file>", "Load specified project file")
         .add("p", qtCliOption::Short);
  options.add("imagery <file>", "Load imagery from 'file'")
         .add("i", qtCliOption::Short | qtCliOption::NamedList);
  options.add("mask <file>", "Load mask imagery from 'file'")
         .add("m", qtCliOption::Short | qtCliOption::NamedList);
  options.add("camera <file>", "Load camera(s) from 'file'")
         .add("c", qtCliOption::Short | qtCliOption::NamedList);
  options.add("tracks <file>", "Load feature tracks from 'file'")
         .add("t", qtCliOption::Short | qtCliOption::NamedList);
  options.add("landmarks <file>", "Load landmarks from 'file'")
         .add("l", qtCliOption::Short | qtCliOption::NamedList);
  args.addOptions(options);

  // Parse arguments
  args.parseOrDie();

  // Create application instance and set icon
  QApplication app(args.qtArgc(), args.qtArgv());
  qtUtil::setApplicationIcon("telesculptor");

  auto const exeDir = QDir{ QApplication::applicationDirPath() };
  auto const plugin_path =
    stdString(exeDir.absoluteFilePath("../lib/kwiver/plugins"));

  // Prefer the log4cplus logger if no logger is specified
  std::string logger;
  if (!kwiversys::SystemTools::GetEnv("VITAL_LOGGER_FACTORY", logger))
  {
    auto const logger_path = plugin_path + "/logger/vital_log4cplus_logger";
    kwiversys::SystemTools::PutEnv("VITAL_LOGGER_FACTORY=" + logger_path);
  }

  // Load KWIVER plugins
  auto& vpm = kwiver::vital::plugin_manager::instance();
  vpm.add_search_path(plugin_path);
  vpm.load_all_plugins(kwiver::vital::plugin_manager::plugin_type::ALGORITHMS);

  // Tell PROJ where to find its data files
  auto projDataDir = exeDir.absoluteFilePath("../share/proj");
  if (QFileInfo{projDataDir}.isDir())
  {
    qputenv("PROJ_LIB", projDataDir.toLocal8Bit());
  }

  // Tell GDAL where to find its data files
  auto gdalDataDir = exeDir.absoluteFilePath("../share/gdal");
  if (QFileInfo{ gdalDataDir }.isDir())
  {
    qputenv("GDAL_DATA", gdalDataDir.toLocal8Bit());
  }

  // Create and show main window
  MainWindow window;
  window.show();

  if (args.isSet("project"))
  {
    window.loadProject(args.value("project"));
  }
  for (auto const& path : args.values("imagery"))
  {
    window.loadImagery(path);
  }
  for (auto const& path : args.values("mask"))
  {
    window.loadMaskImagery(path);
  }
  for (auto const& path : args.values("camera"))
  {
    window.loadCamera(path);
  }
  for (auto const& path : args.values("tracks"))
  {
    window.loadTracks(path);
  }
  for (auto const& path : args.values("landmarks"))
  {
    window.loadLandmarks(path);
  }

  // Hand off to event loop
  return app.exec();
}

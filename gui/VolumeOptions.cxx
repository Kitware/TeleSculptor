// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "VolumeOptions.h"
#include "ui_VolumeOptions.h"

#include "ColorizeSurfaceOptions.h"
#include "WorldView.h"

#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QMenu>
#include <QToolButton>
#include <QWidgetAction>

#include <vtkActor.h>
#include <vtkColorTransferFunction.h>
#include <vtkPolyDataMapper.h>
#include <vtkDataSet.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>

#include <string>

namespace // anonymous
{

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkColorTransferFunction> createBluesColorPalette(
  double* range)
{
  auto lut = vtkSmartPointer<vtkColorTransferFunction>::New();
  std::array<std::array<double, 3>, 17> blues = {{
      {0.031373, 0.188235, 0.419608},
      {0.031373, 0.253195, 0.516063},
      {0.031757, 0.318139, 0.612149},
      {0.080969, 0.38113, 0.661361},
      {0.130427, 0.444152, 0.710327},
      {0.195386, 0.509112, 0.743791},
      {0.260715, 0.573841, 0.777209},
      {0.341423, 0.628958, 0.808704},
      {0.422745, 0.684075, 0.839892},
      {0.523137, 0.739193, 0.861546},
      {0.622684, 0.793464, 0.883429},
      {0.701423, 0.826928, 0.910988},
      {0.778685, 0.8603, 0.937993},
      {0.825928, 0.891795, 0.953741},
      {0.87328, 0.923291, 0.969489},
      {0.922491, 0.954787, 0.985236},
      {0.968627, 0.984314, 1}}};
  for (size_t i = 0; i < blues.size(); ++i)
  {
    lut->AddRGBPoint(
      range[0] + (range[1] - range[0]) * i / (blues.size() - 1),
      blues[i][0], blues[i][1], blues[i][2]);
  }
  // blues from ParaView: ColorMap.json
  lut->SetColorSpaceToLab();
  return lut;
}

}  // namespace <anonymous>

//-----------------------------------------------------------------------------
class VolumeOptionsPrivate
{
public:
  VolumeOptionsPrivate():colorizeSurfaceOptions(nullptr),volumeActor(nullptr)
  {}

  void setPopup(QToolButton* button, QWidget* popup);

  Ui::VolumeOptions UI;
  qtUiState uiState;
  ColorizeSurfaceOptions* colorizeSurfaceOptions;

  vtkActor* volumeActor;
  vtkDataArray* originalColorArray;

};

QTE_IMPLEMENT_D_FUNC(VolumeOptions)

//-----------------------------------------------------------------------------
void VolumeOptionsPrivate::setPopup(QToolButton* button, QWidget* widget)
{
  auto const proxy = new QWidgetAction(button);
  proxy->setDefaultWidget(widget);

  auto const menu = new QMenu(button);
  menu->addAction(proxy);

  button->setMenu(menu);
}

//-----------------------------------------------------------------------------
VolumeOptions::VolumeOptions(const QString &settingsGroup, QWidget* parent,
                             Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new VolumeOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->uiState.restore();

  d->colorizeSurfaceOptions = new ColorizeSurfaceOptions(settingsGroup, this);
  d->originalColorArray = nullptr;
  d->setPopup(d->UI.toolButtonColorizeSurfaceMenu, d->colorizeSurfaceOptions);

  // Connect signals/slots
  connect(d->UI.comboBoxColorizeSurface,
          QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &VolumeOptions::showColorizeSurfaceMenu);

  connect(d->colorizeSurfaceOptions, &ColorizeSurfaceOptions::colorModeChanged,
          this, &VolumeOptions::updateColorizeSurfaceMenu);

  connect(d->UI.doubleSpinBoxSurfaceThreshold,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          qobject_cast<WorldView*>(parent), &WorldView::computeContour);
}

//-----------------------------------------------------------------------------
VolumeOptions::~VolumeOptions()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void VolumeOptions::setActor(vtkActor *actor)
{
  QTE_D();

  d->volumeActor = actor;
  d->colorizeSurfaceOptions->setActor(actor);
}

//-----------------------------------------------------------------------------
void VolumeOptions::initFrameSampling(int nbFrames)
{
  QTE_D();

  d->colorizeSurfaceOptions->initFrameSampling(nbFrames);
}

//-----------------------------------------------------------------------------
int VolumeOptions::getFrameSampling() const
{
  QTE_D();

  return d->colorizeSurfaceOptions->getFrameSampling();
}

//-----------------------------------------------------------------------------
double VolumeOptions::getOcclusionThreshold() const
{
  QTE_D();

  return d->colorizeSurfaceOptions->getOcclusionThreshold();
}

//-----------------------------------------------------------------------------
void VolumeOptions::setCamera(
  kwiver::vital::frame_id_t id, kwiver::vital::camera_sptr const& camera)
{
  QTE_D();

  d->colorizeSurfaceOptions->setCamera(id, camera);
}

//-----------------------------------------------------------------------------
kwiver::vital::camera_map_sptr VolumeOptions::getCameras() const
{
  QTE_D();

  return d->colorizeSurfaceOptions->getCameras();
}

//-----------------------------------------------------------------------------
void VolumeOptions::setVideoConfig(std::string const& path,
                                   kwiver::vital::config_block_sptr config)
{
  QTE_D();

  d->colorizeSurfaceOptions->setVideoConfig(path, config);
}

//-----------------------------------------------------------------------------
kwiver::vital::config_block_sptr VolumeOptions::getVideoConfig() const
{
  QTE_D();

  return d->colorizeSurfaceOptions->getVideoConfig();
}

//-----------------------------------------------------------------------------
std::string VolumeOptions::getVideoPath() const
{
  QTE_D();

  return d->colorizeSurfaceOptions->getVideoPath();
}

//-----------------------------------------------------------------------------
void VolumeOptions::setMaskConfig(std::string const& path,
                                   kwiver::vital::config_block_sptr config)
{
  QTE_D();

  d->colorizeSurfaceOptions->setMaskConfig(path, config);
}

//-----------------------------------------------------------------------------
kwiver::vital::config_block_sptr VolumeOptions::getMaskConfig() const
{
  QTE_D();

  return d->colorizeSurfaceOptions->getMaskConfig();
}

//-----------------------------------------------------------------------------
std::string VolumeOptions::getMaskPath() const
{
  QTE_D();

  return d->colorizeSurfaceOptions->getMaskPath();
}

//-----------------------------------------------------------------------------
void VolumeOptions::colorize()
{
  QTE_D();

  d->colorizeSurfaceOptions->colorize();
}

//-----------------------------------------------------------------------------
void VolumeOptions::forceColorize()
{
  QTE_D();

  d->colorizeSurfaceOptions->forceColorize();
}

//-----------------------------------------------------------------------------
void VolumeOptions::setCurrentFrame(kwiver::vital::frame_id_t frame)
{
  QTE_D();

  d->colorizeSurfaceOptions->setCurrentFrame(frame);
}

//-----------------------------------------------------------------------------
void VolumeOptions::setSurfaceColor(int index)
{
  QTE_D();

  d->UI.comboBoxColorizeSurface->setCurrentIndex(index);
}

//-----------------------------------------------------------------------------
bool VolumeOptions::isColorOptionsEnabled()
{
  QTE_D();

  return  d->UI.comboBoxColorizeSurface->currentIndex() == IMAGE_COLOR;
}


//-----------------------------------------------------------------------------
bool VolumeOptions::validForColoring(vtkDataArray* a, bool& mapScalars)
{
  if (!a)
  {
    return false;
  }
  int numberOfComponents = a->GetNumberOfComponents();
  if (numberOfComponents == 4 || numberOfComponents == 3)
  {
    // RGBA or RGB
    mapScalars = false;
    return true;
  }
  else if (numberOfComponents == 1)
  {
    mapScalars = true;
    return true;
  }
  else
  {
    return false;
  }
}

//-----------------------------------------------------------------------------
void VolumeOptions::showColorizeSurfaceMenu(int index)
{
  QTE_D();
  d->UI.toolButtonColorizeSurfaceMenu->setEnabled(index == IMAGE_COLOR);
  d->colorizeSurfaceOptions->enableMenu(index == IMAGE_COLOR);
  vtkMapper* meshMapper = d->volumeActor->GetMapper();
  switch(index)
  {
  case ORIGINAL_COLOR:
    bool mapScalars;
    if (validForColoring(d->originalColorArray, mapScalars))
    {
      vtkPolyData* volume = vtkPolyData::SafeDownCast(
        d->volumeActor->GetMapper()->GetInput());
      volume->GetPointData()->SetScalars(d->originalColorArray);
      meshMapper->SetScalarVisibility(true);
      if (mapScalars)
      {
        double range[2];
        d->originalColorArray->GetRange(range);
        meshMapper->SetLookupTable(createBluesColorPalette(range));
        meshMapper->SetColorModeToMapScalars();
      }
      else
      {
        // for char, short, int, long arrays 0-255 maps to color component
        // for float arrays 0-1 maps to color component
        meshMapper->SetColorModeToDirectScalars();
      }
    }
    break;
  case IMAGE_COLOR:
    this->forceColorize();
    // we always color by mean first.
    meshMapper->SetColorModeToDirectScalars();
    break;
  }
  d->volumeActor->GetMapper()->SetScalarVisibility(
    index == ORIGINAL_COLOR || index == IMAGE_COLOR);
  emit modified();
}

//-----------------------------------------------------------------------------
void VolumeOptions::setOriginalColorArray(vtkDataArray* dataArray)
{
  QTE_D();
  d->originalColorArray = dataArray;
}

//-----------------------------------------------------------------------------
void VolumeOptions::reshowColorizeSurfaceMenu()
{
  QTE_D();
  if (d->volumeActor)
  {
    d->volumeActor->GetMapper()->Update();
    this->showColorizeSurfaceMenu(d->UI.comboBoxColorizeSurface->currentIndex());
  }
}

//-----------------------------------------------------------------------------
void VolumeOptions::updateColorizeSurfaceMenu(QString const& text)
{
  QTE_D();

  d->UI.toolButtonColorizeSurfaceMenu->setText(text);

  emit modified();
}

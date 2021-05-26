// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "ColorizeSurfaceOptions.h"
#include "MainWindow.h"
#include "ui_ColorizeSurfaceOptions.h"

#include "tools/MeshColoration.h"

#include <vtkActor.h>

#include <vtkLookupTable.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>

#include <qtStlUtil.h>
#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QDebug>

namespace kv = kwiver::vital;

//-----------------------------------------------------------------------------
class ColorizeSurfaceOptionsPrivate
{
public:
  MainWindow* mainWindow = nullptr;

  Ui::ColorizeSurfaceOptions UI;
  qtUiState uiState;

  vtkActor* volumeActor = nullptr;

  kv::config_block_sptr videoConfig;
  std::string videoPath;
  kv::config_block_sptr maskConfig;
  std::string maskPath;
  kv::camera_map_of_sptr<kv::camera> cameras =
    std::make_shared<kv::camera_map_of_<kv::camera>>();

  QString krtdFile;
  QString frameFile;

  kv::frame_id_t currentFrame = -1;
};

QTE_IMPLEMENT_D_FUNC(ColorizeSurfaceOptions)

//-----------------------------------------------------------------------------
ColorizeSurfaceOptions::ColorizeSurfaceOptions(
  const QString &settingsGroup, QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new ColorizeSurfaceOptionsPrivate)
{
  QTE_D();

  // find the main window
  QObject* p = parent;
  d->mainWindow = qobject_cast<MainWindow*>(p);
  while (p && !d->mainWindow)
  {
    p = p->parent();
    d->mainWindow = qobject_cast<MainWindow*>(p);
  }
  Q_ASSERT(d->mainWindow);

  // Set up UI
  d->UI.setupUi(this);

  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->uiState.restore();

  // Connect signals/slots
  connect(d->UI.radioButtonCurrentFrame, &QAbstractButton::clicked,
          this, &ColorizeSurfaceOptions::currentFrameSelected);

  connect(d->UI.radioButtonAllFrames, &QAbstractButton::clicked,
          this, &ColorizeSurfaceOptions::allFrameSelected);

  connect(d->UI.buttonCompute, &QAbstractButton::clicked,
          this, &ColorizeSurfaceOptions::colorize);

  connect(d->UI.comboBoxColorDisplay,
          QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &ColorizeSurfaceOptions::changeColorDisplay);

  connect(d->UI.doubleSpinBoxOcclusionThreshold, &QDoubleSpinBox::editingFinished,
          this, &ColorizeSurfaceOptions::updateOcclusionThreshold);
  connect(d->UI.checkBoxRemoveOccluded,
          &QCheckBox::stateChanged,
          this, &ColorizeSurfaceOptions::removeOccludedChanged);
  connect(d->UI.checkBoxRemoveMasked,
          &QCheckBox::stateChanged,
          this, &ColorizeSurfaceOptions::removeMaskedChanged);

  d->krtdFile = QString();
  d->frameFile = QString();

  d->UI.comboBoxColorDisplay->setDuplicatesEnabled(false);
  this->OcclusionThreshold = 1;
  this->RemoveOccluded = true;
  this->RemoveMasked = true;
  this->InsideColorize = false;
  this->LastColorizedFrame = INVALID_FRAME;
}

//-----------------------------------------------------------------------------
ColorizeSurfaceOptions::~ColorizeSurfaceOptions()
{
  QTE_D();

  d->uiState.save();
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::initFrameSampling(int nbFrames)
{
  QTE_D();

  d->UI.spinBoxFrameSampling->setMaximum(nbFrames-1);
  d->UI.spinBoxFrameSampling->setValue((nbFrames-1)/20);
}

//-----------------------------------------------------------------------------
int ColorizeSurfaceOptions::getFrameSampling() const
{
  QTE_D();

  return d->UI.spinBoxFrameSampling->value();
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::setCurrentFrame(kv::frame_id_t frame)
{
  QTE_D();

  if (d->currentFrame != frame)
  {
    d->currentFrame = frame;

    if (d->UI.radioButtonCurrentFrame->isChecked()
        && d->UI.radioButtonCurrentFrame->isEnabled())
    {
      currentFrameSelected();
    }
  }
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::setActor(vtkActor* actor)
{
  QTE_D();

  d->volumeActor = actor;
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::setVideoConfig(
  std::string const& path, kv::config_block_sptr config)
{
  QTE_D();

  d->videoConfig = config;
  d->videoPath = path;
}

//-----------------------------------------------------------------------------
kv::config_block_sptr ColorizeSurfaceOptions::getVideoConfig() const
{
  QTE_D();

  return d->videoConfig;
}

//-----------------------------------------------------------------------------
std::string ColorizeSurfaceOptions::getVideoPath() const
{
  QTE_D();

  return d->videoPath;
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::setMaskConfig(
  std::string const& path, kv::config_block_sptr config)
{
  QTE_D();
  d->maskConfig = config;
  d->maskPath = path;
  if (d->UI.radioButtonAllFrames->isEnabled())
  {
    this->forceColorize();
    d->UI.checkBoxRemoveMasked->setEnabled(! d->maskPath.empty());
  }
}

//-----------------------------------------------------------------------------
kv::config_block_sptr ColorizeSurfaceOptions::getMaskConfig() const
{
  QTE_D();
  return d->maskConfig;
}

//-----------------------------------------------------------------------------
std::string ColorizeSurfaceOptions::getMaskPath() const
{
  QTE_D();
  return d->maskPath;
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::setCamera(
  kv::frame_id_t id, kv::camera_sptr const& camera)
{
  QTE_D();

  if (camera)
  {
    d->cameras->insert(id, camera);
  }
  else
  {
    d->cameras->erase(id);
  }
}

//-----------------------------------------------------------------------------
kwiver::vital::camera_map_sptr ColorizeSurfaceOptions::getCameras() const
{
  QTE_D();

  return d->cameras;
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::enableMenu(bool state)
{
  QTE_D();

  d->UI.radioButtonAllFrames->setEnabled(state);
  d->UI.radioButtonCurrentFrame->setEnabled(state);
  d->UI.doubleSpinBoxOcclusionThreshold->setEnabled(state);
  d->UI.checkBoxRemoveOccluded->setEnabled(state);
  d->UI.checkBoxRemoveMasked->setEnabled(state && ! d->maskPath.empty());
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::changeColorDisplay()
{
  QTE_D();

  vtkPolyData* volume = vtkPolyData::SafeDownCast(
    d->volumeActor->GetMapper()->GetInput());
  const char* activeScalar = qPrintable(d->UI.comboBoxColorDisplay->currentText());
  volume->GetPointData()->SetActiveScalars(activeScalar);

  vtkMapper* mapper = d->volumeActor->GetMapper();

  if(volume->GetPointData()->GetScalars() &&
     volume->GetPointData()->GetScalars()->GetNumberOfComponents() != 3)
  {
    vtkNew<vtkLookupTable> table;
    table->SetRange(volume->GetPointData()->GetScalars()->GetRange());
    table->Build();
    mapper->SetLookupTable(table.Get());
    mapper->SetColorModeToMapScalars();
    mapper->UseLookupTableScalarRangeOn();
  }
  else
  {
    mapper->SetColorModeToDirectScalars();
    mapper->CreateDefaultLookupTable();
    mapper->UseLookupTableScalarRangeOff();
  }

  mapper->Update();

  emit colorModeChanged(d->UI.buttonGroup->checkedButton()->text());
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::forceColorize()
{
  this->LastColorizedFrame = INVALID_FRAME;
  colorize();
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::colorize()
{
  QTE_D();
  if (! this->InsideColorize)
  {
    this->InsideColorize = true;
    auto colorizedFrame =
      (d->UI.radioButtonCurrentFrame->isChecked() ? d->currentFrame : -1);
    while (this->LastColorizedFrame != colorizedFrame)
    {
      this->LastColorizedFrame = colorizedFrame;

      if (!d->cameras || d->cameras->size() == 0)
      {
        d->UI.comboBoxColorDisplay->setEnabled(true);
        emit colorModeChanged(d->UI.buttonGroup->checkedButton()->text());
        return;
      }
      QEventLoop loop;
      vtkPolyData* volume = vtkPolyData::SafeDownCast(d->volumeActor->GetMapper()
                                                      ->GetInput());
      MeshColoration* coloration =
        new MeshColoration(d->videoConfig, d->videoPath,
                           d->maskConfig, d->maskPath,
                           d->cameras);

      coloration->set_input(volume);
      coloration->set_output(volume);
      coloration->set_frame_sampling(d->UI.spinBoxFrameSampling->value());
      coloration->set_occlusion_threshold(this->OcclusionThreshold);
      coloration->set_remove_occluded(this->RemoveOccluded);
      coloration->set_remove_masked(this->RemoveMasked);
      coloration->set_frame(this->LastColorizedFrame);
      coloration->set_all_frames(false);
      connect(coloration, &MeshColoration::resultReady,
              this, &ColorizeSurfaceOptions::meshColorationHandleResult);
      connect( coloration, &MeshColoration::resultReady, &loop, &QEventLoop::quit );
      connect(coloration, &MeshColoration::finished,
              coloration, &MeshColoration::deleteLater);
      if (this->LastColorizedFrame < 0)
      {
        // only enable the progress bar if coloring with multiple frames
        connect(coloration, &MeshColoration::progressChanged,
                d->mainWindow, &MainWindow::updateToolProgress);
      }
      coloration->start();
      loop.exec();
      colorizedFrame = (d->UI.radioButtonCurrentFrame->isChecked()) ? d->currentFrame : -1;
    }
    this->InsideColorize = false;
  }
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::meshColorationHandleResult(
  MeshColoration* coloration)
{
  QTE_D();
  if (coloration)
  {
    vtkPolyData* volume = coloration->get_output();
    volume->GetPointData()->SetActiveScalars("mean");
    d->UI.comboBoxColorDisplay->setCurrentIndex(
      d->UI.comboBoxColorDisplay->findText("mean"));
    d->UI.comboBoxColorDisplay->setEnabled(true);
    emit colorModeChanged(d->UI.buttonGroup->checkedButton()->text());
  }
  d->UI.checkBoxRemoveOccluded->setEnabled(coloration);
  d->UI.doubleSpinBoxOcclusionThreshold->setEnabled(coloration);
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::enableAllFramesParameters(bool state)
{
  QTE_D();

  d->UI.buttonCompute->setEnabled(state);
  d->UI.spinBoxFrameSampling->setEnabled(state);
  d->UI.comboBoxColorDisplay->setEnabled(false);
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::allFrameSelected()
{
  enableAllFramesParameters(true);
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::currentFrameSelected()
{
  QTE_D();

  enableAllFramesParameters(false);

  if (d->currentFrame != -1)
  {
    colorize();
  }
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::updateOcclusionThreshold()
{
  QTE_D();
  this->setOcclusionThreshold(d->UI.doubleSpinBoxOcclusionThreshold->value());
  this->forceColorize();
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::removeOccludedChanged(int removeOccluded)
{
  this->setRemoveOccluded(removeOccluded);
  this->forceColorize();
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::removeMaskedChanged(int removeMasked)
{
  this->setRemoveMasked(removeMasked);
  this->forceColorize();
}

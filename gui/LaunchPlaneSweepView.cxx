/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
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

#include "LaunchPlaneSweepView.h"

#include "ui_LaunchPlaneSweepView.h"

#include "OutputDialog.h"

#include <qtUiState.h>
#include <qtUiStateItem.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <QProcess>
#include <QByteArray>

///////////////////////////////////////////////////////////////////////////////

//BEGIN LaunchPlaneSweepViewPrivate

//-----------------------------------------------------------------------------
class LaunchPlaneSweepViewPrivate
{
public:

  Ui::LaunchPlaneSweepView UI;
  qtUiState uiState;

  QStringList args;

  QProcess *psl;

  std::string krtdFolder;
  std::string framesFolder;
  std::string frameList;
  std::string landmarksFile;

  void addArg(QWidget* item, std::string value="");
  void addArg(QWidget *item, double value);

  OutputDialog* dialog;
};

QTE_IMPLEMENT_D_FUNC(LaunchPlaneSweepView)

//END LaunchPlaneSweepViewPrivate

//BEGIN LaunchPlaneSweepView

//-----------------------------------------------------------------------------
LaunchPlaneSweepView::LaunchPlaneSweepView(QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new LaunchPlaneSweepViewPrivate)
{
  QTE_D();

  this->setAttribute(Qt::WA_DeleteOnClose);

  // Set up UI
  d->UI.setupUi(this);

  // Set up UI persistence and restore previous state
  d->uiState.setCurrentGroup("LaunchPlaneSweepView");


  // Set up signals/slots
  connect(d->UI.pushButtonCompute, SIGNAL(clicked(bool)),
          this, SLOT(compute()));

  connect(d->UI.checkBoxDepthAuto, SIGNAL(toggled(bool)),
          d->UI.doubleSpinBoxDepthMin, SLOT(setDisabled(bool)));


  connect(d->UI.checkBoxDepthAuto, SIGNAL(toggled(bool)),
          d->UI.doubleSpinBoxDepthMax, SLOT(setDisabled(bool)));

  connect(d->UI.comboBoxMatchCost, SIGNAL(currentIndexChanged(int)),
          this, SLOT(enableColorMatching()));

  d->dialog = new OutputDialog();

  d->psl = new QProcess();

  connect(d->psl, SIGNAL(readyRead()),
         d->dialog, SLOT(ouputPSL()));

  connect(d->psl, SIGNAL(finished(int)),
         this, SLOT(initialState()));

  connect(d->UI.pushButtonStop, SIGNAL(clicked(bool)),
         d->psl, SLOT(kill()));
}

//-----------------------------------------------------------------------------
LaunchPlaneSweepView::~LaunchPlaneSweepView()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepView::setKrtdFolder(std::string krtdFolder)
{
  QTE_D();

  d->krtdFolder = "--krtdFolder="+krtdFolder;
}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepView::setFramesFolder(std::string framesFolder)
{
  QTE_D();

  d->framesFolder = framesFolder;
}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepView::setLandmarksFile(std::string landmarksFile)
{
  QTE_D();

  d->landmarksFile = "--landmarksPLY=" + landmarksFile;
}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepView::compute()
{
  QTE_D();

  std::string pslPath = d->UI.lineEditPSLPath->text().toStdString();

  d->args << "--debug";

  //Getting frame folder

  std::ifstream frameList(d->frameList);
  std::string framePath, frameFolder, imageListFile;

  imageListFile = "--imageListFile=" + d->frameList;

  frameList >> framePath;
  frameFolder = framePath.substr(0,framePath.find_last_of("/"));

  frameList.close();

  frameFolder = "--frameFolder=" + frameFolder;

  d->args <<  QString::fromStdString(frameFolder) ;
  d->args << QString::fromStdString(d->krtdFolder);
  d->args << QString::fromStdString(imageListFile);
  d->args << QString::fromStdString(d->landmarksFile);

  //Parsing arguments from form

  for (int i = 0; i < this->children().size(); ++i)
  {
    if (this->children().at(i)->inherits("QCheckBox"))
    {
      QCheckBox *child = (QCheckBox*) this->children().at(i);

      if (child->isChecked())
      {
        d->addArg(child);
      }
    }
    else if (this->children().at(i)->inherits("QComboBox"))
    {
      QComboBox *child = (QComboBox*) this->children().at(i);

      d->addArg(child, child->currentText().toStdString());
    }
    else if (this->children().at(i)->inherits("QSpinBox"))
    {
      QSpinBox *child = (QSpinBox*) this->children().at(i);

      d->addArg(child,child->value());
    }
  }

  d->addArg(d->UI.lineEditOutputDirectory,d->UI.lineEditOutputDirectory->text().toStdString());


  d->dialog->show();
  d->dialog->setOutputToDisplay(d->psl);

  runningState();

  d->psl->setProcessChannelMode(QProcess::MergedChannels);
  d->psl->start(QString::fromStdString(pslPath),d->args);
}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepView::enableColorMatching()
{

  QTE_D();

  if (d->UI.comboBoxMatchCost->currentText().toStdString() == "SAD")
  {
    d->UI.checkBoxColorMatching->setEnabled(true);
    d->UI.checkBoxColorMatching->setChecked(true);
  }
  else
  {
    d->UI.checkBoxColorMatching->setEnabled(false);
    d->UI.checkBoxColorMatching->setChecked(false);
  }
}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepView::setFrameList(std::string frameList)
{
  QTE_D();

  d->frameList = frameList;
}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepView::initialState()
{
  QTE_D();

  d->args.clear();
  for (int i = 0; i < this->children().size(); ++i) {
    if (this->children().at(i)->inherits("QWidget")) {
      QWidget *child = (QWidget*) this->children().at(i);
      child->setEnabled(true);
    }
  }
  d->UI.pushButtonStop->setEnabled(false);

  d->UI.doubleSpinBoxDepthMin->setEnabled(false);
  d->UI.doubleSpinBoxDepthMax->setEnabled(false);

}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepView::runningState()
{
  QTE_D();

  for (int i = 0; i < this->children().size(); ++i) {
    if (this->children().at(i)->inherits("QWidget")) {
      QWidget *child = (QWidget*) this->children().at(i);
      child->setEnabled(false);
    }
  }
  d->UI.pushButtonStop->setEnabled(true);
}

//END LaunchPlaneSweepView

//-----------------------------------------------------------------------------
void LaunchPlaneSweepViewPrivate::addArg(QWidget *item, std::string value)
{
  std::string arg = item->property("configField").toString().toStdString();

  if(!value.empty())
  {
    arg += "=" + value;
  }
  args << QString::fromStdString(arg);

}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepViewPrivate::addArg(QWidget *item, double value)
{
  std::ostringstream sstream;
  sstream << value;
  std::string valueStr = sstream.str();

  addArg(item,valueStr);
}

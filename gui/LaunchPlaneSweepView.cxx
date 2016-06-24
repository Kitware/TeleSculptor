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
#include <QFileDialog>

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

  QString krtdFolder;
  QString framesFolder;
  QString frameList;
  QString landmarksFile;

  void addArg(QWidget* item, QString value="");
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
         d->dialog, SLOT(ouputProcess()));

  connect(d->psl, SIGNAL(finished(int)),
         this, SLOT(initialState()));

  connect(d->UI.pushButtonStop, SIGNAL(clicked(bool)),
         d->psl, SLOT(kill()));

  connect(d->UI.pushButtonExplore, SIGNAL(clicked(bool)),
         this, SLOT(openFileExplorer()));
}

//-----------------------------------------------------------------------------
LaunchPlaneSweepView::~LaunchPlaneSweepView()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepView::setKrtdFolder(QString krtdFolder)
{
  QTE_D();

  d->krtdFolder = "--krtdFolder="+krtdFolder;
}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepView::setFramesFolder(QString framesFolder)
{
  QTE_D();

  d->framesFolder = framesFolder;
}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepView::setLandmarksFile(QString landmarksFile)
{
  QTE_D();

  d->landmarksFile = "--landmarksPLY=" + landmarksFile;
}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepView::compute()
{
  QTE_D();

  QString pslPath = d->UI.lineEditPSLPath->text();

  d->args << "--debug";

  //Getting frame folder

  std::ifstream frameList(d->frameList.toStdString());
  std::string framePath, frameFolder, imageListFile;

  imageListFile = "--imageListFile=" + d->frameList.toStdString();

  frameList >> framePath;
  frameFolder = framePath.substr(0,framePath.find_last_of("/\\"));

  frameList.close();

  frameFolder = "--frameFolder=" + frameFolder;

  d->args <<  QString::fromStdString(frameFolder) ;
  d->args << d->krtdFolder;
  d->args << QString::fromStdString(imageListFile);
  d->args << d->landmarksFile;

  //Parsing arguments from form

  for (int i = 0; i < this->children().size(); ++i)
  {
    if (this->children().at(i)->inherits("QCheckBox"))
    {
      QCheckBox *child = qobject_cast<QCheckBox*>(this->children().at(i));

      if (child->isChecked())
      {
        d->addArg(child);
      }
    }
    else if (this->children().at(i)->inherits("QComboBox"))
    {
      QComboBox *child = qobject_cast<QComboBox*>(this->children().at(i));

      d->addArg(child, child->currentText());
    }
    else if (this->children().at(i)->inherits("QSpinBox"))
    {
      QSpinBox *child = qobject_cast<QSpinBox*>(this->children().at(i));

      d->addArg(child,child->value());
    }
  }

  d->addArg(d->UI.lineEditOutputDirectory, d->UI.lineEditOutputDirectory->text());


  d->dialog->show();
  d->dialog->setOutputToDisplay(d->psl);

  runningState();

  d->psl->setProcessChannelMode(QProcess::MergedChannels);
  d->psl->start(pslPath,d->args);
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
void LaunchPlaneSweepView::setFrameList(QString frameList)
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
      QWidget *child = qobject_cast<QWidget*>(this->children().at(i));
      child->setEnabled(true);
    }
  }
  d->UI.pushButtonStop->setEnabled(false);

  d->UI.doubleSpinBoxDepthMin->setEnabled(false);
  d->UI.doubleSpinBoxDepthMax->setEnabled(false);

}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepView::openFileExplorer()
{
  QTE_D();
  QString path = QFileDialog::getOpenFileName();

  d->UI.lineEditPSLPath->setText(path);

}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepView::runningState()
{
  QTE_D();

  for (int i = 0; i < this->children().size(); ++i)
  {
    if (this->children().at(i)->inherits("QWidget"))
    {
      QWidget *child = qobject_cast<QWidget*>(this->children().at(i));
      child->setEnabled(false);
    }
  }
  d->UI.pushButtonStop->setEnabled(true);
}

//END LaunchPlaneSweepView

//-----------------------------------------------------------------------------
void LaunchPlaneSweepViewPrivate::addArg(QWidget *item, QString value)
{
  QString arg = item->property("configField").toString();

  if(!value.isEmpty())
  {
    arg = arg + "=" + value;
  }
  args << arg;

}

//-----------------------------------------------------------------------------
void LaunchPlaneSweepViewPrivate::addArg(QWidget *item, double value)
{
  std::ostringstream sstream;
  sstream << value;
  std::string valueStr = sstream.str();

  addArg(item,QString::fromStdString(valueStr));
}

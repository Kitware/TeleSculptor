// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_ACTORCOLORBUTTON_H_
#define TELESCULPTOR_ACTORCOLORBUTTON_H_

#include <qtColorButton.h>

class qtUiState;

class vtkActor;

class ActorColorButtonPrivate;

class ActorColorButton : public qtColorButton
{
  Q_OBJECT

public:
  explicit ActorColorButton(QWidget* parent = nullptr);
  ~ActorColorButton() override;

  void addActor(vtkActor*);

  void persist(qtUiState&, QString const&);

public slots:
  void setColor(QColor) override;

private:
  QTE_DECLARE_PRIVATE_RPTR(ActorColorButton)
  QTE_DECLARE_PRIVATE(ActorColorButton)

  QTE_DISABLE_COPY(ActorColorButton)
};

#endif

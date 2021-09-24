// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_TRACKFEATURESSPROKITTOOL_H_
#define TELESCULPTOR_TRACKFEATURESSPROKITTOOL_H_

#include "AbstractTool.h"
#include <sstream>

class TrackFeaturesSprokitToolPrivate;

class TrackFeaturesSprokitTool : public AbstractTool
{
  Q_OBJECT

public:
  explicit TrackFeaturesSprokitTool(QObject* parent = nullptr);
  ~TrackFeaturesSprokitTool() override;
  Outputs outputs() const override;

  /// Get if the tool can be canceled.
  bool isCancelable() const override { return true; }

  bool execute(QWidget* window = nullptr) override;

protected:
  void run() override;

  virtual std::string create_pipeline_config(QWidget* window = nullptr);

private:
  QTE_DECLARE_PRIVATE_RPTR(TrackFeaturesSprokitTool)
  QTE_DECLARE_PRIVATE(TrackFeaturesSprokitTool)
  QTE_DISABLE_COPY(TrackFeaturesSprokitTool)
};

#endif

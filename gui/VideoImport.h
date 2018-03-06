/*ckwg +29
 * Copyright 2018 by Kitware, Inc.
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

#ifndef MAPTK_VIDEOINPUT_H_
#define MAPTK_VIDEOINPUT_H_

#include <maptk/local_geo_cs.h>

#include <vital/vital_types.h>
#include <vital/config/config_block_types.h>
#include <vital/logger/logger.h>
#include <vital/types/camera.h>
#include <vital/types/metadata.h>
#include <vital/types/metadata_map.h>

#include <qtGlobal.h>

#include <QMetaType>
#include <QtCore/QThread>

class VideoImportPrivate;

class VideoData
{
public:

  kwiver::vital::metadata_map::map_metadata_t metadataMap;
};

Q_DECLARE_METATYPE(std::shared_ptr<VideoData>);

class VideoImport : public QThread
{
  Q_OBJECT

public:
  typedef kwiver::vital::config_block_sptr config_block_sptr;

  VideoImport();
  virtual ~VideoImport();

  void setData(config_block_sptr const&,
               std::string const&,
               kwiver::maptk::local_geo_cs& lgcs);

signals:
  /// Emitted when the tool execution is completed.
  void completed(std::shared_ptr<VideoData>);
  /// Emitted when an intermediate update of the data is available to show progress.
  void updated(int);

public slots:
  void cancel();

protected:
  /// Execute the tool.
  ///
  /// This method must be overridden by tool implementations. The default
  /// implementation of execute() calls this method in a separate thread.
  virtual void run();

private:
  QTE_DECLARE_PRIVATE_RPTR(VideoImport)
  QTE_DECLARE_PRIVATE(VideoImport)
  QTE_DISABLE_COPY(VideoImport)
};

#endif

/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkMaptkSeedWidget.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

/**
 * @class vtkMaptkSeedWidget
 * @brief Place manual seed points
 *
 * The vtkMaptkSeedWidget is a special implementation of the vtkSeedWidget that
 * allows individual seeds to be shown inactive when disabled.
 */

#ifndef vtkMaptkSeedWidget_h
#define vtkMaptkSeedWidget_h

#include <vtkCommand.h>
#include <vtkSeedWidget.h>

class vtkMaptkSeedWidget : public vtkSeedWidget
{
public:
  /**
   * Instantiate this class
   */
  static vtkMaptkSeedWidget* New();

  //@{
  /**
   * Standard methods for a VTK class
   */
  vtkTypeMacro(vtkMaptkSeedWidget, vtkSeedWidget);
  void PrintSelf(ostream& os, vtkIndent indent) override;
  //@}

  /**
   * Use this method to programmatically create a new handle. In interactive
   * mode, (when the widget is in the PlacingSeeds state) this method is
   * automatically invoked. The method returns the handle created.
   * A valid seed representation must exist for the widget to create a new
   * handle.
   */
  virtual vtkHandleWidget* CreateNewHandle() override;

  /**
   * The method for activating and deactivating this widget. This method
   * must be overridden because it is a composite widget and does more than
   * its superclasses' vtkAbstractWidget::SetEnabled() method.
   */
  void SetEnabled(int) override;

  /**
   * Update highlighted / active seed
   */
  void HighlightActiveSeed();

  /**
   * Delete the nth seed.
   */
  void DeleteSeed(int n);

  /**
   * Custom event fired to notify that the active seed has changed.
   */
  enum vtkMaptkSeedWidgetEvents
  {
    ActiveSeedChangedEvent = vtkCommand::UserEvent + 1
  };

protected:
  vtkMaptkSeedWidget();
  ~vtkMaptkSeedWidget() override;

  // Callback interface to capture events when
  // placing the widget.
  static void AddPointAction(vtkAbstractWidget*);
  static void MoveAction(vtkAbstractWidget*);
  static void EndSelectAction(vtkAbstractWidget*);

private:
  vtkMaptkSeedWidget(const vtkMaptkSeedWidget&) = delete;
  void operator=(const vtkMaptkSeedWidget&) = delete;
};

#endif // vtkMaptkSeedWidget_h

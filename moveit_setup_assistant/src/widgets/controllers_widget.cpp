/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Mohamad Ayman.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Mohamad Ayman */

// SA
#include "controllers_widget.h"
#include <moveit_msgs/JointLimits.h>
// Qt
#include <QFormLayout>
#include <QMessageBox>
#include <QDoubleValidator>
#include <QTreeWidgetItem>
#include <QApplication>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// Outer User Interface for MoveIt Configuration Assistant
// ******************************************************************************************
ROSControllersWidget::ROSControllersWidget(QWidget* parent, moveit_setup_assistant::MoveItConfigDataPtr config_data)
  : SetupScreenWidget(parent), config_data_(config_data)
{
  // Basic widget container
  // QWidget* edit_widget = new QWidget(this);
  QVBoxLayout* layout = new QVBoxLayout();

  layout->setAlignment(Qt::AlignTop);

  // Top Header Area ------------------------------------------------
  HeaderWidget* header =
      new HeaderWidget("Controllers settings", "Configuring MoveIt! with the controllers on your robotConfigure MoveIt!"
      											"to use ros_control and integrate with Gazebo. ",
                       this);
  layout->addWidget(header);

  // Tree Box ----------------------------------------------------------------------
  controllers_tree_widget_ = createContentsWidget();

  // Joints edit widget
  joints_widget_ = new DoubleListWidget(this, config_data_, "Joint Collection", "Joint");
  connect(joints_widget_, SIGNAL(cancelEditing()), this, SLOT(cancelEditing()));
  connect(joints_widget_, SIGNAL(doneEditing()), this, SLOT(saveJointsScreen()));
  connect(joints_widget_, SIGNAL(previewSelected(std::vector<std::string>)), this,
          SLOT(previewSelectedJoints(std::vector<std::string>)));

  // Joints Groups Widget
  joint_groups_widget_ = new DoubleListWidget(this, config_data_, "Group Joints Collection", "Group");
  connect(joint_groups_widget_, SIGNAL(cancelEditing()), this, SLOT(cancelEditing()));
  connect(joint_groups_widget_, SIGNAL(doneEditing()), this, SLOT(saveJointsGroupsScreen()));
  connect(joint_groups_widget_, SIGNAL(previewSelected(std::vector<std::string>)), this,
          SLOT(previewSelectedGroup(std::vector<std::string>)));

  // Controller Edit Widget
  controller_edit_widget_ = new ControllerEditWidget(this, config_data_);
  connect(controller_edit_widget_, SIGNAL(cancelEditing()), this, SLOT(cancelEditing()));
  connect(controller_edit_widget_, SIGNAL(deleteController()), this, SLOT(deleteController()));
  connect(controller_edit_widget_, SIGNAL(save()), this, SLOT(saveControllerScreenEdit()));
  connect(controller_edit_widget_, SIGNAL(saveJoints()), this, SLOT(saveControllerScreenJoints()));
  connect(controller_edit_widget_, SIGNAL(saveJointsGroups()), this, SLOT(saveControllerScreenGroups()));

  // Combine into stack
  stacked_layout_ = new QStackedLayout(this);
  stacked_layout_->addWidget(controllers_tree_widget_);  // screen index 0
  stacked_layout_->addWidget(joints_widget_);           // screen index 1
  stacked_layout_->addWidget(controller_edit_widget_);  // screen index 2
  stacked_layout_->addWidget(joint_groups_widget_);     // screen index 3

  // Finish GUI -----------------------------------------------------------

  // Create Widget wrapper for layout
  QWidget* stacked_layout_widget = new QWidget(this);
  stacked_layout_widget->setLayout(stacked_layout_);

  layout->addWidget(stacked_layout_widget);

  this->setLayout(layout);
}

// ******************************************************************************************
// Create the main tree view widget
// ******************************************************************************************
QWidget* ROSControllersWidget::createContentsWidget()
{
  // Main widget
  QWidget* content_widget = new QWidget(this);

  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout(this);

  QHBoxLayout* upper_controls_layout = new QHBoxLayout();

  // Add default controller
  QPushButton* btn_add_default = new QPushButton("&Auto Add FollowJointsTrajectory Controllers For Each Planning Group", this);
  btn_add_default->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  btn_add_default->setMaximumWidth(600);
  connect(btn_add_default, SIGNAL(clicked()), this, SLOT(addDefaultControllers()));
  upper_controls_layout->addWidget(btn_add_default);
  upper_controls_layout->setAlignment(btn_add_default, Qt::AlignRight);

  // Add Controls to layout
  layout->addLayout(upper_controls_layout);

  // Tree Box ----------------------------------------------------------------------
  controllers_tree_ = new QTreeWidget(this);
  controllers_tree_->setHeaderLabel("Current Controllers");
  connect(controllers_tree_, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)), this, SLOT(editSelected()));
  layout->addWidget(controllers_tree_);
  connect(controllers_tree_, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(previewSelected()));
  layout->addWidget(controllers_tree_);

  // Bottom Controls -------------------------------------------------------------

  QHBoxLayout* controls_layout = new QHBoxLayout();

  // Expand/Contract controls
  QLabel* expand_controls = new QLabel(this);
  expand_controls->setText("<a href='expand'>Expand All</a> <a href='contract'>Collapse All</a>");
  connect(expand_controls, SIGNAL(linkActivated(const QString)), this, SLOT(alterTree(const QString)));
  controls_layout->addWidget(expand_controls);

  // Spacer
  QWidget* spacer = new QWidget(this);
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  controls_layout->addWidget(spacer);

  // Delete
  QPushButton* btn_delete_ = new QPushButton("&Delete Controller", this);
  btn_delete_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  btn_delete_->setMaximumWidth(200);
  connect(btn_delete_, SIGNAL(clicked()), this, SLOT(deleteController()));
  controls_layout->addWidget(btn_delete_);
  controls_layout->setAlignment(btn_delete_, Qt::AlignRight);

  // Add Controller Button
  QPushButton* btn_add_ = new QPushButton("&Add Controller", this);
  btn_add_->setMaximumWidth(300);
  connect(btn_add_, SIGNAL(clicked()), this, SLOT(addController()));
  controls_layout->addWidget(btn_add_);
  controls_layout->setAlignment(btn_add_, Qt::AlignRight);

  // Edit
  QPushButton* btn_edit_ = new QPushButton("&Edit Selected", this);
  btn_edit_->setMaximumWidth(300);
  connect(btn_edit_, SIGNAL(clicked()), this, SLOT(editSelected()));
  controls_layout->addWidget(btn_edit_);
  controls_layout->setAlignment(btn_edit_, Qt::AlignRight);

  // Add Controls to layout
  layout->addLayout(controls_layout);

  // Set layout
  content_widget->setLayout(layout);

  return content_widget;
}

// ******************************************************************************************
// Displays data in the ros_controllers_config_ data structure into a QtTableWidget
// ******************************************************************************************
void ROSControllersWidget::loadControllersTree()
{
  // Disable Tree
  controllers_tree_->setUpdatesEnabled(false);  // prevent table from updating until we are completely done
  controllers_tree_->setDisabled(true);         // make sure we disable it so that the cellChanged event is not called
  controllers_tree_->clear();                   // reset the tree

  // Display all controllers by looping through them
  for (std::vector<ROSControlConfig>::iterator controller_it = config_data_->getROSControllers()->begin();
       controller_it != config_data_->getROSControllers()->end(); ++controller_it)
  {
    loadToControllersTree(*controller_it);
  }

  // Reenable Tree
  controllers_tree_->setUpdatesEnabled(true);  // prevent table from updating until we are completely done
  controllers_tree_->setDisabled(false);       // make sure we disable it so that the cellChanged event is not called
  current_edit_controller_.clear();            // no controller is being edited
  alterTree("expand");
}

// ******************************************************************************************
//  Displays data in the controller_config_ data structure into a QtTableWidget
// ******************************************************************************************
void ROSControllersWidget::loadToControllersTree(const ROSControlConfig& controller_it)
{
  // Fonts for tree
  const QFont top_level_font(QFont().defaultFamily(), 11, QFont::Bold);
  const QFont type_font(QFont().defaultFamily(), 11, QFont::Normal, QFont::StyleItalic);

  QTreeWidgetItem* controller;

  controller = new QTreeWidgetItem();
  controller->setText(0, controller_it.name_.c_str());
  controller->setFont(0, top_level_font);
  controller->setData(0, Qt::UserRole, QVariant::fromValue(0));
  controllers_tree_->addTopLevelItem(controller);

  if (!controller_it.joints_.empty())
  {
    // Joints --------------------------------------------------------------
    QTreeWidgetItem* joints = new QTreeWidgetItem(controller);
    joints->setText(0, "Joints");
    joints->setFont(0, type_font);
    joints->setData(0, Qt::UserRole, QVariant::fromValue(1));
    controller->addChild(joints);

    // Loop through all aval. joints
    for (std::vector<std::string>::const_iterator joint_it = controller_it.joints_.begin();
         joint_it != controller_it.joints_.end(); ++joint_it)
    {
      QTreeWidgetItem* joint_item = new QTreeWidgetItem(joints);
      joint_item->setData(0, Qt::UserRole, QVariant::fromValue(2));

      // Add to tree
      joint_item->setText(0, (*joint_it).c_str());
      joints->addChild(joint_item);
    }
  }
}

// ******************************************************************************************
// Called when setup assistant navigation switches to this screen
// ******************************************************************************************
void ROSControllersWidget::focusGiven()
{
  // load controllers tree
  loadControllersTree();
}

// ******************************************************************************************
// Load the popup screen with correct data for joints
// ******************************************************************************************
void ROSControllersWidget::loadJointsScreen(ROSControlConfig* this_controller)
{
  // Retrieve pointer to the shared kinematic model
  const robot_model::RobotModelConstPtr& model = config_data_->getRobotModel();

  // Get the names of the all joints
  const std::vector<std::string>& joints = model->getJointModelNames();

  if (joints.size() == 0)
  {
    QMessageBox::critical(this, "Error Loading", "No joints found for robot model");
    return;
  }

  // Set the available joints (left box)
  joints_widget_->setAvailable(joints);

  // Set the selected joints (right box)
  joints_widget_->setSelected(this_controller->joints_);

  // Set the title
  joints_widget_->title_->setText(
      QString("Edit '").append(QString::fromUtf8(this_controller->name_.c_str())).append("' Joint Collection"));

  // Remember what is currently being edited so we can later save changes
  current_edit_controller_ = this_controller->name_;
}

// ******************************************************************************************
// Load the popup screen with correct data for groups
// ******************************************************************************************
void ROSControllersWidget::loadGroupsScreen(ROSControlConfig* this_controller)
{
  // Load all groups into the subgroup screen
  std::vector<std::string> groups;

  // Display all groups by looping through them
  for (std::vector<srdf::Model::Group>::const_iterator group_it = config_data_->srdf_->groups_.begin();
       group_it != config_data_->srdf_->groups_.end(); ++group_it)
  {
    // add to available groups list
    groups.push_back(group_it->name_);
  }

  // Set the available groups (left box)
  joint_groups_widget_->setAvailable(groups);

  // Set the selected groups (right box)
  joint_groups_widget_->setSelected(this_controller->joints_);

  // Set the title
  joint_groups_widget_->title_->setText(
      QString("Edit '").append(QString::fromUtf8(this_controller->name_.c_str())).append("' Joints groups collection"));

  // Remember what is currently being edited so we can later save changes
  current_edit_controller_ = this_controller->name_;
}

// ******************************************************************************************
// Delete a controller
// ******************************************************************************************
void ROSControllersWidget::deleteController()
{
  std::string controller_name = current_edit_controller_;

  if (controller_name.empty())
  {
    QTreeWidgetItem* item = controllers_tree_->currentItem();
    // Check that something was actually selected
    if (item == NULL)
      return;

    // Get the user custom properties of the currently selected row
    int type_ = item->data(0, Qt::UserRole).value<int>();
    if (type_ == 0)
      controller_name = item->text(0).toUtf8().constData();
  }
  else
    current_edit_controller_.clear();
  if (controller_name.empty())
    return;

  // Confirm user wants to delete controller
  if (QMessageBox::question(this, "Confirm Controller Deletion",
                            QString("Are you sure you want to delete the controller '").append(controller_name.c_str()),
                            QMessageBox::Ok | QMessageBox::Cancel) == QMessageBox::Cancel)
  {
    return;
  }
  // delete actual controller
  if (config_data_->deleteROSController(controller_name))
  {
    ROS_INFO_STREAM_NAMED("Setup Assistnat", "Controller " << controller_name << " deleted succefully");
  }
  else
  {
    ROS_WARN_STREAM_NAMED("Setup Assistnat", "Couldn't delete Controller ");
  }
  
  // Switch to main screen
  showMainScreen();

  // Reload main screen table
  loadControllersTree();
}

// ******************************************************************************************
// Create a new, controller
// ******************************************************************************************
void ROSControllersWidget::addController()
{
  // New Controller
  adding_new_controller_ = true;

  // Load the data
  loadControllerScreen(NULL);  // NULL indicates this is a new controller, not an existing one
  changeScreen(2);
}

// ******************************************************************************************
// Add a Follow Joint Trajectory action Controller for each Planning Group
// ******************************************************************************************
void ROSControllersWidget::addDefaultControllers()
{
  config_data_->addDefaultControllers();
  loadControllersTree();
}

// ******************************************************************************************
// Load the popup screen with correct data for controllers
// ******************************************************************************************
void ROSControllersWidget::loadControllerScreen(ROSControlConfig* this_controller)
{
  // Load the avail controllers. This function only runs once
  controller_edit_widget_->loadControllersTypesComboBox();

  if (this_controller == NULL)  // this is a new screen
  {
    current_edit_controller_.clear();  // provide a blank controller name
    controller_edit_widget_->title_->setText("Create New Controller");
    controller_edit_widget_->btn_delete_->hide();
    controller_edit_widget_->new_buttons_widget_->show();  // helps user choose next step
    controller_edit_widget_->btn_save_->hide();            // this is only for edit mode
  }
  else  // load the controller name into the widget
  {
    current_edit_controller_ = this_controller->name_;
    controller_edit_widget_->title_->setText(
        QString("Edit Controller '").append(current_edit_controller_.c_str()).append("'"));
    controller_edit_widget_->btn_delete_->show();
    controller_edit_widget_->new_buttons_widget_->hide();  // not necessary for existing controllers
    controller_edit_widget_->btn_save_->show();            // this is only for edit mode
  }

  // Set the data in the edit box
  controller_edit_widget_->setSelected(current_edit_controller_);
}

// ******************************************************************************************
// Call when edit screen is canceled
// ******************************************************************************************
void ROSControllersWidget::cancelEditing()
{
  if (!current_edit_controller_.empty() && adding_new_controller_)
  {
    ROSControlConfig* editing = config_data_->findROSController(current_edit_controller_);
    if (editing && editing->joints_.empty())
    {
      config_data_->deleteROSController(current_edit_controller_);
      current_edit_controller_.clear();
      // Load the data to the tree
      loadControllersTree();
    }
  }
  else
  {
    current_edit_controller_.clear();
  }

  // Switch to main screen
  showMainScreen();
}

// ******************************************************************************************
// Called from Double List widget to highlight joints
// ******************************************************************************************
void ROSControllersWidget::previewSelectedJoints(std::vector<std::string> joints)
{
  // Unhighlight all links
  Q_EMIT unhighlightAll();

  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    const robot_model::JointModel* joint_model = config_data_->getRobotModel()->getJointModel(joints[i]);

    // Check that a joint model was found
    if (!joint_model)
    {
      continue;
    }

    // Get the name of the link
    const std::string link = joint_model->getChildLinkModel()->getName();

    if (link.empty())
    {
      continue;
    }

    // Highlight link
    Q_EMIT highlightLink(link, QColor(255, 0, 0));
  }
}

// ******************************************************************************************
// Called from Double List widget to highlight a group
// ******************************************************************************************
void ROSControllersWidget::previewSelectedGroup(std::vector<std::string> groups)
{
  // Unhighlight all links
  Q_EMIT unhighlightAll();

  for (std::size_t i = 0; i < groups.size(); ++i)
  {
    // Highlight group
    Q_EMIT highlightGroup(groups[i]);
  }
}

// ******************************************************************************************
// Called from Double List widget to highlight a subgroup
// ******************************************************************************************
void ROSControllersWidget::previewSelected()
{
}

// ******************************************************************************************
// Call when a new controller is created and ready to progress to next screen
// ******************************************************************************************
void ROSControllersWidget::saveControllerScreenJoints()
{
  // Save the controller
  if (!saveControllerScreen())
    return;

  // Find the controller we are editing based on the controller name string
  ROSControlConfig* editing_controller = config_data_->findROSController(current_edit_controller_);

  loadJointsScreen(editing_controller);

  // Switch to screen
  changeScreen(1);  // 1 is index of joints
}

// ******************************************************************************************
// Call when a new controller is created and ready to progress to next screen
// ******************************************************************************************
void ROSControllersWidget::saveControllerScreenGroups()
{
  // Save the controller
  if (!saveControllerScreen())
    return;

  // Find the controller we are editing based on the controller name string
  ROSControlConfig* editing_controller = config_data_->findROSController(current_edit_controller_);

  loadGroupsScreen(editing_controller);

  // Switch to screen
  changeScreen(3);  // 3 is index of groups
}

// ******************************************************************************************
// Call when joints edit sceen is done and needs to be saved
// ******************************************************************************************
void ROSControllersWidget::saveJointsScreen()
{
  // Find the controller we are editing based on the controller name string
  ROSControlConfig* searched_controller = config_data_->findROSController(current_edit_controller_);

  // clear the old data
  searched_controller->joints_.clear();

  // copy the data
  for (int i = 0; i < joints_widget_->selected_data_table_->rowCount(); ++i)
  {
    searched_controller->joints_.push_back(joints_widget_->selected_data_table_->item(i, 0)->text().toStdString());
  }

  // Switch to main screen
  showMainScreen();

  // Reload main screen table
  loadControllersTree();
}

// ******************************************************************************************
// Call when joints edit sceen is done and needs to be saved
// ******************************************************************************************
void ROSControllersWidget::saveJointsGroupsScreen()
{
  // Find the controller we are editing based on the controller name string
  ROSControlConfig* searched_controller = config_data_->findROSController(current_edit_controller_);

  // clear the old data
  searched_controller->joints_.clear();

  // copy the data
  for (int i = 0; i < joint_groups_widget_->selected_data_table_->rowCount(); ++i)
  {
    // Get list of associated joints
    const robot_model::JointModelGroup* joint_model_group = config_data_->getRobotModel()->getJointModelGroup(
        joint_groups_widget_->selected_data_table_->item(i, 0)->text().toStdString());
    const std::vector<const robot_model::JointModel*>& joint_models = joint_model_group->getActiveJointModels();

    // Iterate through the joints
    for (const robot_model::JointModel* joint : joint_models)
    {
      if (joint->isPassive() || joint->getMimic() != NULL || joint->getType() == robot_model::JointModel::FIXED)
        continue;
      searched_controller->joints_.push_back(joint->getName());
    }
  }

  // Switch to main screen
  showMainScreen();

  // Reload main screen table
  loadControllersTree();
}

// ******************************************************************************************
// Call when joints edit sceen is done and needs to be saved
// ******************************************************************************************
void ROSControllersWidget::saveControllerScreenEdit()
{
  // Save the controller
  if (!saveControllerScreen())
    return;

  // Switch to main screen
  showMainScreen();
}

// ******************************************************************************************
// Call when controller edit sceen is done and needs to be saved
// ******************************************************************************************
bool ROSControllersWidget::saveControllerScreen()
{
  // Get a reference to the supplied strings
  const std::string& controller_name_ = controller_edit_widget_->controller_name_field_->text().trimmed().toStdString();
  const std::string& controller_type_ = controller_edit_widget_->controller_type_field_->currentText().toStdString();

  // Used for editing existing controllers
  ROSControlConfig* searched_controller = NULL;

  // Check that a valid controller name has been given
  if (controller_name_.empty())
  {
    QMessageBox::warning(this, "Error Saving", "A name must be given for the controller!");
    return false;
  }

  // Check if this is an existing controller
  if (!current_edit_controller_.empty())
  {
    // Find the controller we are editing based on the goup name string
    searched_controller = config_data_->findROSController(current_edit_controller_);
  }

  // Check that the controller name is unique
  for (std::vector<ROSControlConfig>::const_iterator controller_it = config_data_->getROSControllers()->begin();
       controller_it != config_data_->getROSControllers()->end(); ++controller_it)
  {
    if (controller_it->name_.compare(controller_name_) == 0)  // the names are the same
    {
      // is this our existing controller? check if controller pointers are same
      if (&(*controller_it) != searched_controller)
      {
        QMessageBox::warning(this, "Error Saving", "A controller already exists with that name!");
        return false;
      }
    }
  }

  adding_new_controller_ = false;

  // Save the new controller name or create the new controller
  if (searched_controller == NULL)  // create new
  {
    ROSControlConfig new_controller;
    new_controller.name_ = controller_name_;
    new_controller.type_ = controller_type_;
    config_data_->addROSController(new_controller);

    adding_new_controller_ = true;  // remember this controller is new
  }
  else
  {
    // Remember old controller name and type
    const std::string old_controller_name = searched_controller->name_;

    // Change controller name
    searched_controller->name_ = controller_name_;
    searched_controller->type_ = controller_type_;
  }

  // Reload main screen table
  loadControllersTree();

  // Update the current edit controller so that we can proceed to the next screen, if user desires
  current_edit_controller_ = controller_name_;

  return true;
}

// ******************************************************************************************
// Edit whatever element is selected in the tree view
// ******************************************************************************************
void ROSControllersWidget::editSelected()
{
  QTreeWidgetItem* item = controllers_tree_->currentItem();
  QTreeWidgetItem* controller_item;
  // Check that something was actually selected
  if (item == NULL)
    return;

  adding_new_controller_ = false;

  int type = item->data(0, Qt::UserRole).value<int>();

  // Get the user custom properties of the currently selected row
  if (type == 2)
  {
    // To which controller does this joint belong
    controller_item = item->parent()->parent();
    current_edit_controller_ = controller_item->text(0).toUtf8().constData();

    ROSControlConfig* this_controller = config_data_->findROSController(current_edit_controller_);
    // Load the data
    loadJointsScreen(this_controller);

    // Switch to screen
    changeScreen(1);  // 1 is index of joints
  }
  else if (type == 1)
  {
    // To which controller do the joints belong
    controller_item = item->parent();
    current_edit_controller_ = controller_item->text(0).toUtf8().constData();

    ROSControlConfig* this_controller = config_data_->findROSController(current_edit_controller_);
    // Load the data
    loadJointsScreen(this_controller);

    // Switch to screen
    changeScreen(1);  // 1 is index of joints
  }
  else if (type == 0)
  {
    // Load the data
    current_edit_controller_ = item->text(0).toUtf8().constData();

    ROSControlConfig* this_controller = config_data_->findROSController(current_edit_controller_);

    loadControllerScreen(this_controller);

    // Switch to screen
    changeScreen(2);
  }
  else
  {
    QMessageBox::critical(this, "Error Loading", "An internal error has occured while loading.");
  }
}

// ******************************************************************************************
// Edit a controller
// ******************************************************************************************
void ROSControllersWidget::editController()
{
  QTreeWidgetItem* item = controllers_tree_->currentItem();

  // Check that something was actually selected
  if (item == NULL)
    return;

  adding_new_controller_ = false;

  loadControllerScreen(config_data_->findROSController(current_edit_controller_));
  changeScreen(2);
}

// ******************************************************************************************
// Switch to current controllers view
// ******************************************************************************************
void ROSControllersWidget::showMainScreen()
{
  stacked_layout_->setCurrentIndex(0);

  // Announce that this widget is not in modal mode
  Q_EMIT isModal(false);
}

// ******************************************************************************************
// Switch which screen is being shown
// ******************************************************************************************
void ROSControllersWidget::changeScreen(int index)
{
  stacked_layout_->setCurrentIndex(index);

  // Announce this widget's mode
  Q_EMIT isModal(index != 0);
}

// ******************************************************************************************
// Expand/Collapse Tree
// ******************************************************************************************
void ROSControllersWidget::alterTree(const QString& link)
{
  if (link.contains("expand"))
    controllers_tree_->expandAll();
  else
    controllers_tree_->collapseAll();
}

}  // namespace

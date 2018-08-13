/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Dave Coleman, Michael 'v4hn' Goerner */


// Qt
#include <QVBoxLayout>
#include <QPushButton>
#include <QMessageBox>
#include <QApplication>
#include <QSplitter>

#include "author_plugin.h"

// ROS
#include <srdfdom/model.h>  // use their struct datastructures
#include <ros/console.h>
#include <ros/ros.h>
// Boost
#include <boost/algorithm/string.hpp>  // for trimming whitespace from user input
#include <boost/filesystem.hpp>        // for creating folders/files
// Read write files
#include <iostream>  // For writing yaml and launch files
#include <fstream>

// plugin lib
// #include <pluginlib/class_list_macros.h>

#include <class_loader/class_loader.hpp>


namespace moveit_setup_assistant
{
// Boost file system
namespace fs = boost::filesystem;

    // ******************************************************************************************
    // Outer User Interface for MoveIt Configuration Assistant
    // ******************************************************************************************
    AuthorPlugin::AuthorPlugin()
    {
        ROS_ERROR_STREAM("Loaded the author plugin");
    }

  void AuthorPlugin::initialize(QWidget* parent, moveit_setup_assistant::MoveItConfigDataPtr config_data)
  {
    // QVBoxLayout* layout = new QVBoxLayout();
    // layout->setAlignment(Qt::AlignTop);

    // // Top Header Area ------------------------------------------------

    // HeaderWidget* header =
    //     new HeaderWidget("Author Information", "Specify contact information of the author and initial maintainer of the "
    //                                             "generated package. catkin requires valid details in the package's "
    //                                             "package.xml",
    //                     this);
    // layout->addWidget(header);

    // QLabel* name_title = new QLabel(this);
    // name_title->setText("Name of the maintainer this MoveIt! configuration:");
    // layout->addWidget(name_title);

    // name_edit_ = new QLineEdit(this);
    // connect(name_edit_, SIGNAL(editingFinished()), this, SLOT(edited_name()));
    // layout->addWidget(name_edit_);

    // QLabel* email_title = new QLabel(this);
    // email_title->setText("Email of the maintainer of this MoveIt! configuration:");
    // layout->addWidget(email_title);

    // email_edit_ = new QLineEdit(this);
    // connect(email_edit_, SIGNAL(editingFinished()), this, SLOT(edited_email()));
    // layout->addWidget(email_edit_);

    // // Finish Layout --------------------------------------------------
    // this->setLayout(layout);
  }
//   : SetupScreenWidget(parent), config_data_(config_data)

}  // namespace
// PLUGINLIB_EXPORT_CLASS(moveit_setup_assistant::AuthorPlugin, moveit_setup_assistant::SetupAssistantWidget)
CLASS_LOADER_REGISTER_CLASS(moveit_setup_assistant::AuthorPlugin, moveit_setup_assistant::SetupAssistantWidget)


// PLUGINLIB_EXPORT_CLASS(moveit_setup_assistant, moveit_setup_assistant::AuthorPlugin);

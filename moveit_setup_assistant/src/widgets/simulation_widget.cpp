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
#include "simulation_widget.h"

// Qt
#include <QVBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QColor>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <regex>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// Constructor
// ******************************************************************************************
SimulationWidget::SimulationWidget(QWidget* parent, moveit_setup_assistant::MoveItConfigDataPtr config_data)
  : SetupScreenWidget(parent), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();
  layout->setAlignment(Qt::AlignTop);

  // Top Header Area ------------------------------------------------

  HeaderWidget* header = new HeaderWidget("Simulation", "Shows what changes that should be made for the robot urdf to "
                                                        "be spawned and rendered "
                                                        "correctly in gazebo simulator. "
                                                        " Green colored text are the changes.",
                                          this);
  layout->addWidget(header);

  // Spacing
  QSpacerItem* blank_space = new QSpacerItem(1, 5);
  layout->addSpacerItem(blank_space);

  // Used to make the new URDF visible
  QPushButton* btn_generate = new QPushButton("&Generate URDF", this);
  btn_generate->setMinimumWidth(180);
  btn_generate->setMinimumHeight(40);
  layout->addWidget(btn_generate);
  layout->setAlignment(btn_generate, Qt::AlignLeft);
  connect(btn_generate, SIGNAL(clicked()), this, SLOT(generateURDFClick()));

  no_changes_ = new QLabel(this);
  no_changes_->setText("No Changes To Be Made");
  layout->addWidget(no_changes_);
  no_changes_->setVisible(false);

  simulation_text_ = new QTextEdit(this);
  layout->addWidget(simulation_text_);
  simulation_text_->setVisible(false);

  QLabel* instructions = new QLabel(this);
  instructions->setText("You can run 'roscd <robot_name>_description' to find where the robot urdf file is.");
  layout->addWidget(instructions);

  // Prepare the urdf string
  config_data_->urdfGazeboCompatible();

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
}

// ******************************************************************************************
// Called when generate URDF button is clicked
// ******************************************************************************************
void SimulationWidget::generateURDFClick()
{
  simulation_text_->setVisible(true);
}

// ******************************************************************************************
// Called when setup assistant navigation switches to this screen
// ******************************************************************************************
void SimulationWidget::focusGiven()
{
  // Check if the urdf do need new elements to be added
  if (config_data_->gazebo_compatible_urdf_string_.length() > 0)
  {
    // Split the added elements from the original urdf to view them in different colors
    std::smatch start_match;
    std::smatch end_match;
    std::regex start_reg_ex("<inertial");
    std::regex end_reg_ex("</inertial");

    // Search for inertial elemnts using regex
    std::regex_search(config_data_->gazebo_compatible_urdf_string_, start_match, start_reg_ex);
    std::regex_search(config_data_->gazebo_compatible_urdf_string_, end_match, end_reg_ex);

    std::string urdf_sub_string;

    // Used to cache the positions of the opening and closing of the inertial elements
    std::vector<int> inertial_opening_matches;
    std::vector<int> inertial_closing_matches;

    inertial_closing_matches.push_back(0);

    // Cache the positions of the openings of the inertial elements
    for (auto it = std::sregex_iterator(config_data_->gazebo_compatible_urdf_string_.begin(),
                                        config_data_->gazebo_compatible_urdf_string_.end(), start_reg_ex);
         it != std::sregex_iterator(); ++it)
    {
      inertial_opening_matches.push_back(it->position());
    }

    inertial_opening_matches.push_back(config_data_->gazebo_compatible_urdf_string_.length());

    // Cache the positions of the closings of the inertial elements
    for (auto it = std::sregex_iterator(config_data_->gazebo_compatible_urdf_string_.begin(),
                                        config_data_->gazebo_compatible_urdf_string_.end(), end_reg_ex);
         it != std::sregex_iterator(); ++it)
    {
      inertial_closing_matches.push_back(it->position());
    }

    for (size_t i = 0; i < inertial_opening_matches.size(); i++)
    {
      // Show the unmodified elements in black
      this->simulation_text_->setTextColor(QColor("black"));
      urdf_sub_string = config_data_->gazebo_compatible_urdf_string_.substr(
          inertial_closing_matches[i], inertial_opening_matches[i] - inertial_closing_matches[i]);
      this->simulation_text_->append(QString(urdf_sub_string.c_str()));

      // Show the added elements in green
      this->simulation_text_->setTextColor(QColor("green"));
      urdf_sub_string = config_data_->gazebo_compatible_urdf_string_.substr(
          inertial_opening_matches[i], inertial_closing_matches[i + 1] - inertial_opening_matches[i] + 11);
      this->simulation_text_->append(QString(urdf_sub_string.c_str()));
      inertial_closing_matches[i + 1] += 11;
    }
  }
  else
  {
    this->simulation_text_->append(QString(config_data_->gazebo_compatible_urdf_string_.c_str()));
    no_changes_->setVisible(true);
  }
}

}  // namespace
